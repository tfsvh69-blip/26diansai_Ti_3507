#include "app_vision_link.h"

#include "FreeRTOS.h"
#include "task.h"

#include "ball_parser.h"
#include "bsp_uart.h"

#define VISION_BOOT_PING_COUNT       (3U)
#define VISION_BOOT_PING_INTERVAL_MS (500U)
#define VISION_OFFLINE_MS            (3000U)
#define VISION_X_TIMEOUT_MS          (100U)

static uint16_t s_nextPingId;
static uint16_t s_waitPongId;
static uint16_t s_runId;
static uint8_t  s_activeTaskId;
static uint16_t s_lastTaskRunId;
static uint8_t  s_lastTaskId;
static uint8_t  s_bootPingCount;
static bool     s_lastTaskStart;
static uint32_t s_seenPongSeq;
static uint32_t s_seenXSeq;
static uint32_t s_lastPingTick;
static uint32_t s_lastPongTick;
static uint32_t s_lastXTick;
static bool     s_hasPong;
static bool     s_pingSent;
static bool     s_xReceived;
static bool     s_xValid;
static bool     s_xNa;
static int16_t  s_xPixel;
static char     s_xText[APP_VISION_X_TEXT_MAX + 1U];

static void AppVisionLink_CopyXText(char *dst, const char *src)
{
    uint32_t i;

    for (i = 0U; i < APP_VISION_X_TEXT_MAX; i++) {
        dst[i] = src[i];
        if (src[i] == '\0') {
            return;
        }
    }
    dst[APP_VISION_X_TEXT_MAX] = '\0';
}

static bool AppVisionLink_Elapsed(uint32_t now, uint32_t then, uint32_t timeout)
{
    return (uint32_t)(now - then) >= pdMS_TO_TICKS(timeout);
}

static uint8_t AppVisionLink_Xor(const char *payload)
{
    uint8_t checksum = 0U;

    while (*payload != '\0') {
        checksum ^= (uint8_t)*payload++;
    }
    return checksum;
}

static void AppVisionLink_SendFrame(const char *payload)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t checksum = AppVisionLink_Xor(payload);

    BspUart0_Lock();
    BspUart0_SendByte('$');
    BspUart0_SendString(payload);
    BspUart0_SendByte('*');
    BspUart0_SendByte((uint8_t)hex[checksum >> 4]);
    BspUart0_SendByte((uint8_t)hex[checksum & 0x0FU]);
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

static void AppVisionLink_SendPing(uint16_t id)
{
    char payload[12];
    uint8_t idx = 0U;
    uint16_t divisor = 10000U;
    bool started = false;

    payload[idx++] = 'P'; payload[idx++] = 'I'; payload[idx++] = 'N'; payload[idx++] = 'G'; payload[idx++] = ',';
    do {
        uint8_t digit = (uint8_t)(id / divisor);
        if ((digit != 0U) || started || (divisor == 1U)) {
            payload[idx++] = (char)('0' + digit);
            started = true;
        }
        id = (uint16_t)(id % divisor);
        divisor = (uint16_t)(divisor / 10U);
    } while (divisor != 0U);
    payload[idx] = '\0';
    AppVisionLink_SendFrame(payload);
}

static void AppVisionLink_SendTask(uint16_t runId, uint8_t taskId, bool start)
{
    char payload[28];
    uint8_t idx = 0U;
    uint16_t divisor = 10000U;
    bool started = false;

    payload[idx++] = 'T'; payload[idx++] = 'A'; payload[idx++] = 'S'; payload[idx++] = 'K'; payload[idx++] = ',';
    do {
        uint8_t digit = (uint8_t)(runId / divisor);
        if ((digit != 0U) || started || (divisor == 1U)) {
            payload[idx++] = (char)('0' + digit);
            started = true;
        }
        runId = (uint16_t)(runId % divisor);
        divisor = (uint16_t)(divisor / 10U);
    } while (divisor != 0U);
    payload[idx++] = ',';
    payload[idx++] = (char)('0' + taskId);
    payload[idx++] = ',';
    if (start) {
        payload[idx++] = 'S'; payload[idx++] = 'T'; payload[idx++] = 'A'; payload[idx++] = 'R'; payload[idx++] = 'T';
    } else {
        payload[idx++] = 'S'; payload[idx++] = 'T'; payload[idx++] = 'O'; payload[idx++] = 'P';
    }
    payload[idx] = '\0';
    AppVisionLink_SendFrame(payload);
}

void AppVisionLink_Init(void)
{
    VisionParser_Reset();
    BspUart0_SetRxHandler(VisionParser_FeedByte);
    s_nextPingId = 0U;
    s_waitPongId = 0U;
    s_runId = 0U;
    s_activeTaskId = 0U;
    s_lastTaskRunId = 0U;
    s_lastTaskId = 0U;
    s_bootPingCount = 0U;
    s_lastTaskStart = false;
    s_seenPongSeq = 0U;
    s_seenXSeq = 0U;
    s_lastPingTick = 0U;
    s_lastPongTick = 0U;
    s_lastXTick = 0U;
    s_hasPong = false;
    s_pingSent = false;
    s_xReceived = false;
    s_xValid = false;
    s_xNa = false;
    s_xPixel = 0;
    s_xText[0] = '\0';
}

void AppVisionLink_Service(void)
{
    VisionData_t data;
    TickType_t now = xTaskGetTickCount();

    (void)VisionParser_GetLatest(&data);
    if ((data.pongSeq != s_seenPongSeq) && data.pongValid) {
        s_seenPongSeq = data.pongSeq;
        if (data.pongId == s_waitPongId) {
            s_lastPongTick = now;
            s_hasPong = true;
        }
    }
    if ((data.xSeq != s_seenXSeq) && data.xReceived) {
        s_seenXSeq = data.xSeq;
        s_lastXTick = now;
        s_xReceived = true;
        s_xValid = data.xValid;
        s_xNa = data.xText[0] == 'N' && data.xText[1] == 'A' && data.xText[2] == '\0';
        s_xPixel = data.xPixel;
        AppVisionLink_CopyXText(s_xText, data.xText);
    }
    if (s_hasPong && AppVisionLink_Elapsed(now, s_lastPongTick, VISION_OFFLINE_MS)) {
        s_hasPong = false;
    }
    /*
     * 上电联调只发三次 PING，防止未接树莓派时持续占用 UART0。
     * PONG、X 和 TASK/ACK 的接收路径在三次发送完成后仍持续工作。
     */
    if ((s_bootPingCount < VISION_BOOT_PING_COUNT) &&
        (!s_pingSent || AppVisionLink_Elapsed(now, s_lastPingTick,
                                               VISION_BOOT_PING_INTERVAL_MS))) {
        s_nextPingId++;
        if (s_nextPingId == 0U) {
            s_nextPingId = 1U;
        }
        s_waitPongId = s_nextPingId;
        AppVisionLink_SendPing(s_waitPongId);
        s_lastPingTick = now;
        s_pingSent = true;
        s_bootPingCount++;
    }
}

void AppVisionLink_TaskStart(uint8_t taskId)
{
    if ((taskId < 1U) || (taskId > 6U)) {
        return;
    }
    s_runId++;
    if (s_runId == 0U) {
        s_runId = 1U;
    }
    s_activeTaskId = taskId;
    s_lastTaskRunId = s_runId;
    s_lastTaskId = taskId;
    s_lastTaskStart = true;
    AppVisionLink_SendTask(s_runId, taskId, true);
}

void AppVisionLink_TaskStop(uint8_t taskId)
{
    if ((s_activeTaskId == 0U) || (taskId != s_activeTaskId)) {
        return;
    }
    AppVisionLink_SendTask(s_runId, taskId, false);
    s_lastTaskRunId = s_runId;
    s_lastTaskId = taskId;
    s_lastTaskStart = false;
    s_activeTaskId = 0U;
}

void AppVisionLink_GetStatus(AppVisionLinkStatus_t *out)
{
    VisionData_t data;
    TickType_t now;

    if (out == NULL) {
        return;
    }
    now = xTaskGetTickCount();
    (void)VisionParser_GetLatest(&data);
    out->online = s_hasPong && !AppVisionLink_Elapsed(now, s_lastPongTick, VISION_OFFLINE_MS);
    out->pingId = s_waitPongId;
    out->xReceived = s_xReceived;
    out->xValid = s_xReceived && s_xValid && !AppVisionLink_Elapsed(now, s_lastXTick, VISION_X_TIMEOUT_MS);
    out->xNa = s_xReceived && s_xNa && !AppVisionLink_Elapsed(now, s_lastXTick, 250U);
    out->xPixel = s_xPixel;
    AppVisionLink_CopyXText(out->xText, s_xText);
    out->ackMatched = data.ackValid && (data.ackRunId == s_lastTaskRunId) &&
                      (data.ackTaskId == s_lastTaskId) &&
                      (data.ackStart == s_lastTaskStart);
    out->ackStart = data.ackStart;
}
