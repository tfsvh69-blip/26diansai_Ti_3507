#include "ball_parser.h"

#include <string.h>

#define VISION_BUF_MAX (48U)

static volatile bool     s_pongValid;
static volatile uint16_t s_pongId;
static volatile uint32_t s_pongSeq;
static volatile bool     s_ackValid;
static volatile uint16_t s_ackRunId;
static volatile uint8_t  s_ackTaskId;
static volatile bool     s_ackStart;
static volatile uint32_t s_ackSeq;
static volatile bool     s_xReceived;
static volatile bool     s_xValid;
static volatile int16_t  s_xPixel;
static volatile uint32_t s_xSeq;
static volatile char     s_xText[VISION_X_TEXT_MAX + 1U];
static volatile uint32_t s_frameOkCnt;
static volatile uint32_t s_crcErrCnt;
static volatile uint32_t s_rxBytes;

static uint8_t  s_buf[VISION_BUF_MAX];
static uint16_t s_idx;
static bool     s_capturing;

static uint8_t VisionParser_HexVal(uint8_t c)
{
    if ((c >= '0') && (c <= '9')) {
        return (uint8_t)(c - '0');
    }
    if ((c >= 'A') && (c <= 'F')) {
        return (uint8_t)(c - 'A' + 10);
    }
    if ((c >= 'a') && (c <= 'f')) {
        return (uint8_t)(c - 'a' + 10);
    }
    return 0xFFU;
}

static bool VisionParser_ParseUint(const uint8_t *buf, uint16_t end,
                                   uint16_t *pos, uint16_t *out)
{
    uint32_t value = 0U;
    bool hasDigit = false;

    while ((*pos < end) && (buf[*pos] >= '0') && (buf[*pos] <= '9')) {
        value = value * 10U + (uint32_t)(buf[*pos] - '0');
        if (value > 65535U) {
            return false;
        }
        hasDigit = true;
        (*pos)++;
    }
    *out = (uint16_t)value;
    return hasDigit;
}

static bool VisionParser_Match(const uint8_t *buf, uint16_t pos,
                               uint16_t end, const char *text)
{
    while (*text != '\0') {
        if ((pos >= end) || (buf[pos] != (uint8_t)*text)) {
            return false;
        }
        pos++;
        text++;
    }
    return pos == end;
}

/*
 * X 帧先保留原始数据字段，再进行协议数值校验。
 * 即使校验和、格式或范围不符合正式协议，OLED 仍可看到上位机真正发来的 X，
 * 便于先确认链路和定位格式问题。
 */
static void VisionParser_CaptureXText(const uint8_t *buf, uint16_t len)
{
    uint16_t pos = 2U;
    uint16_t out = 0U;

    while ((pos < len) && (buf[pos] != '*')) {
        if (out < VISION_X_TEXT_MAX) {
            uint8_t c = buf[pos];

            /* OLED 只有 ASCII 字库；不可显示字符统一替换为问号。 */
            s_xText[out++] = ((c >= 0x20U) && (c <= 0x7EU)) ? (char)c : '?';
        }
        pos++;
    }
    s_xText[out] = '\0';
    s_xValid = false;
}

static void VisionParser_ParsePong(const uint8_t *buf, uint16_t end)
{
    uint16_t pos = 5U;
    uint16_t id;

    if (!VisionParser_ParseUint(buf, end, &pos, &id) || (pos != end)) {
        s_crcErrCnt++;
        return;
    }
    s_pongId = id;
    s_pongValid = true;
    s_pongSeq++;
    s_frameOkCnt++;
}

static void VisionParser_ParseAck(const uint8_t *buf, uint16_t end)
{
    uint16_t pos = 4U;
    uint16_t runId;
    uint16_t taskId;
    bool start;

    if (!VisionParser_ParseUint(buf, end, &pos, &runId) ||
        (pos >= end) || (buf[pos++] != ',' ) ||
        !VisionParser_ParseUint(buf, end, &pos, &taskId) ||
        (taskId < 1U) || (taskId > 6U) ||
        (pos >= end) || (buf[pos++] != ',')) {
        s_crcErrCnt++;
        return;
    }
    if (VisionParser_Match(buf, pos, end, "START")) {
        start = true;
    } else if (VisionParser_Match(buf, pos, end, "STOP")) {
        start = false;
    } else {
        s_crcErrCnt++;
        return;
    }
    s_ackRunId = runId;
    s_ackTaskId = (uint8_t)taskId;
    s_ackStart = start;
    s_ackValid = true;
    s_ackSeq++;
    s_frameOkCnt++;
}

static void VisionParser_ParseX(const uint8_t *buf, uint16_t end)
{
    uint16_t pos = 2U;
    uint16_t pixel;

    if (VisionParser_Match(buf, pos, end, "NA")) {
        s_xValid = false;
        s_frameOkCnt++;
        return;
    }
    if (!VisionParser_ParseUint(buf, end, &pos, &pixel) ||
        (pos != end) || (pixel > VISION_X_PIXEL_MAX)) {
        s_crcErrCnt++;
        return;
    }
    s_xPixel = (int16_t)pixel;
    s_xValid = true;
    s_frameOkCnt++;
}

static void VisionParser_ParseLine(const uint8_t *buf, uint16_t len)
{
    uint16_t star = 0U;
    uint16_t i;
    uint8_t checksum = 0U;
    uint8_t hi;
    uint8_t lo;
    bool isX = (len >= 2U) && (buf[0] == 'X') && (buf[1] == ',');

    if (isX) {
        VisionParser_CaptureXText(buf, len);
    }

    while ((star < len) && (buf[star] != '*')) {
        star++;
    }
    if ((star == 0U) || (star + 3U != len)) {
        s_crcErrCnt++;
        goto publish_x;
    }
    for (i = 0U; i < star; i++) {
        checksum ^= buf[i];
    }
    hi = VisionParser_HexVal(buf[star + 1U]);
    lo = VisionParser_HexVal(buf[star + 2U]);
    if ((hi == 0xFFU) || (lo == 0xFFU) ||
        (checksum != (uint8_t)((hi << 4) | lo))) {
        s_crcErrCnt++;
        goto publish_x;
    }
    if (VisionParser_Match(buf, 0U, 5U, "PONG,") && (star > 5U)) {
        VisionParser_ParsePong(buf, star);
    } else if (VisionParser_Match(buf, 0U, 4U, "ACK,") && (star > 4U)) {
        VisionParser_ParseAck(buf, star);
    } else if (isX && (star > 2U)) {
        VisionParser_ParseX(buf, star);
    } else {
        s_crcErrCnt++;
    }

publish_x:
    /*
     * xSeq 最后发布。任务若在 ISR 更新期间读取快照，可用序号前后一致性
     * 判断数据是否完整，避免把“新序号 + 旧坐标”误当成一帧有效控制输入。
     */
    if (isX) {
        s_xReceived = true;
        s_xSeq++;
    }
}

void VisionParser_Reset(void)
{
    s_pongValid = false;
    s_pongId = 0U;
    s_pongSeq = 0U;
    s_ackValid = false;
    s_ackRunId = 0U;
    s_ackTaskId = 0U;
    s_ackStart = false;
    s_ackSeq = 0U;
    s_xReceived = false;
    s_xValid = false;
    s_xPixel = 0;
    s_xSeq = 0U;
    s_xText[0] = '\0';
    s_frameOkCnt = 0U;
    s_crcErrCnt = 0U;
    s_rxBytes = 0U;
    s_idx = 0U;
    s_capturing = false;
    memset(s_buf, 0, sizeof(s_buf));
}

void VisionParser_FeedByte(uint8_t byte)
{
    s_rxBytes++;
    if (byte == '$') {
        s_idx = 0U;
        s_capturing = true;
        return;
    }
    if (!s_capturing) {
        return;
    }
    if ((byte == '\r') || (byte == '\n')) {
        if (s_idx > 0U) {
            VisionParser_ParseLine(s_buf, s_idx);
        }
        s_idx = 0U;
        s_capturing = false;
        return;
    }
    if (s_idx < VISION_BUF_MAX) {
        s_buf[s_idx++] = byte;
    } else {
        s_capturing = false;
    }
}

bool VisionParser_GetLatest(VisionData_t *out)
{
    uint16_t i;
    uint32_t xSeqBefore;
    uint32_t xSeqAfter = 0U;
    uint8_t attempt;

    if (out == NULL) {
        return false;
    }
    out->pongValid = s_pongValid;
    out->pongId = s_pongId;
    out->pongSeq = s_pongSeq;
    out->ackValid = s_ackValid;
    out->ackRunId = s_ackRunId;
    out->ackTaskId = s_ackTaskId;
    out->ackStart = s_ackStart;
    out->ackSeq = s_ackSeq;
    /*
     * ISR 在写完 X 全部字段后才递增 xSeq。前后序号一致表示本次拷贝完整；
     * 若恰好被新帧中断，则重拷一次。
     */
    for (attempt = 0U; attempt < 2U; attempt++) {
        xSeqBefore = s_xSeq;
        out->xReceived = s_xReceived;
        out->xValid = s_xValid;
        out->xPixel = s_xPixel;
        for (i = 0U; i <= VISION_X_TEXT_MAX; i++) {
            out->xText[i] = s_xText[i];
        }
        xSeqAfter = s_xSeq;
        if (xSeqBefore == xSeqAfter) {
            break;
        }
    }
    out->xSeq = xSeqAfter;
    out->frameOkCnt = s_frameOkCnt;
    out->crcErrCnt = s_crcErrCnt;
    out->rxBytes = s_rxBytes;
    return s_frameOkCnt != 0U;
}
