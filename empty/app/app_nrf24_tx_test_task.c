#include "app_nrf24_tx_test_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_uart.h"
#include "nrf24l01.h"

/*
 * USB 无线串口 V2.0 的地址输入框使用十六进制。
 * 截图中目标地址和本地地址均为 15 52 33 54 55，故板端发射地址按同样字节顺序填写。
 */
static const Nrf24RadioConfig_t s_radioProfiles[] = {
    /* P0：与上位机截图一致，地址按十六进制输入顺序填写。 */
    {{0x15U, 0x52U, 0x33U, 0x54U, 0x55U}, 0x0AU, 0x1AU, 0x02U, 0x0EU},
    /* P1：部分上位机把地址显示顺序与空口写入顺序相反。 */
    {{0x55U, 0x54U, 0x33U, 0x52U, 0x15U}, 0x0AU, 0x1AU, 0x02U, 0x0EU},
    /* P2：厂家参考程序的 16 位 CRC / RF_SETUP=0x0F 组合。 */
    {{0x15U, 0x52U, 0x33U, 0x54U, 0x55U}, 0x0EU, 0x1AU, 0x02U, 0x0FU},
    /* P3：16 位 CRC 且地址顺序相反。 */
    {{0x55U, 0x54U, 0x33U, 0x52U, 0x15U}, 0x0EU, 0x1AU, 0x02U, 0x0FU},
    /* P4：仅用于排查上位机把输入框按十进制字节解释的极端情况。 */
    {{0x0FU, 0x34U, 0x21U, 0x36U, 0x37U}, 0x0AU, 0x1AU, 0x02U, 0x0EU},
    /* P5：十进制字节解释且地址顺序相反。 */
    {{0x37U, 0x36U, 0x21U, 0x34U, 0x0FU}, 0x0AU, 0x1AU, 0x02U, 0x0EU}
};

#define APP_NRF24_RADIO_PROFILE_COUNT \
    ((uint8_t)(sizeof(s_radioProfiles) / sizeof(s_radioProfiles[0])))

static TaskHandle_t s_nrf24TxTaskHandle = NULL;

#if (APP_NRF24_DIAG_UART_LOG != 0U)
static void AppNrf24TxTestTask_SendHex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    BspUart0_SendByte((uint8_t)hex[(value >> 4) & 0x0FU]);
    BspUart0_SendByte((uint8_t)hex[value & 0x0FU]);
}

static void AppNrf24TxTestTask_SendAddress(
    const volatile uint8_t address[NRF24L01_ADDRESS_WIDTH])
{
    uint8_t i;

    for (i = 0U; i < NRF24L01_ADDRESS_WIDTH; i++) {
        AppNrf24TxTestTask_SendHex8(address[i]);
        if ((i + 1U) < NRF24L01_ADDRESS_WIDTH) {
            BspUart0_SendByte((uint8_t)'-');
        }
    }
}

static void AppNrf24TxTestTask_PrintDiag(void)
{
    /*
     * 一行完整输出，用户可直接复制反馈。
     * 字段均来自 volatile 快照，不参与驱动控制。
     */
    BspUart0_Lock();
    BspUart0_SendString("NRF D HB=");
    BspUart0_SendUint(g_nrf24Diag.taskHeartbeat);
    BspUart0_SendString(" STG=");
    BspUart0_SendUint(g_nrf24Diag.stage);
    BspUart0_SendString(" FAIL=");
    BspUart0_SendUint(g_nrf24Diag.initFailCode);
    BspUart0_SendString(" RDY=");
    BspUart0_SendUint(g_nrf24Diag.radioReady);
    BspUart0_SendString(" P=");
    BspUart0_SendUint(g_nrf24Diag.profileIndex);
    BspUart0_SendString(" LOCK=");
    BspUart0_SendUint(g_nrf24Diag.profileLocked);
    BspUart0_SendString(" PA=");
    BspUart0_SendUint(g_nrf24Diag.profileAttemptCount);
    BspUart0_SendString(" GPIO=0x");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.gpioLevelMask);
    BspUart0_SendString(" SPI=");
    BspUart0_SendUint(g_nrf24Diag.spiByteCount);
    BspUart0_SendString(" CFG=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regConfig);
    BspUart0_SendString(" AA=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regEnAa);
    BspUart0_SendString(" ERX=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regEnRxaddr);
    BspUart0_SendString(" AW=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regSetupAw);
    BspUart0_SendString(" RETR=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regSetupRetr);
    BspUart0_SendString(" CH=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regRfCh);
    BspUart0_SendString(" RF=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regRfSetup);
    BspUart0_SendString(" PW=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.regRxPwP0);
    BspUart0_SendString(" TXA=");
    AppNrf24TxTestTask_SendAddress(g_nrf24Diag.txAddress);
    BspUart0_SendString(" RXA=");
    AppNrf24TxTestTask_SendAddress(g_nrf24Diag.rxAddressP0);
    BspUart0_SendString(" STATUS=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.lastStatus);
    BspUart0_SendString(" OBS=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.lastObserveTx);
    BspUart0_SendString(" FIFO=");
    AppNrf24TxTestTask_SendHex8(g_nrf24Diag.lastFifoStatus);
    BspUart0_SendString(" TRY=");
    BspUart0_SendUint(g_nrf24Diag.sendAttempts);
    BspUart0_SendString(" OK=");
    BspUart0_SendUint(g_nrf24Diag.txSuccess);
    BspUart0_SendString(" MAX=");
    BspUart0_SendUint(g_nrf24Diag.txMaxRetry);
    BspUart0_SendString(" TO=");
    BspUart0_SendUint(g_nrf24Diag.txTimeout);
    BspUart0_SendString(" IO=");
    BspUart0_SendUint(g_nrf24Diag.txIoError);
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}
#endif

static void AppNrf24TxTestTask_BuildPayload(
    uint32_t sequence, uint8_t payload[NRF24L01_FIXED_PAYLOAD_WIDTH])
{
    static const char prefix[] = "TMX NRF24 TEST ";
    const uint8_t prefixLength = (uint8_t)(sizeof(prefix) - 1U);
    const uint8_t digitCount = 6U;
    uint8_t i;

    memset(payload, 0, NRF24L01_FIXED_PAYLOAD_WIDTH);

    /*
     * 厂家透传协议固定发送 32 字节：payload[0] 是有效文本长度，
     * payload[1] 起才是真正送到电脑串口的文本，剩余字节清零。
     */
    payload[0] = (uint8_t)(prefixLength + digitCount);
    for (i = 0U; i < prefixLength; i++) {
        payload[1U + i] = (uint8_t)prefix[i];
    }

    sequence %= 1000000UL;
    for (i = 0U; i < digitCount; i++) {
        payload[1U + prefixLength + digitCount - 1U - i] =
            (uint8_t)('0' + (sequence % 10U));
        sequence /= 10U;
    }
}

static void AppNrf24TxTestTask_Entry(void *argument)
{
    TickType_t lastWakeTime;
    uint8_t payload[NRF24L01_FIXED_PAYLOAD_WIDTH];
    uint32_t sequence = 1U;
    bool radioReady = false;
    Nrf24TxResult_t result;
    uint8_t profileIndex = 0U;
#if (APP_NRF24_DIAG_UART_LOG != 0U)
    uint8_t logDivider = 0U;
#endif

    (void)argument;

    /* nRF24L01+ 上电复位最长需约 100ms，延后首次 SPI 配置。 */
    g_nrf24Diag.magic = NRF24_DIAG_MAGIC;
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_POWER_WAIT;
    g_nrf24Diag.radioReady = 0U;
    g_nrf24Diag.profileIndex = profileIndex;
    g_nrf24Diag.profileLocked = 0U;
    g_nrf24Diag.profileAttemptCount = 0U;
    vTaskDelay(APP_NRF24_TX_STARTUP_TICKS);
    lastWakeTime = xTaskGetTickCount();

    for (;;) {
        g_nrf24Diag.taskHeartbeat++;
        g_nrf24Diag.lastSequence = sequence;

        if (!radioReady) {
            radioReady = Nrf24_Init(&s_radioProfiles[profileIndex]);
        }

        if (radioReady) {
            AppNrf24TxTestTask_BuildPayload(sequence, payload);
            result = Nrf24_SendPayload(payload);

            if (result == NRF24_TX_OK) {
                sequence++;
#if (APP_NRF24_DIAG_COMPAT_SCAN != 0U)
                /* 任一参数组收到 ACK 即说明无线参数匹配，后续固定使用该组。 */
                g_nrf24Diag.profileLocked = 1U;
#endif
            } else if ((result == NRF24_TX_TIMEOUT) ||
                       (result == NRF24_TX_NOT_READY) ||
                       (result == NRF24_TX_IO_ERROR)) {
                /* 模块掉线或 SPI 异常时，下一个周期重新初始化；MAX_RT 仅表示对端未应答。 */
                radioReady = false;
#if (APP_NRF24_DIAG_COMPAT_SCAN != 0U)
            } else if ((result == NRF24_TX_MAX_RETRY) &&
                       (g_nrf24Diag.profileLocked == 0U)) {
                g_nrf24Diag.profileAttemptCount++;
                if (g_nrf24Diag.profileAttemptCount >=
                    APP_NRF24_DIAG_PROFILE_TRY_COUNT) {
                    profileIndex++;
                    if (profileIndex >= APP_NRF24_RADIO_PROFILE_COUNT) {
                        profileIndex = 0U;
                    }
                    g_nrf24Diag.profileIndex = profileIndex;
                    g_nrf24Diag.profileAttemptCount = 0U;
                    /* 切换参数后必须从掉电配置流程重新初始化。 */
                    radioReady = false;
                }
#endif
            }
        }

#if (APP_NRF24_DIAG_UART_LOG != 0U)
        logDivider++;
        if (logDivider >= APP_NRF24_DIAG_LOG_DIVIDER) {
            logDivider = 0U;
            AppNrf24TxTestTask_PrintDiag();
        }
#endif

        vTaskDelayUntil(&lastWakeTime, APP_NRF24_TX_TEST_PERIOD_TICKS);
    }
}

void AppNrf24TxTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppNrf24TxTestTask_Entry,
                      "NRF24TX",
                      APP_NRF24_TX_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_NRF24_TX_TEST_TASK_PRIORITY,
                      &s_nrf24TxTaskHandle);
    configASSERT(ret == pdPASS);
}
