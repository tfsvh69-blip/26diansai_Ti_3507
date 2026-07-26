#include "app_nrf24_tx_test_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "nrf24l01.h"

static TaskHandle_t s_nrf24TxTaskHandle = NULL;

static uint8_t AppNrf24TxTestTask_BuildText(
    uint32_t sequence, uint8_t text[NRF24L01_FIXED_PAYLOAD_WIDTH - 1U])
{
    static const char prefix[] = "TMX NRF24 TEST ";
    const uint8_t prefixLength = (uint8_t)(sizeof(prefix) - 1U);
    const uint8_t digitCount = 6U;
    uint8_t i;

    for (i = 0U; i < prefixLength; i++) {
        text[i] = (uint8_t)prefix[i];
    }

    sequence %= 1000000UL;
    for (i = 0U; i < digitCount; i++) {
        text[prefixLength + digitCount - 1U - i] =
            (uint8_t)('0' + (sequence % 10U));
        sequence /= 10U;
    }

    return (uint8_t)(prefixLength + digitCount);
}

static void AppNrf24TxTestTask_Entry(void *argument)
{
    TickType_t lastWakeTime;
    uint8_t text[NRF24L01_FIXED_PAYLOAD_WIDTH - 1U];
    uint8_t textLength;
    uint32_t sequence = 1U;
    bool radioReady = false;
    Nrf24TxResult_t result;

    (void)argument;

    /* nRF24L01+ 上电复位最长需约 100ms，延后首次 SPI 配置。 */
    g_nrf24Diag.magic = NRF24_DIAG_MAGIC;
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_POWER_WAIT;
    g_nrf24Diag.radioReady = 0U;
    vTaskDelay(APP_NRF24_TX_STARTUP_TICKS);
    lastWakeTime = xTaskGetTickCount();

    for (;;) {
        g_nrf24Diag.taskHeartbeat++;
        g_nrf24Diag.lastSequence = sequence;

        if (!radioReady) {
            radioReady = Nrf24_Init(&g_nrf24UsbUartV20Config);
        }

        if (radioReady) {
            textLength = AppNrf24TxTestTask_BuildText(sequence, text);
            result = Nrf24_SendUsbUartText(text, textLength);

            if (result == NRF24_TX_OK) {
                sequence++;
            } else if ((result == NRF24_TX_TIMEOUT) ||
                       (result == NRF24_TX_NOT_READY) ||
                       (result == NRF24_TX_IO_ERROR)) {
                /* 模块掉线或 SPI 异常时，下一个周期重新初始化。 */
                radioReady = false;
            }
        }

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
