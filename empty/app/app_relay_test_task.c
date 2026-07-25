#include "app_relay_test_task.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_relay.h"

/*
 * 继电器通断测试任务：每 APP_RELAY_TEST_PERIOD_TICKS（2 秒）翻转一次吸合/断开状态。
 * 起始为断开，之后 断开→吸合→断开… 循环，便于听继电器咔哒声/看指示灯确认工作。
 * ⚠️ 每次切换都会真实通断继电器所驱动的大电流电磁铁负载，测试时确保负载侧安全。
 */

static TaskHandle_t s_relayTaskHandle = NULL;

static void AppRelayTestTask_Entry(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    bool       on           = false;

    (void)argument;

    /* 起始断开（板级初始化已置断开，这里再收敛一次）。 */
    BspRelay_Off();

    for (;;) {
        vTaskDelayUntil(&lastWakeTime, APP_RELAY_TEST_PERIOD_TICKS);
        on = !on;
        BspRelay_Set(on);
    }
}

void AppRelayTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppRelayTestTask_Entry,
                      "RELAYTEST",
                      APP_RELAY_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_RELAY_TEST_TASK_PRIORITY,
                      &s_relayTaskHandle);
    configASSERT(ret == pdPASS);
}
