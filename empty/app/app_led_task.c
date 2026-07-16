#include "app_led_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_led.h"

/*
 * LED1 心跳任务：唯一职责是每 300ms 翻转 PB25。
 * 它是"调度器是否活着"的最直观指示——只要 LED1 在闪，说明 FreeRTOS 在正常轮转。
 * 特意保持极简、不掺任何业务，避免心跳判断被其它逻辑干扰。
 */

static TaskHandle_t s_ledTaskHandle = NULL;

static void AppLedTask_Entry(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)argument;

    for (;;) {
        /* LED1(PB25) 作为系统心跳灯；LED2/LED3 与蜂鸣器由外设测试任务驱动。 */
        BspLed_Toggle(BSP_LED_1);
        vTaskDelayUntil(&lastWakeTime, APP_LED1_PERIOD_TICKS);
    }
}

void AppLedTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppLedTask_Entry,
                      "LED1",
                      APP_LED_TASK_STACK_WORDS,
                      NULL,
                      APP_LED_TASK_PRIORITY,
                      &s_ledTaskHandle);
    configASSERT(ret == pdPASS);
}
