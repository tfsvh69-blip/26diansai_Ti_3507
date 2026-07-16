#include "app_servo_test_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_servo.h"
#include "bsp_uart.h"

static TaskHandle_t s_servoTestTaskHandle = NULL;

/*
 * 舵机1 慢速来回摆动：20ms 周期，脉宽在 [MIN, MAX] 间往复扫描。
 * 每步变化 10us，单程 (MAX-MIN)/10 步 ≈ 18 步/秒。
 */
#define SWEEP_STEP_US   (10U)

static void AppServoTestTask_Entry(void *argument)
{
    uint16_t pulseUs = SERVO_PULSE_CENTER_US;
    bool     increasing = true;
    TickType_t lastWakeTime;

    (void)argument;

    /* 启动 TIMA0 PWM 输出，舵机上电。 */
    BspServo_SetPulseUs(BSP_SERVO_1, SERVO_PULSE_CENTER_US);
    BspServo_Start();

    BspUart0_SendString("SERVO1: sweep test start, " __DATE__ " " __TIME__ "\r\n");

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        /* 逐 10us 调整脉宽 */
        if (increasing) {
            if (pulseUs < SERVO_PULSE_MAX_US) {
                pulseUs += SWEEP_STEP_US;
            } else {
                increasing = false;
            }
        } else {
            if (pulseUs > SERVO_PULSE_MIN_US) {
                pulseUs -= SWEEP_STEP_US;
            } else {
                increasing = true;
            }
        }

        BspServo_SetPulseUs(BSP_SERVO_1, pulseUs);

        vTaskDelayUntil(&lastWakeTime, APP_SERVO_TEST_PERIOD_TICKS);
    }
}

void AppServoTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppServoTestTask_Entry,
                      "SERVO1",
                      APP_SERVO_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_SERVO_TEST_TASK_PRIORITY,
                      &s_servoTestTaskHandle);
    configASSERT(ret == pdPASS);
}
