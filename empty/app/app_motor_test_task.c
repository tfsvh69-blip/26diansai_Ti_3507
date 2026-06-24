#include "app_motor_test_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

static TaskHandle_t s_motorTestTaskHandle = NULL;

static void AppMotorTestTask_Entry(void *argument)
{
    TickType_t lastWakeTime;

    (void)argument;

    /*
     * 电机1驱动测试：1/8 细分、正向、持续匀速旋转。
     * 按引脚文档 §3.4 顺序：先停 STEP、定方向和细分，再启 STEP，最后使能 ENN。
     * 线序与 VREF 电流已标定完成，步频提到 4kHz 观察正常转速运行。
     * 仍无加减速斜坡，直接启动；后续要更高速需加速度斜坡，否则会失步重新表现为原地抖动。
     */
    BspTmc_SetMicrostep(TMC_MICROSTEP_8);
    BspMotor1_SetDir(MOTOR_DIR_FORWARD);
    BspMotor1_StartStep();
    BspTmc_EnableAll();

    BspUart0_SendString("MOTOR1: 1/8 step, 4kHz, spinning\r\n");

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        /* 持续旋转期间周期性输出心跳，便于确认任务仍在运行。 */
        BspUart0_SendString("MOTOR1: running\r\n");
        vTaskDelayUntil(&lastWakeTime, APP_MOTOR_TEST_PERIOD_TICKS);
    }
}

void AppMotorTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppMotorTestTask_Entry,
                      "MOTOR1",
                      APP_MOTOR_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_MOTOR_TEST_TASK_PRIORITY,
                      &s_motorTestTaskHandle);
    configASSERT(ret == pdPASS);
}
