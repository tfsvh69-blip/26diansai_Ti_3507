#include "app_motor_test_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

static TaskHandle_t s_motorTestTaskHandle = NULL;

static void AppMotorTestTask_Entry(void *argument)
{
    /* 1/8 细分下一整圈所需脉冲数 = 200 全步 × 8 = 1600。 */
    const uint32_t stepsOneRev =
        (uint32_t)BSP_MOTOR_FULL_STEPS_PER_REV * 8U;

    (void)argument;

    /*
     * 电机1测试：1/8 细分、正向、梯形加减速旋转整圈后停止。
     * 1kHz 起步 → 20kHz 巡航 → 1kHz 收尾，避免直接 20kHz 起转失步。
     * 按引脚文档 §3.4 顺序：先设细分和方向，再使能 ENN，最后启动定长步进。
     */
    BspTmc_SetMicrostep(TMC_MICROSTEP_8);
    BspMotor1_SetDir(MOTOR_DIR_FORWARD);
    BspTmc_EnableAll();

    BspUart0_SendString("MOTOR1: 1/8 step, ramp 1k->20k->1k, rotating 1 rev...\r\n");

    /* 启动定长步进，TIMG0 ZERO 中断计步，转完自动停。 */
    BspMotor1_StartRotateSteps(stepsOneRev);

    /* 等待旋转完成（1 rev ≈ 80ms；每 1ms 轮询一次）。 */
    while (!BspMotor1_IsRotateDone()) {
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    BspTmc_DisableAll();
    BspUart0_SendString("MOTOR1: 1 rev done, motor disabled\r\n");

    /* 任务使命完成，挂起自身。 */
    vTaskSuspend(NULL);
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
