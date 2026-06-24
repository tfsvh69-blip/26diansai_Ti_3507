#include "app_motor_test_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

/* 1/8 细分下每圈脉冲数 = 200 全步 × 8 = 1600。 */
#define MOTOR_MICROSTEPS_PER_REV \
    ((uint32_t)BSP_MOTOR_FULL_STEPS_PER_REV * 8U)

static TaskHandle_t s_motorTestTaskHandle = NULL;

/* 启动一次定长旋转：设方向、使能驱动，按指定速度走 revs 圈。 */
static void Motor_StartMotion(BspMotorDir_t dir, uint32_t revs,
                              uint32_t cruisePeriod)
{
    BspMotor1_SetDir(dir);
    BspTmc_EnableAll();
    BspMotor1_StartRotateSteps(revs * MOTOR_MICROSTEPS_PER_REV, cruisePeriod);
}

static void AppMotorTestTask_Entry(void *argument)
{
    /* 上一拍各按键电平，用于检测“释放->按下”沿，保证每次按下只触发一次。 */
    bool keyPrev[BSP_KEY_COUNT] = { false, false, false, false };
    bool keyNow[BSP_KEY_COUNT];
    uint8_t motorBusy = 0U;
    uint32_t i;
    TickType_t lastWakeTime;

    (void)argument;

    /* 细分四路共用，整机只需设一次。 */
    BspTmc_SetMicrostep(TMC_MICROSTEP_8);
    BspUart0_SendString(
        "MOTOR1 key ctrl: K1 slow fwd1, K2 slow rev1, K3 fast fwd2, K4 fast rev2\r\n");

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
            keyNow[i] = BspKey_IsPressed((BspKeyId_t)i);
        }

        if (motorBusy == 0U) {
            /* 空闲：按优先级检测按下沿，触发一次旋转。 */
            if (keyNow[BSP_KEY_1] && !keyPrev[BSP_KEY_1]) {
                Motor_StartMotion(MOTOR_DIR_FORWARD, 1U, BSP_MOTOR_PERIOD_SLOW);
                motorBusy = 1U;
                BspUart0_SendString("KEY1: slow forward 1 rev\r\n");
            } else if (keyNow[BSP_KEY_2] && !keyPrev[BSP_KEY_2]) {
                Motor_StartMotion(MOTOR_DIR_REVERSE, 1U, BSP_MOTOR_PERIOD_SLOW);
                motorBusy = 1U;
                BspUart0_SendString("KEY2: slow reverse 1 rev\r\n");
            } else if (keyNow[BSP_KEY_3] && !keyPrev[BSP_KEY_3]) {
                Motor_StartMotion(MOTOR_DIR_FORWARD, 2U, BSP_MOTOR_PERIOD_FAST);
                motorBusy = 1U;
                BspUart0_SendString("KEY3: fast forward 2 rev\r\n");
            } else if (keyNow[BSP_KEY_4] && !keyPrev[BSP_KEY_4]) {
                Motor_StartMotion(MOTOR_DIR_REVERSE, 2U, BSP_MOTOR_PERIOD_FAST);
                motorBusy = 1U;
                BspUart0_SendString("KEY4: fast reverse 2 rev\r\n");
            }
        } else if (BspMotor1_IsRotateDone()) {
            /* 旋转完成：松开使能，回到空闲。 */
            BspTmc_DisableAll();
            motorBusy = 0U;
            BspUart0_SendString("MOTOR1: done, motor disabled\r\n");
        }

        /*
         * 更新按键基线。
         * 旋转中也持续刷新，使旋转期间按住的键不会在结束瞬间被误判为新按下沿，
         * 必须松开再按才会触发下一次动作。
         */
        for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
            keyPrev[i] = keyNow[i];
        }

        vTaskDelayUntil(&lastWakeTime, APP_MOTOR_KEY_POLL_TICKS);
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
