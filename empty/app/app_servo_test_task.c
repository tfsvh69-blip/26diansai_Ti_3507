#include "app_servo_test_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_servo.h"
#include "bsp_uart.h"

/* ==================================================================
 * 舵机诊断任务：4 路舵机【各自独立】连续来回摆动（不用按键）。
 *
 * 目的：既验证四路都能动，又直观证明"4 个舵机可以完全独立控制"——
 * 这里给 4 路设置不同的起始相位，让它们摆动时处在不同角度、明显不同步。
 *
 * 【独立控制说明】每个舵机在 TIMA0 上有独立的比较寄存器(CCP0/1/2/3)，
 * 只共用一个 50Hz 时基，各自脉宽互不影响。要单独控制某一个，直接：
 *     BspServo_SetPulseUs(BSP_SERVO_2, 1600);   // 只动 2 号，其它不变
 * 即可。本任务的"错相摆动"只是把这一点演示出来。
 *
 * 【避坑】脉宽只经 BspServo_SetPulseUs() 设置（内部已做 period-pulseUs 极性补偿，
 * 且四路方向须一次 setCCPDirection 写全，见 ti_msp_dl_config.c 的说明），切勿绕过它。
 * 脉宽范围已收窄到 800~2200us（见 bsp_servo.h），给舵机机械行程留安全余量。
 * ================================================================== */

/* 每 20ms 扫描步进（微秒）。1400us 行程 / 10us ≈ 140 步 × 20ms ≈ 2.8s 单程。 */
#define SWEEP_STEP_US        (10U)

/* 每秒打印一次四路当前脉宽：20ms × 50 = 1s。 */
#define LOG_PRINT_TICKS      (50U)

static TaskHandle_t s_servoTestTaskHandle = NULL;

/* 四路各自独立的扫描状态。 */
static uint16_t s_pulse[BSP_SERVO_COUNT];
static bool     s_inc[BSP_SERVO_COUNT];

/* 输出无符号十进制整数。 */
static void ServoLog_SendUint(uint32_t value)
{
    char buf[6];
    uint8_t idx = 0U;

    if (value == 0U) {
        BspUart0_SendByte((uint8_t)'0');
        return;
    }
    while ((value > 0U) && (idx < sizeof(buf))) {
        buf[idx++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (idx > 0U) {
        BspUart0_SendByte((uint8_t)buf[--idx]);
    }
}

/* 打印四路当前脉宽：SERVO us S1=.. S2=.. S3=.. S4=.. （值各不相同即证明独立）。 */
static void ServoLog_All(void)
{
    uint32_t i;

    BspUart0_Lock();
    BspUart0_SendString("SERVO us");
    for (i = 0U; i < (uint32_t)BSP_SERVO_COUNT; i++) {
        BspUart0_SendString(" S");
        ServoLog_SendUint(i + 1U);
        BspUart0_SendString("=");
        ServoLog_SendUint((uint32_t)s_pulse[i]);
    }
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

/* 推进一路舵机的三角波扫描（到端点反向）。 */
static void ServoSweep_Step(uint32_t i)
{
    if (s_inc[i]) {
        if (s_pulse[i] >= SERVO_PULSE_MAX_US) {
            s_inc[i] = false;
        } else {
            s_pulse[i] += SWEEP_STEP_US;
        }
    } else {
        if (s_pulse[i] <= SERVO_PULSE_MIN_US) {
            s_inc[i] = true;
        } else {
            s_pulse[i] -= SWEEP_STEP_US;
        }
    }
    BspServo_SetPulseUs((BspServoId_t)i, s_pulse[i]);
}

static void AppServoTestTask_Entry(void *argument)
{
    uint16_t span = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;
    uint32_t logTick = 0U;
    uint32_t i;
    TickType_t lastWakeTime;

    (void)argument;

    /*
     * 四路错相起步：在行程内均匀铺开（MIN、MIN+1/4、MIN+1/2、MIN+3/4），
     * 同向起步 → 摆动时四个舵机明显处于不同角度，直观体现互相独立。
     */
    for (i = 0U; i < (uint32_t)BSP_SERVO_COUNT; i++) {
        s_pulse[i] = (uint16_t)(SERVO_PULSE_MIN_US + (span * i) / (uint32_t)BSP_SERVO_COUNT);
        s_inc[i] = true;
        BspServo_SetPulseUs((BspServoId_t)i, s_pulse[i]);
    }

    /* 启动 TIMA0 PWM 输出（四路一起使能）。 */
    BspServo_Start();

    BspUart0_Lock();
    BspUart0_SendString(
        "SERVO sweep: 4 servos INDEPENDENT phase, 800<->2200us auto, no key.\r\n");
    BspUart0_Unlock();

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        /* 四路各自独立推进一步。 */
        for (i = 0U; i < (uint32_t)BSP_SERVO_COUNT; i++) {
            ServoSweep_Step(i);
        }

        /* 每秒打印一次四路脉宽（数值互不相同即证明独立）。 */
        if (++logTick >= LOG_PRINT_TICKS) {
            logTick = 0U;
            ServoLog_All();
        }

        vTaskDelayUntil(&lastWakeTime, APP_SERVO_TEST_PERIOD_TICKS);
    }
}

void AppServoTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppServoTestTask_Entry,
                      "SERVOSWEEP",
                      APP_SERVO_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_SERVO_TEST_TASK_PRIORITY,
                      &s_servoTestTaskHandle);
    configASSERT(ret == pdPASS);
}
