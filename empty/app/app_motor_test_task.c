#include "app_motor_test_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_motor_status.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

/* ==================================================================
 * 电机1「四按键定圈旋转」测试任务（v1.1）
 *
 * 按键功能：
 *   KEY1(PA28)  正转 1 圈  ( 6400 脉冲)
 *   KEY2(PA31)  反转 1 圈  ( 6400 脉冲)
 *   KEY3(PA30)  正转 3 圈  (19200 脉冲)
 *   KEY4(PA29)  正转 5 圈  (32000 脉冲)
 *
 * 1/32 细分 → 1 圈 = 200 × 32 = 6400 脉冲。
 * 每次移动：梯形加减速起步→巡航→减速→自动停表，
 * 移动期间忽略按键，停稳后才能触发下一次。
 *
 * 文件结构（从上到下）：
 *   1. 常量与数据定义
 *   2. 诊断发布模块（写入 g_motorDiag，供 PERIPH/OLED 读取）
 *   3. 串口日志模块（事件日志 + 周期诊断）
 *   4. 按键→移动映射逻辑
 *   5. 任务入口与初始化
 * ================================================================== */

/* ------------------------------------------------------------------
 * 1. 常量与数据定义
 * ------------------------------------------------------------------ */

/* 1/32 细分每圈脉冲数。 */
#define STEPS_PER_REV               (6400U)

/* 巡航周期（越小越快），period=500 → 8kHz 步频 ≈ 1.25 圈/秒。 */
#define MOVE_CRUISE_PERIOD          (500U)

/* 按键功能对应的圈数与方向。 */
typedef struct {
    uint8_t        revs;   /* 圈数 */
    BspMotorDir_t  dir;    /* 方向 */
} MoveDef_t;

static const MoveDef_t s_moveDef[BSP_KEY_COUNT] = {
    { 1U, MOTOR_DIR_FORWARD  },   /* KEY1: 正转 1 圈 */
    { 1U, MOTOR_DIR_REVERSE  },   /* KEY2: 反转 1 圈 */
    { 3U, MOTOR_DIR_FORWARD  },   /* KEY3: 正转 3 圈 */
    { 5U, MOTOR_DIR_FORWARD  },   /* KEY4: 正转 5 圈 */
};

static TaskHandle_t s_motorTestTaskHandle = NULL;

/* ------------------------------------------------------------------
 * 2. 诊断发布模块
 *
 * 写入 g_motorDiag 全局快照，供 PERIPH 任务在 OLED 上只读显示。
 * 调用约定：每次电机状态变化时调用一次。
 * ------------------------------------------------------------------ */

/* 全局诊断单例（定义在此，声明在 app_motor_status.h）。 */
AppMotorDiag_t g_motorDiag = { false, true, 0U };

static void MotorDiag_Publish(bool running, BspMotorDir_t dir, uint8_t param)
{
    g_motorDiag.running    = running;
    g_motorDiag.dirForward = (dir == MOTOR_DIR_FORWARD);
    g_motorDiag.param      = param;
}

/* ------------------------------------------------------------------
 * 3. 串口日志模块
 *
 * 所有 UART0 输出都经递归互斥量加锁保证整行原子。
 * 分为两类：
 *   MotorLog_Event  — 事件触发（按键按下、移动完成）
 *   MotorLog_Periodic — 每秒周期诊断（剩余步数、当前周期）
 * ------------------------------------------------------------------ */

/* 输出无符号十进制整数（复用的小工具）。 */
static void MotorLog_SendUint(uint32_t value)
{
    char buf[10];
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

/*
 * 事件日志：电机状态变化时打印一行。
 * 格式：MOTOR1 <事件> -> RUN/STOP FWD/REV <圈数> rev
 */
static void MotorLog_Event(const char *evt, uint8_t revs, BspMotorDir_t dir,
                           bool running)
{
    BspUart0_Lock();
    BspUart0_SendString("MOTOR1 ");
    BspUart0_SendString(evt);
    BspUart0_SendString(running ? " -> RUN " : " -> STOP ");
    BspUart0_SendString((dir == MOTOR_DIR_FORWARD) ? "FWD " : "REV ");
    MotorLog_SendUint((uint32_t)revs);
    BspUart0_SendString(" rev\r\n");
    BspUart0_Unlock();
}

/*
 * 周期诊断（每秒一次）：位置模式标志、运行状态、剩余步数、当前周期。
 * 格式：MOTOR DIAG pos=1 run=0/1 left=<剩余步数> per=<当前周期>
 */
static void MotorLog_Periodic(void)
{
    BspUart0_Lock();
    BspUart0_SendString("MOTOR DIAG pos=1 run=");
    BspUart0_SendByte(BspMotor1_IsStopped() ? (uint8_t)'0' : (uint8_t)'1');
    BspUart0_SendString(" left=");
    MotorLog_SendUint(BspMotor1_GetRemainingSteps());
    BspUart0_SendString(" per=");
    MotorLog_SendUint(BspMotor1_GetCurPeriod());
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

/* ------------------------------------------------------------------
 * 4. 按键→移动映射
 *
 * 扫描四个按键的按下沿，返回触发移动的按键索引。
 * 同一拍只响应一个按键；返回 BSP_KEY_COUNT 表示无触发。
 * ------------------------------------------------------------------ */

static uint32_t MotorKey_Scan(const bool keyPrev[BSP_KEY_COUNT],
                              const bool keyNow[BSP_KEY_COUNT])
{
    uint32_t i;
    for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
        if (keyNow[i] && !keyPrev[i]) {
            return i;
        }
    }
    return (uint32_t)BSP_KEY_COUNT;  /* 无按键按下沿 */
}

/*
 * 执行一次定圈移动：设方向→使能驱动→发 MoveSteps（非阻塞，ISR 后台跑）。
 * totalSteps 由圈数 × 每圈脉冲数计算。
 */
static void MotorKey_StartMove(const MoveDef_t *move)
{
    uint32_t totalSteps = (uint32_t)move->revs * STEPS_PER_REV;

    BspMotor1_SetDir(move->dir);
    BspTmc_EnableAll();
    BspMotor1_MoveSteps(totalSteps, MOVE_CRUISE_PERIOD);
}

/* ------------------------------------------------------------------
 * 5. 任务入口
 * ------------------------------------------------------------------ */

static void AppMotorTestTask_Entry(void *argument)
{
    bool keyPrev[BSP_KEY_COUNT] = { false, false, false, false };
    bool keyNow[BSP_KEY_COUNT];
    uint32_t keyIdx;
    uint32_t i;
    uint32_t diagTick = 0U;
    TickType_t lastWakeTime;

    (void)argument;

    /* 细分四路共用，整机只需设一次。上电停在待机，等按键触发。 */
    BspTmc_SetMicrostep(TMC_MICROSTEP_32);
    MotorDiag_Publish(false, MOTOR_DIR_FORWARD, 0U);

    BspUart0_Lock();
    BspUart0_SendString(
        "MOTOR1 ctrl: K1 +1rev, K2 -1rev, K3 +3rev, K4 +5rev (1/32, 6400p/rev)\r\n");
    BspUart0_Unlock();
    MotorLog_Event("idle, press key", 0U, MOTOR_DIR_FORWARD, false);

    /* 用真实电平初始化按键基线，避免上电误判按下沿。 */
    for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
        keyPrev[i] = BspKey_IsPressed((BspKeyId_t)i);
    }

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        /* 采样当前按键电平。 */
        for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
            keyNow[i] = BspKey_IsPressed((BspKeyId_t)i);
        }

        if (BspMotor1_IsStopped()) {
            /* 已停稳：扫描按键按下沿，触发新移动。 */
            keyIdx = MotorKey_Scan(keyPrev, keyNow);
            if (keyIdx < (uint32_t)BSP_KEY_COUNT) {
                const MoveDef_t *move = &s_moveDef[keyIdx];
                MotorKey_StartMove(move);
                MotorDiag_Publish(true, move->dir, move->revs);
                MotorLog_Event("key start", move->revs, move->dir, true);
            }
        }
        /*
         * else: 移动进行中，忽略按键，防止打断位置移动。
         *       按键基线仍会更新，松键后才接受下次触发。
         */

        /* 更新按键基线（无论是否在移动都刷新）。 */
        for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
            keyPrev[i] = keyNow[i];
        }

        /* 移动完成后发布停止快照并打印日志。 */
        if (BspMotor1_IsStopped() && g_motorDiag.running) {
            MotorDiag_Publish(false, MOTOR_DIR_FORWARD, 0U);
            MotorLog_Event("done", 0U, MOTOR_DIR_FORWARD, false);
        }

        /* 每秒诊断打印（20ms × 50 拍）。 */
        if (++diagTick >= 50U) {
            diagTick = 0U;
            MotorLog_Periodic();
        }

        vTaskDelayUntil(&lastWakeTime, APP_MOTOR_KEY_POLL_TICKS);
    }
}

/* ------------------------------------------------------------------
 * 6. 任务初始化（由 app_main.c 的 App_Init 调用）
 * ------------------------------------------------------------------ */

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
