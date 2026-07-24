#include "app_robot_core.h"

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"

/* ==================================================================
 * 机器人题目核心模块：管理全部 6 道题的业务逻辑。
 *
 * 设计约定：
 *   - 每道题 = 一个结点（名称 + init/loop/deinit 三个钩子）。
 *   - UIMENU 任务在进入某题时调 init、每 30ms 轮询调 loop、退出时调 deinit。
 *   - 各题内部自行调用 bsp_motor/bsp_servo 等底层驱动，模块间通过线程安全接口通信。
 *   - 需要高频控制环的题目应在 init 里创建自己的 FreeRTOS 任务/定时器，在 deinit 里销毁。
 *   - 当前全部钩子为占位——仅做蜂鸣器短响 + LED 闪烁表示"已进入"，不做实质动作。
 *
 * 文件结构：1.题目表类型定义 → 2.6 道占位钩子 → 3.题目表 → 4.对 UI 的公共接口。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 1. 题目表类型定义
 * ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    void (*onEnter)(void);
    void (*onLoop)(void);
    void (*onExit)(void);
} RobotTask_t;

/* ------------------------------------------------------------------
 * 2. 6 道题占位钩子（后续逐题填充具体业务逻辑）
 * ------------------------------------------------------------------ */

static void Task1_Enter(void) { /* TODO: 题目1 初始化 */ }

static void Task1_Loop(void)  { /* TODO: 题目1 主循环 */ }

static void Task1_Exit(void)  { /* TODO: 题目1 清理 */ }


static void Task2_Enter(void) { /* TODO: 题目2 初始化 */ }

static void Task2_Loop(void)  { /* TODO: 题目2 主循环 */ }

static void Task2_Exit(void)  { /* TODO: 题目2 清理 */ }


static void Task3_Enter(void) { /* TODO: 题目3 初始化 */ }

static void Task3_Loop(void)  { /* TODO: 题目3 主循环 */ }

static void Task3_Exit(void)  { /* TODO: 题目3 清理 */ }


static void Task4_Enter(void) { /* TODO: 题目4 初始化 */ }

static void Task4_Loop(void)  { /* TODO: 题目4 主循环 */ }

static void Task4_Exit(void)  { /* TODO: 题目4 清理 */ }


static void Task5_Enter(void)
{
    BspLed_On(BSP_LED_1);
    BspLed_Off(BSP_LED_2);
    BspLed_On(BSP_LED_3);

    BspBuzzer_On();
    vTaskDelay(pdMS_TO_TICKS(30U));
    BspBuzzer_Off();
}

static void Task5_Loop(void)  { /* TODO: 题目5 主循环 */ }

static void Task5_Exit(void)  { /* TODO: 题目5 清理 */ }


static void Task6_Enter(void) { /* TODO: 题目6 初始化 */ }

static void Task6_Loop(void)  { /* TODO: 题目6 主循环 */ }

static void Task6_Exit(void)  { /* TODO: 题目6 清理 */ }

/* ------------------------------------------------------------------
 * 3. 题目表（dispatch table）
 * ------------------------------------------------------------------ */

static const RobotTask_t s_robotTasks[] = {
    { "Task 1", Task1_Enter, Task1_Loop, Task1_Exit },
    { "Task 2", Task2_Enter, Task2_Loop, Task2_Exit },
    { "Task 3", Task3_Enter, Task3_Loop, Task3_Exit },
    { "Task 4", Task4_Enter, Task4_Loop, Task4_Exit },
    { "Task 5", Task5_Enter, Task5_Loop, Task5_Exit },
    { "Task 6", Task6_Enter, Task6_Loop, Task6_Exit },
};

#define ROBOT_TASK_COUNT \
    ((uint32_t)(sizeof(s_robotTasks) / sizeof(s_robotTasks[0])))

/* ------------------------------------------------------------------
 * 4. 对 UI 的公共接口
 * ------------------------------------------------------------------ */

uint32_t RobotCore_GetTaskCount(void)
{
    return ROBOT_TASK_COUNT;
}

const char *RobotCore_GetTaskName(uint32_t taskIdx)
{
    if (taskIdx >= ROBOT_TASK_COUNT) {
        return "???";
    }
    return s_robotTasks[taskIdx].name;
}

void RobotCore_EnterTask(uint32_t taskIdx)
{
    if (taskIdx >= ROBOT_TASK_COUNT) {
        return;
    }
    if (s_robotTasks[taskIdx].onEnter != NULL) {
        s_robotTasks[taskIdx].onEnter();
    }

#if (APP_FEATURE_IMU_UART_LOG != 0U)
    /* 调试日志：仅在串口遥测开启时打印，静默模式下不刷串口。 */
    BspUart0_Lock();
    BspUart0_SendString("ROBOT: enter task ");
    BspUart0_SendUint(taskIdx + 1U);
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
#endif
}

void RobotCore_LoopTask(uint32_t taskIdx)
{
    if (taskIdx >= ROBOT_TASK_COUNT) {
        return;
    }
    if (s_robotTasks[taskIdx].onLoop != NULL) {
        s_robotTasks[taskIdx].onLoop();
    }
}

void RobotCore_ExitTask(uint32_t taskIdx)
{
    if (taskIdx >= ROBOT_TASK_COUNT) {
        return;
    }
    if (s_robotTasks[taskIdx].onExit != NULL) {
        s_robotTasks[taskIdx].onExit();
    }

#if (APP_FEATURE_IMU_UART_LOG != 0U)
    BspUart0_Lock();
    BspUart0_SendString("ROBOT: exit task ");
    BspUart0_SendUint(taskIdx + 1U);
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
#endif
}

/* ------------------------------------------------------------------
 * 5. 机器人总任务（占位，后续扩展）
 *
 * 设计意图：一键顺序执行全部 6 道题，每题运行固定时长或等待其自行完成，
 * 完成后打印汇总与耗时。当前为框架占位——仅在串口输出一次提示。
 * 后续填充思路：
 *   - 用状态机枚举 ROBOT_STATE_IDLE / RUNNING_TASK1 / ... / RUNNING_TASK6 / DONE
 *   - 每题完成后自动切到下一题（可引入超时保护）
 *   - 汇总结果打印到 UART0
 * ------------------------------------------------------------------ */

bool RobotMaster_Start(void)
{
#if (APP_FEATURE_IMU_UART_LOG != 0U)
    BspUart0_Lock();
    BspUart0_SendString("ROBOT MASTER: all 6 tasks (placeholder, not implemented)\r\n");
    BspUart0_Unlock();
#endif
    /* TODO: 真正的顺序调度逻辑 */
    return false;
}
