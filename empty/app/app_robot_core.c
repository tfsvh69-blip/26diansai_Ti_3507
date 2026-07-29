#include "app_robot_core.h"

#include <stdbool.h>
#include <stddef.h>

#include "app_config.h"
#include "app_tasks.h"
#include "bsp_uart.h"

/* ==================================================================
 * 机器人题目核心模块：把 6 道题的钩子登记成一张 dispatch 表，供 UIMENU 调用。
 *
 * 设计约定：
 *   - 每道题 = 一张表项（题名 + OnEnter/OnLoop/OnExit 三个钩子）；
 *   - 各题的【具体业务代码在 app/tasks/taskN.c】里，本文件只做"登记 + 分发"；
 *   - UIMENU 进入某题调 EnterTask(→OnEnter)，运行态每 30ms 调 LoopTask(→OnLoop)，
 *     返回菜单调 ExitTask(→OnExit)；按键蜂鸣器反馈由 UIMENU 统一处理。
 *
 * 要加/改某题逻辑：改对应的 app/tasks/taskN.c；要改题名/题数：改本文件的 s_robotTasks[]。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 1. 题目表类型 + 表内容
 * ------------------------------------------------------------------ */

typedef struct {
    const char *name;          /* OLED 菜单显示的题名（当前仅支持 ASCII） */
    void (*onEnter)(void);     /* 进入本题一次性初始化（app/tasks/taskN.c） */
    void (*onLoop)(void);      /* 运行态每 30ms 调用一次（状态机主体） */
    void (*onExit)(void);      /* 返回菜单时收尾（急停/失能/复位） */
} RobotTask_t;

/*
 * 6 道题登记表。题名可按实际赛题改成有意义的短名（如 "LINE","PARK"）。
 * 钩子实现分别在 app/tasks/task1.c ~ task6.c。
 */
static const RobotTask_t s_robotTasks[] = {
    { "DIR TEST", Task1_OnEnter, Task1_OnLoop, Task1_OnExit },
    { "LINE PID", Task2_OnEnter, Task2_OnLoop, Task2_OnExit },
    { "GYRO 90L", Task3_OnEnter, Task3_OnLoop, Task3_OnExit },
    { "SERVO SWP", Task4_OnEnter, Task4_OnLoop, Task4_OnExit },
    { "EMM VEL", Task5_OnEnter, Task5_OnLoop, Task5_OnExit },
    { "Task 6", Task6_OnEnter, Task6_OnLoop, Task6_OnExit },
};

#define ROBOT_TASK_COUNT \
    ((uint32_t)(sizeof(s_robotTasks) / sizeof(s_robotTasks[0])))

/* ------------------------------------------------------------------
 * 2. 对 UI 的公共接口
 * ------------------------------------------------------------------ */

#if (APP_FEATURE_IMU_UART_LOG != 0U)
/* 题目进入/退出调试日志（仅串口遥测开启时输出，静默模式不刷串口）。 */
static void RobotCore_Log(const char *evt, uint32_t taskIdx)
{
    BspUart0_Lock();
    BspUart0_SendString("ROBOT: ");
    BspUart0_SendString(evt);
    BspUart0_SendString(" task ");
    BspUart0_SendUint(taskIdx + 1U);
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}
#endif

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
    RobotCore_Log("enter", taskIdx);
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
    RobotCore_Log("exit", taskIdx);
#endif
}

/* ------------------------------------------------------------------
 * 3. 机器人总任务（占位，后续扩展）
 *
 * 设计意图：一键顺序执行全部 6 道题，每题运行固定时长或等待其自行完成。
 * 当前为框架占位——仅在串口输出一次提示。后续可用状态机枚举
 * ROBOT_STATE_IDLE / RUNNING_TASK1 / ... / DONE，逐题自动切换。
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
