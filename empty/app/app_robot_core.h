#ifndef APP_ROBOT_CORE_H
#define APP_ROBOT_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 机器人题目核心模块（v1.7+）。
 *
 * 本模块管理全部题目（最多 6 道），每道题包含 name/onEnter/onLoop/onExit 三个钩子。
 * 供 UIMENU 任务调用：菜单显示调用 GetTaskName 获取题名，进入/运行/退出调用对应的
 * Enter/Loop/Exit 接口。
 *
 * 设计目的：
 *   1. 题目业务逻辑与 UI 解耦——UI 只负责显示与按键，题目具体做什幺在 robot_core 里；
 *   2. 提供一个"机器人总任务"的调度入口 RobotMaster_Start()，未来可一键运行全部题目；
 *   3. 每题可访问 bsp_motor/bsp_servo/IMU/激光等底层驱动，直接调 BSP 接口即可。
 */

/* ========== 题目接口（UIMENU 调用） ========== */

/* 题目总数。 */
uint32_t RobotCore_GetTaskCount(void);

/* 获取题目名称字符串（只读，无堆分配），供 OLED 菜单显示。 */
const char *RobotCore_GetTaskName(uint32_t taskIdx);

/* 进入题目：调该题的 onEnter 钩子。 */
void RobotCore_EnterTask(uint32_t taskIdx);

/* 题目每周期循环：调该题的 onLoop 钩子（由 UIMENU 的 30ms 节拍驱动）。 */
void RobotCore_LoopTask(uint32_t taskIdx);

/* 退出题目：调该题的 onExit 钩子。 */
void RobotCore_ExitTask(uint32_t taskIdx);

/* ========== 机器人总任务（后续扩展） ========== */

/*
 * 启动"机器人总任务"模式：按顺序自动执行全部 6 道题（每题运行一定时间/条件后
 * 自动切换到下一题）。当前为占位——内部仅打印提示，后续按需填充。
 * 返回值：true 表示全部执行完毕；false 表示被中断或未完成。
 */
bool RobotMaster_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ROBOT_CORE_H */
