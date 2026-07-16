#ifndef APP_PERIPH_TEST_TASK_H
#define APP_PERIPH_TEST_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 外设功能验证任务（v1.1）。
 * 集中做四件事：
 *   1. OLED 屏幕显示综合调试数据（标题、运行秒、LED/BUZZ 状态、电机1状态、累计圈数）；
 *   2. 指示灯 LED2/LED3 交替心跳测试；
 *   3. 蜂鸣器周期性通断控制测试；
 *   4. 从 g_motorDiag 读取电机1诊断信息显示到 OLED（不控制电机，只读）。
 * LED1 仍由 LED1 心跳任务独占，本任务不碰，避免两个任务争用同一 GPIO。
 */
void AppPeriphTestTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_PERIPH_TEST_TASK_H */
