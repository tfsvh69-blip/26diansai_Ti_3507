#ifndef APP_UI_TASK_H
#define APP_UI_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OLED 题目菜单 UI 任务（v1.7+）。
 * 独占板载 OLED（PB8/PB9 软件 I2C）+ 四个按键（KEY1~4），实现：
 *   - 菜单态：一屏列出全部题目，反色高亮当前项；
 *   - 运行态：进入所选题目的运行界面，预留每题业务钩子。
 * 按键：K1=上移、K2=下移、K3=确认进入、K4=运行态返回；
 * 菜单态 K4 启停独立钢球居中闭环（目标 X=320）。
 *
 * 注意：本任务接管 OLED，启用时不要再让 PERIPH 任务刷 OLED（两任务抢
 * 软件 I2C 会花屏）；用 common/app_config.h 的开关互斥即可。
 */
void AppUiTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UI_TASK_H */
