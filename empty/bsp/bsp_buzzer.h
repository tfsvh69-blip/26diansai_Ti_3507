#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 有源蜂鸣器板级封装（v1.1，PA15）。
 * 有源蜂鸣器内部自带振荡电路，MCU 只需给高/低电平控制通断，无需 PWM/定时器。
 * 高电平（经 Q3 驱动）响，低电平停。
 */

/* 初始化为静音（低电平）。GPIO 已在 SYSCFG_DL_GPIO_init 配好，这里仅收敛默认电平。 */
void BspBuzzer_Init(void);

/* 蜂鸣器通（响）。 */
void BspBuzzer_On(void);

/* 蜂鸣器断（停）。 */
void BspBuzzer_Off(void);

/* 统一短促提示音（约 2~3ms），供按键与题目自动完成共用。仅限任务上下文调用。 */
void BspBuzzer_BeepShort(void);

/* 翻转通断状态，供简单通断测试使用。 */
void BspBuzzer_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUZZER_H */
