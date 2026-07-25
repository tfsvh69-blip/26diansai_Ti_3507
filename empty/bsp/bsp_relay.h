#ifndef BSP_RELAY_H
#define BSP_RELAY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 继电器板级封装（v1.1，PA24，接口 P1-3）。
 * 继电器去驱动大电流电磁铁负载，MCU 只给普通 GPIO 高/低电平控制通断，无需 PWM/定时器。
 *
 * 语义以「吸合(通)/断开(停)」为准，与电平的映射由 bsp_relay.c 顶部 BSP_RELAY_ACTIVE_LOW 决定：
 *   默认(=0) 高电平吸合、低电平断开；若实测模块是低电平触发，改该宏为 1 即整体反相。
 * 上电默认断开（安全），避免电磁铁误动作。
 */

/* 初始化为断开（收敛默认状态）。GPIO 已在 SYSCFG_DL_GPIO_init 配好，这里仅收敛默认电平。 */
void BspRelay_Init(void);

/* 继电器吸合（导通，电磁铁通电）。 */
void BspRelay_On(void);

/* 继电器断开（释放，电磁铁断电）。 */
void BspRelay_Off(void);

/* 按布尔设置：on=true 吸合、false 断开。 */
void BspRelay_Set(bool on);

/* 翻转吸合/断开状态，供简单通断测试使用。 */
void BspRelay_Toggle(void);

/*
 * 返回继电器当前逻辑状态：true=吸合(通)、false=断开(停)。
 * 读的是内部维护的逻辑状态（与 On/Off/Set/Toggle 同步），供 OLED 等显示当前状态、
 * 便于对照继电器实际动作核对触发极性是否与 BSP_RELAY_ACTIVE_LOW 一致。
 */
bool BspRelay_IsOn(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_RELAY_H */
