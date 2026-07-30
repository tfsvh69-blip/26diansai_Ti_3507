#ifndef BSP_HOME_SWITCH_H
#define BSP_HOME_SWITCH_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 归零限位开关板级封装（v1.2，PA24，接口 P1-3，原继电器接口已改接轻触开关）。
 * 开关一端接 GND，按下时引脚被拉低；GPIO 已配为内部上拉数字输入
 * （见 ti_msp_dl_config.c），未按下时读到高电平。
 * 供 app/app_lift_homing.c 在开机时驱动 ID1 寻找机械归零参考点。
 */

/* 返回限位开关当前是否被按下（低电平=按下）。只读瞬时电平，无需初始化。 */
bool BspHomeSwitch_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_HOME_SWITCH_H */
