#ifndef BSP_LINE_H
#define BSP_LINE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 8 路灰度循迹（v1.1，接口 H6）：LINE1~LINE8 = PB17~PB24，直接读 GPIO 高低电平。
 * 以小车正朝向看：LINE8=最左（小车左）… LINE1=最右（小车右）。
 * 模块极性：识别到线时信号脚输出低电平（用户实测确认）。若换模块极性相反，
 * 改 bsp_line.c 顶部的 BSP_LINE_ACTIVE_LOW 宏即可整体反相。
 */
#define BSP_LINE_COUNT   (8U)

/*
 * 读取全部 8 路状态，打包成 8 位位图返回（已按模块极性归一化，识别到线=1）：
 *   bit0 = LINE1（最右/小车右）… bit7 = LINE8（最左/小车左）。
 */
uint8_t BspLine_ReadAll(void);

/*
 * 读取单路是否识别到线。ch 为 0~7（0=LINE1 最右，7=LINE8 最左）；越界返回 false。
 */
bool BspLine_IsDetected(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LINE_H */
