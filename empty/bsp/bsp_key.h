#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 四个功能按键编号，对应 KEY1~KEY4。 */
typedef enum {
    BSP_KEY_1 = 0,
    BSP_KEY_2,
    BSP_KEY_3,
    BSP_KEY_4,
    BSP_KEY_COUNT
} BspKeyId_t;

/*
 * 返回按键当前是否按下（低电平为按下）。
 * 仅返回瞬时电平，去抖和按下沿检测由调用方（任务轮询）处理。
 */
bool BspKey_IsPressed(BspKeyId_t key);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_H */
