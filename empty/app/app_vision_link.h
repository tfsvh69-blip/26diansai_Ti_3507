#ifndef APP_VISION_LINK_H
#define APP_VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* OLED 右侧面板宽度只够显示八个 X 原始字符。 */
#define APP_VISION_X_TEXT_MAX (8U)

typedef struct {
    bool     online;
    uint16_t pingId;
    bool     xReceived;
    bool     xValid;
    bool     xNa;
    int16_t  xPixel;
    char     xText[APP_VISION_X_TEXT_MAX + 1U];
    bool     ackMatched;
    bool     ackStart;
} AppVisionLinkStatus_t;

/* 初始化 UART0 接收解析器与通信状态机；调度器启动前调用一次。 */
void AppVisionLink_Init(void);

/* 每个 UIMENU 节拍调用，驱动复位后的三次 PING、在线超时和 X 数据超时。 */
void AppVisionLink_Service(void);

/* 题目开始/结束时发送 TASK 帧；每次开始自动分配新的 run_id。 */
void AppVisionLink_TaskStart(uint8_t taskId);
void AppVisionLink_TaskStop(uint8_t taskId);

/* 读取经过 PONG/X 超时判定后的 OLED 显示快照。 */
void AppVisionLink_GetStatus(AppVisionLinkStatus_t *out);

#endif /* APP_VISION_LINK_H */
