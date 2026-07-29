#ifndef BALL_PARSER_H
#define BALL_PARSER_H

#include <stdbool.h>
#include <stdint.h>

/* 保留最近一帧 X 的原始字段，用于在 OLED 上诊断上位机实际发送格式。 */
#define VISION_X_TEXT_MAX (16U)

/* 相机画面水平像素坐标的正式合法范围（含边界）。 */
#define VISION_X_PIXEL_MIN (0U)
#define VISION_X_PIXEL_MAX (640U)

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 视觉端 UART0 下行协议解析器。
 *
 * 接收 $PONG、$ACK 与 $X 三类 ASCII 帧。PONG/ACK 及正式 X 值须通过 XOR
 * 校验；X 的原始字段会在校验前保留给 OLED 诊断。本模块不依赖硬件或 FreeRTOS，
 * 因此可以直接由 UART0 中断调用。
 */
typedef struct {
    bool     pongValid;
    uint16_t pongId;
    uint32_t pongSeq;

    bool     ackValid;
    uint16_t ackRunId;
    uint8_t  ackTaskId;
    bool     ackStart;
    uint32_t ackSeq;

    bool     xReceived;
    bool     xValid;
    int16_t  xPixel;
    uint32_t xSeq;
    char     xText[VISION_X_TEXT_MAX + 1U];

    uint32_t frameOkCnt;
    uint32_t crcErrCnt;
    uint32_t rxBytes;
} VisionData_t;

/* 复位解析状态、快照和统计量，注册 UART0 接收回调前调用一次。 */
void VisionParser_Reset(void);

/* 逐字节喂入解析器；函数无阻塞、无 FreeRTOS 调用，可在中断中调用。 */
void VisionParser_FeedByte(uint8_t byte);

/* 读取最新快照。返回 true 表示至少接收到一帧校验通过的协议帧。 */
bool VisionParser_GetLatest(VisionData_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BALL_PARSER_H */
