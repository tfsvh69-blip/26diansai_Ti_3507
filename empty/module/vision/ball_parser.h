#ifndef BALL_PARSER_H
#define BALL_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 上位机小球检测报文解析（NMEA 风格 ASCII，走 UART0 下行：上位机 → 下位机）。
 *
 * 一帧一行：
 *   $BALL,<found>,<x>,<y>,<n>*<CHK>\r\n
 *     found  1=检测到球，0=没检测到
 *     x,y    主目标(面积最大)球心像素坐标；去畸变后 640x480，原点左上，x∈[0,639] y∈[0,479]
 *     n      本帧检测到的球总数
 *     CHK    '$' 与 '*' 之间所有字符逐字节 XOR，两位大写十六进制
 * 发送频率约 17 帧/秒（处理帧率），每帧一条。
 *
 * 用法：把 UART0 收到的每个字节喂给 BallParser_FeedByte()（在 UART0 RX 中断里逐字节调用），
 * 解析并校验通过后更新一份可读快照，供上层 BallParser_GetLatest() 读取显示。
 * 本模块不含任何硬件/RTOS 访问，纯 C，可复用（与 module/laser 同一设计风格）。
 */

typedef struct {
    bool     found;       /* 最近一帧是否检测到球 */
    uint16_t x;           /* 主目标球心 x 像素（found=0 时为 0） */
    uint16_t y;           /* 主目标球心 y 像素（found=0 时为 0） */
    uint16_t count;       /* 本帧检测到的球总数 */
    bool     valid;       /* 是否已收到过至少一帧校验通过的数据 */
    uint32_t frameOkCnt;  /* 累计通过校验的帧数 */
    uint32_t crcErrCnt;   /* 累计校验/格式错误的帧数 */
    uint32_t rxBytes;     /* 累计喂入字节数（判断有没有在收） */
} BallData_t;

/* 复位解析状态机与统计量，初始化时调用一次。 */
void BallParser_Reset(void);

/* 逐字节喂入解析器；轻量行缓冲状态机，可在中断中调用。 */
void BallParser_FeedByte(uint8_t byte);

/* 读取最近一次解析结果快照。返回 true 表示已收到过校验通过的帧。 */
bool BallParser_GetLatest(BallData_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BALL_PARSER_H */
