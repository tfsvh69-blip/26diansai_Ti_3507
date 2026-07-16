#ifndef LASER_LD14_H
#define LASER_LD14_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * LD14 单点激光测距模块串口协议解析（移植自参考工程 lidar_manager/ld14）。
 *
 * 协议：195 字节定长帧
 *   [0..3]   帧头 0xAA 0xAA 0xAA 0xAA
 *   [4]      地址(0x00)
 *   [5]      命令字(0x02，读距离)
 *   [6..9]   偏移/长度(固定)
 *   [10..189] 12 个测量点，每点 15 字节：
 *            0-1 距离(mm,小端u16) 2-3 噪声 4-7 峰值 8 置信度 9-12 积分 13-14 温度表征
 *   [190..193] 时间戳(小端u32)
 *   [194]    校验和 = 前 194 字节按字节累加取低 8 位
 *            (兼容两种：含帧头[0..193] 或不含帧头[4..193])
 *
 * 用法：把串口收到的每个字节喂给 LaserLd14_FeedByte()（通常在 UART RX 中断里逐字节调用），
 * 解析成功后 12 个点的非零距离取平均，更新为一个可读的单值距离，
 * 供上层通过 LaserLd14_GetLatest() 读取。本模块不含任何硬件访问，可复用。
 */

typedef struct {
    uint16_t distanceMm;  /* 12 个点非零距离的平均值(mm) */
    uint8_t  confidence;  /* 平均置信度 */
    bool     valid;       /* 是否已收到过至少一帧有效数据 */
    uint32_t frameOkCnt;  /* 累计通过校验的帧数 */
    uint32_t crcErrCnt;   /* 累计校验失败的帧数 */
    uint32_t rxBytes;     /* 累计喂入字节数(判断有没有在收) */
} LaserLd14Data_t;

/* 复位解析状态机与统计量，初始化时调用一次。 */
void LaserLd14_Reset(void);

/* 逐字节喂入解析器；轻量状态机，可在中断中调用。 */
void LaserLd14_FeedByte(uint8_t byte);

/* 读取最近一次解析结果快照。返回 true 表示已收到过有效帧。 */
bool LaserLd14_GetLatest(LaserLd14Data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LASER_LD14_H */
