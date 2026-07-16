#include "laser_ld14.h"

#include <string.h>

#define LD14_FRAME_LEN   (195U)   /* 定长帧总字节数 */
#define LD14_HEADER      (0xAAU)  /* 帧头字节，连续 4 个 */
#define LD14_CMD_DIST    (0x02U)  /* 命令字：读距离 */
#define LD14_POINT_NUM   (12U)    /* 每帧测量点数 */
#define LD14_POINT_SIZE  (15U)    /* 每点字节数 */
#define LD14_POINT_BASE  (10U)    /* 首个测量点在帧内的偏移 */

/*
 * 解析结果快照。ISR 内写、任务读；均为 32/16/8 位对齐单元，
 * Cortex-M0+ 单指令读写天然原子，单值显示不做临界区保护。
 */
static volatile uint16_t s_distanceMm = 0U;
static volatile uint8_t  s_confidence = 0U;
static volatile bool     s_valid = false;
static volatile uint32_t s_frameOkCnt = 0U;
static volatile uint32_t s_crcErrCnt = 0U;
static volatile uint32_t s_rxBytes = 0U;

/* 字节流组帧缓冲与状态：等到攒够一整帧再校验解析。 */
static uint8_t  s_buf[LD14_FRAME_LEN];
static uint16_t s_idx = 0U;        /* 已写入缓冲的字节数；0 表示正在找帧头 */
static uint8_t  s_headerCnt = 0U;  /* 连续 0xAA 计数 */

static uint16_t LaserLd14_ReadU16Le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/*
 * 对整帧做校验并解析。校验和 = 前 194 字节累加取低 8 位，
 * 兼容"含帧头(0..193)"与"不含帧头(4..193)"两种累加口径。
 * 通过后取 12 个点里非零距离的平均，得到单个可读距离值。
 */
static void LaserLd14_ParseFrame(void)
{
    uint32_t crcAll = 0U;
    uint32_t crcNoHeader = 0U;
    uint16_t i;
    uint32_t sumDistance = 0U;
    uint32_t sumConf = 0U;
    uint32_t count = 0U;

    /* 帧头与命令字二次确认（组帧阶段已保证帧头，这里防御性再查一次）。 */
    if ((s_buf[0] != LD14_HEADER) || (s_buf[5] != LD14_CMD_DIST)) {
        s_crcErrCnt++;
        return;
    }

    for (i = 0U; i < (LD14_FRAME_LEN - 1U); ++i) {
        crcAll += s_buf[i];
        if (i >= 4U) {
            crcNoHeader += s_buf[i];
        }
    }

    if (((uint8_t)crcAll != s_buf[LD14_FRAME_LEN - 1U]) &&
        ((uint8_t)crcNoHeader != s_buf[LD14_FRAME_LEN - 1U])) {
        s_crcErrCnt++;
        return;
    }

    /* 遍历 12 个点，累加非零距离与对应置信度。 */
    for (i = 0U; i < LD14_POINT_NUM; ++i) {
        const uint8_t *p = &s_buf[LD14_POINT_BASE + (i * LD14_POINT_SIZE)];
        uint16_t dist = LaserLd14_ReadU16Le(p + 0);

        if (dist != 0U) {
            sumDistance += dist;
            sumConf += p[8];
            count++;
        }
    }

    s_frameOkCnt++;

    if (count == 0U) {
        /* 校验通过但全为无效点（目标超量程/无回波），保留上一次距离。 */
        return;
    }

    s_distanceMm = (uint16_t)(sumDistance / count);
    s_confidence = (uint8_t)(sumConf / count);
    s_valid = true;
}

void LaserLd14_Reset(void)
{
    s_idx = 0U;
    s_headerCnt = 0U;
    memset(s_buf, 0, sizeof(s_buf));
}

void LaserLd14_FeedByte(uint8_t byte)
{
    s_rxBytes++;

    /* 阶段一：尚未对齐帧头，连续找 4 个 0xAA。 */
    if (s_idx == 0U) {
        if (byte == LD14_HEADER) {
            s_headerCnt++;
            if (s_headerCnt >= 4U) {
                s_buf[0] = LD14_HEADER;
                s_buf[1] = LD14_HEADER;
                s_buf[2] = LD14_HEADER;
                s_buf[3] = LD14_HEADER;
                s_idx = 4U;
                s_headerCnt = 0U;
            }
        } else {
            s_headerCnt = 0U;
        }
        return;
    }

    /* 阶段二：帧头已对齐，顺序填充直到攒满一整帧。 */
    s_buf[s_idx] = byte;
    s_idx++;

    if (s_idx >= LD14_FRAME_LEN) {
        LaserLd14_ParseFrame();
        /* 无论成功与否都复位重新找帧头，保证失步后能自动重同步。 */
        s_idx = 0U;
        s_headerCnt = 0U;
    }
}

bool LaserLd14_GetLatest(LaserLd14Data_t *out)
{
    if (out == NULL) {
        return false;
    }

    out->distanceMm = s_distanceMm;
    out->confidence = s_confidence;
    out->valid = s_valid;
    out->frameOkCnt = s_frameOkCnt;
    out->crcErrCnt = s_crcErrCnt;
    out->rxBytes = s_rxBytes;

    return s_valid;
}
