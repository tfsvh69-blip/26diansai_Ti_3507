#include "ball_parser.h"

#include <string.h>

/*
 * 行缓冲上限：一帧最长形如 "BALL,0,639,479,999*XX" 约 22 字节，
 * 取 48 留足余量；超长即判为噪声，丢弃并等待下一个 '$' 重同步。
 * 缓冲不含起始 '$'（收到 '$' 才开始捕获，'$' 本身不入缓冲）。
 */
#define BALL_BUF_MAX     (48U)
#define BALL_PREFIX      "BALL,"
#define BALL_PREFIX_LEN  (5U)

/*
 * 解析结果快照。ISR 内写、任务读；均为 32/16/8 位对齐单元，
 * Cortex-M0+ 单指令读写天然原子。found/x/y/count 是一组关联量，
 * 极偶发的跨帧撕裂（例如 found 为新帧、坐标为旧帧）仅影响一次显示刷新，
 * 对 3Hz 的状态显示无实质影响，故与 module/laser 一致不做临界区保护。
 */
static volatile bool     s_found = false;
static volatile uint16_t s_x = 0U;
static volatile uint16_t s_y = 0U;
static volatile uint16_t s_count = 0U;
static volatile bool     s_valid = false;
static volatile uint32_t s_frameOkCnt = 0U;
static volatile uint32_t s_crcErrCnt = 0U;
static volatile uint32_t s_rxBytes = 0U;

/* 行组帧缓冲与状态。 */
static uint8_t  s_buf[BALL_BUF_MAX];
static uint16_t s_idx = 0U;         /* 已写入缓冲的字符数 */
static bool     s_capturing = false;/* 是否已收到 '$' 正在捕获一行 */

/* 单个十六进制字符转 0..15，非法返回 0xFF。 */
static uint8_t BallParser_HexVal(uint8_t c)
{
    if ((c >= '0') && (c <= '9')) {
        return (uint8_t)(c - '0');
    }
    if ((c >= 'A') && (c <= 'F')) {
        return (uint8_t)(c - 'A' + 10);
    }
    if ((c >= 'a') && (c <= 'f')) {
        return (uint8_t)(c - 'a' + 10);
    }
    return 0xFFU;
}

/*
 * 从 buf[*pos] 起解析一个非负十进制整数，跳过后停在分隔符处。
 * 返回 true 表示至少解析到一位数字；数值经 uint16 上限饱和。
 * 结束后 *pos 指向第一个非数字字符（分隔符 ',' 或 '*'）。
 */
static bool BallParser_ParseUint(const uint8_t *buf, uint16_t len,
                                 uint16_t *pos, uint16_t *outVal)
{
    uint32_t v = 0U;
    bool     hasDigit = false;

    while ((*pos < len) && (buf[*pos] >= '0') && (buf[*pos] <= '9')) {
        v = (v * 10U) + (uint32_t)(buf[*pos] - '0');
        if (v > 0xFFFFU) {
            v = 0xFFFFU;  /* 饱和，防溢出 */
        }
        hasDigit = true;
        (*pos)++;
    }

    *outVal = (uint16_t)v;
    return hasDigit;
}

/*
 * 对一整行（buf[0..len-1]，不含起始 '$' 与结尾 CR/LF）做校验并解析：
 *   1. 定位 '*'，校验 '$'..'*' 之间字符的逐字节 XOR 与其后两位十六进制一致；
 *   2. 确认前缀 "BALL,"；
 *   3. 依次解析 found,x,y,n 四个字段。
 * 任一步失败计入 crcErrCnt 并保留上一次快照。
 */
static void BallParser_ParseLine(const uint8_t *buf, uint16_t len)
{
    uint16_t starPos = 0U;
    uint16_t i;
    uint8_t  xorCalc = 0U;
    uint8_t  hi;
    uint8_t  lo;
    uint16_t pos;
    uint16_t found;
    uint16_t x;
    uint16_t y;
    uint16_t n;

    /* 定位校验分隔符 '*'。 */
    while ((starPos < len) && (buf[starPos] != '*')) {
        starPos++;
    }
    /* '*' 后至少还需两位十六进制校验值：要求 buf[starPos+1]、buf[starPos+2] 均存在。 */
    if ((starPos + 2U) >= len) {
        s_crcErrCnt++;
        return;
    }

    /* '$' 与 '*' 之间所有字符逐字节 XOR（buf 不含 '$'，即 buf[0..starPos-1]）。 */
    for (i = 0U; i < starPos; i++) {
        xorCalc ^= buf[i];
    }

    hi = BallParser_HexVal(buf[starPos + 1U]);
    lo = BallParser_HexVal(buf[starPos + 2U]);
    if ((hi == 0xFFU) || (lo == 0xFFU)) {
        s_crcErrCnt++;
        return;
    }
    if (xorCalc != (uint8_t)((hi << 4) | lo)) {
        s_crcErrCnt++;
        return;
    }

    /* 前缀确认 "BALL,"。 */
    if ((starPos < BALL_PREFIX_LEN) ||
        (memcmp(buf, BALL_PREFIX, BALL_PREFIX_LEN) != 0)) {
        s_crcErrCnt++;
        return;
    }

    /* 逐字段解析 found,x,y,n，字段间以 ',' 分隔。 */
    pos = BALL_PREFIX_LEN;
    if (!BallParser_ParseUint(buf, starPos, &pos, &found)) { s_crcErrCnt++; return; }
    if ((pos >= starPos) || (buf[pos] != ',')) { s_crcErrCnt++; return; }
    pos++;
    if (!BallParser_ParseUint(buf, starPos, &pos, &x)) { s_crcErrCnt++; return; }
    if ((pos >= starPos) || (buf[pos] != ',')) { s_crcErrCnt++; return; }
    pos++;
    if (!BallParser_ParseUint(buf, starPos, &pos, &y)) { s_crcErrCnt++; return; }
    if ((pos >= starPos) || (buf[pos] != ',')) { s_crcErrCnt++; return; }
    pos++;
    if (!BallParser_ParseUint(buf, starPos, &pos, &n)) { s_crcErrCnt++; return; }

    /* 全部通过：更新快照。 */
    s_found = (found != 0U);
    s_x = x;
    s_y = y;
    s_count = n;
    s_valid = true;
    s_frameOkCnt++;
}

void BallParser_Reset(void)
{
    s_idx = 0U;
    s_capturing = false;
    memset(s_buf, 0, sizeof(s_buf));
}

void BallParser_FeedByte(uint8_t byte)
{
    s_rxBytes++;

    /* 帧起始：任何时候收到 '$' 都重新开始捕获（自动丢弃残缺的上一行）。 */
    if (byte == (uint8_t)'$') {
        s_idx = 0U;
        s_capturing = true;
        return;
    }

    if (!s_capturing) {
        return;  /* 尚未对齐帧头，忽略 */
    }

    /* 行结束：换行触发解析；'\r' 一并作为结束符处理。 */
    if ((byte == (uint8_t)'\n') || (byte == (uint8_t)'\r')) {
        if (s_idx > 0U) {
            BallParser_ParseLine(s_buf, s_idx);
        }
        s_idx = 0U;
        s_capturing = false;
        return;
    }

    /* 行内字符入缓冲；超长视为噪声，丢弃并等待下一个 '$'。 */
    if (s_idx < BALL_BUF_MAX) {
        s_buf[s_idx] = byte;
        s_idx++;
    } else {
        s_capturing = false;
    }
}

bool BallParser_GetLatest(BallData_t *out)
{
    if (out == NULL) {
        return false;
    }

    out->found = s_found;
    out->x = s_x;
    out->y = s_y;
    out->count = s_count;
    out->valid = s_valid;
    out->frameOkCnt = s_frameOkCnt;
    out->crcErrCnt = s_crcErrCnt;
    out->rxBytes = s_rxBytes;

    return s_valid;
}
