#include "emm42_v5.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "bsp_uart.h"

/*
 * 张大头 Emm42_V5.0 协议实现：组帧 → 交给 bsp_uart 的 UART1 整帧下发。
 * 命令帧结构 [地址][功能码][参数...][0x6B]，参数一律【大端】（高字节在前）。
 * 移植自 26RuiKang STM32 工程的 Emm_V5.c，改动点：
 *   - 去掉每条命令后的 osDelay（改为非阻塞，由题目状态机按 30ms 轮询节拍错开）；
 *   - 发送出口换成 BspUart1_SendBytes，不依赖 HAL/DMA；
 *   - 增加回复帧统计，便于上板时判断总线是否真的通。
 */

/* ------------------------------------------------------------------
 * 回复接收：ISR 逐字节累积，遇校验字节 0x6B 视为一帧结束
 * ------------------------------------------------------------------ */

static volatile uint32_t s_txFrameCount;
static volatile uint32_t s_rxByteCount;
static volatile uint32_t s_rxFrameCount;

/* 正在组装的回复帧 */
static volatile uint8_t  s_rxBuf[EMM42_REPLY_MAX_LEN];
static volatile uint8_t  s_rxLen;

/* 最后一帧完整回复的快照 */
static volatile uint8_t  s_lastReply[EMM42_REPLY_MAX_LEN];
static volatile uint8_t  s_lastReplyLen;

/*
 * 独立钢球控制线程会与 UI 题目状态机并发共用 UART1。协议层用互斥量保证整帧
 * 不交叉，并在帧后统一留 5ms 给共享总线上的驱动器处理/回复。
 */
#define EMM42_TX_FRAME_GAP_MS (5U)
static SemaphoreHandle_t s_txMutex;

/*
 * UART1 接收中断回调（中断上下文，只做累积，不调用任何 FreeRTOS API）。
 * Emm42 回复帧无固定帧头、长度随功能码变化，但都以校验字节 0x6B 结尾，
 * 故以 0x6B 作为分帧依据；数据段中偶然出现的 0x6B 会导致提前分帧，
 * 但本模块只把回复用于"总线是否通"的诊断，不解析字段，可以接受。
 */
static void Emm42_OnRxByte(uint8_t byte)
{
    s_rxByteCount++;

    if (s_rxLen < EMM42_REPLY_MAX_LEN) {
        s_rxBuf[s_rxLen] = byte;
        s_rxLen++;
    }

    if ((byte == EMM42_CHECK_BYTE) || (s_rxLen >= EMM42_REPLY_MAX_LEN)) {
        uint8_t i;
        for (i = 0U; i < s_rxLen; i++) {
            s_lastReply[i] = s_rxBuf[i];
        }
        s_lastReplyLen = s_rxLen;
        s_rxLen        = 0U;
        s_rxFrameCount++;   /* 最后自增：任务侧靠它判断快照是否已完整写入 */
    }
}

/* ------------------------------------------------------------------
 * 组帧下发
 * ------------------------------------------------------------------ */

static void Emm42_SendFrame(const uint8_t *cmd, uint16_t len)
{
    bool schedulerRunning = xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;

    if ((s_txMutex != NULL) && schedulerRunning) {
        (void)xSemaphoreTake(s_txMutex, portMAX_DELAY);
    }

    BspUart1_SendBytes(cmd, len);
    s_txFrameCount++;

    if ((s_txMutex != NULL) && schedulerRunning) {
        vTaskDelay(pdMS_TO_TICKS(EMM42_TX_FRAME_GAP_MS));
        (void)xSemaphoreGive(s_txMutex);
    }
}

void Emm42_Init(void)
{
    s_txFrameCount = 0U;
    s_rxByteCount  = 0U;
    s_rxFrameCount = 0U;
    s_rxLen        = 0U;
    s_lastReplyLen = 0U;

    if (s_txMutex == NULL) {
        s_txMutex = xSemaphoreCreateMutex();
        configASSERT(s_txMutex != NULL);
    }

    /* 注册 RX 中断回调（UART1 外设本身已在 BspBoard_Init 中初始化完毕）。 */
    BspUart1_Init(Emm42_OnRxByte);
}

void Emm42_Enable(uint8_t addr, bool enable, bool sync)
{
    /* [地址][0xF3][0xAB][使能状态][多机同步][0x6B] */
    uint8_t cmd[6] = {
        addr, 0xF3U, 0xABU, (uint8_t)(enable ? 1U : 0U), (uint8_t)(sync ? 1U : 0U),
        EMM42_CHECK_BYTE
    };
    Emm42_SendFrame(cmd, (uint16_t)sizeof(cmd));
}

void Emm42_VelControl(uint8_t addr, Emm42Dir_t dir, uint16_t rpm, uint8_t acc, bool sync)
{
    uint8_t cmd[8];

    /* 转速限幅，避免误传大数把电机直接拉到超速。 */
    if (rpm > EMM42_MAX_RPM) {
        rpm = (uint16_t)EMM42_MAX_RPM;
    }

    /* [地址][0xF6][方向][转速高8位][转速低8位][加速度][多机同步][0x6B] */
    cmd[0] = addr;
    cmd[1] = 0xF6U;
    cmd[2] = (uint8_t)dir;
    cmd[3] = (uint8_t)(rpm >> 8);
    cmd[4] = (uint8_t)(rpm & 0xFFU);
    cmd[5] = acc;
    cmd[6] = (uint8_t)(sync ? 1U : 0U);
    cmd[7] = EMM42_CHECK_BYTE;

    Emm42_SendFrame(cmd, (uint16_t)sizeof(cmd));
}

void Emm42_SetSpeedRpm(uint8_t addr, int16_t rpm, uint8_t acc)
{
    if (rpm == 0) {
        /* 0 转速直接走急停命令，比下发 vel=0 更快停住。 */
        Emm42_StopNow(addr, false);
        return;
    }

    if (rpm > 0) {
        Emm42_VelControl(addr, EMM42_DIR_CW, (uint16_t)rpm, acc, false);
    } else {
        Emm42_VelControl(addr, EMM42_DIR_CCW, (uint16_t)(-rpm), acc, false);
    }
}

void Emm42_PosControl(uint8_t addr, Emm42Dir_t dir, uint16_t rpm, uint8_t acc,
                      uint32_t clk, bool absolute, bool sync)
{
    uint8_t cmd[13];

    if (rpm > EMM42_MAX_RPM) {
        rpm = (uint16_t)EMM42_MAX_RPM;
    }

    /* [地址][0xFD][方向][转速16位][加速度][脉冲数32位][相对/绝对][多机同步][0x6B] */
    cmd[0]  = addr;
    cmd[1]  = 0xFDU;
    cmd[2]  = (uint8_t)dir;
    cmd[3]  = (uint8_t)(rpm >> 8);
    cmd[4]  = (uint8_t)(rpm & 0xFFU);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24);
    cmd[7]  = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);
    cmd[9]  = (uint8_t)(clk & 0xFFU);
    cmd[10] = (uint8_t)(absolute ? 1U : 0U);
    cmd[11] = (uint8_t)(sync ? 1U : 0U);
    cmd[12] = EMM42_CHECK_BYTE;

    Emm42_SendFrame(cmd, (uint16_t)sizeof(cmd));
}

void Emm42_StopNow(uint8_t addr, bool sync)
{
    /* [地址][0xFE][0x98][多机同步][0x6B] */
    uint8_t cmd[5] = {addr, 0xFEU, 0x98U, (uint8_t)(sync ? 1U : 0U), EMM42_CHECK_BYTE};
    Emm42_SendFrame(cmd, (uint16_t)sizeof(cmd));
}

void Emm42_SyncMotion(void)
{
    /* 广播帧：[0x00][0xFF][0x66][0x6B] */
    uint8_t cmd[4] = {EMM42_ADDR_BROADCAST, 0xFFU, 0x66U, EMM42_CHECK_BYTE};
    Emm42_SendFrame(cmd, (uint16_t)sizeof(cmd));
}

void Emm42_ResetCurPosToZero(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0AU, 0x6DU, EMM42_CHECK_BYTE};
    Emm42_SendFrame(cmd, (uint16_t)sizeof(cmd));
}

void Emm42_ResetClogProtection(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0EU, 0x52U, EMM42_CHECK_BYTE};
    Emm42_SendFrame(cmd, (uint16_t)sizeof(cmd));
}

void Emm42_ReadSysParams(uint8_t addr, Emm42SysParam_t param)
{
    uint8_t cmd[4];
    uint8_t i = 0U;

    cmd[i++] = addr;
    switch (param) {
        case EMM42_PARAM_VERSION: cmd[i++] = 0x1FU; break;
        case EMM42_PARAM_VBUS:    cmd[i++] = 0x24U; break;
        case EMM42_PARAM_VEL:     cmd[i++] = 0x35U; break;
        case EMM42_PARAM_CPOS:    cmd[i++] = 0x36U; break;
        case EMM42_PARAM_PERR:    cmd[i++] = 0x37U; break;
        case EMM42_PARAM_FLAG:    cmd[i++] = 0x3AU; break;
        case EMM42_PARAM_STATE:   cmd[i++] = 0x43U; cmd[i++] = 0x7AU; break;
        default: return;   /* 未知参数不发帧 */
    }
    cmd[i++] = EMM42_CHECK_BYTE;

    Emm42_SendFrame(cmd, (uint16_t)i);
}

/* ------------------------------------------------------------------
 * 诊断接口
 * ------------------------------------------------------------------ */

uint32_t Emm42_GetTxFrameCount(void)
{
    return s_txFrameCount;
}

uint32_t Emm42_GetRxByteCount(void)
{
    return s_rxByteCount;
}

uint32_t Emm42_GetRxFrameCount(void)
{
    return s_rxFrameCount;
}

bool Emm42_GetLastReply(uint8_t *buf, uint8_t *len)
{
    uint32_t seqBefore;
    uint32_t seqAfter;
    uint8_t  i;
    uint8_t  n;

    if ((buf == NULL) || (len == NULL)) {
        return false;
    }

    /*
     * 不关中断，用帧计数做序号校验：拷贝前后计数一致说明期间没有新帧写入，
     * 快照完整；不一致就再拷一次（回复帧稀疏，两次连续被打断几乎不可能）。
     */
    for (i = 0U; i < 2U; i++) {
        seqBefore = s_rxFrameCount;
        if (seqBefore == 0U) {
            return false;   /* 一帧都没收到 */
        }

        n = s_lastReplyLen;
        if (n > EMM42_REPLY_MAX_LEN) {
            n = EMM42_REPLY_MAX_LEN;
        }
        {
            uint8_t k;
            for (k = 0U; k < n; k++) {
                buf[k] = s_lastReply[k];
            }
        }
        seqAfter = s_rxFrameCount;

        if (seqAfter == seqBefore) {
            *len = n;
            return true;
        }
    }

    *len = 0U;
    return false;
}
