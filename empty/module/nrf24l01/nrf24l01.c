#include "nrf24l01.h"

#include <stddef.h>
#include <string.h>

#include "bsp_nrf24_port.h"

/* SPI 指令。 */
#define NRF24_CMD_R_REGISTER   (0x00U)
#define NRF24_CMD_W_REGISTER   (0x20U)
#define NRF24_CMD_W_TX_PAYLOAD (0xA0U)
#define NRF24_CMD_FLUSH_TX     (0xE1U)
#define NRF24_CMD_FLUSH_RX     (0xE2U)
#define NRF24_CMD_NOP          (0xFFU)

/* 寄存器地址。 */
#define NRF24_REG_CONFIG       (0x00U)
#define NRF24_REG_EN_AA        (0x01U)
#define NRF24_REG_EN_RXADDR    (0x02U)
#define NRF24_REG_SETUP_AW     (0x03U)
#define NRF24_REG_SETUP_RETR   (0x04U)
#define NRF24_REG_RF_CH        (0x05U)
#define NRF24_REG_RF_SETUP     (0x06U)
#define NRF24_REG_STATUS       (0x07U)
#define NRF24_REG_OBSERVE_TX   (0x08U)
#define NRF24_REG_RX_ADDR_P0   (0x0AU)
#define NRF24_REG_TX_ADDR      (0x10U)
#define NRF24_REG_RX_PW_P0     (0x11U)
#define NRF24_REG_FIFO_STATUS  (0x17U)
#define NRF24_REG_DYNPD        (0x1CU)
#define NRF24_REG_FEATURE      (0x1DU)

/* STATUS 标志。 */
#define NRF24_STATUS_MAX_RT    (0x10U)
#define NRF24_STATUS_TX_DS     (0x20U)
#define NRF24_STATUS_IRQ_MASK  (0x70U)

#define NRF24_TX_TIMEOUT_US              (12000U)
#define NRF24_TX_POLL_INTERVAL_US        (50U)

/* USB 无线串口 V2.0 实机联调成功的无线参数。 */
const Nrf24RadioConfig_t g_nrf24UsbUartV20Config = {
    {0x15U, 0x52U, 0x33U, 0x54U, 0x55U},
    0x0EU, /* PWR_UP + 16 位 CRC + TX 模式。 */
    0x1AU, /* 自动重传间隔 500us，最多重传 10 次。 */
    0x02U, /* 2.402GHz。 */
    0x0FU  /* 2Mbps、0dBm，保持与 USB 模块实测匹配。 */
};

volatile Nrf24Diag_t g_nrf24Diag = {
    NRF24_DIAG_MAGIC
};

static bool s_initialized = false;

static uint8_t Nrf24_SpiExchange(uint8_t value)
{
    g_nrf24Diag.spiByteCount++;
    return BspNrf24Port_SpiExchange(value);
}

static uint8_t Nrf24_BeginCommand(uint8_t command)
{
    uint8_t status;

    BspNrf24Port_SetCsn(false);
    status = Nrf24_SpiExchange(command);
    g_nrf24Diag.spiCommandCount++;
    g_nrf24Diag.lastCommand = command;
    g_nrf24Diag.lastSpiStatus = status;
    return status;
}

static void Nrf24_EndCommand(void)
{
    BspNrf24Port_SetCsn(true);
}

static uint8_t Nrf24_Command(uint8_t command)
{
    uint8_t status = Nrf24_BeginCommand(command);

    Nrf24_EndCommand();
    return status;
}

static uint8_t Nrf24_ReadReg(uint8_t reg)
{
    uint8_t value;

    (void)Nrf24_BeginCommand(
        (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1FU)));
    value = Nrf24_SpiExchange(NRF24_CMD_NOP);
    Nrf24_EndCommand();
    return value;
}

static void Nrf24_ReadBuffer(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t i;

    (void)Nrf24_BeginCommand(
        (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1FU)));
    for (i = 0U; i < length; i++) {
        data[i] = Nrf24_SpiExchange(NRF24_CMD_NOP);
    }
    Nrf24_EndCommand();
}

static void Nrf24_WriteReg(uint8_t reg, uint8_t value)
{
    (void)Nrf24_BeginCommand(
        (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1FU)));
    (void)Nrf24_SpiExchange(value);
    Nrf24_EndCommand();
}

static void Nrf24_WriteBuffer(
    uint8_t command, const uint8_t *data, uint8_t length)
{
    uint8_t i;

    (void)Nrf24_BeginCommand(command);
    for (i = 0U; i < length; i++) {
        (void)Nrf24_SpiExchange(data[i]);
    }
    Nrf24_EndCommand();
}

static uint8_t Nrf24_ReadStatus(void)
{
    return Nrf24_Command(NRF24_CMD_NOP);
}

static void Nrf24_CaptureRegisterSnapshot(void)
{
    uint8_t txAddress[NRF24L01_ADDRESS_WIDTH];
    uint8_t rxAddress[NRF24L01_ADDRESS_WIDTH];
    uint8_t i;

    g_nrf24Diag.regConfig = Nrf24_ReadReg(NRF24_REG_CONFIG);
    g_nrf24Diag.regEnAa = Nrf24_ReadReg(NRF24_REG_EN_AA);
    g_nrf24Diag.regEnRxaddr = Nrf24_ReadReg(NRF24_REG_EN_RXADDR);
    g_nrf24Diag.regSetupAw = Nrf24_ReadReg(NRF24_REG_SETUP_AW);
    g_nrf24Diag.regSetupRetr = Nrf24_ReadReg(NRF24_REG_SETUP_RETR);
    g_nrf24Diag.regRfCh = Nrf24_ReadReg(NRF24_REG_RF_CH);
    g_nrf24Diag.regRfSetup = Nrf24_ReadReg(NRF24_REG_RF_SETUP);
    g_nrf24Diag.regRxPwP0 = Nrf24_ReadReg(NRF24_REG_RX_PW_P0);
    g_nrf24Diag.regDynpd = Nrf24_ReadReg(NRF24_REG_DYNPD);
    g_nrf24Diag.regFeature = Nrf24_ReadReg(NRF24_REG_FEATURE);
    g_nrf24Diag.lastStatus = Nrf24_ReadStatus();
    g_nrf24Diag.lastObserveTx = Nrf24_ReadReg(NRF24_REG_OBSERVE_TX);
    g_nrf24Diag.lastFifoStatus = Nrf24_ReadReg(NRF24_REG_FIFO_STATUS);

    Nrf24_ReadBuffer(NRF24_REG_TX_ADDR, txAddress, NRF24L01_ADDRESS_WIDTH);
    Nrf24_ReadBuffer(NRF24_REG_RX_ADDR_P0, rxAddress, NRF24L01_ADDRESS_WIDTH);
    for (i = 0U; i < NRF24L01_ADDRESS_WIDTH; i++) {
        g_nrf24Diag.txAddress[i] = txAddress[i];
        g_nrf24Diag.rxAddressP0[i] = rxAddress[i];
    }

    g_nrf24Diag.lastArcCount = g_nrf24Diag.lastObserveTx & 0x0FU;
    g_nrf24Diag.lostPacketCount =
        (g_nrf24Diag.lastObserveTx >> 4) & 0x0FU;
    g_nrf24Diag.gpioLevelMask = BspNrf24Port_ReadPinLevels();
}

static uint8_t Nrf24_VerifyConfiguration(
    const Nrf24RadioConfig_t *radioConfig)
{
    uint8_t i;

    if (g_nrf24Diag.regConfig != radioConfig->config) {
        return NRF24_INIT_FAIL_CONFIG;
    }
    if (g_nrf24Diag.regEnAa != 0x01U) {
        return NRF24_INIT_FAIL_EN_AA;
    }
    if (g_nrf24Diag.regEnRxaddr != 0x01U) {
        return NRF24_INIT_FAIL_EN_RXADDR;
    }
    if (g_nrf24Diag.regSetupAw != 0x03U) {
        return NRF24_INIT_FAIL_SETUP_AW;
    }
    if (g_nrf24Diag.regSetupRetr != radioConfig->setupRetr) {
        return NRF24_INIT_FAIL_SETUP_RETR;
    }
    if (g_nrf24Diag.regRfCh != radioConfig->rfChannel) {
        return NRF24_INIT_FAIL_RF_CH;
    }
    if (g_nrf24Diag.regRfSetup != radioConfig->rfSetup) {
        return NRF24_INIT_FAIL_RF_SETUP;
    }
    if (g_nrf24Diag.regRxPwP0 != NRF24L01_FIXED_PAYLOAD_WIDTH) {
        return NRF24_INIT_FAIL_RX_PW_P0;
    }
    if (g_nrf24Diag.regDynpd != 0x00U) {
        return NRF24_INIT_FAIL_DYNPD;
    }
    if (g_nrf24Diag.regFeature != 0x00U) {
        return NRF24_INIT_FAIL_FEATURE;
    }

    for (i = 0U; i < NRF24L01_ADDRESS_WIDTH; i++) {
        if (g_nrf24Diag.txAddress[i] != radioConfig->peerAddress[i]) {
            return NRF24_INIT_FAIL_TX_ADDRESS;
        }
        if (g_nrf24Diag.rxAddressP0[i] != radioConfig->peerAddress[i]) {
            return NRF24_INIT_FAIL_RX_ADDRESS_P0;
        }
    }

    return NRF24_INIT_FAIL_NONE;
}

static void Nrf24_CaptureTxResult(uint8_t status)
{
    g_nrf24Diag.lastStatus = status;
    g_nrf24Diag.lastObserveTx = Nrf24_ReadReg(NRF24_REG_OBSERVE_TX);
    g_nrf24Diag.lastFifoStatus = Nrf24_ReadReg(NRF24_REG_FIFO_STATUS);
    g_nrf24Diag.lastArcCount = g_nrf24Diag.lastObserveTx & 0x0FU;
    g_nrf24Diag.lostPacketCount =
        (g_nrf24Diag.lastObserveTx >> 4) & 0x0FU;
    g_nrf24Diag.gpioLevelMask = BspNrf24Port_ReadPinLevels();
}

bool Nrf24_Init(const Nrf24RadioConfig_t *radioConfig)
{
    uint8_t failCode;
    uint8_t configPowerDown;

    g_nrf24Diag.magic = NRF24_DIAG_MAGIC;
    g_nrf24Diag.initAttempts++;
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_PORT_INIT;
    g_nrf24Diag.radioReady = 0U;
    g_nrf24Diag.initFailCode = NRF24_INIT_FAIL_NONE;

    if (radioConfig == NULL) {
        g_nrf24Diag.initFailCode = NRF24_INIT_FAIL_NULL_ADDRESS;
        g_nrf24Diag.stage = NRF24_DIAG_STAGE_INIT_FAILED;
        g_nrf24Diag.initFailure++;
        return false;
    }

    s_initialized = false;
    BspNrf24Port_Init();
    BspNrf24Port_SetCe(false);
    g_nrf24Diag.gpioLevelMask = BspNrf24Port_ReadPinLevels();
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_WRITE_CONFIG;
    configPowerDown = radioConfig->config & (uint8_t)~0x03U;

    /*
     * 配置阶段保持掉电，全部寄存器写完后再 PWR_UP。
     * TX_ADDR 与 RX_ADDR_P0 必须相同，发送端才能在通道 0 接收自动应答。
     */
    Nrf24_WriteReg(NRF24_REG_CONFIG, configPowerDown);
    Nrf24_WriteReg(NRF24_REG_EN_AA, 0x01U);
    Nrf24_WriteReg(NRF24_REG_EN_RXADDR, 0x01U);
    Nrf24_WriteReg(NRF24_REG_SETUP_AW, 0x03U);
    Nrf24_WriteReg(NRF24_REG_SETUP_RETR, radioConfig->setupRetr);
    Nrf24_WriteReg(NRF24_REG_RF_CH, radioConfig->rfChannel);
    Nrf24_WriteReg(NRF24_REG_RF_SETUP, radioConfig->rfSetup);
    Nrf24_WriteReg(NRF24_REG_RX_PW_P0, NRF24L01_FIXED_PAYLOAD_WIDTH);
    Nrf24_WriteReg(NRF24_REG_DYNPD, 0x00U);
    Nrf24_WriteReg(NRF24_REG_FEATURE, 0x00U);
    Nrf24_WriteBuffer((uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR),
                      radioConfig->peerAddress,
                      NRF24L01_ADDRESS_WIDTH);
    Nrf24_WriteBuffer((uint8_t)(NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_P0),
                      radioConfig->peerAddress,
                      NRF24L01_ADDRESS_WIDTH);

    (void)Nrf24_Command(NRF24_CMD_FLUSH_TX);
    (void)Nrf24_Command(NRF24_CMD_FLUSH_RX);
    Nrf24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    Nrf24_WriteReg(NRF24_REG_CONFIG, radioConfig->config);

    /* 从 Power Down 到 Standby-I 最长约 1.5ms，留足 2ms 后再完整回读。 */
    BspNrf24Port_DelayUs(2000U);
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_READBACK;
    Nrf24_CaptureRegisterSnapshot();
    failCode = Nrf24_VerifyConfiguration(radioConfig);
    g_nrf24Diag.initFailCode = failCode;

    if (failCode != NRF24_INIT_FAIL_NONE) {
        g_nrf24Diag.stage = NRF24_DIAG_STAGE_INIT_FAILED;
        g_nrf24Diag.initFailure++;
        return false;
    }

    s_initialized = true;
    g_nrf24Diag.radioReady = 1U;
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_READY;
    g_nrf24Diag.initSuccess++;
    return true;
}

Nrf24TxResult_t Nrf24_SendPayload(
    const uint8_t payload[NRF24L01_FIXED_PAYLOAD_WIDTH])
{
    uint32_t elapsedUs;
    uint8_t status = 0U;

    g_nrf24Diag.sendAttempts++;

    if (!s_initialized) {
        g_nrf24Diag.radioReady = 0U;
        g_nrf24Diag.lastTxResult = NRF24_TX_NOT_READY;
        g_nrf24Diag.stage = NRF24_DIAG_STAGE_NOT_READY;
        return NRF24_TX_NOT_READY;
    }
    if (payload == NULL) {
        g_nrf24Diag.lastTxResult = NRF24_TX_INVALID_PAYLOAD;
        return NRF24_TX_INVALID_PAYLOAD;
    }

    g_nrf24Diag.lastPayloadLength = payload[0];
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_LOAD_PAYLOAD;
    BspNrf24Port_SetCe(false);
    Nrf24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    (void)Nrf24_Command(NRF24_CMD_FLUSH_TX);
    Nrf24_WriteBuffer(
        NRF24_CMD_W_TX_PAYLOAD, payload, NRF24L01_FIXED_PAYLOAD_WIDTH);

    /* CE 高脉冲需至少 10us；15us 后拉低，随后轮询 STATUS，不依赖 IRQ 引脚。 */
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_CE_PULSE;
    BspNrf24Port_SetCe(true);
    BspNrf24Port_DelayUs(15U);
    BspNrf24Port_SetCe(false);
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_WAIT_STATUS;

    for (elapsedUs = 0U;
         elapsedUs < NRF24_TX_TIMEOUT_US;
         elapsedUs += NRF24_TX_POLL_INTERVAL_US) {
        status = Nrf24_ReadStatus();

        if ((status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) ==
            (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) {
            Nrf24_CaptureTxResult(status);
            Nrf24_WriteReg(NRF24_REG_STATUS,
                           NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
            (void)Nrf24_Command(NRF24_CMD_FLUSH_TX);
            s_initialized = false;
            g_nrf24Diag.radioReady = 0U;
            g_nrf24Diag.lastTxResult = NRF24_TX_IO_ERROR;
            g_nrf24Diag.stage = NRF24_DIAG_STAGE_IO_ERROR;
            g_nrf24Diag.txIoError++;
            return NRF24_TX_IO_ERROR;
        }
        if ((status & NRF24_STATUS_TX_DS) != 0U) {
            Nrf24_CaptureTxResult(status);
            Nrf24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
            g_nrf24Diag.lastTxResult = NRF24_TX_OK;
            g_nrf24Diag.stage = NRF24_DIAG_STAGE_TX_OK;
            g_nrf24Diag.txSuccess++;
            return NRF24_TX_OK;
        }
        if ((status & NRF24_STATUS_MAX_RT) != 0U) {
            Nrf24_CaptureTxResult(status);
            Nrf24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
            (void)Nrf24_Command(NRF24_CMD_FLUSH_TX);
            g_nrf24Diag.lastTxResult = NRF24_TX_MAX_RETRY;
            g_nrf24Diag.stage = NRF24_DIAG_STAGE_MAX_RETRY;
            g_nrf24Diag.txMaxRetry++;
            return NRF24_TX_MAX_RETRY;
        }

        BspNrf24Port_DelayUs(NRF24_TX_POLL_INTERVAL_US);
    }

    Nrf24_CaptureTxResult(status);
    (void)Nrf24_Command(NRF24_CMD_FLUSH_TX);
    Nrf24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_IRQ_MASK);
    s_initialized = false;
    g_nrf24Diag.radioReady = 0U;
    g_nrf24Diag.lastTxResult = NRF24_TX_TIMEOUT;
    g_nrf24Diag.stage = NRF24_DIAG_STAGE_TIMEOUT;
    g_nrf24Diag.txTimeout++;
    return NRF24_TX_TIMEOUT;
}

Nrf24TxResult_t Nrf24_SendUsbUartText(
    const uint8_t *text, uint8_t textLength)
{
    uint8_t payload[NRF24L01_FIXED_PAYLOAD_WIDTH];

    if ((text == NULL) || (textLength == 0U) ||
        (textLength >= NRF24L01_FIXED_PAYLOAD_WIDTH)) {
        g_nrf24Diag.lastTxResult = NRF24_TX_INVALID_PAYLOAD;
        return NRF24_TX_INVALID_PAYLOAD;
    }

    /* USB 无线串口的第 0 字节是有效文本长度，后续字节才是透传正文。 */
    memset(payload, 0, sizeof(payload));
    payload[0] = textLength;
    memcpy(&payload[1], text, textLength);

    return Nrf24_SendPayload(payload);
}
