#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF24L01_ADDRESS_WIDTH       (5U)
#define NRF24L01_FIXED_PAYLOAD_WIDTH (32U)
#define NRF24_DIAG_MAGIC             (0x4E524632UL)

typedef enum {
    NRF24_DIAG_STAGE_RESET = 0,
    NRF24_DIAG_STAGE_POWER_WAIT,
    NRF24_DIAG_STAGE_PORT_INIT,
    NRF24_DIAG_STAGE_WRITE_CONFIG,
    NRF24_DIAG_STAGE_READBACK,
    NRF24_DIAG_STAGE_READY,
    NRF24_DIAG_STAGE_LOAD_PAYLOAD,
    NRF24_DIAG_STAGE_CE_PULSE,
    NRF24_DIAG_STAGE_WAIT_STATUS,
    NRF24_DIAG_STAGE_TX_OK,
    NRF24_DIAG_STAGE_MAX_RETRY,
    NRF24_DIAG_STAGE_TIMEOUT,
    NRF24_DIAG_STAGE_IO_ERROR,
    NRF24_DIAG_STAGE_INIT_FAILED,
    NRF24_DIAG_STAGE_NOT_READY
} Nrf24DiagStage_t;

typedef enum {
    NRF24_INIT_FAIL_NONE = 0,
    NRF24_INIT_FAIL_NULL_ADDRESS,
    NRF24_INIT_FAIL_CONFIG,
    NRF24_INIT_FAIL_EN_AA,
    NRF24_INIT_FAIL_EN_RXADDR,
    NRF24_INIT_FAIL_SETUP_AW,
    NRF24_INIT_FAIL_SETUP_RETR,
    NRF24_INIT_FAIL_RF_CH,
    NRF24_INIT_FAIL_RF_SETUP,
    NRF24_INIT_FAIL_RX_PW_P0,
    NRF24_INIT_FAIL_DYNPD,
    NRF24_INIT_FAIL_FEATURE,
    NRF24_INIT_FAIL_TX_ADDRESS,
    NRF24_INIT_FAIL_RX_ADDRESS_P0
} Nrf24InitFail_t;

typedef enum {
    NRF24_TX_OK = 0,
    NRF24_TX_MAX_RETRY,
    NRF24_TX_TIMEOUT,
    NRF24_TX_NOT_READY,
    NRF24_TX_IO_ERROR
} Nrf24TxResult_t;

/* 一组可完整复现的 NRF24L01+ 发射参数。 */
typedef struct {
    uint8_t peerAddress[NRF24L01_ADDRESS_WIDTH];
    uint8_t config;
    uint8_t setupRetr;
    uint8_t rfChannel;
    uint8_t rfSetup;
} Nrf24RadioConfig_t;

/*
 * Keil Debug Watch 诊断快照。
 * 全局变量 g_nrf24Diag 只用于观测，不作为任务间业务通信。
 */
typedef struct {
    uint32_t magic;
    uint32_t taskHeartbeat;
    uint32_t spiCommandCount;
    uint32_t spiByteCount;
    uint32_t initAttempts;
    uint32_t initSuccess;
    uint32_t initFailure;
    uint32_t sendAttempts;
    uint32_t txSuccess;
    uint32_t txMaxRetry;
    uint32_t txTimeout;
    uint32_t txIoError;
    uint32_t lastSequence;
    uint8_t stage;
    uint8_t initFailCode;
    uint8_t radioReady;
    uint8_t lastTxResult;
    uint8_t lastCommand;
    uint8_t lastSpiStatus;
    uint8_t lastStatus;
    uint8_t lastObserveTx;
    uint8_t lastFifoStatus;
    uint8_t lastArcCount;
    uint8_t lostPacketCount;
    uint8_t lastPayloadLength;
    uint8_t gpioLevelMask;
    uint8_t regConfig;
    uint8_t regEnAa;
    uint8_t regEnRxaddr;
    uint8_t regSetupAw;
    uint8_t regSetupRetr;
    uint8_t regRfCh;
    uint8_t regRfSetup;
    uint8_t regRxPwP0;
    uint8_t regDynpd;
    uint8_t regFeature;
    uint8_t profileIndex;
    uint8_t profileLocked;
    uint8_t profileAttemptCount;
    uint8_t reserved;
    uint8_t txAddress[NRF24L01_ADDRESS_WIDTH];
    uint8_t rxAddressP0[NRF24L01_ADDRESS_WIDTH];
} Nrf24Diag_t;

extern volatile Nrf24Diag_t g_nrf24Diag;

/*
 * 按指定参数初始化：通道 0 自动应答、32 字节固定载荷。
 */
bool Nrf24_Init(const Nrf24RadioConfig_t *radioConfig);
Nrf24TxResult_t Nrf24_SendPayload(
    const uint8_t payload[NRF24L01_FIXED_PAYLOAD_WIDTH]);

#ifdef __cplusplus
}
#endif

#endif
