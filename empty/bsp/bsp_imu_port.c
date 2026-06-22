#include "bsp_imu_port.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define BSP_IMU_I2C_TIMEOUT_LOOPS       (120000UL)
#define BSP_IMU_I2C_RECOVERY_PULSES     (18U)
#define BSP_IMU_I2C_ACK_TIMEOUT_LOOPS   (250U)
#define BSP_IMU_I2C_DELAY_CYCLES        (CPUCLK_FREQ / 200000UL)

/* PB2/PB3 已切换到 GPIO 软件 I2C 模式后无需重复初始化，用此标志跳过。 */
static bool s_softI2cConfigured = false;

static void BspImuPort_Delay(void)
{
    DL_Common_delayCycles(BSP_IMU_I2C_DELAY_CYCLES);
}

static void BspImuPort_UseSoftI2cPins(void)
{
    /*
     * 首次调用时关闭 I2C1 硬件控制器并将 PB2/PB3 切换为 GPIO 模拟 I2C。
     * 此后 I2C1 不再使用，不需要每次事务都重复初始化。
     */
    if (s_softI2cConfigured) {
        return;
    }

    DL_I2C_disableController(IMU_I2C_1_INST);
    DL_I2C_resetControllerTransfer(IMU_I2C_1_INST);
    DL_I2C_flushControllerTXFIFO(IMU_I2C_1_INST);
    DL_I2C_flushControllerRXFIFO(IMU_I2C_1_INST);
    DL_I2C_disableControllerReadOnTXEmpty(IMU_I2C_1_INST);
    DL_I2C_disableControllerACK(IMU_I2C_1_INST);

    DL_GPIO_initDigitalInputFeatures(IMU_I2C_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IMU_I2C_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setPins(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
    DL_GPIO_setPins(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);
    DL_GPIO_disableOutput(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
    DL_GPIO_disableOutput(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);

    s_softI2cConfigured = true;
}

static void BspImuPort_SetScl(bool high)
{
    if (high) {
        DL_GPIO_setPins(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
        DL_GPIO_disableOutput(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
    } else {
        DL_GPIO_clearPins(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
        DL_GPIO_enableOutput(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
    }
}

static void BspImuPort_SetSda(bool high)
{
    if (high) {
        DL_GPIO_setPins(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);
        DL_GPIO_disableOutput(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);
    } else {
        DL_GPIO_clearPins(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);
        DL_GPIO_enableOutput(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);
    }
}

static bool BspImuPort_ReadScl(void)
{
    return (DL_GPIO_readPins(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN) != 0U);
}

static bool BspImuPort_ReadSda(void)
{
    return (DL_GPIO_readPins(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN) != 0U);
}

static bool BspImuPort_WaitSclHigh(void)
{
    uint32_t timeout = BSP_IMU_I2C_TIMEOUT_LOOPS;

    /*
     * 正常情况下 SCL 由主机释放后会立刻被上拉拉高。
     * 若外部短路或从机长时间 clock stretching，这里限时退出，避免任务卡死。
     */
    while (!BspImuPort_ReadScl()) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

static bool BspImuPort_IsBusReleased(void)
{
    return (BspImuPort_ReadScl() && BspImuPort_ReadSda());
}

static bool BspImuPort_Start(void)
{
    BspImuPort_SetSda(true);
    BspImuPort_SetScl(true);
    if (!BspImuPort_WaitSclHigh()) {
        return false;
    }
    BspImuPort_Delay();
    BspImuPort_SetSda(false);
    BspImuPort_Delay();
    BspImuPort_SetScl(false);
    BspImuPort_Delay();
    return true;
}

static bool BspImuPort_Stop(void)
{
    BspImuPort_SetSda(false);
    BspImuPort_Delay();
    BspImuPort_SetScl(true);
    if (!BspImuPort_WaitSclHigh()) {
        return false;
    }
    BspImuPort_Delay();
    BspImuPort_SetSda(true);
    BspImuPort_Delay();
    return true;
}

static bool BspImuPort_SendByte(uint8_t value)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        BspImuPort_SetSda(((value & 0x80U) != 0U));
        BspImuPort_Delay();
        BspImuPort_SetScl(true);
        if (!BspImuPort_WaitSclHigh()) {
            return false;
        }
        BspImuPort_Delay();
        BspImuPort_SetScl(false);
        value <<= 1;
        BspImuPort_Delay();
    }

    BspImuPort_SetSda(true);
    return true;
}

static bool BspImuPort_WaitAck(void)
{
    uint32_t timeout = BSP_IMU_I2C_ACK_TIMEOUT_LOOPS;
    bool ack;

    /*
     * 第 9 个时钟释放 SDA，由从机拉低表示 ACK。
     * 若未收到 ACK，调用方会生成 STOP，确保总线回到空闲。
     */
    BspImuPort_SetSda(true);
    BspImuPort_Delay();
    BspImuPort_SetScl(true);
    if (!BspImuPort_WaitSclHigh()) {
        return false;
    }
    BspImuPort_Delay();

    while (BspImuPort_ReadSda()) {
        if (timeout-- == 0U) {
            BspImuPort_SetScl(false);
            BspImuPort_Delay();
            return false;
        }
    }

    ack = !BspImuPort_ReadSda();
    BspImuPort_SetScl(false);
    BspImuPort_Delay();
    return ack;
}

static bool BspImuPort_ReadByte(uint8_t *value, bool ack)
{
    uint8_t bit;
    uint8_t data = 0U;

    BspImuPort_SetSda(true);
    for (bit = 0U; bit < 8U; bit++) {
        data <<= 1;
        BspImuPort_SetScl(true);
        if (!BspImuPort_WaitSclHigh()) {
            return false;
        }
        BspImuPort_Delay();
        if (BspImuPort_ReadSda()) {
            data++;
        }
        BspImuPort_SetScl(false);
        BspImuPort_Delay();
    }

    /*
     * 多字节读取时除最后一个字节外都 ACK；
     * 最后一个字节必须 NACK，再 STOP，否则 LSM6DSV16X 可能继续保持发送态并拉低 SDA。
     */
    BspImuPort_SetSda(!ack);
    BspImuPort_Delay();
    BspImuPort_SetScl(true);
    if (!BspImuPort_WaitSclHigh()) {
        return false;
    }
    BspImuPort_Delay();
    BspImuPort_SetScl(false);
    BspImuPort_SetSda(true);
    BspImuPort_Delay();

    *value = data;
    return true;
}

bool BspImuPort_WriteReg(uint8_t devAddr, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    bool ok = false;

    if ((len != 0U) && (data == NULL)) {
        return false;
    }

    BspImuPort_UseSoftI2cPins();
    if (!BspImuPort_IsBusReleased()) {
        BspImuPort_RecoverBus();
        if (!BspImuPort_IsBusReleased()) {
            return false;
        }
    }

    if (!BspImuPort_Start()) {
        return false;
    }

    do {
        if ((!BspImuPort_SendByte((uint8_t)((devAddr << 1) | 0U))) || (!BspImuPort_WaitAck())) {
            break;
        }

        if ((!BspImuPort_SendByte(reg)) || (!BspImuPort_WaitAck())) {
            break;
        }

        for (i = 0U; i < len; i++) {
            if ((!BspImuPort_SendByte(data[i])) || (!BspImuPort_WaitAck())) {
                break;
            }
        }

        ok = (i == len);
    } while (false);

    (void)BspImuPort_Stop();
    return ok;
}

bool BspImuPort_ReadReg(uint8_t devAddr, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint16_t i;
    bool ok = false;

    if ((data == NULL) || (len == 0U)) {
        return false;
    }

    BspImuPort_UseSoftI2cPins();
    if (!BspImuPort_IsBusReleased()) {
        BspImuPort_RecoverBus();
        if (!BspImuPort_IsBusReleased()) {
            return false;
        }
    }

    if (!BspImuPort_Start()) {
        return false;
    }

    do {
        if ((!BspImuPort_SendByte((uint8_t)((devAddr << 1) | 0U))) || (!BspImuPort_WaitAck())) {
            break;
        }

        if ((!BspImuPort_SendByte(reg)) || (!BspImuPort_WaitAck())) {
            break;
        }

        if (!BspImuPort_Start()) {
            break;
        }

        if ((!BspImuPort_SendByte((uint8_t)((devAddr << 1) | 1U))) || (!BspImuPort_WaitAck())) {
            break;
        }

        for (i = 0U; i < len; i++) {
            if (!BspImuPort_ReadByte(&data[i], (i + 1U) < len)) {
                break;
            }
        }

        ok = (i == len);
    } while (false);

    (void)BspImuPort_Stop();
    return ok;
}

bool BspImuPort_ProbeAddress(uint8_t devAddr, uint8_t probeReg)
{
    /*
     * 扫描时只写入一个寄存器地址，用 ACK 判断设备是否存在。
     * 避免裸读导致从机输出未知寄存器数据，减少 SDA 被拉低或总线状态被扰乱的概率。
     */
    if (!BspImuPort_WriteReg(devAddr, probeReg, NULL, 0U)) {
        BspImuPort_RecoverBus();
        return false;
    }

    return true;
}

bool BspImuPort_ReadIntLevel(void)
{
    return (DL_GPIO_readPins(IMU_INT_PORT, IMU_INT_PIN) != 0U);
}

void BspImuPort_RecoverBus(void)
{
    uint8_t i;

    /*
     * 若从机停在读事务中，SDA 可能持续为低并等待后续 SCL。
     * 这里用 GPIO 给 SCL 输出恢复脉冲，随后显式生成 STOP。
     */
    BspImuPort_UseSoftI2cPins();

    for (i = 0U; i < BSP_IMU_I2C_RECOVERY_PULSES; i++) {
        if (BspImuPort_ReadSda()) {
            break;
        }

        BspImuPort_SetScl(false);
        BspImuPort_Delay();
        BspImuPort_SetScl(true);
        (void)BspImuPort_WaitSclHigh();
        BspImuPort_Delay();
    }

    if (BspImuPort_ReadSda()) {
        (void)BspImuPort_Stop();
    }
}

void BspImuPort_GetBusState(bool *sclHigh, bool *sdaHigh, uint32_t *status)
{
    /*
     * 软件 I2C 模式下直接读取 GPIO DIN，能反映 PB2/PB3 物理电平。
     * status 仍返回 I2C1 控制器状态，主要用于保留旧日志格式。
     */
    BspImuPort_UseSoftI2cPins();

    if (sclHigh != NULL) {
        *sclHigh = BspImuPort_ReadScl();
    }

    if (sdaHigh != NULL) {
        *sdaHigh = BspImuPort_ReadSda();
    }

    if (status != NULL) {
        *status = DL_I2C_getControllerStatus(IMU_I2C_1_INST);
    }
}

void BspImuPort_EnterPinTestMode(void)
{
    /*
     * 引脚测试模式用于确认 PB2/PB3 是否真正连接到 B02/B03。
     * 关闭 I2C 外设后，把两根线切成 GPIO 输入上拉，默认释放为高电平。
     */
    BspImuPort_UseSoftI2cPins();
}

void BspImuPort_SetPinTestLevel(bool sclHigh, bool sdaHigh)
{
    /*
     * 用开漏模拟方式输出测试波形：
     * high 表示释放为输入，由上拉拉高；low 表示开启输出并拉低。
     */
    BspImuPort_SetScl(sclHigh);
    BspImuPort_SetSda(sdaHigh);
}

void BspImuPort_ReadPinTestLevel(bool *sclHigh, bool *sdaHigh)
{
    /*
     * 直接读取 GPIO DIN 寄存器，用于确认 MCU 看到的 PB2/PB3 实际电平。
     * 若 SET 为释放高但 READ 为 0，通常表示外部器件、短路或接线把该线拉低。
     */
    if (sclHigh != NULL) {
        *sclHigh = BspImuPort_ReadScl();
    }

    if (sdaHigh != NULL) {
        *sdaHigh = BspImuPort_ReadSda();
    }
}
