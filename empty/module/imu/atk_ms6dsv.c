#include "atk_ms6dsv.h"

#include <math.h>
#include <stddef.h>

#include "Delay.h"
#include "bsp_imu_port.h"
#include "lsm6dsv16x_reg.h"
#include "ti_msp_dl_config.h"

#define ATK_MS6DSV_USE_BOOT_RESET       (0U)
#define ATK_MS6DSV_RESET_TIMEOUT_MS     (200U)
#define ATK_MS6DSV_BOOT_WAIT_MS         (20U)
#define ATK_MS6DSV_FIFO_READ_LIMIT      (32U)
#define ATK_MS6DSV_RAD_TO_DEG           (57.2957795f)

static int32_t AtkMs6dsv_WriteReg(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)handle;

    return BspImuPort_WriteReg(IMU_MS6DSV_I2C_ADDR_7BIT, reg, buf, len) ? 0 : -1;
}

static int32_t AtkMs6dsv_ReadReg(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)handle;

    return BspImuPort_ReadReg(IMU_MS6DSV_I2C_ADDR_7BIT, reg, buf, len) ? 0 : -1;
}

static void AtkMs6dsv_DelayMs(uint32_t millisec)
{
    Delay_ms(millisec);
}

static stmdev_ctx_t s_lsm6dsvCtx = {
    .write_reg = AtkMs6dsv_WriteReg,
    .read_reg = AtkMs6dsv_ReadReg,
    .mdelay = AtkMs6dsv_DelayMs,
    .handle = NULL,
};

static uint8_t s_lastInitId = 0U;
static bool s_lastInitIdValid = false;
static const char *s_lastInitStep = "NONE";

static uint32_t AtkMs6dsv_HalfBitsToFloatBits(uint16_t h)
{
    uint16_t hExp;
    uint16_t hSig;
    uint32_t fSgn;
    uint32_t fExp;
    uint32_t fSig;

    hExp = (uint16_t)(h & 0x7C00U);
    fSgn = ((uint32_t)h & 0x8000U) << 16;

    switch (hExp) {
        case 0x0000U:
            hSig = (uint16_t)(h & 0x03FFU);
            if (hSig == 0U) {
                return fSgn;
            }

            hSig <<= 1;
            while ((hSig & 0x0400U) == 0U) {
                hSig <<= 1;
                hExp++;
            }
            fExp = ((uint32_t)(127U - 15U - hExp)) << 23;
            fSig = ((uint32_t)(hSig & 0x03FFU)) << 13;
            return fSgn + fExp + fSig;

        case 0x7C00U:
            return fSgn + 0x7F800000U + (((uint32_t)(h & 0x03FFU)) << 13);

        default:
            return fSgn + (((uint32_t)(h & 0x7FFFU) + 0x1C000U) << 13);
    }
}

static float AtkMs6dsv_HalfToFloat(uint16_t h)
{
    union {
        float value;
        uint32_t bits;
    } conv;

    conv.bits = AtkMs6dsv_HalfBitsToFloatBits(h);
    return conv.value;
}

static void AtkMs6dsv_SflpToQuaternion(float quat[4], const uint16_t sflp[3])
{
    float sumsq = 0.0f;
    uint8_t i;

    /*
     * LSM6DSV16X 的 SFLP 游戏旋转向量只给出四元数前三项。
     * 第四项由单位四元数约束恢复，超过单位长度时先归一化保护。
     */
    quat[0] = AtkMs6dsv_HalfToFloat(sflp[0]);
    quat[1] = AtkMs6dsv_HalfToFloat(sflp[1]);
    quat[2] = AtkMs6dsv_HalfToFloat(sflp[2]);

    for (i = 0U; i < 3U; i++) {
        sumsq += quat[i] * quat[i];
    }

    if (sumsq > 1.0f) {
        float norm = sqrtf(sumsq);
        quat[0] /= norm;
        quat[1] /= norm;
        quat[2] /= norm;
        sumsq = 1.0f;
    }

    quat[3] = sqrtf(1.0f - sumsq);
}

static int16_t AtkMs6dsv_FloatDegToCentideg(float deg)
{
    float scaled = deg * 100.0f;

    if (scaled > 32767.0f) {
        return 32767;
    }

    if (scaled < -32768.0f) {
        return -32768;
    }

    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static bool AtkMs6dsv_ConfigureSensorFusion(void)
{
    lsm6dsv16x_fifo_sflp_raw_t fifoSflp = {0};

    /*
     * 配置参考正点原子 DEMO_SENSOR_FUSION_ANOTC。
     * 芯片 ODR 没有精确 100Hz 档位，这里用 120Hz 让 10ms 任务读取时不会欠采样。
     */
    s_lastInitStep = "XL_ODR";
    if (lsm6dsv16x_xl_data_rate_set(&s_lsm6dsvCtx, LSM6DSV16X_ODR_AT_120Hz) != 0) {
        return false;
    }
    s_lastInitStep = "GY_ODR";
    if (lsm6dsv16x_gy_data_rate_set(&s_lsm6dsvCtx, LSM6DSV16X_ODR_AT_120Hz) != 0) {
        return false;
    }
    s_lastInitStep = "XL_FS";
    if (lsm6dsv16x_xl_full_scale_set(&s_lsm6dsvCtx, LSM6DSV16X_2g) != 0) {
        return false;
    }
    s_lastInitStep = "GY_FS";
    if (lsm6dsv16x_gy_full_scale_set(&s_lsm6dsvCtx, LSM6DSV16X_125dps) != 0) {
        return false;
    }
    s_lastInitStep = "FIFO_WTM";
    if (lsm6dsv16x_fifo_watermark_set(&s_lsm6dsvCtx, 8U) != 0) {
        return false;
    }

    fifoSflp.game_rotation = PROPERTY_ENABLE;
    s_lastInitStep = "SFLP_BATCH";
    if (lsm6dsv16x_fifo_sflp_batch_set(&s_lsm6dsvCtx, fifoSflp) != 0) {
        return false;
    }
    s_lastInitStep = "FIFO_MODE";
    if (lsm6dsv16x_fifo_mode_set(&s_lsm6dsvCtx, LSM6DSV16X_STREAM_MODE) != 0) {
        return false;
    }
    s_lastInitStep = "SFLP_OFF";
    if (lsm6dsv16x_sflp_game_rotation_set(&s_lsm6dsvCtx, PROPERTY_DISABLE) != 0) {
        return false;
    }
    s_lastInitStep = "SFLP_ODR";
    if (lsm6dsv16x_sflp_data_rate_set(&s_lsm6dsvCtx, LSM6DSV16X_SFLP_120Hz) != 0) {
        return false;
    }
    s_lastInitStep = "SFLP_ON";
    if (lsm6dsv16x_sflp_game_rotation_set(&s_lsm6dsvCtx, PROPERTY_ENABLE) != 0) {
        return false;
    }

    s_lastInitStep = "DONE";
    return true;
}

AtkMs6dsvStatus_t AtkMs6dsv_Init(void)
{
    uint8_t id = 0U;
#if (ATK_MS6DSV_USE_BOOT_RESET != 0U)
    uint32_t timeout;
    lsm6dsv16x_reset_t resetState;
#endif

    s_lastInitIdValid = false;
    s_lastInitStep = "WHOAMI";
    if (lsm6dsv16x_device_id_get(&s_lsm6dsvCtx, &id) != 0) {
        return ATK_MS6DSV_ERROR_COMM;
    }

    /*
     * 记录初始化阶段实际读到的 WHO_AM_I。
     * 若返回 ID 错误，可通过串口直接看到读值，便于区分读错寄存器和芯片型号不一致。
     */
    s_lastInitId = id;
    s_lastInitIdValid = true;

    if (id != LSM6DSV16X_ID) {
        s_lastInitStep = "ID_CHECK";
        return ATK_MS6DSV_ERROR_ID;
    }

    /*
     * 当前模块在 RESTORE_CTRL_REGS/boot 阶段会把 SDA 拉低，导致后续通信失败。
     * 默认跳过芯片 boot reset，直接进入运行配置；如需对比，可临时打开宏。
     */
#if (ATK_MS6DSV_USE_BOOT_RESET != 0U)
    s_lastInitStep = "RESET_SET";
    if (lsm6dsv16x_reset_set(&s_lsm6dsvCtx, LSM6DSV16X_RESTORE_CTRL_REGS) != 0) {
        return ATK_MS6DSV_ERROR_COMM;
    }

    Delay_ms(ATK_MS6DSV_BOOT_WAIT_MS);
    BspImuPort_RecoverBus();

    timeout = ATK_MS6DSV_RESET_TIMEOUT_MS;
    do {
        Delay_ms(1U);
        s_lastInitStep = "RESET_GET";
        if (lsm6dsv16x_reset_get(&s_lsm6dsvCtx, &resetState) != 0) {
            return ATK_MS6DSV_ERROR_COMM;
        }

        if (timeout-- == 0U) {
            return ATK_MS6DSV_ERROR_CONFIG;
        }
    } while (resetState != LSM6DSV16X_READY);
#else
    s_lastInitStep = "RESET_SKIP";
    BspImuPort_RecoverBus();
    Delay_ms(2U);
#endif

    s_lastInitStep = "BDU";
    if (lsm6dsv16x_block_data_update_set(&s_lsm6dsvCtx, PROPERTY_ENABLE) != 0) {
        return ATK_MS6DSV_ERROR_COMM;
    }

    return AtkMs6dsv_ConfigureSensorFusion() ? ATK_MS6DSV_OK : ATK_MS6DSV_ERROR_CONFIG;
}

bool AtkMs6dsv_ReadWhoAmI(uint8_t devAddr, uint8_t *id)
{
    if (id == NULL) {
        return false;
    }

    /*
     * 调试用裸读 WHO_AM_I，允许同时探测 0x6A/0x6B，
     * 用于区分地址错误、ACK 失败和 ID 不匹配。
     */
    return BspImuPort_ReadReg(devAddr, LSM6DSV16X_WHO_AM_I, id, 1U);
}

bool AtkMs6dsv_GetLastInitId(uint8_t *id)
{
    if ((id == NULL) || (!s_lastInitIdValid)) {
        return false;
    }

    *id = s_lastInitId;
    return true;
}

const char *AtkMs6dsv_GetLastInitStep(void)
{
    return s_lastInitStep;
}

bool AtkMs6dsv_ReadEuler(AtkMs6dsvEuler_t *euler)
{
    lsm6dsv16x_fifo_status_t fifoStatus;
    lsm6dsv16x_fifo_out_raw_t fifoRaw;
    uint16_t readLimit;
    bool updated = false;

    if (euler == NULL) {
        return false;
    }

    if (lsm6dsv16x_fifo_status_get(&s_lsm6dsvCtx, &fifoStatus) != 0) {
        return false;
    }

    euler->fifoLevel = (fifoStatus.fifo_level > 255U) ? 255U : (uint8_t)fifoStatus.fifo_level;
    euler->intLevel = BspImuPort_ReadIntLevel();
    readLimit = fifoStatus.fifo_level;
    if (readLimit > ATK_MS6DSV_FIFO_READ_LIMIT) {
        readLimit = ATK_MS6DSV_FIFO_READ_LIMIT;
    }

    while (readLimit-- > 0U) {
        if (lsm6dsv16x_fifo_out_raw_get(&s_lsm6dsvCtx, &fifoRaw) != 0) {
            return false;
        }

        if (fifoRaw.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
            uint16_t sflp[3];
            float quat[4];
            float pitch;
            float roll;
            float yaw;

            sflp[0] = (uint16_t)((uint16_t)fifoRaw.data[0] | ((uint16_t)fifoRaw.data[1] << 8));
            sflp[1] = (uint16_t)((uint16_t)fifoRaw.data[2] | ((uint16_t)fifoRaw.data[3] << 8));
            sflp[2] = (uint16_t)((uint16_t)fifoRaw.data[4] | ((uint16_t)fifoRaw.data[5] << 8));

            AtkMs6dsv_SflpToQuaternion(quat, sflp);

            /*
             * 角度换算沿用正点原子例程的四元数顺序和符号约定。
             * 输出单位为 0.01 度，避免串口格式化依赖浮点 printf。
             */
            pitch = atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                           1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3])) * ATK_MS6DSV_RAD_TO_DEG;
            if (pitch > 0.0f) {
                pitch = 180.0f - pitch;
            } else {
                pitch = -pitch - 180.0f;
            }

            roll = -asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1])) * ATK_MS6DSV_RAD_TO_DEG;
            yaw = -atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                          1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2])) * ATK_MS6DSV_RAD_TO_DEG;

            euler->rollCentideg = AtkMs6dsv_FloatDegToCentideg(roll);
            euler->pitchCentideg = AtkMs6dsv_FloatDegToCentideg(pitch);
            euler->yawCentideg = AtkMs6dsv_FloatDegToCentideg(yaw);
            updated = true;
        }
    }

    return updated;
}
