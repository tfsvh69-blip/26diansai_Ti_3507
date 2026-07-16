#include "app_imu_uart_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "atk_ms6dsv.h"
#include "bsp_imu_port.h"
#include "bsp_uart.h"
#include "laser_ld14.h"

static TaskHandle_t s_imuUartTaskHandle = NULL;

#define APP_IMU_WHO_AM_I_REG           (0x0FU)

static void AppImuUartTask_SendUint(uint32_t value)
{
    char buf[10];
    uint8_t index = 0U;

    if (value == 0U) {
        BspUart0_SendByte((uint8_t)'0');
        return;
    }

    while ((value > 0U) && (index < sizeof(buf))) {
        buf[index++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (index > 0U) {
        BspUart0_SendByte((uint8_t)buf[--index]);
    }
}

/* 输出带符号十进制整数，用于加速度(mg)/角速度(mdps)等可能超出 int16 的量。 */
static void AppImuUartTask_SendInt32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        BspUart0_SendByte((uint8_t)'-');
        /* 先转 int64 再取负，避免 INT32_MIN 取负溢出。 */
        magnitude = (uint32_t)(-(int64_t)value);
    } else {
        magnitude = (uint32_t)value;
    }

    AppImuUartTask_SendUint(magnitude);
}

static void AppImuUartTask_SendHexByte(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    BspUart0_SendString("0x");
    BspUart0_SendByte((uint8_t)hex[(value >> 4) & 0x0FU]);
    BspUart0_SendByte((uint8_t)hex[value & 0x0FU]);
}

static void AppImuUartTask_SendHex32(uint32_t value)
{
    int8_t shift;

    BspUart0_SendString("0x");
    for (shift = 28; shift >= 0; shift -= 4) {
        uint8_t nibble = (uint8_t)((value >> shift) & 0x0FU);
        BspUart0_SendByte((uint8_t)((nibble < 10U) ? ('0' + nibble) : ('A' + nibble - 10U)));
    }
}

static void AppImuUartTask_SendCentideg(int16_t centideg)
{
    uint32_t value;
    uint32_t integer;
    uint32_t fraction;

    if (centideg < 0) {
        BspUart0_SendByte((uint8_t)'-');
        value = (uint32_t)(-((int32_t)centideg));
    } else {
        value = (uint32_t)centideg;
    }

    integer = value / 100U;
    fraction = value % 100U;

    AppImuUartTask_SendUint(integer);
    BspUart0_SendByte((uint8_t)'.');
    BspUart0_SendByte((uint8_t)('0' + (fraction / 10U)));
    BspUart0_SendByte((uint8_t)('0' + (fraction % 10U)));
}

static void AppImuUartTask_SendStatus(AtkMs6dsvStatus_t status)
{
    uint8_t id;
    bool idValid;
    const char *step;

    idValid = AtkMs6dsv_GetLastInitId(&id);
    step = AtkMs6dsv_GetLastInitStep();
    BspUart0_Lock();
    BspUart0_SendString("IMU INIT FAIL:");
    AppImuUartTask_SendUint((uint32_t)status);
    BspUart0_SendString(" STEP=");
    BspUart0_SendString(step);
    if (idValid) {
        BspUart0_SendString(" LAST_ID=");
        AppImuUartTask_SendHexByte(id);
    }
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

static void AppImuUartTask_SendWhoAmIProbe(void)
{
    uint8_t id6a = 0U;
    uint8_t id6b = 0U;
    bool ok6a;
    bool ok6b;

    /*
     * SA0 接地时理论地址为 0x6A，同时探测 0x6B 可快速判断
     * SA0 实际电平、接线和 ACK 是否符合预期。
     */
    ok6a = AtkMs6dsv_ReadWhoAmI(0x6AU, &id6a);
    ok6b = AtkMs6dsv_ReadWhoAmI(0x6BU, &id6b);

    BspUart0_Lock();
    BspUart0_SendString("IMU WHOAMI 0x6A=");
    if (ok6a) {
        AppImuUartTask_SendHexByte(id6a);
    } else {
        BspUart0_SendString("ERR");
    }

    BspUart0_SendString(" 0x6B=");
    if (ok6b) {
        AppImuUartTask_SendHexByte(id6b);
    } else {
        BspUart0_SendString("ERR");
    }
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

/* 在已加锁区域内输出 " SCL=x SDA=x STAT=0x...\r\n"，调用方持锁。 */
static void AppImuUartTask_SendBusStatus(bool sclHigh, bool sdaHigh, uint32_t status)
{
    BspUart0_SendString(" SCL=");
    BspUart0_SendByte(sclHigh ? (uint8_t)'1' : (uint8_t)'0');
    BspUart0_SendString(" SDA=");
    BspUart0_SendByte(sdaHigh ? (uint8_t)'1' : (uint8_t)'0');
    BspUart0_SendString(" STAT=");
    AppImuUartTask_SendHex32(status);
    BspUart0_SendString("\r\n");
}

static void AppImuUartTask_SendBusState(void)
{
    bool sclHigh;
    bool sdaHigh;
    uint32_t status;

    /*
     * 先释放 I2C 控制器的异常传输状态，再读物理线状态。
     * 若恢复后 SCL/SDA 仍为低，基本就是硬件总线被拉低。
     */
    BspImuPort_RecoverBus();
    BspImuPort_GetBusState(&sclHigh, &sdaHigh, &status);

    BspUart0_Lock();
    BspUart0_SendString("IMU BUS");
    AppImuUartTask_SendBusStatus(sclHigh, sdaHigh, status);
    BspUart0_Unlock();
}

static void AppImuUartTask_SendI2cScan(void)
{
    bool sclHigh;
    bool sdaHigh;
    uint32_t status;
    bool found6a = false;
    bool found6b = false;

    /*
     * 只探测 MS6DSV 可能出现的两个地址。
     * 全地址扫描在异常状态下容易被 SDA/ACK 残留状态误导，调试阶段先避免刷出假地址。
     */
    BspImuPort_GetBusState(&sclHigh, &sdaHigh, &status);
    if ((!sclHigh) || (!sdaHigh)) {
        BspUart0_Lock();
        BspUart0_SendString("IMU SCAN: bus stuck");
        AppImuUartTask_SendBusStatus(sclHigh, sdaHigh, status);
        BspUart0_Unlock();
        return;
    }

    found6a = BspImuPort_ProbeAddress(0x6AU, APP_IMU_WHO_AM_I_REG);
    found6b = BspImuPort_ProbeAddress(0x6BU, APP_IMU_WHO_AM_I_REG);

    BspUart0_Lock();
    BspUart0_SendString("IMU SCAN:");
    if (found6a) {
        BspUart0_SendString(" 0x6A");
    }

    if (found6b) {
        BspUart0_SendString(" 0x6B");
    }

    if ((!found6a) && (!found6b)) {
        BspUart0_SendString(" none");
    }
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

/* 在已加锁区域内追加激光测距1的读数：" D1=<mm>mm"，无有效帧时输出 " D1=---"。调用方持锁。 */
static void AppImuUartTask_SendLaser(void)
{
    LaserLd14Data_t laser;

    BspUart0_SendString(" D1=");
    if (LaserLd14_GetLatest(&laser)) {
        AppImuUartTask_SendUint((uint32_t)laser.distanceMm);
        BspUart0_SendString("mm");
    } else {
        /* 尚未收到有效帧：可能激光未上电/未接/波特率不符，输出占位便于排查。 */
        BspUart0_SendString("---");
    }
}

static void AppImuUartTask_SendImu(const AtkMs6dsvEuler_t *euler, const AtkMs6dsvImuRaw_t *raw)
{
    /*
     * 一整行输出融合欧拉角 + 三轴加速度 + 三轴角速度 + 激光测距1。
     * 角度单位为度(保留 2 位小数)；AX/AY/AZ 单位 mg；GX/GY/GZ 单位 mdps；
     * D1 为激光测距1(mm)；INT 为 PA16 电平。
     * 整行较长，由调用方按 APP_IMU_PRINT_DIVIDER 节流打印频率（已降到便于阅读的低速率）。
     */
    BspUart0_Lock();
    BspUart0_SendString("IMU R=");
    AppImuUartTask_SendCentideg(euler->rollCentideg);
    BspUart0_SendString(" P=");
    AppImuUartTask_SendCentideg(euler->pitchCentideg);
    BspUart0_SendString(" Y=");
    AppImuUartTask_SendCentideg(euler->yawCentideg);
    BspUart0_SendString(" AX=");
    AppImuUartTask_SendInt32(raw->accMg[0]);
    BspUart0_SendString(" AY=");
    AppImuUartTask_SendInt32(raw->accMg[1]);
    BspUart0_SendString(" AZ=");
    AppImuUartTask_SendInt32(raw->accMg[2]);
    BspUart0_SendString(" GX=");
    AppImuUartTask_SendInt32(raw->gyrMdps[0]);
    BspUart0_SendString(" GY=");
    AppImuUartTask_SendInt32(raw->gyrMdps[1]);
    BspUart0_SendString(" GZ=");
    AppImuUartTask_SendInt32(raw->gyrMdps[2]);
    BspUart0_SendString(" FIFO=");
    AppImuUartTask_SendUint((uint32_t)euler->fifoLevel);
    /* 与陀螺仪数据同一行输出激光测距1，实现"一起发"。 */
    AppImuUartTask_SendLaser();
    BspUart0_SendString(" INT=");
    BspUart0_SendByte(euler->intLevel ? (uint8_t)'1' : (uint8_t)'0');
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

static void AppImuUartTask_RunPinTest(void)
{
    bool phase = false;

    /*
     * 该模式只用于硬件排查：不访问 IMU，只用开漏模拟方式翻转并读回 PB2/PB3。
     * high 表示释放为输入由上拉拉高，low 表示 MCU 主动拉低。
     */
    BspImuPort_EnterPinTestMode();
    BspUart0_Lock();
    BspUart0_SendString("IMU PINTEST PB2=SCL PB3=SDA, SET then READ GPIO DIN\r\n");
    BspUart0_Unlock();

    for (;;) {
        bool sclHigh = phase;
        bool sdaHigh = !phase;
        bool sclRead;
        bool sdaRead;

        BspImuPort_SetPinTestLevel(sclHigh, sdaHigh);
        BspImuPort_ReadPinTestLevel(&sclRead, &sdaRead);

        BspUart0_Lock();
        BspUart0_SendString("IMU PINTEST SET SCL=");
        BspUart0_SendByte(sclHigh ? (uint8_t)'1' : (uint8_t)'0');
        BspUart0_SendString(" SDA=");
        BspUart0_SendByte(sdaHigh ? (uint8_t)'1' : (uint8_t)'0');
        BspUart0_SendString(" READ SCL=");
        BspUart0_SendByte(sclRead ? (uint8_t)'1' : (uint8_t)'0');
        BspUart0_SendString(" SDA=");
        BspUart0_SendByte(sdaRead ? (uint8_t)'1' : (uint8_t)'0');
        BspUart0_SendString("\r\n");
        BspUart0_Unlock();

        phase = !phase;
        vTaskDelay(APP_IMU_I2C_PIN_TEST_PERIOD_TICKS);
    }
}

static void AppImuUartTask_Entry(void *argument)
{
    (void)argument;

#if (APP_IMU_I2C_PIN_TEST_ENABLE != 0U)
    AppImuUartTask_RunPinTest();
#else
    TickType_t lastWakeTime;
    AtkMs6dsvStatus_t initStatus;
    AtkMs6dsvEuler_t euler = {0};
    AtkMs6dsvImuRaw_t imuRaw = {0};
    uint32_t initFailCount = 0U;
    uint32_t printDivider = 0U;

    BspUart0_Lock();
    BspUart0_SendString("IMU UART 100Hz START, SWI2C addr=0x6A\r\n");
    BspUart0_Unlock();

    initStatus = AtkMs6dsv_Init();
    while (initStatus != ATK_MS6DSV_OK) {
        AppImuUartTask_SendStatus(initStatus);
        AppImuUartTask_SendWhoAmIProbe();
        AppImuUartTask_SendBusState();
        if ((initFailCount % 5U) == 0U) {
            AppImuUartTask_SendI2cScan();
        }
        initFailCount++;
        vTaskDelay(pdMS_TO_TICKS(1000U));

        /*
         * 传感器上电、总线上拉或接线调整后可能恢复，
         * 失败状态下每秒重试一次，避免必须手动复位开发板。
         */
        initStatus = AtkMs6dsv_Init();
    }

    BspUart0_Lock();
    BspUart0_SendString("IMU INIT OK\r\n");
    /* 打印一次单位说明，之后每行不再重复单位以缩短行长。 */
    BspUart0_SendString("IMU FORMAT: R/P/Y=deg AX/AY/AZ=mg GX/GY/GZ=mdps D1=mm(激光测距1)\r\n");
    BspUart0_Unlock();
    lastWakeTime = xTaskGetTickCount();

    for (;;) {
        /*
         * 融合欧拉角走 FIFO，每周期(100Hz)读取以持续排空 FIFO 并保持角度新鲜，
         * 无新数据时保留上一次角度值。
         */
        (void)AtkMs6dsv_ReadEuler(&euler);

        /*
         * 打印节流：整行输出降到 100Hz/APP_IMU_PRINT_DIVIDER=20Hz，避免长串口行挂起调度器。
         *
         * 方案A（CPU 优化）：加速度/角速度当前只用于串口显示，尚无 100Hz 消费者。
         * 软件 I2C 一次 6 字节寄存器读约 1ms 且 CPU 全程忙等，两组共 ~2ms/周期。
         * 因此把寄存器直读移到「要打印的那一拍」才做，读取开销从 ~2ms/10ms(20%)
         * 降到 ~0.4ms/10ms(4%)。将来若有算法需要 100Hz 原始数据，
         * 再把 ReadImuRaw 移回每周期，并优先恢复硬件 I2C(TPR=0=400kHz) 提速。
         */
        if (++printDivider >= APP_IMU_PRINT_DIVIDER) {
            printDivider = 0U;
            (void)AtkMs6dsv_ReadImuRaw(&imuRaw);
            AppImuUartTask_SendImu(&euler, &imuRaw);
        }

        vTaskDelayUntil(&lastWakeTime, APP_IMU_UART_PERIOD_TICKS);
    }
#endif
}

void AppImuUartTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppImuUartTask_Entry,
                      "IMU100Hz",
                      APP_IMU_UART_TASK_STACK_WORDS,
                      NULL,
                      APP_IMU_UART_TASK_PRIORITY,
                      &s_imuUartTaskHandle);
    configASSERT(ret == pdPASS);
}
