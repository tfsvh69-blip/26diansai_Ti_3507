#include "app_imu_uart_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "atk_ms6dsv.h"
#include "bsp_imu_port.h"
#include "bsp_uart.h"
#if (APP_FEATURE_LASER != 0U)
#include "laser_ld14.h"
#endif

/*
 * IMU + 激光测距1 输出任务（本工程 CPU 最大消耗，读写全走软件 I2C 忙等）。
 *
 * 【主循环 AppImuUartTask_Entry】
 *   1) 初始化 IMU，失败则每秒重试并打印详细排查信息（WHOAMI/总线/扫描）。
 *   2) 成功后进入 100Hz 循环：每 10ms 读一次融合欧拉角(走 FIFO/SFLP，持续排空 FIFO)。
 *   3) 打印节流：每 APP_IMU_PRINT_DIVIDER(=20) 拍才发一整行 → 5Hz，避免刷屏太快/占串口。
 *      只有在"要打印那拍"才额外读一次加速度/角速度寄存器(方案A，省软件 I2C 开销)，
 *      并把激光测距1(D1)追加到同一行，实现"和陀螺仪一起发"。
 *
 * 【文件其余部分】全是把数字格式化到 UART0 的小工具(SendUint/SendInt32/SendCentideg/
 *   SendHex... )和排查用打印(SendStatus/SendWhoAmIProbe/SendBusState/SendI2cScan)，
 *   以及一个只在 APP_IMU_I2C_PIN_TEST_ENABLE=1 时启用的 PB2/PB3 引脚物理测试模式。
 *   激光距离本身在 UART2 RX 中断里解析，这里只通过 LaserLd14_GetLatest() 取快照。
 *
 * 【为什么它最占 CPU】软件 I2C 每位靠 delayCycles 空转(~5µs/延时)，一次 6 字节读≈1ms，
 *   100Hz 读融合角≈15~25% CPU。要提速优先换回硬件 I2C，详见 docs/PROJECT_CONTEXT.md。
 */

static TaskHandle_t s_imuUartTaskHandle = NULL;

#define APP_IMU_WHO_AM_I_REG           (0x0FU)

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

    BspUart0_SendUint(magnitude);
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

    BspUart0_SendUint(integer);
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
    BspUart0_SendUint((uint32_t)status);
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

/*
 * 在已加锁区域内追加激光测距1的读数：" D1=<mm>mm"，无有效帧时输出 " D1=---"。调用方持锁。
 * APP_FEATURE_LASER=0 时函数体为空（不打印 D1 字段），调用点无需再套 #if。
 */
static void AppImuUartTask_SendLaser(void)
{
#if (APP_FEATURE_LASER != 0U)
    LaserLd14Data_t laser;

    BspUart0_SendString(" D1=");
    if (LaserLd14_GetLatest(&laser)) {
        BspUart0_SendUint((uint32_t)laser.distanceMm);
        BspUart0_SendString("mm");
    } else {
        /* 尚未收到有效帧：可能激光未上电/未接/波特率不符，输出占位便于排查。 */
        BspUart0_SendString("---");
    }
#endif
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
    BspUart0_SendUint((uint32_t)euler->fifoLevel);
    /* 与陀螺仪数据同一行输出激光测距1，实现"一起发"（LASER 关闭时该调用为空）。 */
    AppImuUartTask_SendLaser();
    BspUart0_SendString(" INT=");
    BspUart0_SendByte(euler->intLevel ? (uint8_t)'1' : (uint8_t)'0');
    BspUart0_SendString("\r\n");
    BspUart0_Unlock();
}

/*
 * IMU 暂不可用（初始化失败/后台重试中）时的精简遥测行：
 * 只报 IMU 状态占位，仍照常输出激光 D1，保证激光遥测不被 IMU 故障拖累。
 */
static void AppImuUartTask_SendReportNoImu(void)
{
    BspUart0_Lock();
    BspUart0_SendString("IMU ---(retry)");
    AppImuUartTask_SendLaser();
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

/* IMU 初始化失败累计次数（用于每 5 次失败刷一次 I2C 扫描）。 */
static uint32_t s_imuInitFailCount = 0U;

/*
 * 试一次 IMU 初始化：成功打印 "IMU INIT OK" 返回 true；失败打印诊断返回 false。
 * fullDiag=false（首次尝试）只打 SendStatus；true（循环重试）加打 WHOAMI/总线/扫描。
 * 合并了原先首次尝试与循环重试各写一遍的 init+打印逻辑。
 */
static bool AppImuUartTask_TryInit(bool fullDiag)
{
    AtkMs6dsvStatus_t status = AtkMs6dsv_Init();

    if (status == ATK_MS6DSV_OK) {
        BspUart0_Lock();
        BspUart0_SendString("IMU INIT OK\r\n");
        BspUart0_Unlock();
        return true;
    }

    AppImuUartTask_SendStatus(status);
    if (fullDiag) {
        AppImuUartTask_SendWhoAmIProbe();
        AppImuUartTask_SendBusState();
        if ((s_imuInitFailCount % 5U) == 0U) {
            AppImuUartTask_SendI2cScan();
        }
        s_imuInitFailCount++;
    }
    return false;
}

static void AppImuUartTask_Entry(void *argument)
{
    (void)argument;

#if (APP_IMU_I2C_PIN_TEST_ENABLE != 0U)
    AppImuUartTask_RunPinTest();
#else
    TickType_t lastWakeTime;
    AtkMs6dsvEuler_t euler = {0};
    AtkMs6dsvImuRaw_t imuRaw = {0};
    uint32_t printDivider = 0U;
    uint32_t reinitDivider = 0U;
    bool imuOk;

    BspUart0_Lock();
    BspUart0_SendString("IMU UART 100Hz START, SWI2C addr=0x6A\r\n");
    /* 打印一次单位说明，之后每行不再重复单位以缩短行长。 */
    BspUart0_SendString("IMU FORMAT: R/P/Y=deg AX/AY/AZ=mg GX/GY/GZ=mdps D1=mm(激光测距1)\r\n");
    BspUart0_Unlock();

    /*
     * 【非阻塞初始化·解耦激光】首次尝试初始化 IMU（简诊断）；失败不再死等，
     * 转入主循环里每秒重试一次（全诊断）。主循环照常 100Hz 运行、按节流打印，
     * 即使 IMU 缺失/损坏，激光 D1 等遥测也照常输出，不被 IMU 故障拖累。
     */
    imuOk = AppImuUartTask_TryInit(false);

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        if (imuOk) {
            /*
             * 融合欧拉角走 FIFO，每周期(100Hz)读取以持续排空 FIFO 并保持角度新鲜，
             * 无新数据时保留上一次角度值。
             */
            (void)AtkMs6dsv_ReadEuler(&euler);
        } else if (++reinitDivider >= APP_IMU_REINIT_DIVIDER) {
            /* 非阻塞重试：约每秒尝试重新初始化并打印全诊断，期间不阻塞主循环。 */
            reinitDivider = 0U;
            imuOk = AppImuUartTask_TryInit(true);
        }

        /*
         * 打印节流：整行输出降到 100Hz/APP_IMU_PRINT_DIVIDER=5Hz。
         * 方案A（CPU 优化）：加速度/角速度只在「要打印的那一拍」才寄存器直读，省软件 I2C 忙等。
         * IMU 不可用时改发精简行（仍含激光 D1）。
         */
        if (++printDivider >= APP_IMU_PRINT_DIVIDER) {
            printDivider = 0U;
            if (imuOk) {
                (void)AtkMs6dsv_ReadImuRaw(&imuRaw);
                AppImuUartTask_SendImu(&euler, &imuRaw);
            } else {
                AppImuUartTask_SendReportNoImu();
            }
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
