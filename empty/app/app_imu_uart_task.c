#include "app_imu_uart_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "atk_ms6dsv.h"
#include "bsp_imu_port.h"
#include "bsp_uart.h"

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

static void AppImuUartTask_SendEuler(const AtkMs6dsvEuler_t *euler)
{
    /*
     * 100Hz 输出尽量保持短行，避免 115200 波特率下串口发送反过来拖慢任务。
     * 角度单位为度，保留 2 位小数；INT 为 PA16 当前电平。
     */
    BspUart0_Lock();
    BspUart0_SendString("IMU R=");
    AppImuUartTask_SendCentideg(euler->rollCentideg);
    BspUart0_SendString(" P=");
    AppImuUartTask_SendCentideg(euler->pitchCentideg);
    BspUart0_SendString(" Y=");
    AppImuUartTask_SendCentideg(euler->yawCentideg);
    BspUart0_SendString(" FIFO=");
    AppImuUartTask_SendUint((uint32_t)euler->fifoLevel);
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
    uint32_t noDataCount = 0U;
    uint32_t initFailCount = 0U;

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
    BspUart0_Unlock();
    lastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (AtkMs6dsv_ReadEuler(&euler)) {
            noDataCount = 0U;
            AppImuUartTask_SendEuler(&euler);
        } else {
            /*
             * 初始化后短时间没有 FIFO 数据是正常现象。
             * 只按 1s 节流提示，避免故障时刷屏影响观察。
             */
            noDataCount++;
            if (noDataCount >= 100U) {
                noDataCount = 0U;
                BspUart0_SendString("IMU WAIT DATA\r\n");
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
