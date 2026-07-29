/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * MSPM0G3507 手写板级 DriverLib 初始化配置。
 *
 * 当前工程已与 TI SysConfig 生成流程解耦，本文件由工程直接维护。
 * 函数名保留 SYSCFG_DL_*，是为了兼容已有 BSP 调用，不代表仍依赖 SysConfig 软件。
 */

#include "ti_msp_dl_config.h"

/* 运行时记录实际时钟参考源，供启动日志读取。 */
volatile bool g_sysClockUsingHFXT = false;

/*
 * 主用 80MHz 配置：以 40MHz 外部晶振 HFXT 为 SYSPLL 参考。
 * 40MHz 经 PDIV /2 = 20MHz 入 PLL；再经 QDIV *8 得到 160MHz VCO；
 * CLK0 分频 /2（rDivClk0=0 表示 /2）得到 80MHz MCLK。
 * CLK0 输出的 80MHz 由外部 40MHz 晶振锁定，比内部 RC 更准更稳。
 */
static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfigHFXT80MHz = {
    .rDivClk2x = 0,
    .rDivClk1 = 0,
    .rDivClk0 = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_HFCLK,
    .qDiv = 8,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_2,
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
};

/*
 * 备用 80MHz 配置：晶振起振失败时回退到内部 SYSOSC(32MHz)。
 * 32MHz 经 PDIV /2 = 16MHz，QDIV *10 得到 160MHz VCO，/2 得到 80MHz。
 * 保证晶振异常时整机仍能以 80MHz 运行、串口可用，不会卡死无输出。
 */
static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfigSYSOSC80MHz = {
    .rDivClk2x = 0,
    .rDivClk1 = 0,
    .rDivClk0 = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_SYSOSC,
    .qDiv = 10,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_2,
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
};

/*
 * 尝试启用 40MHz 外部晶振 HFXT，带超时轮询。
 * 关键：monitor 传 false，避免落入 DriverLib 内部“死等 HFCLK_GOOD”的无限循环——
 * 若晶振不起振（虚焊、负载电容不对等），那里会永久卡死、串口毫无输出、板子像变砖。
 * 这里改为自己带超时轮询，超时即关闭 HFXT 返回 false，交由上层回退内部 SYSOSC。
 */
static bool SYSCFG_DL_tryStartHFXT(void)
{
    uint32_t timeout = 1000000UL;

    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ, 8U, false);

    while (timeout-- > 0U) {
        if ((DL_SYSCTL_getClockStatus() & SYSCTL_CLKSTATUS_HFCLKGOOD_MASK) ==
            DL_SYSCTL_CLK_STATUS_HFCLK_GOOD) {
            return true;
        }
    }

    DL_SYSCTL_disableHFXT();
    return false;
}

void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_I2C_1_init();
    SYSCFG_DL_SYSTICK_init();
}

void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Main_reset(UART_1_INST);
    DL_UART_Main_reset(UART_2_INST);
    DL_I2C_reset(IMU_I2C_1_INST);
    DL_TimerG_reset(MOTOR_STEP_TIMER_INST);
    DL_TimerG_reset(MOTOR2_STEP_TIMER_INST);
    DL_TimerG_reset(MOTOR3_STEP_TIMER_INST);
    DL_TimerG_reset(MOTOR4_STEP_TIMER_INST);
    DL_Timer_reset(SERVO_TIMER_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_UART_Main_enablePower(UART_1_INST);
    DL_UART_Main_enablePower(UART_2_INST);
    DL_I2C_enablePower(IMU_I2C_1_INST);
    DL_TimerG_enablePower(MOTOR_STEP_TIMER_INST);
    DL_TimerG_enablePower(MOTOR2_STEP_TIMER_INST);
    DL_TimerG_enablePower(MOTOR3_STEP_TIMER_INST);
    DL_TimerG_enablePower(MOTOR4_STEP_TIMER_INST);
    DL_Timer_enablePower(SERVO_TIMER_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

void SYSCFG_DL_GPIO_init(void)
{
    /*
     * 40MHz 外部晶振 HFXT：PA5=HFXIN、PA6=HFXOUT 必须配为模拟功能，否则晶振无法起振。
     * 本函数在 SYSCFG_DL_SYSCTL_init() 之前调用，确保启用 HFXT 时引脚已就绪。
     */
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

    /* UART0 使用 PA10=TX、PA11=RX；两者为核心板慎用引脚，已按用户确认接入串口模块。 */
    DL_GPIO_initPeripheralOutputFunctionFeatures(GPIO_UART_0_IOMUX_TX,
        GPIO_UART_0_IOMUX_TX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_UART_0_IOMUX_RX,
        GPIO_UART_0_IOMUX_RX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /*
     * UART1 使用 PA17=TX、PB5=RX，接张大头 Emm42_V5.0 闭环驱动（v1.1 排针 H7）。
     * RX 端加内部上拉：驱动器未接/未上电时接收线保持高电平（空闲态），避免误触发起始位。
     * 注意接线说明 R1——多台驱动器的 TX 并联在 PB5 上（当前 3 台：摆杆/左轮/右轮，
     * 见 module/emm42/emm42_robot.h），需外部肖特基做线与后才可同时接。
     */
    DL_GPIO_initPeripheralOutputFunctionFeatures(GPIO_UART_1_IOMUX_TX,
        GPIO_UART_1_IOMUX_TX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_UART_1_IOMUX_RX,
        GPIO_UART_1_IOMUX_RX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /*
     * UART2 使用 PB15=TX、PB16=RX，接激光测距1（v1.1 排针 H21）。
     * RX 端加内部上拉，激光模块拔出/未上电时接收线保持高电平，避免误触发。
     */
    DL_GPIO_initPeripheralOutputFunctionFeatures(GPIO_UART_2_IOMUX_TX,
        GPIO_UART_2_IOMUX_TX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_UART_2_IOMUX_RX,
        GPIO_UART_2_IOMUX_RX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /* 三个指示灯 LED1(PB25)/LED2(PA7)/LED3(PB12)，v1.1 均高电平点亮，默认由 bsp_led.c 熄灭。 */
    DL_GPIO_initDigitalOutputFeatures(LED_LED1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(LED_LED2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(LED_LED3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /* 板载 OLED 软件 I2C：PB8=SDA、PB9=SCL，普通推挽输出，空闲拉高。 */
    DL_GPIO_initDigitalOutputFeatures(OLED_PIN_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(OLED_PIN_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /* 蜂鸣器 PA15：有源蜂鸣器高电平响，普通推挽输出，默认低电平（不响）。 */
    DL_GPIO_initDigitalOutputFeatures(BUZZER_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /*
     * 舵机 PWM（v1.1）：四路共用 TIMA0，50Hz。
     * SERVO1=PA8(TIMA0_CCP0)、SERVO2=PA9(CCP1)、SERVO3=PB4(CCP2)、SERVO4=PA12(CCP3)。
     * 当前先启用 SERVO1~4 四路复用输出，后续按需开关各通道。
     */
    DL_GPIO_initPeripheralOutputFunction(SERVO1_IOMUX, SERVO1_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(SERVO2_IOMUX, SERVO2_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(SERVO3_IOMUX, SERVO3_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(SERVO4_IOMUX, SERVO4_IOMUX_FUNC);

    /*
     * 步进电机 / TMC2209（v1.1）。
     * 电机1 STEP=PB10(TIMG0_CCP0 复用)，DIR=PB11；四路共用 ENN=PA13、MS1=PB0、MS2=PB1。
     * STEP 为定时器复用输出；DIR/ENN/MS1/MS2 为普通 GPIO 输出。
     */
    DL_GPIO_initPeripheralOutputFunction(MOTOR1_STEP_IOMUX, MOTOR1_STEP_IOMUX_FUNC);
    DL_GPIO_initDigitalOutputFeatures(MOTOR1_DIR_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /* 电机2/3/4 STEP 为各自定时器 CCP0 复用输出，DIR 为普通推挽 GPIO；逻辑方向由 bsp_motor 标定。 */
    DL_GPIO_initPeripheralOutputFunction(MOTOR2_STEP_IOMUX, MOTOR2_STEP_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(MOTOR3_STEP_IOMUX, MOTOR3_STEP_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(MOTOR4_STEP_IOMUX, MOTOR4_STEP_IOMUX_FUNC);
    DL_GPIO_initDigitalOutputFeatures(MOTOR2_DIR_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(MOTOR3_DIR_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(MOTOR4_DIR_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(TMC_ENN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(TMC_MS1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(TMC_MS2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /*
     * IMU PB2/PB3 默认仍保留 I2C1 复用配置，端口层访问前会切换为 GPIO 软件 I2C。
     * 当前硬件已外接上拉，代码仍保留 MCU 内部上拉用于调试。
     * PB2/PB3 不在核心板慎用引脚列表内；SDA 的 IOMUX 是 PINCM16，不是板口编号 U2.17。
     */
    DL_GPIO_initPeripheralInputFunctionFeatures(IMU_I2C_SCL_IOMUX,
        IMU_I2C_SCL_IOMUX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IMU_I2C_SCL_IOMUX);
    DL_GPIO_initPeripheralInputFunctionFeatures(IMU_I2C_SDA_IOMUX,
        IMU_I2C_SDA_IOMUX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IMU_I2C_SDA_IOMUX);

    /* IMU INT 当前先作为轮询输入使用，后续需要外部中断时再单独接入 ISR。 */
    DL_GPIO_initDigitalInputFeatures(IMU_INT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* 四个功能按键：数字输入、内部上拉、迟滞；按下为低电平，任务轮询读取。 */
    DL_GPIO_initDigitalInputFeatures(KEY1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /*
     * 8 路灰度循迹 LINE1~LINE8（PB17~PB24）：数字输入 + 内部上拉 + 迟滞。
     * 识别到线=低电平，故用上拉：模块未接/悬空时读到高(未识别)，是安全默认；
     * 模块信号为推挽输出，弱内部上拉不会与之冲突。任务低频轮询读取电平。
     */
    DL_GPIO_initDigitalInputFeatures(LINE1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE5_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE6_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE7_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(LINE8_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /*
     * 继电器 RELAY（PA24）：推挽输出 + 内部下拉。下拉配合上电清零，降低上电瞬间
     * 误吸合的概率（接线文档风险 R7）。驱动强度 LOW 足够（控制脚为高阻输入）。
     */
    DL_GPIO_initDigitalOutputFeatures(RELAY_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /*
     * NRF24L01+ GPIO 模拟 SPI：CE/SCK/MOSI 为推挽输出，MISO 为输入。
     * CSN(PA1) 只能开漏驱动，配置为输入上拉；拉低时由端口层临时开启输出。
     */
    DL_GPIO_initDigitalOutputFeatures(NRF24_CE_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(NRF24_SCK_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(NRF24_MOSI_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalInputFeatures(NRF24_MISO_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(NRF24_CSN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /*
     * 上电安全默认状态（v1.1）：
     * GPIOA：ENN(PA13) 拉高禁用四路驱动；NRF24 CSN 释放为高，其余输出为低。
     * GPIOB：LED1/LED3 灭；DIR 原始低电平；MS1=0/MS2=0；OLED SCL/SDA 空闲拉高。
     * BspMotor_Init 随后会写入实车标定后的逻辑方向与默认 1/32 细分。
     * STEP(PB10) 由定时器输出，这里只使能输出、不手动置位。
     */
    DL_GPIO_setPins(GPIOA, TMC_ENN_PIN);
    DL_GPIO_clearPins(GPIOA,
        BUZZER_PIN | LED_LED2_PIN | RELAY_PIN | NRF24_CE_PIN |
        NRF24_CSN_PIN | NRF24_SCK_PIN | NRF24_MOSI_PIN);
    DL_GPIO_enableOutput(GPIOA,
        TMC_ENN_PIN | BUZZER_PIN | LED_LED2_PIN | RELAY_PIN |
        NRF24_CE_PIN | NRF24_SCK_PIN | NRF24_MOSI_PIN);
    DL_GPIO_disableOutput(GPIOA, NRF24_CSN_PIN);

    DL_GPIO_setPins(GPIOB, OLED_PIN_SCL_PIN | OLED_PIN_SDA_PIN);
    DL_GPIO_clearPins(GPIOB,
        LED_LED1_PIN | LED_LED3_PIN | MOTOR1_DIR_PIN | TMC_MS1_PIN | TMC_MS2_PIN |
        MOTOR2_DIR_PIN | MOTOR3_DIR_PIN | MOTOR4_DIR_PIN);
    DL_GPIO_enableOutput(GPIOB,
        LED_LED1_PIN | LED_LED3_PIN | MOTOR1_STEP_PIN | MOTOR1_DIR_PIN |
        TMC_MS1_PIN | TMC_MS2_PIN | OLED_PIN_SCL_PIN | OLED_PIN_SDA_PIN |
        MOTOR2_STEP_PIN | MOTOR2_DIR_PIN | MOTOR3_STEP_PIN | MOTOR3_DIR_PIN |
        MOTOR4_STEP_PIN | MOTOR4_DIR_PIN);
}

void SYSCFG_DL_SYSCTL_init(void)
{
    /* 低功耗模式保持默认 SLEEP0。 */
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    /*
     * MCLK 切到 SYSPLL 前先配置 Flash 等待周期和 ULPCLK 分频。
     * UART0 使用 MFCLK 4MHz，避免 BUSCLK/ULPCLK 分频影响波特率。
     */
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);

    /*
     * 先尝试启用 40MHz 外部晶振作为 PLL 参考；起振失败则回退内部 SYSOSC。
     * 两种情况 PLL 输出都是 80MHz MCLK，CPU 主频不变，FreeRTOS 节拍无需改动。
     * UART/I2C/步进定时器仍走内部 MFCLK 4MHz，与本次时钟源切换解耦。
     */
    g_sysClockUsingHFXT = SYSCFG_DL_tryStartHFXT();

    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);

    if (g_sysClockUsingHFXT) {
        DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfigHFXT80MHz);
    } else {
        DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfigSYSOSC80MHz);
    }

    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
}

void SYSCFG_DL_SYSTICK_init(void)
{
    /* FreeRTOS 工程入口不要调用此函数，避免抢占 FreeRTOS SysTick。 */
    DL_SYSTICK_init(80);
    DL_SYSTICK_enable();
}

static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

void SYSCFG_DL_UART_0_init(void)
{
    /*
     * UART0：115200 8N1，使用 MFCLK/1。
     * 4MHz 下 115200 的 16x 过采样分频为 IBRD=2、FBRD=11。
     */
    DL_UART_Main_setClockConfig(UART_0_INST,
        (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);
    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_MAIN_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST,
        UART_0_IBRD_115200_MFCLK, UART_0_FBRD_115200_MFCLK);
    /*
     * 接收 FIFO 阈值设为 1 字节：配合 BspUart0_SetRxHandler 注册的 RX 中断按字节及时取走，
     * 避免上位机报文(约17帧/秒)在轮询节拍间隙把 4 字节 FIFO 溢出丢字节。
     * 中断与 NVIC 由 bsp_uart.c 的 BspUart0_SetRxHandler 在注册回调后再放开；
     * 若不注册回调，则仅是阈值配置、无中断触发，原轮询接口照常工作。
     */
    DL_UART_Main_setRXFIFOThreshold(UART_0_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enable(UART_0_INST);
}

static const DL_UART_Main_ClockConfig gUART_1ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_1Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

void SYSCFG_DL_UART_1_init(void)
{
    /*
     * UART1：张大头 Emm42_V5.0 闭环步进驱动，115200 8N1，使用 MFCLK/1。
     * 与 UART0 同为 4MHz + 16x 过采样，故分频值相同（IBRD=2、FBRD=11）。
     * RX FIFO 阈值 1 字节：驱动器回复帧很短（4~8 字节），按字节取走避免残留。
     * 中断与 NVIC 由 bsp_uart.c 的 BspUart1_Init 在注册回调后再放开。
     */
    DL_UART_Main_setClockConfig(UART_1_INST,
        (DL_UART_Main_ClockConfig *) &gUART_1ClockConfig);
    DL_UART_Main_init(UART_1_INST, (DL_UART_Main_Config *) &gUART_1Config);
    DL_UART_Main_setOversampling(UART_1_INST, DL_UART_MAIN_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST,
        UART_1_IBRD_115200_MFCLK, UART_1_FBRD_115200_MFCLK);
    DL_UART_Main_setRXFIFOThreshold(UART_1_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enable(UART_1_INST);
}

static const DL_UART_Main_ClockConfig gUART_2ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_2Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

void SYSCFG_DL_UART_2_init(void)
{
    /*
     * UART2：激光测距1，230400 8N1，使用 MFCLK/1。
     * 采用 8x 过采样，4MHz 下 230400 的分频为 IBRD=2、FBRD=11（误差 -0.08%）。
     * 接收 FIFO 阈值设为 1 字节，配合中断按字节及时取走，避免高波特率下溢出/尾字节滞留。
     * 中断与 NVIC 由 bsp_uart.c 的 BspUart2_Init 在注册回调后再放开。
     */
    DL_UART_Main_setClockConfig(UART_2_INST,
        (DL_UART_Main_ClockConfig *) &gUART_2ClockConfig);
    DL_UART_Main_init(UART_2_INST, (DL_UART_Main_Config *) &gUART_2Config);
    DL_UART_Main_setOversampling(UART_2_INST, DL_UART_MAIN_OVERSAMPLING_RATE_8X);
    DL_UART_Main_setBaudRateDivisor(UART_2_INST,
        UART_2_IBRD_230400_MFCLK, UART_2_FBRD_230400_MFCLK);
    DL_UART_Main_setRXFIFOThreshold(UART_2_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enable(UART_2_INST);
}

static const DL_I2C_ClockConfig gIMU_I2C_1ClockConfig = {
    .clockSel    = DL_I2C_CLOCK_MFCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1
};

void SYSCFG_DL_I2C_1_init(void)
{
    /*
     * I2C1：当前仅保留外设初始化兼容旧接口，**不使能硬件控制器**。
     *
     * IMU 实际访问由 bsp_imu_port.c 切换 PB2/PB3 为 GPIO 软件 I2C。
     * 若在此处使能 I2C1 控制器，则 PB2/PB3 会被硬件 I2C 外设接管，
     * 在调度器启动到 IMU 任务首次 GPIO 软件 I2C 接管之间存在空窗期；
     * 期间硬件控制器可能意外在总线上产生 glitch，导致 LSM6DSV16X 的
     * I2C 状态机进入错误状态（不响应任何事务，只能断电重启恢复）。
     *
     * 因此仅做时钟配置和 FIFO 冲洗，不使能控制器，确保总线全程由
     * GPIO 软件 I2C 控制，避免硬件/软件 I2C 切换的竞争窗口。
     */
    DL_I2C_setClockConfig(IMU_I2C_1_INST,
        (DL_I2C_ClockConfig *) &gIMU_I2C_1ClockConfig);
    DL_I2C_resetControllerTransfer(IMU_I2C_1_INST);
    DL_I2C_setTimerPeriod(IMU_I2C_1_INST, IMU_I2C_1_TPR_10KHZ_MFCLK);
    DL_I2C_flushControllerTXFIFO(IMU_I2C_1_INST);
    DL_I2C_flushControllerRXFIFO(IMU_I2C_1_INST);
    DL_I2C_setControllerTXFIFOThreshold(IMU_I2C_1_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(IMU_I2C_1_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(IMU_I2C_1_INST);
    DL_I2C_disableControllerACK(IMU_I2C_1_INST);
    /* 不调用 DL_I2C_enableController()，见上方注释。 */
}

/*
 * 电机1 STEP 定时器：TIMG0_CCP0 输出连续方波作为步进脉冲。
 * 时钟源选 MFCLK(4MHz)，与 UART/I2C 同源，避免 BUSCLK/ULPCLK 分频带来的步频不确定。
 * 预分频 ÷1（物理最小，prescale=0）→ 定时器时钟 = MFCLK 4MHz；周期 200 → 20kHz 步频；占空比约 50%。
 * 初始化后定时器保持停止，由 bsp_motor 在使能电机时再启动计数。
 */
static const DL_TimerG_ClockConfig gMotorStepClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_MFCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = MOTOR_STEP_TIMER_PRESCALE
};

static const DL_TimerG_PWMConfig gMotorStepConfig = {
    .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period            = MOTOR_STEP_TIMER_PERIOD,
    .isTimerWithFourCC = false,
    .startTimer        = DL_TIMER_STOP
};

/*
 * 初始化一路 STEP 定时器为 CCP0 方波 PWM 输出（四路电机配置完全相同）。
 * 默认周期 200 → 20kHz，运行时由 bsp_motor 写入相同 LOAD 值统一调速。
 * 初始化后定时器保持停止，由 bsp_motor 在使能电机时再启动计数。
 */
static void SYSCFG_DL_initStepTimer(GPTIMER_Regs *inst)
{
    DL_TimerG_setClockConfig(inst,
        (DL_TimerG_ClockConfig *) &gMotorStepClockConfig);
    DL_TimerG_initPWMMode(inst,
        (DL_TimerG_PWMConfig *) &gMotorStepConfig);

    /* CCP0 输出：初值低、不反相、使用功能值驱动。 */
    DL_TimerG_setCaptureCompareOutCtl(inst,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(inst,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(inst,
        MOTOR_STEP_TIMER_DUTY, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(inst);
    DL_TimerG_setCCPDirection(inst, DL_TIMER_CC0_OUTPUT);
}

void SYSCFG_DL_TIMER_STEP_init(void)
{
    /*
     * 四路电机 STEP 定时器：电机1(TIMG0)兼作梯形斜坡"主控"（唯一开 ZERO 中断），
     * 电机2/3/4(TIMG8/TIMG12/TIMG6)为"跟随"，只输出 STEP、周期由 bsp_motor 镜像主控，
     * 使四电机同频同向一起转。四者初始化配置完全相同。
     */
    SYSCFG_DL_initStepTimer(MOTOR_STEP_TIMER_INST);
    SYSCFG_DL_initStepTimer(MOTOR2_STEP_TIMER_INST);
    SYSCFG_DL_initStepTimer(MOTOR3_STEP_TIMER_INST);
    SYSCFG_DL_initStepTimer(MOTOR4_STEP_TIMER_INST);
}

/*
 * 舵机 TIMA0 初始化：四路 50Hz PWM，边沿对齐(向下计数)。
 * MFCLK=4MHz → prescale=3(÷4) → 1MHz → period=20000 → 50Hz。
 * 四个 CCP 通道初值由 bsp_servo 动态更新。
 *
 * 【极性说明·经实测】该定时器 CCP 输出恒为"周期起点置高、向下计到 CC 时置低"，
 * 即 高电平 = period - CC，写入的 CC 值实际对应"低电平宽度"。
 * 曾尝试把 pwmMode 改为 EDGE_ALIGN_UP 想让 CC==高电平脉宽，但实测无效
 * (PA8 直流仍 ~3V)，故此处保持 EDGE_ALIGN，改由 bsp_servo 层写入
 * (period - pulseUs) 做极性补偿，使 PA8 实际高脉冲宽度 == pulseUs。
 */
static const DL_Timer_ClockConfig gServoClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_MFCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = SERVO_TIMER_PRESCALE
};

static const DL_Timer_PWMConfig gServoPWMConfig = {
    .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period            = SERVO_TIMER_PERIOD,
    .isTimerWithFourCC = true,
    .startTimer        = DL_TIMER_STOP
};

void SYSCFG_DL_TIMER_SERVO_init(void)
{
    DL_Timer_setClockConfig(SERVO_TIMER_INST,
        (DL_Timer_ClockConfig *) &gServoClockConfig);
    DL_Timer_initPWMMode(SERVO_TIMER_INST,
        (DL_Timer_PWMConfig *) &gServoPWMConfig);

    /* CCP0=SERVO1：初值低、不反相、功能值驱动。 */
    DL_Timer_setCaptureCompareOutCtl(SERVO_TIMER_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptCompUpdateMethod(SERVO_TIMER_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(SERVO_TIMER_INST,
        1500U, DL_TIMER_CC_0_INDEX);   /* 中位 1.5ms */

    /* CCP1=SERVO2 */
    DL_Timer_setCaptureCompareOutCtl(SERVO_TIMER_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptCompUpdateMethod(SERVO_TIMER_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptureCompareValue(SERVO_TIMER_INST,
        1500U, DL_TIMER_CC_1_INDEX);

    /* CCP2=SERVO3 */
    DL_Timer_setCaptureCompareOutCtl(SERVO_TIMER_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMER_CC_2_INDEX);
    DL_Timer_setCaptCompUpdateMethod(SERVO_TIMER_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_2_INDEX);
    DL_Timer_setCaptureCompareValue(SERVO_TIMER_INST,
        1500U, DL_TIMER_CC_2_INDEX);

    /* CCP3=SERVO4 */
    DL_Timer_setCaptureCompareOutCtl(SERVO_TIMER_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMER_CC_3_INDEX);
    DL_Timer_setCaptCompUpdateMethod(SERVO_TIMER_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_3_INDEX);
    DL_Timer_setCaptureCompareValue(SERVO_TIMER_INST,
        1500U, DL_TIMER_CC_3_INDEX);

    DL_Timer_enableClock(SERVO_TIMER_INST);
    /*
     * 【关键·勿拆成 4 次调用】DL_Timer_setCCPDirection 是「整体覆盖」CCPD 寄存器
     * (gptimer->CCPD = ccpConfig)，不是「或入一位」。四路方向必须**一次调用**把
     * CC0/1/2/3_OUTPUT(0x1|0x2|0x4|0x8=0xF) 全部或在一起写入。
     * 若分 4 次单独调用，只有最后一次(CC3)生效，CCP0/1/2 退回输入态、舵机1/2/3 无 PWM
     * 输出、信号线浮在 ~0.3V 不动——这正是"只有舵机4动"的根因(2026-07-16 实测定位)。
     */
    DL_Timer_setCCPDirection(SERVO_TIMER_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT |
        DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT);
}
