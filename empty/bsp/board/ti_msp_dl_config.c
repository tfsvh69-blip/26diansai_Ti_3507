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

/*
 * 80MHz 主频配置。
 * SYSOSC=32MHz，先经 PDIV /2，再经 QDIV *10 得到 160MHz VCO。
 * CLK0 再 /2 得到 80MHz MCLK。
 */
static DL_SYSCTL_SYSPLLConfig gSYSPLLConfig80MHz = {
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
    DL_I2C_reset(IMU_I2C_1_INST);
    DL_TimerG_reset(MOTOR_STEP_TIMER_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_I2C_enablePower(IMU_I2C_1_INST);
    DL_TimerG_enablePower(MOTOR_STEP_TIMER_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

void SYSCFG_DL_GPIO_init(void)
{
    /* UART0 使用 PA10=TX、PA11=RX；两者为核心板慎用引脚，已按用户确认接入串口模块。 */
    DL_GPIO_initPeripheralOutputFunctionFeatures(GPIO_UART_0_IOMUX_TX,
        GPIO_UART_0_IOMUX_TX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_UART_0_IOMUX_RX,
        GPIO_UART_0_IOMUX_RX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /* PB22 LED 实测为高电平点亮，GPIO 初始化后由 bsp_led.c 统一设置默认灭灯。 */
    DL_GPIO_initDigitalOutputFeatures(LED_LED1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /*
     * 步进电机 / TMC2209 控制脚（天猛星扩展板 v1.0）。
     * PB8/PB9 在本板改作 TMC 细分 MS1/MS2，不再用于 OLED。
     * STEP 为 TIMG0_CCP0 复用输出；DIR/ENN/MS1/MS2 为普通 GPIO 输出。
     */
    DL_GPIO_initPeripheralOutputFunction(MOTOR1_STEP_IOMUX, MOTOR1_STEP_IOMUX_FUNC);
    DL_GPIO_initDigitalOutputFeatures(MOTOR1_DIR_IOMUX,
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

    /*
     * 上电安全默认状态（对应引脚文档 §3.4 初始化顺序）：
     * 先 ENN 拉高禁用四路驱动；DIR 正向(低)；MS1=0/MS2=0 → 1/8 细分；LED 灭。
     * STEP 由定时器输出，这里不手动置位。
     */
    DL_GPIO_setPins(GPIOA, TMC_ENN_PIN);
    DL_GPIO_enableOutput(GPIOA, TMC_ENN_PIN);

    DL_GPIO_clearPins(GPIOB,
        LED_LED1_PIN | MOTOR1_DIR_PIN | TMC_MS1_PIN | TMC_MS2_PIN);
    DL_GPIO_enableOutput(GPIOB,
        LED_LED1_PIN | MOTOR1_STEP_PIN | MOTOR1_DIR_PIN | TMC_MS1_PIN | TMC_MS2_PIN);
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
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_configSYSPLL(&gSYSPLLConfig80MHz);
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
    DL_UART_Main_enable(UART_0_INST);
}

static const DL_I2C_ClockConfig gIMU_I2C_1ClockConfig = {
    .clockSel    = DL_I2C_CLOCK_MFCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1
};

void SYSCFG_DL_I2C_1_init(void)
{
    /*
     * I2C1：当前仅保留外设初始化兼容旧接口。
     * IMU 实际访问由 bsp_imu_port.c 切换 PB2/PB3 为 GPIO 软件 I2C。
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
    DL_I2C_enableController(IMU_I2C_1_INST);
}

/*
 * 电机1 STEP 定时器：TIMG0_CCP0 输出连续方波作为步进脉冲。
 * 时钟源选 MFCLK(4MHz)，与 UART/I2C 同源，避免 BUSCLK/ULPCLK 分频带来的步频不确定。
 * 预分频 /40 → 100kHz；周期 25 → 4000Hz 步频；占空比约 50%（脉宽足够 TMC2209 识别）。
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

void SYSCFG_DL_TIMER_STEP_init(void)
{
    DL_TimerG_setClockConfig(MOTOR_STEP_TIMER_INST,
        (DL_TimerG_ClockConfig *) &gMotorStepClockConfig);
    DL_TimerG_initPWMMode(MOTOR_STEP_TIMER_INST,
        (DL_TimerG_PWMConfig *) &gMotorStepConfig);

    /* CCP0 输出：初值低、不反相、使用功能值驱动。 */
    DL_TimerG_setCaptureCompareOutCtl(MOTOR_STEP_TIMER_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(MOTOR_STEP_TIMER_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_STEP_TIMER_INST,
        MOTOR_STEP_TIMER_DUTY, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(MOTOR_STEP_TIMER_INST);
    DL_TimerG_setCCPDirection(MOTOR_STEP_TIMER_INST, DL_TIMER_CC0_OUTPUT);
}
