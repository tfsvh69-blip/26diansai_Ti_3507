/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 * MSPM0G3507 手写板级 DriverLib 初始化声明。
 *
 * 当前工程不依赖 TI SysConfig 软件生成代码；本文件中的宏和值由工程直接维护。
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)
#define CPUCLK_FREQ                                                     80000000

/* UART0 手写 115200 8N1 配置：MFCLK=4MHz，16x 过采样，误差约 -0.08%。 */
#define UART_0_INST                                                       UART0
#define UART_0_INST_FREQUENCY                                          4000000
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_115200_MFCLK                                            (2U)
#define UART_0_FBRD_115200_MFCLK                                           (11U)

/* IMU：ATK-MS6DSV/LSM6DSV16X，PB2/PB3 当前由端口层切为 GPIO 软件 I2C，SA0 接地后 7bit 地址为 0x6A。 */
#define IMU_I2C_1_INST                                                      I2C1
#define IMU_I2C_1_INST_FREQUENCY                                         4000000
#define IMU_I2C_1_TPR_10KHZ_MFCLK                                          (39U)
#define IMU_MS6DSV_I2C_ADDR_7BIT                                          (0x6A)

/* UART0 复用引脚：PA10=TX，PA11=RX；PA10/PA11 属于核心板慎用引脚，已按用户确认使用。 */
#define GPIO_UART_0_TX_PORT                                               (GPIOA)
#define GPIO_UART_0_TX_PIN                                      (DL_GPIO_PIN_10)
#define GPIO_UART_0_IOMUX_TX                                      (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_TX_FUNC                         IOMUX_PINCM21_PF_UART0_TX
#define GPIO_UART_0_RX_PORT                                               (GPIOA)
#define GPIO_UART_0_RX_PIN                                      (DL_GPIO_PIN_11)
#define GPIO_UART_0_IOMUX_RX                                      (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_RX_FUNC                         IOMUX_PINCM22_PF_UART0_RX

/* LED1：PB22，当前板子实测为高电平点亮。 */
#define LED_LED1_PORT                                                    (GPIOB)
#define LED_LED1_PIN                                            (DL_GPIO_PIN_22)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM50)

/*
 * OLED：GPIO 模拟 I2C。
 * 注意：天猛星扩展板 v1.0 上 PB8/PB9 已改作 TMC 细分 MS1/MS2，本板不再接 OLED。
 * 这里保留宏仅为兼容 module/oled 的编译，运行时不会再初始化/驱动这两脚。
 */
#define OLED_PORT                                                        (GPIOB)
#define OLED_PIN_SCL_PIN                                         (DL_GPIO_PIN_9)
#define OLED_PIN_SCL_IOMUX                                       (IOMUX_PINCM26)
#define OLED_PIN_SDA_PIN                                         (DL_GPIO_PIN_8)
#define OLED_PIN_SDA_IOMUX                                       (IOMUX_PINCM25)

/*
 * 步进电机 / TMC2209（天猛星扩展板 v1.0）。
 * 四路驱动共用 TMC_ENN / TMC_MS1 / TMC_MS2；当前先实现电机1。
 */
/* 电机1 STEP：PB10（PINCM27），使用 TIMG0_CCP0 硬件定时器输出方波。 */
#define MOTOR1_STEP_PORT                                                 (GPIOB)
#define MOTOR1_STEP_PIN                                          (DL_GPIO_PIN_10)
#define MOTOR1_STEP_IOMUX                                        (IOMUX_PINCM27)
#define MOTOR1_STEP_IOMUX_FUNC                        IOMUX_PINCM27_PF_TIMG0_CCP0
#define MOTOR_STEP_TIMER_INST                                             (TIMG0)
/*
 * STEP 定时器：源 MFCLK=4MHz，预分频 ÷1（物理最小，prescale 寄存器=0）。
 * 定时器时钟 = 4MHz；周期 200 → 20kHz 步频，作为加减速的巡航（最高）速度。
 * 直接 20kHz 起转会失步，故 bsp_motor 内做梯形加减速：从慢速起步、加速到本周期对应的
 * 20kHz 巡航、末段再减速；起步/加速参数见 bsp_motor.c。
 */
#define MOTOR_STEP_TIMER_PRESCALE                                           (0U)
#define MOTOR_STEP_TIMER_PERIOD                                           (200U)
#define MOTOR_STEP_TIMER_DUTY                                             (100U)

/* TIMG0 中断号，用于 StartRotateSteps 内部开关 NVIC。 */
#define MOTOR_STEP_TIMER_IRQn                                    (TIMG0_INT_IRQn)

/* 电机1 DIR：PB11（PINCM28），普通 GPIO，低电平为正向。 */
#define MOTOR1_DIR_PORT                                                  (GPIOB)
#define MOTOR1_DIR_PIN                                           (DL_GPIO_PIN_11)
#define MOTOR1_DIR_IOMUX                                         (IOMUX_PINCM28)

/* TMC 总使能 ENN：PA13（PINCM35），低有效，四路共用。 */
#define TMC_ENN_PORT                                                     (GPIOA)
#define TMC_ENN_PIN                                             (DL_GPIO_PIN_13)
#define TMC_ENN_IOMUX                                            (IOMUX_PINCM35)

/* TMC 细分 MS1：PB8（PINCM25，原 OLED SDA），四路共用。 */
#define TMC_MS1_PORT                                                     (GPIOB)
#define TMC_MS1_PIN                                              (DL_GPIO_PIN_8)
#define TMC_MS1_IOMUX                                            (IOMUX_PINCM25)
/* TMC 细分 MS2：PB9（PINCM26，原 OLED SCL），四路共用。 */
#define TMC_MS2_PORT                                                     (GPIOB)
#define TMC_MS2_PIN                                              (DL_GPIO_PIN_9)
#define TMC_MS2_IOMUX                                            (IOMUX_PINCM26)

/*
 * 四个功能按键：一端接 GND，按下为低电平，使用 MCU 内部上拉。
 * A28/A31/A30/A17 均不在核心板慎用引脚列表内。
 */
#define KEY1_PORT                                                        (GPIOA)
#define KEY1_PIN                                                (DL_GPIO_PIN_28)
#define KEY1_IOMUX                                                (IOMUX_PINCM3)
#define KEY2_PORT                                                        (GPIOA)
#define KEY2_PIN                                                (DL_GPIO_PIN_31)
#define KEY2_IOMUX                                                (IOMUX_PINCM6)
#define KEY3_PORT                                                        (GPIOA)
#define KEY3_PIN                                                (DL_GPIO_PIN_30)
#define KEY3_IOMUX                                                (IOMUX_PINCM5)
#define KEY4_PORT                                                        (GPIOA)
#define KEY4_PIN                                                (DL_GPIO_PIN_17)
#define KEY4_IOMUX                                               (IOMUX_PINCM39)

/* IMU 接线：PB2=SCL，PB3=SDA，PA16=INT 输入；SCL/SDA 已外接上拉，代码保留 MCU 内部上拉用于调试。 */
#define IMU_I2C_SCL_PORT                                                  (GPIOB)
#define IMU_I2C_SCL_PIN                                           (DL_GPIO_PIN_2)
#define IMU_I2C_SCL_IOMUX                                         (IOMUX_PINCM15)
#define IMU_I2C_SCL_IOMUX_FUNC                         IOMUX_PINCM15_PF_I2C1_SCL
#define IMU_I2C_SDA_PORT                                                  (GPIOB)
#define IMU_I2C_SDA_PIN                                           (DL_GPIO_PIN_3)
#define IMU_I2C_SDA_IOMUX                                         (IOMUX_PINCM16)
#define IMU_I2C_SDA_IOMUX_FUNC                         IOMUX_PINCM16_PF_I2C1_SDA
#define IMU_INT_PORT                                                      (GPIOA)
#define IMU_INT_PIN                                               (DL_GPIO_PIN_16)
#define IMU_INT_IOMUX                                             (IOMUX_PINCM38)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_TIMER_STEP_init(void);
void SYSCFG_DL_SYSTICK_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
