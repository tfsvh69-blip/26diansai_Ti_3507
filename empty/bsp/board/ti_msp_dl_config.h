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

/*
 * 40MHz 外部晶振 HFXT（天猛星核心板 X1，接 PA5/HFXIN、PA6/HFXOUT）。
 * 作为 SYSPLL 参考，倍频后仍输出 80MHz MCLK；起振失败时自动回退内部 SYSOSC。
 * 引脚需配为模拟功能：PA5=PINCM10、PA6=PINCM11（由器件手册 IOMUX 表确认）。
 */
#define GPIO_HFXIN_IOMUX                                          (IOMUX_PINCM10)
#define GPIO_HFXOUT_IOMUX                                         (IOMUX_PINCM11)

/* 运行时记录当前系统时钟参考源：true=40MHz 外部晶振，false=内部 SYSOSC 回退。 */
extern volatile bool g_sysClockUsingHFXT;

/* UART0 手写 115200 8N1 配置：MFCLK=4MHz，16x 过采样，误差约 -0.08%。 */
#define UART_0_INST                                                       UART0
#define UART_0_INST_IRQn                                        (UART0_INT_IRQn)
#define UART_0_INST_FREQUENCY                                          4000000
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_115200_MFCLK                                            (2U)
#define UART_0_FBRD_115200_MFCLK                                           (11U)

/* IMU：ATK-MS6DSV/LSM6DSV16X，PB2/PB3 由端口层切为 GPIO 软件 I2C，SA0 接地后 7bit 地址为 0x6A。 */
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

/*
 * UART2：激光测距1（v1.1 排针 H21），MCU 视角 PB15=TX / PB16=RX，230400 8N1。
 * 时钟源 MFCLK=4MHz，采用 8x 过采样：分频 IBRD=2、FBRD=11，
 * 实际波特率 4MHz/(8*(2+11/64))=230215bps，误差 -0.08%（与 UART0 115200@16x 同一分频）。
 * 若实测激光模块波特率不同，改 UART_2_BAUD_RATE 与下方分频即可。
 * 接收采用中断逐字节喂解析器，见 bsp_uart.c 的 UART2_IRQHandler。
 */
#define UART_2_INST                                                       UART2
#define UART_2_INST_IRQn                                        (UART2_INT_IRQn)
#define UART_2_INST_FREQUENCY                                          4000000
#define UART_2_BAUD_RATE                                                (230400)
#define UART_2_IBRD_230400_MFCLK                                            (2U)
#define UART_2_FBRD_230400_MFCLK                                           (11U)

/* UART2 复用引脚：PB15=TX（PINCM32），PB16=RX（PINCM33），复用功能 PF2。 */
#define GPIO_UART_2_TX_PORT                                               (GPIOB)
#define GPIO_UART_2_TX_PIN                                      (DL_GPIO_PIN_15)
#define GPIO_UART_2_IOMUX_TX                                      (IOMUX_PINCM32)
#define GPIO_UART_2_IOMUX_TX_FUNC                         IOMUX_PINCM32_PF_UART2_TX
#define GPIO_UART_2_RX_PORT                                               (GPIOB)
#define GPIO_UART_2_RX_PIN                                      (DL_GPIO_PIN_16)
#define GPIO_UART_2_IOMUX_RX                                      (IOMUX_PINCM33)
#define GPIO_UART_2_IOMUX_RX_FUNC                         IOMUX_PINCM33_PF_UART2_RX

/* LED1：PB25（PINCM56），v1.1 高电平点亮（IO→阳极）。SDK 头文件已确认 IOMUX_PINCM56=GPIOB_DIO25。 */
#define LED_LED1_PORT                                                    (GPIOB)
#define LED_LED1_PIN                                            (DL_GPIO_PIN_25)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM56)
/* LED2：PA7（PINCM12），v1.1 高电平点亮。 */
#define LED_LED2_PORT                                                    (GPIOA)
#define LED_LED2_PIN                                             (DL_GPIO_PIN_7)
#define LED_LED2_IOMUX                                           (IOMUX_PINCM12)
/* LED3：PB12（PINCM29），v1.1 高电平点亮。 */
#define LED_LED3_PORT                                                    (GPIOB)
#define LED_LED3_PIN                                            (DL_GPIO_PIN_12)
#define LED_LED3_IOMUX                                           (IOMUX_PINCM29)

/*
 * NRF24L01+：GPIO 模拟 SPI，IRQ 不接并由任务轮询 STATUS。
 * PA1/CSN 为开漏脚，外部 4.7kΩ 上拉至 3V3；PA0/MISO 为输入。
 */
#define NRF24_CE_PORT                                                    (GPIOA)
#define NRF24_CE_PIN                                            (DL_GPIO_PIN_22)
#define NRF24_CE_IOMUX                                           (IOMUX_PINCM47)
#define NRF24_CSN_PORT                                                   (GPIOA)
#define NRF24_CSN_PIN                                            (DL_GPIO_PIN_1)
#define NRF24_CSN_IOMUX                                           (IOMUX_PINCM2)
#define NRF24_SCK_PORT                                                   (GPIOA)
#define NRF24_SCK_PIN                                           (DL_GPIO_PIN_27)
#define NRF24_SCK_IOMUX                                          (IOMUX_PINCM60)
#define NRF24_MOSI_PORT                                                  (GPIOA)
#define NRF24_MOSI_PIN                                          (DL_GPIO_PIN_14)
#define NRF24_MOSI_IOMUX                                         (IOMUX_PINCM36)
#define NRF24_MISO_PORT                                                  (GPIOA)
#define NRF24_MISO_PIN                                           (DL_GPIO_PIN_0)
#define NRF24_MISO_IOMUX                                          (IOMUX_PINCM1)

/*
 * OLED：板载 0.96 寸 OLED，GPIO 模拟 I2C（v1.1）。
 * v1.1 板 PB8/PB9 恢复为板载 OLED（软件 I2C），TMC 细分改用 PB0/PB1，两者不再冲突。
 * 约定：SCL=PB9、SDA=PB8（沿用江协驱动既有约定，若与实物相反在此对调即可）。
 * 驱动只做推挽输出、不读 ACK，故按普通数字输出初始化即可。
 */
#define OLED_PORT                                                        (GPIOB)
#define OLED_PIN_SCL_PIN                                         (DL_GPIO_PIN_9)
#define OLED_PIN_SCL_IOMUX                                       (IOMUX_PINCM26)
#define OLED_PIN_SDA_PIN                                         (DL_GPIO_PIN_8)
#define OLED_PIN_SDA_IOMUX                                       (IOMUX_PINCM25)

/* 蜂鸣器 BUZZER：PA15（PINCM37），有源蜂鸣器高电平响，普通推挽 GPIO 输出（v1.1）。 */
#define BUZZER_PORT                                                      (GPIOA)
#define BUZZER_PIN                                              (DL_GPIO_PIN_15)
#define BUZZER_IOMUX                                             (IOMUX_PINCM37)

/*
 * 步进电机 / TMC2209（v1.1）。
 * 四路驱动共用 TMC_ENN(PA13) / TMC_MS1(PB0) / TMC_MS2(PB1)；当前先实现电机1。
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

/* TIMG0 中断号，连续旋转起转/停表时在 bsp_motor 内开关 NVIC。 */
#define MOTOR_STEP_TIMER_IRQn                                    (TIMG0_INT_IRQn)

/* 电机1 DIR：PB11（PINCM28），普通 GPIO，低电平为正向。 */
#define MOTOR1_DIR_PORT                                                  (GPIOB)
#define MOTOR1_DIR_PIN                                           (DL_GPIO_PIN_11)
#define MOTOR1_DIR_IOMUX                                         (IOMUX_PINCM28)
/* 电机1 编号别名（指向历史无编号宏），使 bsp_motor 四路映射表四行同构。 */
#define MOTOR1_STEP_TIMER_INST                              MOTOR_STEP_TIMER_INST
#define MOTOR1_STEP_TIMER_IRQn                              MOTOR_STEP_TIMER_IRQn

/*
 * 电机2/3/4（v1.1）：STEP 各占独立定时器 CCP0 输出，DIR 为普通 GPIO；
 * 四路共用 ENN/MS1/MS2。四路 STEP 定时器全部 MFCLK 4MHz、÷1、周期 200(默认 20kHz)，
 * 运行时由 bsp_motor 统一写入相同 LOAD 值 → 四电机同频同向"一起转"。
 * 电机2：STEP=PB6(PINCM23,TIMG8_CCP0 PF5)、DIR=PB7(PINCM24)。
 * 电机3：STEP=PB13(PINCM30,TIMG12_CCP0 PF4)、DIR=PB14(PINCM31)。
 * 电机4：STEP=PB26(PINCM57,TIMG6_CCP0 PF5)、DIR=PB27(PINCM58)。
 */
#define MOTOR2_STEP_PORT                                                 (GPIOB)
#define MOTOR2_STEP_PIN                                          (DL_GPIO_PIN_6)
#define MOTOR2_STEP_IOMUX                                        (IOMUX_PINCM23)
#define MOTOR2_STEP_IOMUX_FUNC                        IOMUX_PINCM23_PF_TIMG8_CCP0
#define MOTOR2_STEP_TIMER_INST                                           (TIMG8)
#define MOTOR2_DIR_PORT                                                  (GPIOB)
#define MOTOR2_DIR_PIN                                           (DL_GPIO_PIN_7)
#define MOTOR2_DIR_IOMUX                                         (IOMUX_PINCM24)
/* 电机2 TIMG8 中断号：四路独立驱动后每路各开自己的 ZERO 中断做梯形斜坡/计步。 */
#define MOTOR2_STEP_TIMER_IRQn                                   (TIMG8_INT_IRQn)

#define MOTOR3_STEP_PORT                                                 (GPIOB)
#define MOTOR3_STEP_PIN                                         (DL_GPIO_PIN_13)
#define MOTOR3_STEP_IOMUX                                        (IOMUX_PINCM30)
#define MOTOR3_STEP_IOMUX_FUNC                       IOMUX_PINCM30_PF_TIMG12_CCP0
#define MOTOR3_STEP_TIMER_INST                                          (TIMG12)
#define MOTOR3_DIR_PORT                                                  (GPIOB)
#define MOTOR3_DIR_PIN                                          (DL_GPIO_PIN_14)
#define MOTOR3_DIR_IOMUX                                         (IOMUX_PINCM31)
/* 电机3 TIMG12 中断号。 */
#define MOTOR3_STEP_TIMER_IRQn                                  (TIMG12_INT_IRQn)

#define MOTOR4_STEP_PORT                                                 (GPIOB)
#define MOTOR4_STEP_PIN                                         (DL_GPIO_PIN_26)
#define MOTOR4_STEP_IOMUX                                        (IOMUX_PINCM57)
#define MOTOR4_STEP_IOMUX_FUNC                        IOMUX_PINCM57_PF_TIMG6_CCP0
#define MOTOR4_STEP_TIMER_INST                                           (TIMG6)
#define MOTOR4_DIR_PORT                                                  (GPIOB)
#define MOTOR4_DIR_PIN                                          (DL_GPIO_PIN_27)
#define MOTOR4_DIR_IOMUX                                         (IOMUX_PINCM58)
/* 电机4 TIMG6 中断号。 */
#define MOTOR4_STEP_TIMER_IRQn                                    (TIMG6_INT_IRQn)

/* TMC 总使能 ENN：PA13（PINCM35），低有效，四路共用。 */
#define TMC_ENN_PORT                                                     (GPIOA)
#define TMC_ENN_PIN                                             (DL_GPIO_PIN_13)
#define TMC_ENN_IOMUX                                            (IOMUX_PINCM35)

/* TMC 细分 MS1：PB0（PINCM13），四路共用（v1.1）。 */
#define TMC_MS1_PORT                                                     (GPIOB)
#define TMC_MS1_PIN                                              (DL_GPIO_PIN_0)
#define TMC_MS1_IOMUX                                            (IOMUX_PINCM13)
/* TMC 细分 MS2：PB1（PINCM14），四路共用（v1.1）。 */
#define TMC_MS2_PORT                                                     (GPIOB)
#define TMC_MS2_PIN                                              (DL_GPIO_PIN_1)
#define TMC_MS2_IOMUX                                            (IOMUX_PINCM14)

/*
 * 舵机 PWM（v1.1）：四路共用 TIMA0，50Hz 周期，边沿对齐 PWM。
 * MFCLK=4MHz，CPS 预分频=3 → ÷4 → 定时器时钟 1MHz → 50Hz 周期=20000。
 * 注意：MSPM0G CPS 编码为 divider=CPS+1（非 2^CPS），实测 CPS=2 得到 67.4Hz。
 * SERVO1~4 对应 TIMA0 C0~C3，PF 值见各宏。
 */
#define SERVO_TIMER_INST                                                 (TIMA0)
#define SERVO_TIMER_PRESCALE                                              (3U)   /* +1 = /4 */
#define SERVO_TIMER_PERIOD                                             (20000U)   /* 1MHz/50Hz */

/* SERVO1：PA8（PINCM19），TIMA0_CCP0 PF=5 */
#define SERVO1_PORT                                                      (GPIOA)
#define SERVO1_PIN                                               (DL_GPIO_PIN_8)
#define SERVO1_IOMUX                                             (IOMUX_PINCM19)
#define SERVO1_IOMUX_FUNC                             IOMUX_PINCM19_PF_TIMA0_CCP0

/* SERVO2：PA9（PINCM20），TIMA0_CCP1 PF=5 */
#define SERVO2_PORT                                                      (GPIOA)
#define SERVO2_PIN                                               (DL_GPIO_PIN_9)
#define SERVO2_IOMUX                                             (IOMUX_PINCM20)
#define SERVO2_IOMUX_FUNC                             IOMUX_PINCM20_PF_TIMA0_CCP1

/* SERVO3：PB4（PINCM17），TIMA0_CCP2 PF=5 */
#define SERVO3_PORT                                                      (GPIOB)
#define SERVO3_PIN                                               (DL_GPIO_PIN_4)
#define SERVO3_IOMUX                                             (IOMUX_PINCM17)
#define SERVO3_IOMUX_FUNC                             IOMUX_PINCM17_PF_TIMA0_CCP2

/* SERVO4：PA12（PINCM34），TIMA0_CCP3 PF=6 */
#define SERVO4_PORT                                                      (GPIOA)
#define SERVO4_PIN                                              (DL_GPIO_PIN_12)
#define SERVO4_IOMUX                                             (IOMUX_PINCM34)
#define SERVO4_IOMUX_FUNC                             IOMUX_PINCM34_PF_TIMA0_CCP3

/*
 * 四个功能按键（v1.1）：一端接 GND，按下为低电平，使用 MCU 内部上拉。
 * KEY1=PA28、KEY2=PA31、KEY3=PA30、KEY4=PA29，均在 GPIOA bit28~31，可一次端口读。
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
#define KEY4_PIN                                                (DL_GPIO_PIN_29)
#define KEY4_IOMUX                                                (IOMUX_PINCM4)

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

/*
 * 7 路灰度循迹（v1.1，接口 H6）：LINE1~LINE7 = PB17~PB23，全为数字输入。
 * LINE1=最左(小车左) … LINE7=最右(小车右)；识别到线=高电平（模块实测极性）。
 * PINCM 已按 SDK mspm0g350x.h 核实（注意 PB20=PINCM48，非连续）。
 * ⚠️ PB17~PB23 非 5V 容忍：模块信号须 3.3V 电平，否则需分压/电平转换（接线文档风险 R2）。
 */
#define LINE1_PORT                                                       (GPIOB)
#define LINE1_PIN                                               (DL_GPIO_PIN_17)
#define LINE1_IOMUX                                              (IOMUX_PINCM43)
#define LINE2_PORT                                                       (GPIOB)
#define LINE2_PIN                                               (DL_GPIO_PIN_18)
#define LINE2_IOMUX                                              (IOMUX_PINCM44)
#define LINE3_PORT                                                       (GPIOB)
#define LINE3_PIN                                               (DL_GPIO_PIN_19)
#define LINE3_IOMUX                                              (IOMUX_PINCM45)
#define LINE4_PORT                                                       (GPIOB)
#define LINE4_PIN                                               (DL_GPIO_PIN_20)
#define LINE4_IOMUX                                              (IOMUX_PINCM48)
#define LINE5_PORT                                                       (GPIOB)
#define LINE5_PIN                                               (DL_GPIO_PIN_21)
#define LINE5_IOMUX                                              (IOMUX_PINCM49)
#define LINE6_PORT                                                       (GPIOB)
#define LINE6_PIN                                               (DL_GPIO_PIN_22)
#define LINE6_IOMUX                                              (IOMUX_PINCM50)
#define LINE7_PORT                                                       (GPIOB)
#define LINE7_PIN                                               (DL_GPIO_PIN_23)
#define LINE7_IOMUX                                              (IOMUX_PINCM51)

/*
 * 继电器 RELAY（v1.1，接口 P1-3）：PA24，普通 GPIO 推挽输出。
 * 驱动大电流电磁铁负载；默认约定高电平=吸合(通)、低电平=断开(停)，上电默认断开。
 * ⚠️ 接线文档风险 R7：上电前高阻可能误吸合，硬件已建议加 10kΩ 下拉 + 100Ω 串联限流；
 * 代码侧亦用内部下拉 + 上电清零，把误吸合概率再压一层。PA24 不在核心板慎用引脚列表内。
 */
#define RELAY_PORT                                                       (GPIOA)
#define RELAY_PIN                                               (DL_GPIO_PIN_24)
#define RELAY_IOMUX                                              (IOMUX_PINCM54)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_TIMER_STEP_init(void);
void SYSCFG_DL_TIMER_SERVO_init(void);
void SYSCFG_DL_SYSTICK_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
