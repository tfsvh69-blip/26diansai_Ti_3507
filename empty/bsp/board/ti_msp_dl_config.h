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

/* OLED：GPIO 模拟 I2C。 */
#define OLED_PORT                                                        (GPIOB)
#define OLED_PIN_SCL_PIN                                         (DL_GPIO_PIN_9)
#define OLED_PIN_SCL_IOMUX                                       (IOMUX_PINCM26)
#define OLED_PIN_SDA_PIN                                         (DL_GPIO_PIN_8)
#define OLED_PIN_SDA_IOMUX                                       (IOMUX_PINCM25)

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
void SYSCFG_DL_SYSTICK_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
