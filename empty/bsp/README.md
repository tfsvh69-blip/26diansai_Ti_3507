# bsp

用于放置板级初始化、GPIO、UART、SPI、I2C、TIM、ADC、DMA 等硬件相关封装。

`board/` 保存手写板级 DriverLib 初始化配置；`delay/` 保存原 SysTick 忙等延时模块。

当前 `bsp_imu_port.c` 负责 ATK-MS6DSV 的 PB2/PB3 GPIO 软件 I2C 端口层，读寄存器流程对齐正点原子官方例程，并包含 SDA 被拉低时的 SCL 恢复脉冲。
