# bsp

用于放置板级初始化、GPIO、UART、SPI、I2C、TIM、ADC、DMA 等硬件相关封装。

`board/` 保存手写板级 DriverLib 初始化配置；`delay/` 保存原 SysTick 忙等延时模块。

当前 `bsp_imu_port.c` 负责 ATK-MS6DSV 的 PB2/PB3 GPIO 软件 I2C 端口层，读寄存器流程对齐正点原子官方例程，并包含 SDA 被拉低时的 SCL 恢复脉冲。

`bsp_uart.c` 负责两路串口：

- **UART0**（PA10/PA11，115200 8N1）：发送侧用**递归互斥量**保证多任务整行日志原子（`BspUart0_Lock/Unlock/SendString`）；接收侧可用 `BspUart0_SetRxHandler` 注册 RX 中断回调，`UART0_IRQHandler` 逐字节喂上层解析器（当前接上位机 `$BALL` 小球报文 → `module/vision`）。RX 中断与旧的轮询自检 `BspUart0_ReadByte` 互斥。
- **UART2**（PB15/PB16，230400 8N1）：`BspUart2_Init` 注册回调 + `UART2_IRQHandler` 逐字节喂激光测距解析器（`module/laser`）。

两个 ISR 都遵守"只快速取空 FIFO、不调用非 FromISR 的 FreeRTOS API"。串口配置与乱码排查见 [`../docs/UART_DEBUG_GUIDE.md`](../docs/UART_DEBUG_GUIDE.md)。

`bsp_motor.c` 负责 4 路步进电机 / TMC2209（小车驱动）：**四路完全独立**，每路各占一个硬件定时器（TIMG0/8/12/6）输出 STEP，并各开自己的 ZERO 中断做梯形加减速与精确计步，互不影响；四路共用 ENN/MS1/MS2。对上提供 RPM 单位的易读接口——`BspMotor_SetSpeedRpm(id, ±rpm)` 连续转、`BspMotor_MoveSteps(id, ±steps, rpm)` 定距，全部非阻塞。接口速查见 [`../app/README.md`](../app/README.md) 的"小车开发速查"。`bsp_servo.c` 为 4 路舵机 TIMA0 50Hz PWM。
