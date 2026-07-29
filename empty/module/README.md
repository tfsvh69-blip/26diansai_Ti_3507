# module

用于放置电机、传感器、遥控器、OLED、通信协议解析等可复用模块。

模块可以依赖 `bsp/` 提供的硬件能力，但应避免直接依赖 `app/` 的业务任务。

当前包含以下子模块：

| 子模块 | 职责 | 数据来源 | 消费者 |
|---|---|---|---|
| `imu/` | 封装 ATK-MS6DSV/LSM6DSV16X 初始化、SFLP FIFO 读取、四元数转欧拉角 | GPIO 软件 I2C（`bsp_imu_port`） | `app_imu_uart_task` |
| `oled/` | OLED 显示驱动（软件 I2C，仅 ASCII 字库） | — | `app_ui_task` |
| `laser/` | LD14 单点激光测距 195 字节定长帧解析（`LaserLd14_FeedByte/GetLatest`，纯 C、可中断调用） | UART2 RX 中断逐字节喂入 | `app_imu_uart_task`（并入遥测行）、`app_ui_task`（状态栏 `D:`） |
| `vision/` | 视觉端协议解析 `$PONG`/`$ACK`/`$X`（NMEA 风格 + XOR 校验，`VisionParser_FeedByte/GetLatest`，纯 C、可中断调用） | UART0 RX 中断逐字节喂入 | `app_vision_link`（在线状态、题目 ACK 与 X 反馈） |
| `nrf24l01/` | NRF24L01+ 寄存器驱动：固定载荷发送、自动重传、状态轮询、超时保护，含 USB 无线串口文本发送 | GPIO 模拟 SPI（`bsp_nrf24_port`） | `app_nrf24_tx_test_task` |
| `diff_drive/` | 四轮差速圆弧：按带符号半径算左右 RPM 并映射到 M1/M2、M3/M4 | — （纯算法+`bsp_motor` 出口） | `app/tasks/task2.c` |
| `emm42/` | 张大头 Emm42_V5.0 闭环步进：`emm42_v5.c` UART 协议层（命令组帧：速度/位置/使能/急停/同步/读参数 + 回复诊断统计）+ `emm42_robot.c` 角色映射层（本车专用，把协议地址 1/2/3 包成 摆杆/左轮/右轮，`app/tasks/task5.c` 优先用这层） | 发送经 `BspUart1_SendBytes`；回复由 UART1 RX 中断逐字节喂入 | `app/tasks/task5.c` |

> `laser/` 与 `vision/` 都是"串口字节流 → 组帧校验 → 线程安全最新值快照"的通信协议解析器，不含任何硬件/RTOS 访问，可整体复用。`emm42/` 方向相反，是"命令 → 组帧 → 串口下发"的发送侧协议模块（回复只做诊断统计）。协议细节见 [`../docs/MESSAGE_LIST.md`](../docs/MESSAGE_LIST.md)。
