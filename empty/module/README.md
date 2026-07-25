# module

用于放置电机、传感器、遥控器、OLED、通信协议解析等可复用模块。

模块可以依赖 `bsp/` 提供的硬件能力，但应避免直接依赖 `app/` 的业务任务。

当前包含四个子模块：

| 子模块 | 职责 | 数据来源 | 消费者 |
|---|---|---|---|
| `imu/` | 封装 ATK-MS6DSV/LSM6DSV16X 初始化、SFLP FIFO 读取、四元数转欧拉角 | GPIO 软件 I2C（`bsp_imu_port`） | `app_imu_uart_task` |
| `oled/` | OLED 显示驱动（软件 I2C，仅 ASCII 字库） | — | `app_ui_task` |
| `laser/` | LD14 单点激光测距 195 字节定长帧解析（`LaserLd14_FeedByte/GetLatest`，纯 C、可中断调用） | UART2 RX 中断逐字节喂入 | `app_imu_uart_task`（并入遥测行）、`app_ui_task`（状态栏 `D:`） |
| `vision/` | 上位机小球检测报文解析 `$BALL,found,x,y,n*CHK`（NMEA 风格 + XOR 校验，`BallParser_FeedByte/GetLatest`，纯 C、可中断调用） | UART0 RX 中断逐字节喂入 | `app_ui_task`（右侧文字面板） |

> `laser/` 与 `vision/` 都是"串口字节流 → 组帧校验 → 线程安全最新值快照"的通信协议解析器，不含任何硬件/RTOS 访问，可整体复用。协议细节见 [`../docs/MESSAGE_LIST.md`](../docs/MESSAGE_LIST.md)。
