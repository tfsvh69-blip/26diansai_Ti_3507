# 项目上下文

## 基本信息

- 芯片：Texas Instruments MSPM0G3507，Cortex-M0+。
- 工程：Keil uVision，目标名 `empty_LP_MSPM0G3507_nortos_keil`。
- RTOS：FreeRTOS Kernel V11.3.0，使用 `ARM_CM0` 移植层和 `heap_4.c`。
- 当前主频：80 MHz，`SYSOSC 32MHz -> PDIV /2 -> QDIV *10 -> VCO 160MHz -> CLK0 /2 -> MCLK 80MHz`。
- FreeRTOS tick：1000 Hz，即 1 ms。

## 当前分层

| 目录 | 说明 |
|---|---|
| `app/` | 主入口、FreeRTOS 任务创建、业务流程 |
| `bsp/` | 板级初始化、GPIO、UART、延时等硬件封装 |
| `bsp/board/` | 手写板级 DriverLib 初始化配置，当前不依赖 TI SysConfig 生成 |
| `common/` | 任务配置、FreeRTOS 配置、公共消息定义 |
| `module/oled/` | OLED 显示驱动和字模数据 |
| `module/imu/` | ATK-MS6DSV/LSM6DSV16X 初始化、SFLP 姿态读取和四元数转欧拉角 |
| `docs/` | 项目上下文、任务表、接线表、AI 维护记录 |
| `third_party/FreeRTOS/` | FreeRTOS 内核源码 |
| `third_party/ti_driverlib/` | TI DriverLib 文件 |
| `third_party/st_lsm6dsv16x/` | ST LSM6DSV16X 官方寄存器驱动，供 IMU 模块封装调用 |
| `keil/` | Keil 工程、启动文件、链接脚本和构建输出 |

## SysConfig 解耦状态

- Keil 工程的 `BeforeMake` 已关闭，不再执行 `syscfg.bat`。
- `bsp/board/empty.syscfg` 仅作为历史参考文件保留，不参与 Keil 工程和构建。
- `bsp/board/ti_msp_dl_config.c/h` 已改为手写维护文件，保留 `SYSCFG_DL_*` 函数名只是为了兼容现有 BSP 调用。
- 当前不需要安装 TI SysConfig 软件也可以编译、烧录和维护 UART/GPIO/时钟配置。

## 当前硬件

- LED：PB22，实测高电平点亮，低电平熄灭；当前由 `LED1` 任务每 300ms 翻转一次，用作 FreeRTOS 心跳。
- OLED：GPIO 模拟 I2C，SCL=PB9，SDA=PB8。
- UART0：MFCLK 4MHz，115200 8N1，PA10=TX，PA11=RX。
- PA10/PA11 属于核心板特殊功能风险引脚，本次已按用户确认用于 UART0。
- ATK-MS6DSV：GPIO 软件 I2C，SCL=PB2/B02，SDA=PB3/B03，INT=PA16/A16，SA0 接地后 7bit 地址为 `0x6A`；SCL/SDA 已外接上拉，代码仍启用 MCU 内部上拉用于调试，软件时序按正点原子官方例程显式处理 ACK/NACK/STOP。
- IMU 姿态输出：`IMU100Hz` 任务每 10ms 通过 UART0 输出 Roll/Pitch/Yaw；LSM6DSV16X 内部加速度、陀螺仪和 SFLP 使用 120Hz ODR，因为芯片枚举无精确 100Hz 档位。
- 当前 IMU 已实测可连续输出姿态数据；`FIFO=0/1/2` 小范围跳动属于 120Hz 产数与 100Hz 读取节拍不完全同步的正常现象，只要角度连续、FIFO 不持续累积即可。

## FreeRTOS 注意事项

- 不要调用 `SYSCFG_DL_init()` 作为 RTOS 工程总初始化入口；该函数会调用 `SYSCFG_DL_SYSTICK_init()`，与 FreeRTOS SysTick 冲突。
- 当前应使用 `BspBoard_Init()`，内部只调用电源、GPIO、SYSCTL、UART0 初始化，然后初始化 BSP LED。
- 任务周期使用 `vTaskDelay()` 或 `vTaskDelayUntil()`，不要在任务中长时间忙等。
- ISR 中调用 FreeRTOS API 必须使用 `FromISR` 版本。

## 串口问题解决笔记

本次乱码的根因不是文本编码，而是 UART 时钟源不确定导致波特率实际值不稳定。最终稳定方案是启用 `MFCLK`，让 UART0 使用确定的 4MHz 时钟源；115200 8N1 使用 16x 过采样，分频 `IBRD=2`、`FBRD=11`。

当前 UART0 使用 PA10=TX、PA11=RX。串口助手设置为 115200、8N1、无流控。`app/app_uart_test_task.c` 轮询接收：收到任意非换行字符后回显 `UART RX OK`。PB22 不再由串口命令控制，固定作为心跳灯使用。

## UART 后续维护规则

以后新增或修改 UART 前，先阅读 `docs/UART_DEBUG_GUIDE.md`。必须先确认 UART 实例、真实 TX/RX 引脚、GND 共地、电平类型、UART 时钟源和分频值，再烧录验证。出现乱码时优先按 bit 宽和时钟源排查，不要先猜文本编码或反复试波特率。
