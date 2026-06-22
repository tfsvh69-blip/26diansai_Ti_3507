# 硬件接线表

本文记录当前工程中硬件模块与 MSPM0G3507 的连接关系，用于接线检查、故障排查和后续修改引脚时同步维护。

## 核心板特殊功能引脚

尽量不要使用以下引脚；这些引脚属于特殊功能引脚，使用可能会导致核心板异常：

| 禁用/慎用引脚 |
|---|
| A23 |
| A21 |
| A20 |
| A19 |
| A18 |
| A5 |
| A6 |
| A4 |
| A3 |
| A2 |

## 当前硬件连接

| 硬件 | 信号 | MCU 引脚 | 工程宏 | IOMUX/封装脚位 | 电气/功能说明 | 排查现象 |
|---|---|---|---|---|---|---|
| LED1 | LED1 控制 | PB22 | `LED_LED1_PIN` | `IOMUX_PINCM50`，package pin 21 | GPIO 输出，实测高电平点亮、低电平熄灭 | FreeRTOS 启动后每 300ms 翻转一次，用作系统心跳 |
| OLED | SCL | PB9 | `OLED_PIN_SCL_PIN` | `IOMUX_PINCM26`，package pin 61 | GPIO 模拟 I2C 时钟线，上拉输出 | OLED 无显示时优先检查该线是否接反或悬空 |
| OLED | SDA | PB8 | `OLED_PIN_SDA_PIN` | `IOMUX_PINCM25`，package pin 60 | GPIO 模拟 I2C 数据线，上拉输出 | OLED 无显示时优先检查该线是否接反或悬空 |
| UART0 | TX | PA10 | `GPIO_UART_0_TX_PIN` | `IOMUX_PINCM21` / `IOMUX_PINCM21_PF_UART0_TX` | UART0 发送，MFCLK/115200 8N1；PA10 属于核心板特殊功能风险引脚，已按用户确认使用 | 串口助手应收到启动提示、IMU 输出或 `UART RX OK` 回显 |
| UART0 | RX | PA11 | `GPIO_UART_0_RX_PIN` | `IOMUX_PINCM22` / `IOMUX_PINCM22_PF_UART0_RX` | UART0 接收，用于接收串口助手发来的命令；PA11 属于核心板特殊功能风险引脚 | 发送任意非换行字符后回显 `UART RX OK` |
| ATK-MS6DSV | IMU_SCL | PB2 | `IMU_I2C_SCL_PIN` | `IOMUX_PINCM15` / `IOMUX_PINCM15_PF_I2C1_SCL`，U2.15 | GPIO 软件 I2C SCL；已外接上拉，代码保留 MCU 内部上拉用于调试 | 串口应输出 `IMU INIT OK`，否则优先查 SCL 是否接到 B02 |
| ATK-MS6DSV | IMU_SDA | PB3 | `IMU_I2C_SDA_PIN` | `IOMUX_PINCM16` / `IOMUX_PINCM16_PF_I2C1_SDA`，U2.17 | GPIO 软件 I2C SDA；已外接上拉，代码保留 MCU 内部上拉用于调试；SA0 接地后 7bit 地址 `0x6A` | 初始化失败码 `2` 多为 I2C ACK/接线/地址/上拉问题 |
| ATK-MS6DSV | IMU_INT | PA16 | `IMU_INT_PIN` | `IOMUX_PINCM38`，U2.67 | GPIO 输入，下拉；当前任务轮询读取电平，暂未接入 ISR | 串口每行 `INT=0/1` 反映当前 PA16 电平 |

> 当前 UART0 TX 已从 PB0 改为 PA10。PA10/PA11 均属于核心板特殊功能风险引脚，本次按用户确认使用。

## 当前 OLED 上电显示内容

烧录并运行当前程序后，OLED 应显示以下 ASCII 内容：

| 行 | 内容 |
|---|---|
| 第 1 行 | `MSPM0G3507` |
| 第 2 行 | `FreeRTOS OK` |
| 第 3 行 | `IMU SWI2C 0x6A` |
| 第 4 行 | `PB22 Heartbeat` |

若 PB22 LED 正常闪烁但 OLED 无显示，优先检查 OLED 供电、GND、SCL=PB9、SDA=PB8，以及 OLED I2C 地址是否为驱动中使用的 `0x78` 写地址。

## PB22 心跳灯

- `App_Init()` 当前启动 `AppLedTask_Init()`。
- PB22 每 300ms 翻转一次，用于判断 FreeRTOS 调度是否正常运行。
- 若 PB22 不闪，优先排查程序是否进入 `App_Init()`、是否卡在外设初始化、是否触发 `configASSERT` 或 HardFault。

## UART0 接收回显测试

- 串口助手设置：115200 8N1，无流控。
- UART0 TX 使用 PA10，接串口模块 RX；UART0 RX 使用 PA11，接串口模块 TX。
- `main()` 在板级初始化后会先输出 `BOOT: board init ok`。
- `main()` 在应用初始化完成后会再输出 `BOOT: start scheduler`。
- 收到任意非换行字符后回显 `UART RX OK`。
- 回车和换行会被忽略，避免串口助手自动追加换行造成重复提示。
- 多段拼接的 IMU 调试日志已使用 UART0 行级锁，避免多个任务输出互相穿插；单条短字符串仍可直接发送。

## ATK-MS6DSV 姿态输出

- 模块：正点原子 ATK-MS6DSV，核心器件 LSM6DSV16X。
- 接线：SCL=PB2/B02，SDA=PB3/B03，INT=PA16/A16，SA0 接地。
- I2C：当前使用 PB2/PB3 GPIO 软件 I2C，流程对齐正点原子官方例程；SA0 接地时 7bit 地址为 `0x6A`。
- SCL/SDA 已按用户反馈外接上拉，代码仍启用 MCU 内部上拉用于调试；正式使用建议 PB2/PB3 分别外接 4.7k~10k 到 3.3V。
- 软件 I2C 读寄存器流程为“写地址+寄存器地址、重复起始、读数据、最后一字节 NACK、STOP”；若最后一字节误 ACK，LSM6DSV16X 可能继续保持发送态并拉低 SDA。
- 读取节拍：`IMU100Hz` 任务每 10ms 读取一次并通过 UART0 输出一次。
- 芯片 ODR：LSM6DSV16X 无精确 100Hz 档位，当前加速度、陀螺仪和 SFLP 配为 120Hz，任务侧按 100Hz 对外输出。
- 串口格式示例：`IMU R=4.88 P=-29.17 Y=-20.35 FIFO=1 INT=0`。
- 当前已实测姿态数据可连续输出，`FIFO=0/1/2` 小范围跳动正常；若 FIFO 持续增大，表示读取跟不上，若长时间为 0 且角度不更新，表示 SFLP/FIFO 可能未持续产数。
- 若串口输出 `IMU INIT FAIL:2`，表示 I2C 通信失败，优先检查模块供电、GND 共地、SCL/SDA 是否接反、内部上拉是否不足、SA0 是否确实接地，以及 7bit 地址是否为 `0x6A`。
- 初始化失败时会额外输出 `IMU WHOAMI 0x6A=... 0x6B=...`；正常 LSM6DSV16X 应在实际地址读到 `0x70`，两个地址都为 `ERR` 多半是接线、供电或上拉问题。
- 若初始化输出 `IMU INIT FAIL:1 LAST_ID=0x..`，表示 I2C 读到了 WHO_AM_I，但芯片 ID 与 LSM6DSV16X 期望的 `0x70` 不一致，应优先确认实际芯片型号、模块版本和寄存器读时序。
- 若初始化输出 `IMU INIT FAIL:2 STEP=... LAST_ID=0x70`，表示 WHO_AM_I 已经读通，通信失败发生在后续 reset/config 阶段，应优先按 `STEP` 定位是 `RESET_SET`、`RESET_GET`、`BDU` 还是传感器融合配置。
- 2026-06-18 实测失败点为 `STEP=RESET_SET LAST_ID=0x70`，即 ST 驱动 `RESTORE_CTRL_REGS`/boot reset 会导致 SDA 被拉低；当前代码默认跳过该 reset，直接进入运行配置。
- 初始化失败时还会切到 GPIO 软件 I2C，并给 SCL 输出 18 个恢复脉冲，再输出 `IMU BUS SCL=... SDA=... STAT=...`；恢复后 SCL/SDA 都应为 `1`，若任意一根为 `0`，优先查短路、接反、模块供电或上拉不足。
- 初始化首次失败和之后每 5 次失败会输出 `IMU SCAN: ...`；当前只探测 SA0 可能对应的 `0x6A/0x6B`，避免异常状态下全地址扫描刷出假 ACK。
- 若输出 `IMU SCAN: bus stuck SCL=1 SDA=0`，不要按地址列表排查；这表示 SDA 物理线被拉低，此时所有地址都可能被误判 ACK，应优先断开 IMU SDA 线观察 PB3 是否回到高电平。

## IMU I2C 引脚物理测试

- 当前已关闭：`common/app_config.h` 中 `APP_IMU_I2C_PIN_TEST_ENABLE` 为 `0`。
- 该模式下不初始化 IMU，串口输出 `IMU PINTEST PB2=SCL PB3=SDA, SET then READ GPIO DIN`。
- PB2/SCL 与 PB3/SDA 会每 500ms 交替“拉低/释放高”，随后 MCU 直接读取 GPIO 输入寄存器并输出 `READ SCL=... SDA=...`。
- 若 `SET` 为 `1` 但 `READ` 为 `0`，表示该线释放后仍被外部器件、短路或接线拉低；若 `SET` 为 `0` 但 `READ` 为 `1`，表示 MCU 没能拉低该 GPIO 或 pinmux/端口配置异常。
- 测完必须把 `APP_IMU_I2C_PIN_TEST_ENABLE` 改回 `0`，否则不会进入正常 IMU 读取流程。
- 2026-06-18 用户实测串口 `SET SCL=0 SDA=1 READ SCL=0 SDA=1` 与 `SET SCL=1 SDA=0 READ SCL=1 SDA=0` 持续一致，说明 MCU 端 PB2/PB3 GPIO 读写和开漏释放行为正常；该测试不能单独证明外部板口到 IMU 模块线缆完全无误。

## SysConfig 解耦记录

- Keil 工程已关闭 `BeforeMake` 中的 `syscfg.bat` 调用。
- `bsp/board/empty.syscfg` 只作为历史参考文件保留，不参与构建。
- `bsp/board/ti_msp_dl_config.c/h` 由工程手写维护，不再由 TI SysConfig 重新生成。

## 串口乱码根因记录

- 不要继续从文本编码角度排查此前的乱码；根因是 UART 时钟源和实际波特率不稳定。
- 当前稳定方案：UART0 使用 MFCLK 4MHz，115200 8N1，16x 过采样，`IBRD=2`、`FBRD=11`。
- 当前 UART0 使用 PA10=TX、PA11=RX；若串口不生效，优先确认串口模块 RX 是否接到 PA10、TX 是否接到 PA11，并保证 GND 共地。
- 后续 UART 配置和乱码排查必须先按 `docs/UART_DEBUG_GUIDE.md` 执行，避免再次靠猜波特率、猜引脚或猜文本编码排查。

## 维护规则

- 新增硬件后，在本文增加硬件名称、信号、MCU 引脚、宏名、电气说明和预期现象。
- 修改 GPIO 初始化、外设复用或硬件接线时，必须同步检查并更新本文。
- 使用 A 口特殊功能引脚前，必须先说明原因和风险，并等待人工确认。
