# 项目上下文

## 基本信息

- 芯片：Texas Instruments MSPM0G3507，Cortex-M0+。
- 工程：Keil uVision，目标名 `empty_LP_MSPM0G3507_nortos_keil`。
- RTOS：FreeRTOS Kernel V11.3.0，使用 `ARM_CM0` 移植层和 `heap_4.c`。
- 当前主频：80 MHz，**由 40MHz 外部晶振 HFXT 经 SYSPLL 倍频锁定**：`HFXT 40MHz -> PDIV /2 -> 20MHz -> QDIV *8 -> VCO 160MHz -> CLK0 /2 -> MCLK 80MHz`。晶振起振失败自动回退内部 SYSOSC（`32MHz -> PDIV /2 -> QDIV *10 -> 160MHz -> 80MHz`），启动串口打印实际时钟源，全局标志 `g_sysClockUsingHFXT`。
- 外设时钟：UART0/I2C1/步进定时器均走内部 MFCLK 4MHz，与主频时钟源解耦，波特率/分频常数不随晶振切换变化。
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
| `module/laser/` | 激光测距1（LD14）串口协议解析器，纯软件、按字节喂入、可复用 |
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

- 引脚以 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md`（v1.1 机器人主控板）为准，v1.0 扩展板配置已废弃。
- LED：LED1=PB25(心跳灯，`LED1` 任务每 300ms 翻转)、LED2=PA7、LED3=PB12，均高电平点亮；LED2/LED3 由 `PERIPH` 外设测试任务翻转。
- 蜂鸣器：PA15，有源蜂鸣器高电平响，普通 GPIO；由 `PERIPH` 任务做通断测试。
- OLED：板载 0.96 寸 OLED，软件 I2C，SCL=PB9/SDA=PB8；`PERIPH` 任务刷屏显示调试测试数据。TMC 细分已改到 PB0/PB1，与 OLED 不再冲突。
- 电机1~4（TMC2209 STEP/DIR）：STEP=PB10/PB6/PB13/PB26（TIMG0/TIMG8/TIMG12/TIMG6 各自 CCP0）、DIR=PB11/PB7/PB14/PB27，ENN=PA13(低有效,四路共用)、MS1=PB0/MS2=PB1(四路共用细分)。`MOTORTEST` 任务 KEY1/KEY2 让四电机一起正/反转测试；实现为"电机1 斜坡主控 + 电机2/3/4 镜像同频跟随"（只有 TIMG0 开中断）。
- 舵机1~4（TIMA0_CCP0~3 PWM）：SERVO=PA8/PA9/PB4/PA12，50Hz；`SERVOTEST` 任务 KEY3/KEY4 让四舵机一起在两位置切换测试。脉宽经 `bsp_servo` 极性补偿(period-pulseUs)，避开 EDGE_ALIGN 反相坑，勿绕过直接写定时器。
- UART0：MFCLK 4MHz，115200 8N1，PA10=TX，PA11=RX。
- 激光测距1（UART2）：PB15=TX/PB16=RX，MFCLK 4MHz + 8x 过采样，230400 8N1；**RX 中断**逐字节喂 `module/laser` 的 LD14 解析器（无独立任务），距离由 `IMU100Hz` 任务在整行末尾追加 `D1=<mm>mm` 输出。波特率 230400 依参考工程推定，实物不符改 `UART_2_BAUD_RATE`。⚠️ 激光 TX 若 5V 而 PB16 非 5V 容忍，接前先量电平（风险 R2）。
- PA10/PA11 属于核心板特殊功能风险引脚，本次已按用户确认用于 UART0。
- 40MHz 晶振：PA5=HFXIN、PA6=HFXOUT，作 SYSPLL 参考锁定 80MHz 主频（详见接线表“系统时钟”节）。
- ATK-MS6DSV：**GPIO 软件 I2C**（开漏模拟），SCL=PB2/B02，SDA=PB3/B03，INT=PA16/A16，SA0 接地后 7bit 地址 `0x6A`；扩展板已焊 4.7k 上拉到 3.3V，IMU 供电 3.3V。端口层 `bsp_imu_port.c` 首次访问时关闭 I2C1 硬件控制器并切 PB2/PB3 为 GPIO 模式。
- IMU 姿态输出：`IMU100Hz` 任务每 10ms 读一次融合欧拉角(FIFO/SFLP)，通过 UART0 按 **5Hz**(打印节流 `APP_IMU_PRINT_DIVIDER=20`)输出 Roll/Pitch/Yaw + 三轴加速度(mg) + 三轴角速度(mdps) + 激光测距1(D1,mm)。加速度/角速度走输出寄存器直读，采用方案A：只在打印那拍(5Hz)才读，避免软件 I2C 忙等开销(一次 6 字节读≈1ms，两组≈2ms/周期)。整行同时追加激光测距1，实现"和陀螺仪数据一起发、频率不高、便于阅读"。LSM6DSV16X 内部加速度、陀螺仪和 SFLP 使用 120Hz ODR，因为芯片枚举无精确 100Hz 档位。加速度量程 ±2g、角速度量程 ±125dps。将来算法需要 100Hz 原始数据时，把读取移回每周期并优先恢复硬件 I2C 提速。
- 当前 IMU 已实测可连续输出姿态数据；`FIFO=0/1/2` 小范围跳动属于 120Hz 产数与 100Hz 读取节拍不完全同步的正常现象，只要角度连续、FIFO 不持续累积即可。

## CPU 占用预算（估算，80MHz）

> 目的：定位"谁在吃 CPU"，便于后续优化。核心结论：**吃 CPU 的几乎全是"软件位操作 + 忙等延时"（软件 I2C / 软件 I2C 刷屏），不是算法**。数值为量级估算，非精确测量。

按平均占用从高到低：

| 排名 | 部分 | 触发/频率 | 单次耗时 | 平均 CPU | 性质 |
|---|---|---|---:|---:|---|
| 1 | **IMU 融合角读取**（`IMU100Hz`→软件 I2C 读 FIFO_STATUS + 1~2 条 SFLP 记录） | 100Hz（每 10ms） | ~1.5~2.8ms/次 | **~15~25%** | CPU 忙等（`DL_Common_delayCycles` 空转，每位 3×5µs≈15µs，一字节≈0.1~0.14ms） |
| 2 | **OLED 整屏刷新**（`PERIPH`→`OLED_Update` 软件 I2C 推 ~1050 字节） | 2Hz（每 500ms） | **~45~57ms/次（突发）** | ~9~11% | CPU 忙等（`Delay_us(2)`，每字节≈54µs）；突发期与其它同优先级任务分时轮转 |
| 3 | **激光测距 RX 中断**（UART2 230400，每字节 1 次中断喂解析器） | 跟随激光帧率，连续流最坏 ~23000 次/秒 | ~80 周期/字节 + 每帧 CRC~1500 周期 | ~2~3%（满速流时） | 中断，随实际字节率线性缩放 |
| 4 | **IMU 原始加速度/角速度读取**（打印那拍 2×6 字节软件 I2C 读） | 5Hz | ~2ms/次 | ~1% | CPU 忙等（方案A 已从 100Hz 降到 5Hz） |
| 5 | **电机 STEP 中断**（`TIMG0_IRQHandler` 梯形斜坡计算） | 仅电机转动时，巡航 ~8kHz | ~150 周期/次 | ~1.5%（仅转动时） | 中断；**四电机一起转也只有 TIMG0 一个 ISR**（2/3/4 镜像跟随、不产生中断），CPU 开销与单电机相同 |
| — | LED / 舵机 / 串口回显 / 电机按键轮询等 | 300ms~20ms | 微秒级 | <0.5% | 可忽略 |

**空闲态**（电机不转、激光在收、OLED 在刷、IMU 在读）估算总占用约 **30~40% CPU**，其余为 FreeRTOS 空闲任务。

**要降 CPU 的话，优先级从高到低**：

1. **IMU 换回硬件 I2C（400kHz）**：软件 I2C 忙等是最大头。硬件 I2C 用 DMA/中断收发不占 CPU，一次 6 字节读从 ~1ms 降到 ~0.15ms，可把第 1 项从 ~20% 砍到个位数。（历史因软/硬 I2C 切换会把老 IMU 推入锁死态而回退纯软件，换新模块后可重新评估。）
2. **OLED 降刷新率或改局部刷新**：`OLED_Update` 整屏很贵。用 `OLED_UpdateArea` 只刷变化区域，或把 `PERIPH` 周期从 500ms 拉长；也可把 OLED 放到独立低优先级任务，避免突发阻塞同优先级任务。
3. **激光 RX FIFO 阈值调高 + 批量处理**：若确认激光帧率不高，可把 RX FIFO 阈值从 1 提到 4，减少中断次数（当前为 1 是为了不丢尾字节，权衡后再定）。
4. **IMU 融合角读取按需降频**：若上层不需要 100Hz 角度，可把 `APP_IMU_UART_PERIOD_TICKS` 拉长。

## FreeRTOS 注意事项

- 不要调用 `SYSCFG_DL_init()` 作为 RTOS 工程总初始化入口；该函数会调用 `SYSCFG_DL_SYSTICK_init()`，与 FreeRTOS SysTick 冲突。
- 当前应使用 `BspBoard_Init()`，内部依次调用电源、GPIO、SYSCTL、UART0、UART2、I2C1、步进定时器、舵机定时器初始化，再初始化 BSP LED/电机/蜂鸣器，最后建 UART0 递归互斥量。
- 任务周期使用 `vTaskDelay()` 或 `vTaskDelayUntil()`，不要在任务中长时间忙等。
- ISR 中调用 FreeRTOS API 必须使用 `FromISR` 版本。

## 串口问题解决笔记

本次乱码的根因不是文本编码，而是 UART 时钟源不确定导致波特率实际值不稳定。最终稳定方案是启用 `MFCLK`，让 UART0 使用确定的 4MHz 时钟源；115200 8N1 使用 16x 过采样，分频 `IBRD=2`、`FBRD=11`。

当前 UART0 使用 PA10=TX、PA11=RX。串口助手设置为 115200、8N1、无流控。`app/app_uart_test_task.c` 轮询接收：收到任意非换行字符后回显 `UART RX OK`。LED1(PB25) 不再由串口命令控制，固定作为心跳灯使用。

## UART 后续维护规则

以后新增或修改 UART 前，先阅读 `docs/UART_DEBUG_GUIDE.md`。必须先确认 UART 实例、真实 TX/RX 引脚、GND 共地、电平类型、UART 时钟源和分频值，再烧录验证。出现乱码时优先按 bit 宽和时钟源排查，不要先猜文本编码或反复试波特率。
