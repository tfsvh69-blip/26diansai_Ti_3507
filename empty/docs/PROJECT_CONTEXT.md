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
| `module/vision/` | 上位机小球检测报文 `$BALL`（NMEA+XOR）解析器，纯软件、按字节喂入、可复用 |
| `module/emm42/` | 张大头 Emm42_V5.0 闭环步进驱动器：`emm42_v5.c`(UART 协议层，命令组帧+回复诊断统计，经 `bsp_uart` 的 UART1 收发) + `emm42_robot.c`(角色映射层，把协议地址包成"摆杆/左轮/右轮"，本车专用) |
| `module/nrf24l01/` | NRF24L01+ 寄存器驱动，提供带自动重传、状态轮询和超时保护的固定载荷发送接口，以及 USB 无线串口文本发送接口 |
| `module/diff_drive/` | 四轮差速圆弧模块：根据带符号半径计算左右 RPM，并一次映射到 M1/M2、M3/M4；标称左右轮距 201 mm |
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
- 蜂鸣器：PA15，有源蜂鸣器高电平响，普通 GPIO；`UIMENU` 对任一按键按下沿短促嘀声（~2ms 忙等后立即关断），`PERIPH` 任务可做通断测试。
- OLED：板载 0.96 寸 OLED，软件 I2C，SCL=PB9/SDA=PB8；`PERIPH` 任务刷屏显示调试测试数据。TMC 细分已改到 PB0/PB1，与 OLED 不再冲突。
- 电机1~4（TMC2209 STEP/DIR）：STEP=PB10/PB6/PB13/PB26（TIMG0/TIMG8/TIMG12/TIMG6 各自 CCP0）、DIR=PB11/PB7/PB14/PB27，ENN=PA13(低有效,四路共用)、MS1=PB0/MS2=PB1(四路共用细分)。**v1.9 起四路完全独立**：每路各开自己的 ZERO 中断完成定距计步，可各自不同速度/方向/距离；所有速度、定距和停止命令直接下发，不做梯形加减速。实车安装位置为 M1=左前、M2=左后、M3=右前、M4=右后；M1/M2 与其余两路机械方向相反，已在 `BspMotor_Init()` 全局取反标定，之后正 RPM/正 steps 对四路均表示小车前进。对上接口为 `BspMotor_SetSpeedRpm(id,±rpm)` / `BspMotor_MoveSteps(id,±steps,rpm)`，见 `bsp_motor.h`。直接高速起转可能失步，应先低速实测。`MOTORTEST` 任务（默认禁用）用新接口对四路同时下发做验证。
- 四轮差速圆弧：`module/diff_drive/diff_drive.h` 以标称左右轮中心距 201 mm 计算圆弧内外轮 RPM，并将左侧命令映射给 M1/M2、右侧映射给 M3/M4。`DiffDrive_RunRadiusTurn(半径,中心RPM)` 是任务可直接调用的立即执行接口，不缩放用户给定速度；超出安全 RPM 则拒绝命令。当前前后轮中心距也是 201 mm、轮胎宽度 27 mm；后二者不进入理想差速公式，但会造成固定四轮转弯侧滑，实车可通过调整 `trackWidthMm` 校正有效轮距。该模块保留为可复用圆弧运动学，详见 `docs/DIFF_DRIVE.md`。
- 题目二 `LINE PID`（✅ 已上车测试通过）用于正式 8 路灰度循迹：按物理左→右的 LINE8→LINE1 加权计算误差，PID 生成左右轮差速，并经 UART1 的 Emm42 角色层控制 ID2 左轮和 ID3 右轮；ID1 摆杆不参与。入场走节拍化状态机（先失能复位再依次使能两路，`OnEnter` 不直接下发命令）；误差经低通滤波+死区再喂给 PID 抑制中心抖动；速度命令加速度档位用非 0 曲线档位平滑变速，PID 目标 RPM 每拍直接下发、不叠加任务层软件斜坡；左右轮差速限幅用"整体平移"保住转向权限（`Task2_ClampWheelPair`）；OLED 显示秒表计时（到终点自动定格），因主频/tick 精度疑点乘了实测校准系数；秒表时间达到固定阈值 `T2_DECEL_START_MS` 后基础速度开始线性下降（梯度、下限两个参数可调），跟转弯减速取更小值生效。基础速度、PID 和丢线保护参数集中在 `app/tasks/task2.c` 顶部，详见 `docs/FREERTOS_TASKS.md`。
- 舵机1~4（TIMA0_CCP0~3 PWM）：SERVO=PA8/PA9/PB4/PA12，50Hz；`SERVOTEST` 任务 KEY3/KEY4 让四舵机一起在两位置切换测试。脉宽经 `bsp_servo` 极性补偿(period-pulseUs)，避开 EDGE_ALIGN 反相坑，勿绕过直接写定时器。
- 继电器（RELAY）：**PA24**(PINCM54，接口 P1-3)，普通 GPIO 推挽输出驱动大电流电磁铁负载，无需 PWM/定时器。**极性已实测确认：高电平=吸合、低电平=断开**(`bsp_relay.c::BSP_RELAY_ACTIVE_LOW=0`；换低电平触发模块把该宏改 1 即整体反相)；上电默认断开(PA24 下拉+清零，`BspRelay_Init` 在 `BspBoard_Init` 里再收敛一次)。封装 `bsp/bsp_relay.h`：`BspRelay_On/Off/Set(bool)/Toggle/IsOn`，语义以「吸合/断开」为准，业务代码 `#include "bsp_relay.h"` 即可直接调。**正常运行由业务代码(`app/tasks/taskN.c`)按需调接口控制、不自动切换**；每 2s 自动切换的 `RELAYTEST` 自检任务默认禁用(见功能开关)。OLED 底部状态栏最前显示 `R:ON/OFF`(`APP_FEATURE_RELAY=1`)。接线风险 R7：PA24 上电前高阻可能误吸合，硬件建议加 10kΩ 下拉+100Ω 限流。
- UART0：MFCLK 4MHz，115200 8N1，PA10=TX，PA11=RX。当前 **PA11(RX) 接上位机(视觉主机)TX**，经 **UART0 RX 中断**逐字节喂 `module/vision` 解析上位机 `$BALL,found,x,y,n*CHK` 小球检测报文（约 17 帧/秒，已实测正常接收），结果由 `UIMENU` 显示在 OLED 右侧文字面板（开关 `APP_FEATURE_BALL_VISION`）。RX 中断与轮询自检 `APP_FEATURE_UART_ECHO` 互斥（编译期护栏）。
- 张大头 Emm42_V5.0 闭环步进（UART1）：PA17=TX(→驱动器RX)/PB5=RX(←驱动器TX)，MFCLK 4MHz + 16x 过采样，115200 8N1，v1.1 排针 H7。**2026-07-29 起总线上挂 3 台设备**，靠设备地址(1/2/3)区分：地址1=摆杆高低调节(LIFT)、地址2=左轮(WHEEL_L)、地址3=右轮(WHEEL_R)。与 TMC2209 开环 STEP/DIR 完全不同：这是**串口命令式闭环驱动**，MCU 只发命令帧、不产生脉冲、不占定时器，速度/位置由驱动器内部执行。**题目二与题目五使用此系统；4 路 TMC2209 开环电机（bsp_motor，保留给题目一、题目三及独立测试）本轮不参与巡线。**
  - **两层 API**：`module/emm42/emm42_v5.h` 是纯协议层，提供 `Emm42_Enable/SetSpeedRpm/VelControl/PosControl/StopNow/SyncMotion/ReadSysParams` 及诊断计数（`Emm42_GetRxByteCount` 判断总线是否通），只认设备地址、不知道地址对应哪个部件；`module/emm42/emm42_robot.h` 在其上按本车实际用途包一层角色映射（`Emm42RobotId_t`: `EMM42_ROBOT_LIFT/WHEEL_L/WHEEL_R`），提供 `Emm42Robot_Init/Enable/SetSpeedRpm/Stop/StopAll/GetAddr`，业务代码（`app/tasks/task5.c`）应优先用这层，不直接碰地址数字。
  - ⚠️ 方向/差速运动学**尚未标定**：正负 RPM 与"抬升/下降"、"前进/后退"的实际对应关系待确认，目前 `emm42_robot` 只做"按角色转发到对应地址"的基础驱动，不做方向补偿也不做左右轮差速运动学（后续会补类似 `bsp_motor.h::SetDirInvert` 的标定接口和类似 `module/diff_drive` 的差速运动学）。
  - 所有接口非阻塞、不等回复、不做延时，调用方须自行留帧间隔——第 5 题 `task5.c` 的做法是**每 30ms 轮询拍最多发一帧**（3 路错拍：LIFT/WHEEL_L/WHEEL_R）。回复走 UART1 RX 中断（无独立任务），以校验字节 `0x6B` 分帧，仅用于诊断不解析字段。波特率取 Emm42 出厂默认 115200（与参考工程一致）；若驱动器被改为接线表建议的 38400，换 `ti_msp_dl_config.h` 中已算好的另一组 IBRD/FBRD 即可。
  - ⚠️ 风险 R1：多台驱动器 TX 并联于 PB5（现为 3 台，原接线文档述两台，风险等比放大），需外部 BAT54S 肖特基线与 + 4.7kΩ 上拉后才可同时接；改造完成前可用 `task5.c` 的 `T5_ENABLE_LIFT/WHEEL_L/WHEEL_R` 跳过未接的角色。
  - ⚠️ 第 3 台设备的物理接线点：v1.1 PCB 的 H7 排针按权威接线文档只设计了 Motor1/Motor2 两个物理连接头，第 3 台需从同一 UART1 总线额外引出，具体接法待确认后补充到 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md`（该文档是硬件权威来源，本仓库文档不代为修改，只在此处标注待办）。
- 激光测距1（UART2）：PB15=TX/PB16=RX，MFCLK 4MHz + 8x 过采样，230400 8N1；**RX 中断**逐字节喂 `module/laser` 的 LD14 解析器（无独立任务），距离由 `IMU100Hz` 任务在整行末尾追加 `D1=<mm>mm` 输出。波特率 230400 依参考工程推定，实物不符改 `UART_2_BAUD_RATE`。⚠️ 激光 TX 若 5V 而 PB16 非 5V 容忍，接前先量电平（风险 R2）。**【2026-07-29 临时禁用，`APP_FEATURE_LASER=0`，减 CPU/中断占用，需要时改回 1】**：关闭后 `App_Init` 不注册 UART2 RX 回调，中断不使能，`LaserLd14_GetLatest()` 恒返回 false。
- NRF24L01+：GPIO 模拟 SPI 模式0，CE=PA22、CSN=PA1（开漏，外部4.7kΩ上拉3V3）、SCK=PA27、MOSI=PA14、MISO=PA0，IRQ不接。`NRF24TX` 任务**【2026-07-29 临时禁用，`APP_FEATURE_NRF24_TX_TEST=0`，减 CPU 占用，需要时改回 1】**，启用时每500ms按 USB 无线串口 V2.0 的 32字节协议发送 `TMX NRF24 TEST nnnnnn`；已实机验证参数为地址 `15 52 33 54 55`、2.402GHz、2Mbps、16位CRC、0dBm、自动应答。`nrf24l01.h` 对外提供 `g_nrf24UsbUartV20Config`、`Nrf24_Init()`、`Nrf24_SendPayload()` 和 `Nrf24_SendUsbUartText()`，后续业务可直接复用。
- PA10/PA11 属于核心板特殊功能风险引脚，本次已按用户确认用于 UART0。
- 40MHz 晶振：PA5=HFXIN、PA6=HFXOUT，作 SYSPLL 参考锁定 80MHz 主频（详见接线表“系统时钟”节）。
- ATK-MS6DSV：**GPIO 软件 I2C**（开漏模拟），SCL=PB2/B02，SDA=PB3/B03，INT=PA16/A16，SA0 接地后 7bit 地址 `0x6A`；扩展板已焊 4.7k 上拉到 3.3V，IMU 供电 3.3V。端口层 `bsp_imu_port.c` 首次访问时关闭 I2C1 硬件控制器并切 PB2/PB3 为 GPIO 模式。
- IMU 姿态输出：**【2026-07-29 临时禁用，`APP_FEATURE_IMU=0`，减 CPU 占用（软件 I2C 100Hz 读取是当前开销最大的任务），需要时改回 1】**：关闭后 `IMU100Hz` 任务不创建，`AppImuUartTask_GetYaw()` 恒返回 false，OLED 状态栏 Yaw 显示 `---`，依赖 Yaw 的题目三 `GYRO 90L` 无法测试。启用时行为：`IMU100Hz` 任务每 10ms 读一次融合欧拉角(FIFO/SFLP)，通过 UART0 按 **5Hz**(打印节流 `APP_IMU_PRINT_DIVIDER=20`)输出 Roll/Pitch/Yaw + 三轴加速度(mg) + 三轴角速度(mdps) + 激光测距1(D1,mm)。加速度/角速度走输出寄存器直读，采用方案A：只在打印那拍(5Hz)才读，避免软件 I2C 忙等开销(一次 6 字节读≈1ms，两组≈2ms/周期)。整行同时追加激光测距1，实现"和陀螺仪数据一起发、频率不高、便于阅读"。LSM6DSV16X 内部加速度、陀螺仪和 SFLP 使用 120Hz ODR，因为芯片枚举无精确 100Hz 档位。加速度量程 ±2g、角速度量程 ±125dps。将来算法需要 100Hz 原始数据时，把读取移回每周期并优先恢复硬件 I2C 提速。
- 当前 IMU 已实测可连续输出姿态数据；`FIFO=0/1/2` 小范围跳动属于 120Hz 产数与 100Hz 读取节拍不完全同步的正常现象，只要角度连续、FIFO 不持续累积即可。

### IMU Yaw（偏航角）行为约定（已实测）

- **范围**：−180.00° ~ +180.00°（内部单位：厘度 0.01°，即 −18000 ~ +18000）。
- **转向与 Yaw 变化方向**：小车左转时 Yaw 递减（例如 180° → 150° → 0° → −179°）；右转时 Yaw 递增。
- **复位初始值不确定**：每次 MCU 复位后，IMU SFLP 融合输出的初始 Yaw 值不同（融合需要时间收敛，且无绝对航向参考——仅靠陀螺仪+加速度计无法提供绝对偏航）。**这不影响相对角度闭环**：任务只需记录起始 Yaw 计算偏移量（如目标 = 起始 − 90°），不依赖绝对 Yaw 值。
- **跨界跳变处理**：Yaw 在 ±180° 边界会跳变（如 −179° → +179°），计算角度差时必须用最短路径差值算法（见 `task3.c::AngleDiffCd()`），不能直接做减法。
- **漂移**：无磁力计/外部参考时，Yaw 长期会有陀螺仪积分漂移；短时间（几秒到几十秒）内相对精度足够用于 90°/180° 转弯闭环。

## CPU 占用预算（估算，80MHz）

> 目的：定位"谁在吃 CPU"，便于后续优化。核心结论：**吃 CPU 的几乎全是"软件位操作 + 忙等延时"（软件 I2C / 软件 I2C 刷屏），不是算法**。数值为量级估算，非精确测量。

按平均占用从高到低：

| 排名 | 部分 | 触发/频率 | 单次耗时 | 平均 CPU | 性质 |
|---|---|---|---:|---:|---|
| 1 | **IMU 融合角读取**（`IMU100Hz`→软件 I2C 读 FIFO_STATUS + 1~2 条 SFLP 记录） | 100Hz（每 10ms） | ~1.5~2.8ms/次 | **~15~25%** | CPU 忙等（`DL_Common_delayCycles` 空转，每位 3×5µs≈15µs，一字节≈0.1~0.14ms） |
| 2 | **OLED 局部刷新**（`PERIPH`→`OLED_UpdateArea(0,0,128,24)` 只推前 3 页 ~384 字节） | 2Hz（每 500ms） | **~20ms/次（突发）** | ~4~5% | CPU 忙等（`Delay_us(2)`）；已从整屏 8 页(~50ms)改为只刷显示用的前 3 页 |
| 3 | **激光测距 RX 中断**（UART2 230400，每字节 1 次中断喂解析器） | 跟随激光帧率，连续流最坏 ~23000 次/秒 | ~80 周期/字节 + 每帧 CRC~1500 周期 | ~2~3%（满速流时） | 中断，随实际字节率线性缩放 |
| 4 | **IMU 原始加速度/角速度读取**（打印那拍 2×6 字节软件 I2C 读） | 5Hz | ~2ms/次 | ~1% | CPU 忙等（方案A 已从 100Hz 降到 5Hz） |
| 5 | **电机 STEP 中断**（`TIMG0/8/12/6_IRQHandler` 定距计步） | 仅电机转动时，每路 = 该路步频（常 ~kHz 级） | ~150 周期/次 | ~1.5%/路（仅转动时） | 中断；**v1.9 四路各自独立 ISR**，同时转动时开销约为单路的 N 倍（N=转动路数），常规速度下仍很小 |
| 6 | **小球报文 RX 中断**（UART0 115200，$BALL 约 17 帧/秒×~25 字节） | ~425 次/秒 | ~80 周期/字节 + 每帧解析~数百周期 | <0.5% | 中断；字节率远低于激光，几乎可忽略 |
| — | LED / 舵机 / 串口回显 / 电机按键轮询等 | 300ms~20ms | 微秒级 | <0.5% | 可忽略 |

**空闲态**（电机不转、激光在收、OLED 在刷、IMU 在读）估算总占用约 **30~40% CPU**，其余为 FreeRTOS 空闲任务。

**要降 CPU 的话，优先级从高到低**：

1. **IMU 换回硬件 I2C（400kHz）**：软件 I2C 忙等是最大头。硬件 I2C 用 DMA/中断收发不占 CPU，一次 6 字节读从 ~1ms 降到 ~0.15ms，可把第 1 项从 ~20% 砍到个位数。（历史因软/硬 I2C 切换会把老 IMU 推入锁死态而回退纯软件，换新模块后可重新评估。）
2. **OLED 降刷新率或改局部刷新**：`OLED_Update` 整屏很贵。用 `OLED_UpdateArea` 只刷变化区域，或把 `PERIPH` 周期从 500ms 拉长；也可把 OLED 放到独立低优先级任务，避免突发阻塞同优先级任务。
3. **激光 RX FIFO 阈值调高 + 批量处理**：若确认激光帧率不高，可把 RX FIFO 阈值从 1 提到 4，减少中断次数（当前为 1 是为了不丢尾字节，权衡后再定）。
4. **IMU 融合角读取按需降频**：若上层不需要 100Hz 角度，可把 `APP_IMU_UART_PERIOD_TICKS` 拉长。

## 功能总开关

- `common/app_config.h` 顶部有 `APP_FEATURE_*`（1/0），`App_Init` 据此门控各任务创建；用开发板时把不用的外设置 0（不建任务/不占 CPU/不刷串口，硬件初始化保留）。关掉的功能会被链接器移除，实测关 IMU+LASER 后代码从 ~30KB 降到 ~18KB。
- 激光 D1 随 IMU 遥测整行输出，`APP_FEATURE_IMU=0` 时该行不打印（即使 LASER=1，激光仍后台接收但无打印出口）。
- **【2026-07-29】** `APP_FEATURE_IMU`、`APP_FEATURE_LASER`、`APP_FEATURE_NRF24_TX_TEST` 三个当前临时改为 `0`（默认值曾是 `1`），目的是给张大头 Emm42 步进测试腾 CPU/中断资源；只是关闭，代码逻辑未删改，需要陀螺仪/激光/无线串口时随时改回 `1`。详见下方各条目及 `FREERTOS_TASKS.md` 对应任务/中断小节的禁用说明。
- `APP_FEATURE_BALL_VISION`（默认 1）：UART0 RX 中断解析上位机 `$BALL` 报文，供 OLED 右侧面板显示；与 `APP_FEATURE_UART_ECHO` 争用 UART0 RX，二者互斥（同时置 1 编译期 `#error` 拦截，需串口收发自检时先关 BALL）。
- 继电器开关 `APP_FEATURE_RELAY`（默认 1）与 `APP_FEATURE_RELAY_SELFTEST`（默认 0）**已解耦**（2026-07-25）：前者=继电器功能(板级初始化 + OLED 状态栏 `R:ON/OFF` 显示 + 对外接口 `BspRelay_*` 可调用)；后者=自检任务 `RELAYTEST`(每 2s 自动切换吸合/断开，仅上电验证用)。正常运行继电器由业务代码经 `bsp_relay` 接口按需控制、不自动切换；需上电自检时把 `APP_FEATURE_RELAY_SELFTEST` 置 1。

- `APP_FEATURE_EMM42`（默认 1）：张大头 Emm42_V5.0 闭环步进。它**不创建任务**，只在 `App_Init` 里调 `Emm42Robot_Init()`（内部转调 `Emm42_Init()`）注册 UART1 回复接收中断并清诊断计数；命令由第 5 题 `task5.c` 经 `Emm42Robot_*` 角色化接口按需下发。UART1 的板级初始化在 `BspBoard_Init` 中始终执行，置 0 只是不收回复。

## 稳健性配置（2026-07-16 加固）

- **栈溢出检测已开启**：`configCHECK_FOR_STACK_OVERFLOW 2`。之前未定义(=0)导致 `vApplicationStackOverflowHook` 形同虚设，现命中即触发（钩子会关中断、串口打印溢出的任务名、快闪 LED1）。可用 `uxTaskGetStackHighWaterMark` 查余量。
- **故障可视化**：`configASSERT` 失败转调 `vAssertCalled`（main.c），StackOverflow/MallocFailed 钩子也一样——关中断后串口打印原因(`FATAL: ...`)并快闪 LED1，不再静默死循环。故障处理里只用不加锁的 `BspUart0_SendByte`（互斥量在关中断上下文会死锁）。
- **IMU 初始化非阻塞**：IMU 任务首次 init 失败后不再死等，转入主循环每秒重试（`APP_IMU_REINIT_DIVIDER`）；期间主循环照常 100Hz 运行、5Hz 打印，激光 D1 等遥测正常输出——解决"IMU 坏了连激光也发不出"的耦合。IMU 未就绪时打印精简行 `IMU ---(retry) D1=..`。
- **其它打磨项**：UART2 RX ISR 加溢出(OVRERR)清标志；`g_motorDiag` 写/读都用 `taskENTER_CRITICAL` 取一致快照；OLED 改局部刷新(前 3 页)；`configUSE_TIMERS=0` 去掉空转的软件定时器任务省 ~1KB。
- **勿删的移植层宏**：`FreeRTOSConfig.h` 的 `configENABLE_MPU/TRUSTZONE/FPB/RUN_FREERTOS_SECURE_ONLY` 是本 SDK 的 ARM_CM0(统一 MPU 支持)移植层强制要求的，`portmacro.h` 会 `#error`；`SECURE_ONLY=1` 对无 TrustZone 的 M0+ 是正确的单映像配置，别当"残留"删。

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
