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
| 40MHz 晶振 | HFXIN | PA5 | `GPIO_HFXIN_IOMUX` | `IOMUX_PINCM10`，package pin 45 | 核心板 X1（40MHz ±10ppm，15pF），配为模拟功能，作 SYSPLL 参考倍频到 80MHz | 起振成功串口打印 `BOOT: MCLK 80MHz <- HFXT 40MHz OK` |
| 40MHz 晶振 | HFXOUT | PA6 | `GPIO_HFXOUT_IOMUX` | `IOMUX_PINCM11`，package pin 46 | 与 PA5 一起接晶振，配为模拟功能 | 起振失败自动回退内部 SYSOSC，串口打印 `BOOT: HFXT FAIL, ...` |
| LED1 | LED1 控制 | PB25 | `LED_LED1_PIN` | `IOMUX_PINCM56` | GPIO 输出，v1.1 高电平点亮（IO→阳极） | FreeRTOS 启动后每 300ms 翻转一次，用作系统心跳 |
| LED2 | LED2 控制 | PA7 | `LED_LED2_PIN` | `IOMUX_PINCM12` | GPIO 输出，v1.1 高电平点亮 | 外设测试任务每 500ms 翻转 |
| LED3 | LED3 控制 | PB12 | `LED_LED3_PIN` | `IOMUX_PINCM29` | GPIO 输出，v1.1 高电平点亮 | 外设测试任务每 500ms 翻转（与 LED2 相位相反） |
| 蜂鸣器 | BUZZER | PA15 | `BUZZER_PIN` | `IOMUX_PINCM37` | GPIO 输出，有源蜂鸣器高电平响（经 Q3 驱动），普通 GPIO 无需 PWM | 外设测试任务每 5s 通一拍做通断测试；上电自检短响一次 |
| OLED | SCL | PB9 | `OLED_PIN_SCL_PIN` | `IOMUX_PINCM26` | 板载 OLED 软件 I2C SCL，推挽输出 | 显示 `3507 v1.1 TEST` 及 tick/秒/蜂鸣器状态 |
| OLED | SDA | PB8 | `OLED_PIN_SDA_PIN` | `IOMUX_PINCM25` | 板载 OLED 软件 I2C SDA，推挽输出 | 约定 SCL=PB9/SDA=PB8，若显示异常可对调 |
| 电机1 | M1_STEP | PB10 | `MOTOR1_STEP_PIN` | `IOMUX_PINCM27` / `IOMUX_PINCM27_PF_TIMG0_CCP0`，U2.49 | TIMG0_CCP0 硬件定时器输出，连续旋转+梯形加减速可变频方波（L1≈6.4kHz ~ L5≈32kHz）；TIMG0 ZERO 中断在线调速 | 运行期间 PB10 有连续方波；停止时减速到起步速度后定时器停止 |
| 电机1 | M1_DIR | PB11 | `MOTOR1_DIR_PIN` | `IOMUX_PINCM28`，U2.47 | GPIO 输出，低=正向，高=反向 | 方向不对就翻转该电平或调线序 |
| 电机2 | M2_STEP | PB6 | `MOTOR2_STEP_PIN` | `IOMUX_PINCM23` / `IOMUX_PINCM23_PF_TIMG8_CCP0` | TIMG8_CCP0 方波，跟随电机1同频 | 四电机测试时应与 M1 同步出脉冲 |
| 电机2 | M2_DIR | PB7 | `MOTOR2_DIR_PIN` | `IOMUX_PINCM24` | GPIO 输出，低=正向 | 四路 DIR 由 `BspMotorAll_SetDir` 一起设 |
| 电机3 | M3_STEP | PB13 | `MOTOR3_STEP_PIN` | `IOMUX_PINCM30` / `IOMUX_PINCM30_PF_TIMG12_CCP0` | TIMG12_CCP0(32位) 方波，跟随电机1 | — |
| 电机3 | M3_DIR | PB14 | `MOTOR3_DIR_PIN` | `IOMUX_PINCM31` | GPIO 输出，低=正向 | — |
| 电机4 | M4_STEP | PB26 | `MOTOR4_STEP_PIN` | `IOMUX_PINCM57` / `IOMUX_PINCM57_PF_TIMG6_CCP0` | TIMG6_CCP0 方波，跟随电机1 | — |
| 电机4 | M4_DIR | PB27 | `MOTOR4_DIR_PIN` | `IOMUX_PINCM58` | GPIO 输出，低=正向 | — |
| 舵机1~4 | SERVO1~4 PWM | PA8/PA9/PB4/PA12 | `SERVO1..4_PIN` | TIMA0_CCP0~3，PINCM19/20/17/34 | 50Hz PWM，脉宽经 bsp_servo 极性补偿 | KEY3/KEY4 测试四路一起动 |
| TMC2209 | TMC_ENN | PA13 | `TMC_ENN_PIN` | `IOMUX_PINCM35`，U2.30 | GPIO 输出，低有效，四路驱动共用使能 | 高电平电机失力；测试任务会拉低使能 |
| TMC2209 | TMC_MS1 | PB0 | `TMC_MS1_PIN` | `IOMUX_PINCM13` | GPIO 输出，四路共用细分（v1.1） | 黄排针模块：MS1=1,MS2=0 → 1/32 细分 |
| TMC2209 | TMC_MS2 | PB1 | `TMC_MS2_PIN` | `IOMUX_PINCM14` | GPIO 输出，四路共用细分（v1.1） | 见引脚文档 §2.2 黄排针细分表（LL=1/8, HH=1/16, HL=1/32, LH=1/64） |
| 按键1 | KEY1 | PA28 | `KEY1_PIN` | `IOMUX_PINCM3`，U2.4 | GPIO 输入，内部上拉；一端接 GND，按下为低 | 电机1 **启停切换** |
| 按键2 | KEY2 | PA31 | `KEY2_PIN` | `IOMUX_PINCM6`，U2.6 | GPIO 输入，内部上拉；一端接 GND，按下为低 | 电机1 **换向**（运行中先减速停稳再反向） |
| 按键3 | KEY3 | PA30 | `KEY3_PIN` | `IOMUX_PINCM5`，U2.53 | GPIO 输入，内部上拉；一端接 GND，按下为低 | 电机1 **加速一档**（L1→L5） |
| 按键4 | KEY4 | PA29 | `KEY4_PIN` | `IOMUX_PINCM4` | GPIO 输入，内部上拉；一端接 GND，按下为低 | 电机1 **减速一档**（L5→L1） |
| UART0 | TX | PA10 | `GPIO_UART_0_TX_PIN` | `IOMUX_PINCM21` / `IOMUX_PINCM21_PF_UART0_TX` | UART0 发送，MFCLK/115200 8N1；PA10 属于核心板特殊功能风险引脚，已按用户确认使用 | 串口助手应收到启动提示、IMU 输出或 `UART RX OK` 回显 |
| UART0 | RX | PA11 | `GPIO_UART_0_RX_PIN` | `IOMUX_PINCM22` / `IOMUX_PINCM22_PF_UART0_RX` | UART0 接收，用于接收串口助手发来的命令；PA11 属于核心板特殊功能风险引脚 | 发送任意非换行字符后回显 `UART RX OK` |
| 激光测距1 | UART2 TX(MCU) | PB15 | `GPIO_UART_2_TX_PIN` | `IOMUX_PINCM32` / `IOMUX_PINCM32_PF_UART2_TX`，v1.1 排针 H21 | UART2 发送，MFCLK/230400 8N1；接激光模块 RX（本模块只收不发，此脚一般不用） | — |
| 激光测距1 | UART2 RX(MCU) | PB16 | `GPIO_UART_2_RX_PIN` | `IOMUX_PINCM33` / `IOMUX_PINCM33_PF_UART2_RX`，v1.1 排针 H21 | UART2 接收，接激光模块 TX；RX 中断逐字节喂 `LaserLd14` 解析器 | 串口每行 `D1=<mm>mm`；一直 `D1=---` 见下方排查 |
| ATK-MS6DSV | IMU_SCL | PB2 | `IMU_I2C_SCL_PIN` | `IOMUX_PINCM15` / `IOMUX_PINCM15_PF_I2C1_SCL`，U2.15 | **GPIO 软件 I2C SCL**（开漏模拟）；扩展板已焊 4.7k 上拉到 3.3V | 串口应输出 `IMU INIT OK`，否则优先查 SCL 是否接到 B02 |
| ATK-MS6DSV | IMU_SDA | PB3 | `IMU_I2C_SDA_PIN` | `IOMUX_PINCM16` / `IOMUX_PINCM16_PF_I2C1_SDA`，U2.17 | **GPIO 软件 I2C SDA**（开漏模拟）；SA0 接地后 7bit 地址 `0x6A`；IMU 供电 3.3V | 初始化失败码 `2` 多为 I2C ACK/接线/地址/上拉问题 |
| ATK-MS6DSV | IMU_INT | PA16 | `IMU_INT_PIN` | `IOMUX_PINCM38`，U2.67 | GPIO 输入，下拉；当前任务轮询读取电平，暂未接入 ISR | 串口每行 `INT=0/1` 反映当前 PA16 电平 |

> 当前 UART0 TX 已从 PB0 改为 PA10。PA10/PA11 均属于核心板特殊功能风险引脚，本次按用户确认使用。

## OLED / LED / 蜂鸣器 外设测试（v1.1）

- v1.1 板 OLED 恢复到板载 PB8/PB9（软件 I2C），TMC 细分改用 PB0/PB1，两者不再冲突。
- 三项验证统一由 `PERIPH` 外设测试任务完成（`app/app_periph_test_task.c`）：
  - **OLED**：`OLED_Init()` 后每 500ms 刷屏，显示 `3507 MOTOR1 v1.1`(标题) / `T:..s L:.. B:..`(运行秒+LED+蜂鸣器) / `M1:RUN FWD R3`(电机1 运行·方向·圈数，读 `g_motorDiag`)。
  - **指示灯**：LED2(PA7)、LED3(PB12) 每 500ms 交替翻转（相位相反）；LED1(PB25) 仍由心跳任务独占。
  - **蜂鸣器**：PA15 当前保持静音（已验证正常）；上电自检时短响一次。
- OLED 软件 I2C 只做推挽输出、不读 ACK，`SYSCFG_DL_GPIO_init` 已把 PB8/PB9 配为普通数字输出、空闲拉高。
- 位延时 `OLED.c::IIC_delay` 已从 10us 降到 2us，减少全屏刷新对同优先级任务（按键轮询）的忙等阻塞。
- 约定 SCL=PB9、SDA=PB8；文档未标注具体归属，若实物相反在 `ti_msp_dl_config.h` 对调两宏即可。

## 电机 / 舵机 四按键测试（KEY1/2 电机，KEY3/4 舵机）

- 步进驱动 TMC2209：四路 STEP 各占独立定时器（M1=TIMG0/PB10、M2=TIMG8/PB6、M3=TIMG12/PB13、M4=TIMG6/PB26），四路 DIR=PB11/PB7/PB14/PB27；ENN=PA13(低有效)、MS1=PB0/MS2=PB1 四路共用。
- 上电默认安全状态：ENN 拉高禁用、四路 STEP 停止、四路 DIR 正向、MS1=0/MS2=0；运行前由任务设为 MS1=1/MS2=0(1/32 细分)。
- **电机测试任务 `MOTORTEST`**（20ms 轮询 KEY1/KEY2，移动期间忽略按键）：
  - **KEY1：4 电机一起正转 2 圈**；**KEY2：4 电机一起反转 2 圈**（1/32 细分，1 圈=6400 脉冲，巡航周期 500≈1.25 圈/秒）。
  - 实现：`BspMotorAll_MoveSteps()` → 电机1(TIMG0)做梯形斜坡主控并计步，电机2/3/4 镜像同一周期跟随、同启同停，四电机同频同向"一起转"；走完自动一起减速停表。
- **舵机测试任务 `SERVOSWEEP`**（20ms，无按键）：
  - 4 舵机(PA8/PA9/PB4/PA12=TIMA0 CCP0/1/2/3)以**不同相位各自独立**在 **800~2200us** 间来回摆动，演示四路可完全独立控制；单独控制某路用 `BspServo_SetPulseUs(BSP_SERVO_x, us)`。
  - **两个坑（已修）**：① 脉宽只经 `BspServo_SetPulseUs()`（内部 `period-pulseUs` 极性补偿，勿绕过）；② 四路方向须**一次** `DL_Timer_setCCPDirection(TIMA0, CC0|CC1|CC2|CC3_OUTPUT)` 写全——该寄存器是整体覆盖，分 4 次调只有最后一次生效，会导致只有舵机4动、舵机1/2/3 信号线浮 0.3V。
  - 脉宽范围 800~2200us 是给舵机机械行程留安全余量；极值不灵是舵机限位所致，非 MCU（补偿在任何脉宽都精确）。
- **梯形加减速**（双模式并存）：
  - **位置模式**（MoveSteps）：500Hz(period=8000)起步，每脉冲 ±`MOTOR_RAMP_DELTA=4` 朝目标逼近；步数太少时自动退化为三角形曲线。ISR 每脉冲计步，自动判断加速/巡航/减速三段，走完+回起步速度后停表。
  - **连续模式**（RunContinuous，保留供后续扩展）：起步→加速到巡航→手动 RequestStop 后减速停止。
- 诊断快照 `g_motorDiag` 由 `MOTORTEST` 写入、PERIPH 在 OLED 只读显示（`app_motor_status.h`），反映四电机测试的运行/方向/圈数。
- 串口输出：上电 `MOTOR test: K1=4 motors FWD 2rev, K2=...`；每次触发 `MOTORx4 key start -> RUN FWD/REV 2 rev`，完成 `MOTORx4 done -> STOP`；每秒 `MOTORx4 DIAG run=x left=<剩余步数> per=<当前周期>`。舵机测试打印 `SERVOx4 -> posA/posB <us>`。
- 踩坑确认：步进电机**从静止直接起高频会失步**（实测 20kHz 直接起转只抖 5~10°），高速必须配加减速斜坡；速度/加速度现为 500Hz 起步 + RAMP_DELTA=4，换驱动芯片后实测运行平稳。

### 电机1调试结论（已实测可正常旋转）

- **原地剧烈抖动 = 相线圈配对错**：4 根电机线必须按"同一线圈两根"成对接入驱动 A1/A2、B1/B2，不能两个线圈各掏一根凑一对。断电用万用表电阻档量出两组导通对(同线圈约 1~10Ω，异线圈开路)即可确认。本工程实测把线序改对后电机正常旋转。
- 区分"线序错"与"共振/丢步"的方法：把步频降到 200Hz 观察——降频后能顺转多为共振/丢步；200Hz 仍只原地抖则是线圈配对错。
- **TMC2209 电流靠 VREF 电位器标定**(不接 UART)：VREF 设的是每相 RMS 电流，`Irms ≈ VREF×0.71`(常见 0.11Ω Rsense 模块，具体随模块 Rsense 变)。调法：塑料螺丝刀小步拧、边量 VREF 电压边调，最终以电机/驱动温热不烫(<60~70℃)、手捏轴有明显反抗力矩为准。本工程已由用户调到合适数值。
- **无加减速直接启动有步频上限（历史记录）**：当前已实现梯形加减速（1kHz 起步→加速到巡航→末段对称减速），此条为历史踩坑保留。若未来去掉加减速，需注意直接起高频会失步。
- 电机不转排查顺序：① VM(4S) 是否上电；② TMC2209 VREF 电流是否调到有力矩；③ ENN 是否确实拉低；④ PB10 是否有方波；⑤ A/B 相线序是否接错。

## LED1(PB25) 心跳灯

- `App_Init()` 当前启动 `AppLedTask_Init()`。
- LED1(PB25) 每 300ms 翻转一次，用于判断 FreeRTOS 调度是否正常运行。
- 若 LED1 不闪，优先排查程序是否进入 `App_Init()`、是否卡在外设初始化、是否触发 `configASSERT` 或 HardFault。

## UART0 接收回显测试

- 串口助手设置：115200 8N1，无流控。
- UART0 TX 使用 PA10，接串口模块 RX；UART0 RX 使用 PA11，接串口模块 TX。
- `main()` 在板级初始化后会先输出 `BOOT: board init ok`。
- `main()` 在应用初始化完成后会再输出 `BOOT: start scheduler`。
- 收到任意非换行字符后回显 `UART RX OK`。
- 回车和换行会被忽略，避免串口助手自动追加换行造成重复提示。
- 多段拼接的 IMU 调试日志已使用 UART0 行级锁，避免多个任务输出互相穿插；单条短字符串仍可直接发送。

## ATK-MS6DSV 姿态输出

> **重要提醒 1（临时锁死）**：全部设备上电状态下烧录后，IMU 常读不出来（代码正确也一样）。
> 根因是烧录期间 IMU 未断电，芯片内部状态机未经历上电复位（POR），残留状态无法通过软件复位恢复。
> **解决：彻底断电再重新上电。** ⚠️ v1.1 板 IMU 的 3V3 由开发板 LDO（来自 +5V）供，+5V 有 XL4015(电池) 和 Type-C **两路**（风险 R3）；只掉一路、只按 RST、只重烧都不算断电，**两路都拔、等几秒**才让 IMU 经历 POR。
>
> **重要提醒 2（永久损坏，2026-07-16 实测）**：若"两路电全拔彻底断电"后仍 `INIT FAIL:2 STEP=WHOAMI`、`0x6A/0x6B=ERR`、`SCAN none`、总线 `SCL=1 SDA=1`，说明器件在总线上完全不 ACK，多半是**模块硬件坏了**——软件救不了（连地址都不 ACK 就发不进任何命令）。**直接换一颗新 MS6DSV 模块验证**，本工程曾据此换模块后立即恢复正常。
>
> **排查顺序**：两路电全拔断电重启 → 仍不行且引脚/供电确认无误 → 换新模块。

- 模块：正点原子 ATK-MS6DSV，核心器件 LSM6DSV16X。
- 接线：SCL=PB2/B02，SDA=PB3/B03，INT=PA16/A16，SA0 接地。
- I2C：**当前使用 GPIO 软件 I2C**（开漏模拟方式，400 个 CPU 周期延迟），端口层 `bsp_imu_port.c` 首次访问时关闭 I2C1 硬件控制器，将 PB2/PB3 切换为 GPIO 输入上拉 + 开漏输出模式；SA0 接地时 7bit 地址为 `0x6A`。
- **【防护·勿回退】`SYSCFG_DL_I2C_1_init()` 只做时钟/FIFO 配置、不调用 `DL_I2C_enableController()`**：否则上电到 IMU 任务接管软件 I2C 的空窗期内，硬件 I2C 会接管 PB2/PB3，可能在总线上打 glitch 把传感器推入锁死态（反复冲击疑似加速永久损坏）。让 PB2/PB3 全程由 GPIO 软件 I2C 控制，别为兼容旧接口把 `enableController` 加回去。
- 软件 I2C 使用标准重复起始时序：写寄存器地址 → 重复起始 → 读 N 字节，最后一字节 NACK 后 STOP（避免 LSM6DSV16X 继续保持发送态拉低 SDA）。
- 所有 I2C 等待循环均带超时，总线被拉死时会超时返回失败而非卡任务。
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

## 激光测距1（LD14，UART2）

- 模块：单点激光测距（协议移植自参考工程 `26RuiKang-AppleRobot-STM32-5.1` 的 `lidar_manager`/`ld14`）。
- 接线（v1.1 排针 **H21**，脚序 1=+5V / 2=GND / 3=MCU RX / 4=MCU TX）：模块 **TX → PB16(MCU RX)**，模块 RX → PB15(MCU TX)。**只用到 MCU RX**，激光持续外发数据帧，MCU 只接收。
- 串口参数：**230400 8N1**（依据参考工程 SC16 通道A 配置推定）。MFCLK 4MHz + 8x 过采样，`IBRD=2`/`FBRD=11`，实测波特率误差 -0.08%。**若实物模块波特率不同，改 `ti_msp_dl_config.h` 的 `UART_2_BAUD_RATE` 及分频即可**。
- 协议：195 字节定长帧，帧头 `0xAA×4` + 命令字 `0x02` + 12 点(每点 15 字节，偏移 10 起) + 时间戳 + 校验和（前 194 字节累加低 8 位，兼容含/不含帧头两种口径）。解析后取 12 点非零距离**平均**作为单值距离。
- 接收路径：**UART2 RX 中断**（`UART2_IRQHandler` in `bsp_uart.c`）逐字节喂 `LaserLd14_FeedByte()`。230400 连续流下任务轮询来不及，故必须走中断；RX FIFO 阈值设为 1 字节、中断内循环取空。ISR 内不调用 FreeRTOS API。
- 输出：距离由 `IMU100Hz` 任务在打印整行时一并输出，格式 `... D1=<mm>mm ...`，与陀螺仪数据同一行、5Hz 刷新（见 `FREERTOS_TASKS.md`）。
- 排查：
  - 一直 `D1=---`：说明没收到过有效帧。优先查 ① 模块是否上电（H21-1=+5V、H21-2=GND 共地）；② 模块 TX 是否接到 **PB16**；③ 波特率是否真的 230400（不符则改宏）；④ 是否与 H21 上的板载/其它设备 TX 冲突（排针与板载设备二选一）。
  - `D1` 有值但明显不对/跳变：多为波特率略偏或校验口径问题，可临时打印 `LaserLd14Data_t` 的 `frameOkCnt`/`crcErrCnt`/`rxBytes` 定位是"没在收"还是"收了但过不了校验"。
- ⚠️ **电平风险（原理图风险 R2）**：激光模块 TX 若为 5V 电平，而 PB16 **不是 5V 容忍引脚**，长期直连有损坏风险。接线前务必先量模块 TX 输出电平；若为 5V 需串分压/加电平转换。

## IMU I2C 引脚物理测试

- 当前已关闭：`common/app_config.h` 中 `APP_IMU_I2C_PIN_TEST_ENABLE` 为 `0`。
- 该模式下不初始化 IMU，串口输出 `IMU PINTEST PB2=SCL PB3=SDA, SET then READ GPIO DIN`。
- PB2/SCL 与 PB3/SDA 会每 500ms 交替“拉低/释放高”，随后 MCU 直接读取 GPIO 输入寄存器并输出 `READ SCL=... SDA=...`。
- 若 `SET` 为 `1` 但 `READ` 为 `0`，表示该线释放后仍被外部器件、短路或接线拉低；若 `SET` 为 `0` 但 `READ` 为 `1`，表示 MCU 没能拉低该 GPIO 或 pinmux/端口配置异常。
- 测完必须把 `APP_IMU_I2C_PIN_TEST_ENABLE` 改回 `0`，否则不会进入正常 IMU 读取流程。
- 2026-06-18 用户实测串口 `SET SCL=0 SDA=1 READ SCL=0 SDA=1` 与 `SET SCL=1 SDA=0 READ SCL=1 SDA=0` 持续一致，说明 MCU 端 PB2/PB3 GPIO 读写和开漏释放行为正常；该测试不能单独证明外部板口到 IMU 模块线缆完全无误。

## 系统时钟 / 40MHz 外部晶振（HFXT）

- 核心板 X1 为 40MHz 无源晶振（±10ppm，15pF），接 PA5/HFXIN、PA6/HFXOUT，配 C34/C35=15pF。
- **当前 MCLK 80MHz 由该 40MHz 晶振经 SYSPLL 倍频锁定**：40MHz →PDIV/2=20MHz →QDIV×8=160MHz VCO →/2=80MHz。之前版本 `disableHFXT()` 未用晶振、全跑内部 SYSOSC，现已切换。
- **起振失败自动回退**：`SYSCFG_DL_tryStartHFXT()` 带超时轮询启用 HFXT（monitor=false，不进库内死等循环），超时即回退内部 SYSOSC（32MHz→16MHz→×10=160MHz→80MHz），保证晶振虚焊/异常时整机仍以 80MHz 运行、串口有输出，不会卡死变砖。
- 启动时串口打印实际时钟源：`BOOT: MCLK 80MHz <- HFXT 40MHz OK`（晶振正常）或 `BOOT: HFXT FAIL, MCLK 80MHz <- internal SYSOSC`（回退）。全局标志 `g_sysClockUsingHFXT`。
- **外设时钟不受影响**：UART0、I2C1、步进定时器仍走内部 MFCLK 4MHz，波特率/分频常数与本次切换解耦（UART 115200 的 IBRD=2/FBRD=11 不变）。晶振只锁定 CPU 主频，不改 FreeRTOS 节拍。
- 若打印 `HFXT FAIL`：优先查 X1 焊接、C34/C35 负载电容、PA5/PA6 是否被占用；此时系统功能正常，只是主频精度回到内部 RC（约 ±1%）。

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
