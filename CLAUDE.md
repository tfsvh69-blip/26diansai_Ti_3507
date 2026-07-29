# CLAUDE.md

本文件为本仓库的 AI 协作者提供工作约定。

## 最高优先级与文档同步

- 每次阅读、分析、修改本仓库前，必须先阅读根目录的 `CLAUDE.md`，再阅读根目录的 `AGENTS.md`；两份文件共同定义协作规则。
- `CLAUDE.md` 记录详细工程约定，`AGENTS.md` 记录简明的仓库协作指南。修改任一文件中涉及编码、构建、硬件、任务、测试或协作流程的规则时，必须在同一次改动中同步更新 `CLAUDE.md` 和 `AGENTS.md`，保持含义一致。
- 新增或修改的代码注释、README、`docs/` 文档、协作指南及其他面向开发者的说明统一使用中文；命令、文件路径、代码标识符、协议字段和必要的专有名词除外。

## 编码约定

- 代码注释统一使用中文。
- 关键逻辑、硬件操作、中断处理、任务入口、状态切换和安全保护必须写必要注释；简单自明的语句不需要堆无意义注释。

## 构建方式

本工程使用 **Keil uVision**，工程文件位于 [empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx](empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx)。

- 用 Keil 打开工程，`Build (F7)` 编译，`Download` 烧录到板子。
- Keil 的 `BeforeMake` 已关闭 `syscfg.bat`，**不需要安装 TI SysConfig** 也可以编译。
- 烧录后打开串口助手（115200 8N1，无流控），连接 PA10=TX、PA11=RX，应依次看到 `BOOT: board init ok`、`BOOT: start scheduler`，随后 PB25(LED1) 每 300ms 闪烁。

## 项目架构

**芯片**：Texas Instruments MSPM0G3507（Cortex-M0+），主频 80 MHz。  
**RTOS**：FreeRTOS V11.3.0，ARM_CM0 移植层，heap_4.c，tick 1000 Hz（1 ms）。

### 目录分层

| 目录 | 职责 |
|---|---|
| `empty/app/` | FreeRTOS 任务创建、业务流程、控制状态机；`app_robot_core` 为 6 道题目 dispatch 登记模块 |
| `empty/app/tasks/` | **各赛题业务代码**：第 N 题 = `taskN.c`，每题一套状态机骨架（`OnEnter/OnLoop/OnExit`）；`app_tasks.h` 声明钩子并含开发说明 |
| `empty/bsp/` | 板级外设初始化、GPIO、UART、延时硬件封装 |
| `empty/bsp/board/` | `ti_msp_dl_config.c/h` 手写板级 DriverLib 初始化（不由 SysConfig 生成） |
| `empty/module/` | 可复用模块（IMU、OLED） |
| `empty/algo/` | 纯算法（PID、角度工具、滤波、运动学），不依赖硬件；任务层只保留可调参数宏 |
| `empty/common/` | 任务/FreeRTOS 配置宏（`app_config.h`）、消息定义 |
| `empty/third_party/FreeRTOS/` | FreeRTOS 内核源码 |
| `empty/third_party/ti_driverlib/` | TI DriverLib |
| `empty/third_party/st_lsm6dsv16x/` | ST 官方寄存器驱动（供 `module/imu` 封装调用） |
| `empty/docs/` | 项目文档、任务表、消息表、接线表、AI 维护记录 |

### 启动流程

`main()` → `BspBoard_Init()` → `App_Init()`（创建 FreeRTOS 任务）→ `vTaskStartScheduler()`

**不要调用 `SYSCFG_DL_init()`** 做总初始化，它会调用 `SYSCFG_DL_SYSTICK_init()` 与 FreeRTOS SysTick 冲突；应使用 `BspBoard_Init()`。

### 当前 FreeRTOS 任务

任务周期、栈大小、优先级集中在 [empty/common/app_config.h](empty/common/app_config.h)。

| 任务名 | 文件 | 周期 | 说明 |
|---|---|---|---|
| `LED1` | `app/app_led_task.c` | 300 ms | LED1(PB25) 心跳灯，用于判断 FreeRTOS 是否正常调度 |
| `UART0TX` | `app/app_uart_test_task.c` | 10 ms 轮询 | UART0 接收回显，收到非换行字符返回 `UART RX OK` |
| `UIMENU` | `app/app_ui_task.c` | 30 ms 轮询 | **OLED 题目菜单 UI**：4 键(K1上/K2下/K3确认/K4返回)选题并进入运行界面，任一按键均短促嘀声（~2ms 忙等后立即关断），**独占 OLED 与 KEY1~4**；题目业务委托 `app_robot_core` → `app/tasks/taskN.c` 的 `OnEnter/OnLoop/OnExit`。当前 5 道题均为硬件/函数测试（非正式赛题）：task1 `DIR TEST` 四轮方向核对、task2 `ARC TEST` 差速圆弧、task3 `GYRO 90L` 陀螺仪闭环左转 90°（✅ 已调通）、task4 `SERVO SWP` 四路舵机 2s 间隔 0°↔270° 翻转、task5 `EMM VEL` 张大头 Emm42_V5.0 闭环步进【方向标定测试】（UART1，3 路角色：摆杆高低调节/左轮/右轮，地址 1/2/3；依次单独让 ID1→ID2→ID3 以正 RPM 转 3s 再停 1s，不循环，供肉眼观察实际方向后反馈标定）；task6 骨架待填。M1/M2 已全局取反标定 |
| `IMU100Hz` | `app/app_imu_uart_task.c` | 10 ms | 读取 ATK-MS6DSV 姿态 + 追加激光测距1(D1)，按 5Hz 整行输出 Roll/Pitch/Yaw/加减速度/D1 |
| `MOTORTEST` | `app/app_motor_test_task.c` | 20 ms 轮询 | **【默认禁用】** KEY1/KEY2 让 4 个电机（各自独立接口同时下发）正/反转 2 圈测试（按键已让给 UIMENU） |
| `SERVOSWEEP` | `app/app_servo_test_task.c` | 20 ms | **【默认禁用】4 个舵机各自独立错相摆动**（800~2200us，无按键）；单控用 `BspServo_SetPulseUs(id,us)` |
| `PERIPH` | `app/app_periph_test_task.c` | 500 ms | **【默认禁用】** OLED 已交给 UIMENU（互斥防抢屏）；原为外设验证：OLED/LED2/LED3/蜂鸣器 |

> 激光测距1（UART2）不是任务，而是 **UART2 RX 中断**逐字节喂 `module/laser` 的 LD14 解析器；距离由 `IMU100Hz` 任务读取并随整行输出。
> 小球检测（UART0）同样不是任务，而是 **UART0 RX 中断**逐字节喂 `module/vision` 的 `$BALL` 解析器（上位机→下位机，NMEA+XOR）；结果由 `UIMENU` 读取显示在 OLED 右侧文字面板。见 [empty/docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md) 的 `$BALL` 报文小节。
> 题目菜单 UI 见 [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md) 的「OLED 题目菜单 UI」小节；**题目业务逻辑逐题填 [empty/app/tasks/](empty/app/tasks/) 的 `taskN.c`**（第 N 题 = `taskN.c`），题名/题数登记在 `app_robot_core.c` 的 `s_robotTasks[]`。四路步进电机为**各自独立**驱动（`bsp_motor.h`，RPM 单位、带符号定方向）。OLED 当前只能显示 ASCII（无中文字库）。

### 功能总开关（按需启用外设）

[empty/common/app_config.h](empty/common/app_config.h) 顶部有一组 `APP_FEATURE_*`（1=启用/0=禁用），`App_Init` 用它门控各任务创建。用开发板时把不需要的外设置 0 即可（不创建任务、不占 CPU、不刷串口；板级硬件初始化仍保留）：`APP_FEATURE_LED_HEARTBEAT / UART_ECHO / UI_MENU / PERIPH_OLED / SERVO / MOTOR / IMU / IMU_UART_LOG / LASER / BALL_VISION / LINE_TRACK / RELAY / RELAY_SELFTEST / EMM42`。**当前默认**：`UI_MENU=1`（题目菜单）；因 UI 独占 OLED 与按键，`PERIPH_OLED / SERVO / MOTOR` 默认置 0。`UART_ECHO=0`（关接收自检）、`IMU_UART_LOG=0`（IMU 不打印串口遥测——串口彻底静默；但 IMU 读取 + Yaw 快照照常运行供 OLED 状态栏）。调试时把这两个改回 1 即可恢复串口输出。`BALL_VISION=1`（UART0 RX 中断解析上位机 `$BALL` 报文，OLED 右侧文字面板显示）——它与 `UART_ECHO` 争用 UART0 RX，二者互斥（同时置 1 编译期 `#error` 拦截）。`LINE_TRACK=1`（7 路灰度循迹 PB17~PB23，OLED 菜单右半显示状态）。`EMM42=1`（张大头 Emm42_V5.0 闭环步进：UART1 板级初始化 + 注册回复接收中断，当前总线上 3 台设备（地址 1/2/3 = 摆杆高低调节/左轮/右轮），命令由第 5 题 `task5.c` 经 `module/emm42/emm42_robot.h` 的角色化接口 `Emm42Robot_*` 下发（内部再转发到 `emm42_v5.h` 的按地址协议接口）；它不创建任务，仅注册中断。方向/差速运动学尚未标定，见 `emm42_robot.h` 文件头说明）。继电器开关**已解耦**：`RELAY=1`（继电器 PA24 功能——板级初始化 + OLED 状态栏 `R:ON/OFF` 显示 + 对外接口 `BspRelay_*` 可直接调用），`RELAY_SELFTEST=0`（每 2s 自动切换的自检任务 `RELAYTEST` 默认关；正常运行继电器由业务代码经 `bsp_relay` 接口按需控制、不自动切换，上电自检时才置 1）。**`IMU=0`、`LASER=0`、`NRF24_TX_TEST=0`（2026-07-29 临时关闭，减少 CPU/中断占用，需要时改回 1 即可，代码逻辑未删改）**：关闭后 `IMU100Hz`/`NRF24TX` 任务不创建、UART2 激光 RX 中断不使能，OLED 状态栏 Yaw/距离显示 `---`；题目三 `GYRO 90L` 依赖 IMU，此开关关闭期间无法测试。

修改或新增任务后必须同步更新 [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md)。

### EMM42 方向标定状态

- 角色层 `module/emm42/emm42_robot.c` 统一处理 Emm42 正方向：ID2（左轮）直通，ID3（右轮）取反，ID1（摆杆）尚待实机确认且暂按直通。
- 题目五 `EMM VEL` 仍按 ID1→ID2→ID3 依次以正 RPM 测试；每路使能后固定等待约 300ms 再下发速度帧，保证包括首路 ID1 在内的驱动器有处理使能命令的时间。后续确认 ID1 方向时，只修改角色层标定表，不修改 `task5.c`。
- 左右轮差速运动学尚未实现。

### 循迹通道状态

- 灰度循迹已启用 8 路：H6 的 LINE1~LINE8 分别接 PB17~PB24；`bsp_line` 位图 bit0~bit7 对应 LINE1~LINE8。
- 硬件物理左→右为 LINE8→LINE1，OLED 菜单右半下部同步显示 `87654321` 及 bit7→bit0 的对应 8 位状态；当前模块识别到线为低电平，`BSP_LINE_ACTIVE_LOW=1` 归一化为 `1`=识别到线；PB17~PB24 均非 5V 容忍，必须使用 3.3V 信号电平。

### 题目二正式循迹（已上车测试通过，进入稳定迭代阶段）

- 题目二当前为 `LINE PID`：用 8 路灰度循迹经 UART1 控制 Emm42 的 ID2 左轮与 ID3 右轮前进，ID1 摆杆不参与；PID 增益、基础速度等具体数值由用户在实车上持续调参，以 `app/tasks/task2.c` 顶部当前值为准，不要以文档里的旧数值为准。
- **入场时序（状态机）**：`Task2_OnEnter()` 只复位变量、不直接下发 Emm42 命令；实际动作在 `OnLoop` 的状态机里节拍化完成：`T2_STATE_RESET_DISABLE`（先失能 ID2/ID3 清残留状态）→ `T2_STATE_RESET_WAIT`（等 `T2_RESET_SETTLE_TICKS`）→ 依次使能 ID2、ID3，各使能后等 `T2_ENABLE_SETTLE_TICKS` → `T2_STATE_RUN`。**`T2_ENABLE_SETTLE_TICKS` 已实测验证**：3 拍(90ms) 会有约一半概率使能不生效（驱动器来不及处理），6 拍(180ms) 才稳定，不要再往下调；`T2_RESET_SETTLE_TICKS` 没有类似的失败实测数据，是偏保守加的，需要压缩入场延时时优先调这个。
- 误差先做一阶低通滤波（`T2_ERROR_FILTER_ALPHA`）再叠加死区 `T2_ERROR_DEADBAND`（`|误差|` 小于此值直接当 0），抑制离散传感器命中路数在相邻两档（如 2 路/3 路）来回跳变导致的中心附近小幅摆动；转向输出越大基础速度按比例自动降低（`T2_CORNER_SLOWDOWN_GAIN`/`T2_MIN_BASE_RPM`），实现直道快、弯道稳。速度命令加速度档位 `T2_EMM_ACC` 用非 0 曲线档位（说明书 §6.3.1）——0 会立即生效但实测车身会突兀抖动，任务层不再叠加软件斜坡，平滑完全交给这个硬件曲线档位。
- 差速安全限幅 `Task2_ClampWheelPair()` 对左右轮目标 RPM 做"整体平移"而非各自独立 clamp：独立 clamp 会在外侧轮触顶时压扁差速、内侧轮却不受影响，导致弯道实际转向权限远小于 PID 算出的量（高速冲出弯道的典型根因）；`T2_MAX_WHEEL_RPM` 须 `>= T2_BASE_RPM` 且总跨度 `>= 2*T2_MAX_STEER_RPM`。
- 终点检测：先连续 `T2_FINISH_ARM_TICKS` 拍命中细线确认已离开出发线才"武装"终点检测，武装后再连续 `T2_FINISH_HIT_TICKS` 拍命中路数 `>= T2_FINISH_HIT_MIN`（当前 6，含 7/8 路）判定跑完一圈，硬停车锁定并等待 K4 手动退出，避免出发瞬间被误判为终点。
- K4 退出与终点急停都需要背靠背给 ID2/ID3 下发多帧（停/停/失能/失能），帧间用 `vTaskDelay(T2_EMM_CMD_GAP_MS)` 隔开，避免共享 UART1 总线互相干扰丢帧；ID1 摆杆不属于本题，不发送任何命令。
- **定时减速**：秒表校准后时间达到 `T2_DECEL_START_MS`（固定 14500ms）后，基础速度按 `T2_BASE_RPM − T2_DECEL_GRADIENT_RPM_PER_SEC×超时秒数` 线性下降到 `T2_DECEL_MIN_RPM` 为止，与转弯减速算出的基础速度取更小值生效；用于接近预期完赛时间时主动放慢，也给跑得比预期久的情况兜底。
- **OLED 秒表**：`Task2_GetUiStatus()` 返回 `"T:12.3s"`，`s_elapsedTicks` 从 `OnEnter` 起每拍累加、到 `T2_STATE_FINISHED` 后停止累加（定格并追加 `" DONE"`），显示位置复用题目五的 `UI_RUN_NAME_Y` 行（`app_ui_task.c` 的 `UI_TASK2_INDEX`）。显示时长会乘以 `T2_STOPWATCH_CAL_SCALE` 校准系数——实测计时比真实时间偏快（怀疑是 FreeRTOS tick 依赖的主频跟工程假设的 80MHz 有偏差，根因未定位），先用系数硬补偿；如果继续偏，按"新系数 = 当前系数 × (最新实测秒数/最新显示秒数)"滚动修正，不用每次从 1.0 重算。

### 当前硬件连接（v1.1，以 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md` 为准）

| 硬件 | 引脚 | 说明 |
|---|---|---|
| LED1 | PB25 | 高电平点亮，心跳灯 |
| LED2 | PA7 | 高电平点亮，外设测试任务翻转 |
| LED3 | PB12 | 高电平点亮，外设测试任务翻转 |
| 蜂鸣器 | PA15 | 有源蜂鸣器，高电平响，普通 GPIO |
| OLED SCL | PB9 | 板载 OLED GPIO 软件 I2C |
| OLED SDA | PB8 | 板载 OLED GPIO 软件 I2C |
| 舵机1~4 PWM | PA8/PA9/PB4/PA12 | TIMA0_CCP0~3，50Hz PWM |
| 电机1 STEP/DIR | PB10/PB11 | TIMG0_CCP0 / GPIO，左前轮；BSP 已取反，逻辑正向时 DIR 高（各路独立） |
| 电机2 STEP/DIR | PB6/PB7 | TIMG8_CCP0 / GPIO，左后轮；BSP 已取反，逻辑正向时 DIR 高（独立调速调向） |
| 电机3 STEP/DIR | PB13/PB14 | TIMG12_CCP0 / GPIO，右前轮；逻辑正向时 DIR 低（独立调速调向） |
| 电机4 STEP/DIR | PB26/PB27 | TIMG6_CCP0 / GPIO，右后轮；逻辑正向时 DIR 低（独立调速调向） |
| TMC 使能 ENN | PA13 | 低有效，四路共用 |
| TMC 细分 MS1 | PB0 | 四路共用细分 |
| TMC 细分 MS2 | PB1 | 四路共用细分 |
| 按键 KEY1~4 | PA28/PA31/PA30/PA29 | 按下接地，内部上拉 |
| UART0 TX | PA10 | MFCLK 4MHz，115200 8N1 |
| UART0 RX | PA11 | — |
| 张大头 UART1 TX | PA17 | MFCLK 4MHz，115200 8N1；→ Emm42_V5.0 驱动器 RX（v1.1 排针 H7） |
| 张大头 UART1 RX | PB5 | ← 驱动器 TX；当前 3 台设备共用此总线（地址1=摆杆高低调节、2=左轮、3=右轮）；⚠️多台驱动器 TX 并联（接线文档风险 R1，原文档述两台，现为三台风险等比放大），需外部肖特基线与后才可同时接 |
| 激光测距1 UART2 TX | PB15 | MFCLK 4MHz，230400 8N1（激光只收不发，一般不用） |
| 激光测距1 UART2 RX | PB16 | 接激光模块 TX，RX 中断逐字节喂 LD14 解析器；⚠️非 5V 容忍，激光 TX 若 5V 先量电平 |
| ATK-MS6DSV SCL | PB2 | GPIO 软件 I2C，已外接上拉 |
| ATK-MS6DSV SDA | PB3 | GPIO 软件 I2C，SA0 接地，7bit 地址 `0x6A` |
| ATK-MS6DSV INT | PA16 | GPIO 输入，下拉 |

修改引脚或新增硬件后必须同步更新 [empty/docs/HARDWARE_WIRING.md](empty/docs/HARDWARE_WIRING.md)。

## 关键约定

### 硬件接线图优先（铁律）

任何涉及引脚、外设复用、接线的操作（判断某引脚接了什么、新增/修改引脚用途、分配指示灯/信号脚等），**必须先查阅权威硬件接线图** [`pcb引脚配置文档/v1.1/机器人控制板_接线说明.md`](pcb引脚配置文档/v1.1/机器人控制板_接线说明.md) 及同目录网表文件，确认该引脚在**当前硬件版本**上的实际物理连接和功能后再动手；不得凭代码现状、注释、git 历史或记忆推断引脚用途。

> 反例（2026-07-15）：PB22 在 git 历史里曾是 LED1，代码与注释也残留该痕迹，但它在 v1.1 实际是 `LINE6`、已接到扩展板承担其它功能，不能想当然拿去做心跳灯。凡"复用某个看似空闲的引脚"之前，务必先在接线图里核实其真实连接。

### 禁用/慎用引脚

以下引脚属于核心板特殊功能引脚，**不得随意使用**；确需使用须先说明原因、风险，并等待人工确认：

A23、A21、A20、A19、A18、A11、A10、A5、A6、A4、A3、A2

> 当前 PA10/PA11（UART0）已按用户确认使用，属于例外。

### FreeRTOS 规则

- ISR 中只做快速处理；调用 FreeRTOS API 必须使用 `FromISR` 版本。
- 任务周期循环用 `vTaskDelay` / `vTaskDelayUntil`，不要长时间忙等。
- 不要在高频任务、控制环或中断中动态分配内存。
- 控制输出前必须检查 enable、离线、超时和限幅。

### UART 配置原则

- UART0 固定使用 MFCLK 4MHz、115200 8N1，分频 `IBRD=2`、`FBRD=11`。
- 新增或修改 UART 前先阅读 [empty/docs/UART_DEBUG_GUIDE.md](empty/docs/UART_DEBUG_GUIDE.md)。
- 乱码排查优先怀疑"时钟源或分频不对"，不要先猜文本编码或反复试波特率。
- UART0 多任务共享用**递归互斥量**保证整行原子（`BspUart0_Init` 在 `BspBoard_Init` 中于调度器启动前创建）：`BspUart0_SendString` 单次调用自动加锁，多段拼接用 `BspUart0_Lock/Unlock` 包裹。**不再挂起调度器**，发送整行时其它任务照常时间片轮转，避免全局卡顿；但仍需注意单串口日志总频率。

### SysConfig 解耦

- `bsp/board/empty.syscfg` 仅作历史参考，不参与构建。
- `bsp/board/ti_msp_dl_config.c/h` 手写维护，保留 `SYSCFG_DL_*` 函数名只是为了兼容现有 BSP 调用。

### IMU 注意事项

- `APP_IMU_I2C_PIN_TEST_ENABLE`（在 `common/app_config.h`）置 1 时，`IMU100Hz` 任务不读 IMU，只翻转 PB2/PB3 并读 GPIO 电平用于硬件排查；排查完必须改回 0。
- LSM6DSV16X 无精确 100Hz 档位，当前 ODR 配为 120Hz；`FIFO=0/1/2` 小范围跳动正常。
- `ATK_MS6DSV_USE_BOOT_RESET` 默认为 0，跳过 boot reset，原因是实测 `RESET_SET` 会导致 SDA 被拉低。
- 软件 I2C 读最后一字节必须回 NACK 再发 STOP，否则 LSM6DSV16X 会持续占用 SDA。

### IMU Yaw（偏航角）行为约定（已实测）

- **范围**：−180.00° ~ +180.00°（内部厘度 0.01°，即 −18000 ~ +18000）。
- **转向与 Yaw 变化方向**：小车**左转 → Yaw 递减**（如 180° → 150° → 0° → −179°）；右转 → Yaw 递增。
- **复位初始值不确定**：每次 MCU 复位后，IMU SFLP 融合初始 Yaw 不同，**不影响相对角度闭环**——只需记录起始 Yaw 计算偏移量（如目标 = 起始 − 90°），不依赖绝对值。
- **跨界跳变**：Yaw 在 ±180° 边界跳变（−179° ↔ +179°），计算角度差必须用最短路径差值算法（如 `task3.c::AngleDiffCd()`），不可直接减法。
- **漂移**：无磁力计/外部参考时，Yaw 长期会漂移；短时间（几秒到几十秒）内相对精度足够用于 90°/180° 转弯闭环。

### 跨模块通信

当前暂未启用跨任务消息。后续新增时优先使用 FreeRTOS queue / event group / stream buffer，并在 [empty/docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md) 中记录。应用之间不要通过全局变量或直接包含头文件传递业务数据。

## 修改代码后自查

1. 是否符合分层边界（app/module/algo/bsp/common）？
2. 是否遗漏 FreeRTOS 同步、中断安全、`FromISR` 版本？
3. 是否误用了核心板特殊功能引脚？
4. 是否新增了未记录的任务、消息、模式或硬件接线？
5. 是否需要更新 `docs/` 下的 `FREERTOS_TASKS.md`、`MESSAGE_LIST.md`、`HARDWARE_WIRING.md`？
6. 关键代码是否写了必要的中文注释？

## 维护文档索引

| 文档 | 用途 |
|---|---|
| [docs/PROJECT_CONTEXT.md](empty/docs/PROJECT_CONTEXT.md) | 当前架构、硬件、主要模块、关键约定 |
| [docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md) | 任务名、周期、优先级、栈大小、输入输出 |
| [docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md) | 跨任务消息定义（当前为空） |
| [docs/HARDWARE_WIRING.md](empty/docs/HARDWARE_WIRING.md) | 硬件接线、引脚、电气说明、排查备注 |
| [docs/UART_DEBUG_GUIDE.md](empty/docs/UART_DEBUG_GUIDE.md) | UART 配置原则与乱码排查流程 |
| [docs/AI_MEMORY.md](empty/docs/AI_MEMORY.md) | 用户固定要求、重要决策、踩坑记录 |
