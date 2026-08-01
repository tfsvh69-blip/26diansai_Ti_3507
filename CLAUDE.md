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
| `UIMENU` | `app/app_ui_task.c` | 30 ms 轮询 | **OLED 题目菜单 UI**：4 键(K1上/K2下/K3确认/K4返回)选题并进入运行界面，任一按键均短促嘀声（约2~3ms），**独占 OLED 与 KEY1~4**；题目业务委托 `app_robot_core` → `app/tasks/taskN.c` 的 `OnEnter/OnLoop/OnExit`，运行态 K3 按下沿另经 `RobotCore_ConfirmTask()` 转发给题目的 `onConfirm` 钩子（当前题目三、四、五登记，其余题目为 `NULL`）。当前 5 道题均为硬件/函数测试（非正式赛题）：task1 `VIDEO 5S` 通过视觉协议录制 5 秒无叠加标注的正常画面、task2 `LINE PID` 8 路灰度 PID 循迹、task3 `BALL SWING` 先让 ID1 限位回零并抬升，随后自动以 X=350 启动钢球闭环并等待 K3；按键后启动单次左右摆球并开始计时+录像（OLED 显示 `T:x.xs`，5 秒 `T3_RECORD_DURATION_MS` 后停录像停计时，球继续走完三段），ID2/ID3 只使能并保持 0 RPM（独立 `T3_*`/`T3_BALL_*` 参数组，详见下方说明）、task4 `LINE 6S` **两段式 K3 启动**（第一次进球杆位置闭环、确认钢珠稳定后第二次才发车循迹），行驶期间同步保持钢珠在目标 `X`（`T4_BALL_TARGET_X_PX`），累计前进 `T4_STOP_AFTER_MS` 后缓停，具体秒数与全部调参数值以 `task4.c` 当前值为准（详见下方说明）、task5 `Five` **2026-08 同样改为两段式 K3 启动**（第一次进球杆位置闭环、确认稳定后第二次才发车循迹，加速度/顶速参考 task4 当前值），武装后循迹命中 `T5_STOP_LINE_HIT_MIN`（当前 5）路黑线即判定到达终点，随即两轮改为同一转速直直前进，维持 `T5_AFTER_LINE_MS`（当前 1500ms）后转入缓停；蜂鸣器第二次响、通知视觉端结束录像不等这段直行走完，而是在其中更早的 `T5_BEEP_DELAY_MS`（当前 800ms）先触发，之后再用与起步同一根软件斜坡 `T5_RAMP_RPM_PER_SEC` 对称降速到 0（详见下方说明）、task6 `ID1 POS` 仅对 Emm42 ID1 做**位置模式 API 冒烟测试**（进题目自动位置清零定原点 → 按 `T6_RETURN_ENABLE`/`T6_ABS_TEST_ENABLE` 决定单程/往返/绝对验证，当前默认正转 10 圈后停住量丝杆导程，OLED 题名行显示阶段与目标脉冲）。任务二、四、五自动完成时使用与按键完全相同的短促提示音。M1/M2 已全局取反标定 |
| `BALLCTRL` | `app/app_ball_control_task.c` | 10 ms 轮询；仅在视觉新样本到达时才计算 | **上电默认 OFF**；菜单态 K4 用模块默认 `BALL_CTRL_*` 参数启动目标 `X=350` 的单点验证，右侧 OLED 的 `X:`/`B:` 分别反馈实测位置与闭环状态，再按 K4 安全停止；题目经 `AppBallControl_RequestTargetWithProfile()` 传入独立 profile 启停各自目标 `X`。**已切到位置模式**：α-β 估计位置/速度后走无 I 项的纯 PD，`targetPulse = LEVEL_TRIM_PULSE + SIGN×(Kx×error − Kv×velocity)`，每帧下发绝对位置目标，不是速度。误差进 `settleDeadbandPx` 死区【且】球基本停住才回真实水平点保持；唯一仍主动出手的保护是边缘保护（球快滚出摆杆，命令回水平并锁定 `FAULT_EDGE`，等题目自行处理）。`profile` 另含 `maxPulseStepPerFrame`（软件限速，把目标突变摊到多帧，0=不限速，菜单/题目三保持 0 不变）；`positionAcc` 每题目独立标定，见下方「EMM42 位置模式」小节的 ACC 陷阱。**临时调试用**：`app_ball_control_task.c` 顶部 `BALL_CTRL_DEBUG_LOG_ENABLE`（默认 0）打开后会把每次算出的目标/测量/滤波位置/速度/命令脉冲/状态打到 UART0（TX 空闲，不影响视觉 RX），排查完记得关掉，长期开着占 CPU 和串口带宽 |
| `IMU100Hz` | `app/app_imu_uart_task.c` | 10 ms | 读取 ATK-MS6DSV 姿态 + 追加激光测距1(D1)，按 5Hz 整行输出 Roll/Pitch/Yaw/加减速度/D1 |
| `MOTORTEST` | `app/app_motor_test_task.c` | 20 ms 轮询 | **【默认禁用】** KEY1/KEY2 让 4 个电机（各自独立接口同时下发）正/反转 2 圈测试（按键已让给 UIMENU） |
| `SERVOSWEEP` | `app/app_servo_test_task.c` | 20 ms | **【默认禁用】4 个舵机各自独立错相摆动**（800~2200us，无按键）；单控用 `BspServo_SetPulseUs(id,us)` |
| `PERIPH` | `app/app_periph_test_task.c` | 500 ms | **【默认禁用】** OLED 已交给 UIMENU（互斥防抢屏）；原为外设验证：OLED/LED2/LED3/蜂鸣器 |

> 激光测距1（UART2）不是任务，而是 **UART2 RX 中断**逐字节喂 `module/laser` 的 LD14 解析器；距离由 `IMU100Hz` 任务读取并随整行输出。
> 小球检测（UART0）同样不是任务，而是 **UART0 RX 中断**逐字节喂 `module/vision` 的 `$BALL` 解析器（上位机→下位机，NMEA+XOR）；结果由 `UIMENU` 读取显示在 OLED 右侧文字面板。见 [empty/docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md) 的 `$BALL` 报文小节。
> 题目菜单 UI 见 [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md) 的「OLED 题目菜单 UI」小节；**题目业务逻辑逐题填 [empty/app/tasks/](empty/app/tasks/) 的 `taskN.c`**（第 N 题 = `taskN.c`），题名/题数登记在 `app_robot_core.c` 的 `s_robotTasks[]`。四路步进电机为**各自独立**驱动（`bsp_motor.h`，RPM 单位、带符号定方向）。OLED 当前只能显示 ASCII（无中文字库）。
> K4 在运行态表示退出当前题目；菜单态首次按下启动使用默认 `BALL_CTRL_*` 参数的 `X=350` 单点验证，OLED 右侧 `X:`/`B:` 反馈实测位置与闭环状态，闭环运行时再次按下安全停止。题目三与后台闭环都使用 ID1，进入题目三前会先等待闭环释放 ID1。

> **题目三当前行为**：`BALL SWING` 不执行循迹。进入后等待 `BALLCTRL` 释放 ID1，按“失能→限位回零→抬升”完成非阻塞归零；随后自动请求 X=350 的钢球闭环并等待 K3，期间持续保持中心目标。K3 后 ID2/ID3 依次使能并保持 0 RPM。K3 触发三段摆动那一刻同时开始计时与录像（`deferVideoStart=true`，OLED 显示 `T:x.xs` 加阶段简称 `ARM`/`LFT`/`MID`/`FIN`/`REC`/`DONE`），累计校准时间达 `T3_RECORD_DURATION_MS`（5000ms）后停录像并暂停计时，小球继续走完三段不受影响。`T3_BALL_CENTER_X_PX=350` 仅用于 K3 前的中心保持；K3 后按 `T3_BALL_LEFT_TARGET_X_PX=230`→`T3_BALL_MIDDLE_TARGET_X_PX=380`→`T3_BALL_FINAL_TARGET_X_PX=450` 三段执行。前两段首次进入各自到达带即切下一段；最终段测量达到 `T3_BALL_FINAL_GUARD_X_PX=500` 时重新投递最终目标回拉，按位置误差≤6 px、速度≤10 px/s 持续 `T3_BALL_FINAL_HOLD_TIME_MS=200ms` 后短鸣并等待 K4。`FAULT_EDGE` 走停止、等待 OFF、按当前阶段重新请求的恢复握手；K4 停止并失能 ID1、ID2、ID3。所有 `T3_*`/`T3_BALL_*` 参数仅影响本题。

> **题目四钢珠平衡 + 循迹**：ID1 的位置零点由开机自动归零流程建立，不再需要人工在菜单页按 K4。**2026-08 新增两段式 K3 启动**：进题目后什么都不动（ID1 不闭环、轮子不使能，OLED 提示按 K3）；**第一次 K3** 向 `BALLCTRL` 投递独立 `T4_BALL_*` profile 并等待后台真正进入闭环（`RUNNING`/`HOLDING`）；**第二次 K3** 才走 ID2/ID3 使能时序、开始循迹前进、秒表开始计时，且才通知视觉端开始录像（`RobotCore_NotifyTaskStarted()`，题目表 `deferVideoStart=true`，其余非本节题目仍是进题目即自动开始录像）。行驶和缓停期间 ID1 均保持闭环，K4 退出或 `T4_STATE_TIME_STOPPED`/`T4_STATE_FINISHED`（到时缓停或触线终点保护）都会通过 `RobotCore_NotifyTaskFinished()` 结束录像并响一声提示音。**`T4_STATE_TIME_STOPPED`/`T4_STATE_FINISHED` 是真正的终止态**：`OnLoop` 一进入这两个状态就在函数最前面直接 `return`，不会再被后面任何逻辑（循迹读数、终点判定、`FAULT_EDGE` 恢复握手）意外拨回 `RUN`——这是修过一次真实 bug 后的设计，`FAULT_EDGE` 恢复握手完成时会检查"本次是否已经跑完"（`s_runCompleted`），跑完了就直接停在终止态，不会误判成还要继续循迹。`T4_BALL_*`、循迹 PID、起步斜坡、缓停时长等全部参数与任务三互不影响，具体数值以 `task4.c` 当前值为准，不要以文档旧数字为准（改动频繁）。

> **题目五钢珠平衡 + 循迹到终点线**：2026-08 由题目四整体框架改造而来，同样是两段式 K3 启动（第一次 K3 启动独立 `T5_BALL_*` profile 的球杆闭环并等待就绪，OLED 提示 `T5 K3=BALL`/`T5 B WAIT`/`T5 K3=GO`；第二次 K3 才使能 ID2/ID3 循迹前进、录像开始、秒表开始计时），题目表 `deferVideoStart=true`。**终点判定与题目二/四不同**：先连续 `T5_ARM_TICKS` 拍命中 `≤T5_ARM_HIT_MAX` 路细线武装，武装后单拍命中 `≥T5_STOP_LINE_HIT_MIN`（当前 5，即 8 路里的 5 路，非题目二/四系的 6 路）**且累计时间 `≥T5_FINISH_MIN_ELAPSED_MS`（当前 3000ms）**才判定到达终点——赛道环形、起跑线=终点线，仅凭"武装+命中路数"防不住发车不久又经过同一根线被误判，2026-08 加了这层时间下限（与题目二 `T2_FINISH_MIN_ELAPSED_MS` 同一动机）。判定到达终点后立即把左右轮命令改成判定那一刻的平均转速（两轮同值，即"保持当前速度直直地前进"，不再有转向修正），维持 `T5_AFTER_LINE_MS`（当前 1500ms）后转入缓停。蜂鸣器第二次响、结束本次录像不等这 1500ms 走完，而是在其中更早的 `T5_BEEP_DELAY_MS`（当前 800ms）先触发（`s_finishBeeped` 保证只响一次）——录像/计分只覆盖到终点线+800ms，之后车辆仍按同一转速再直行到满 1500ms 才开始减速，给机械留裕量。1500ms 结束后用**与起步同一根** `T5_RAMP_RPM_PER_SEC` 软件斜坡反向线性降速到 0（期间继续循迹 PID 修正，尽量平稳停住）——用户明确要求缓停加速度与起步加速度对称，因此不像题目五旧版本那样另用一个固定时长的减速参数。行驶、终点保持和缓停期间 ID1 均保持闭环，`FAULT_EDGE` 自动恢复握手与「本次是否已经跑完」（`s_runCompleted`）判断逻辑均与题目四一致。**转向输出软件限速** `T5_STEER_SLEW_RPM_PER_SEC`（题目二/四没有，题目五独有）：PID 算出的转向量只作为目标，实际下发的 `s_appliedSteerRpm` 每拍最多向目标靠近 `本值×T5_DT_SEC`，把离散灰度传感器命中路数切换带来的台阶式转向量摊成渐变，抑制过弯瞬间大幅横摆把摆杆上的钢珠带得晃动（2026-08 实车反馈"Kp 偏大导致过弯横摆、球被甩"后新增，起点值待现场调）。`T5_BALL_*`、`T5_STOP_LINE_HIT_MIN`、`T5_AFTER_LINE_MS`、`T5_BEEP_DELAY_MS`、`T5_RAMP_RPM_PER_SEC`、`T5_STEER_SLEW_RPM_PER_SEC`、循迹 PID 等全部参数与题目四/二互不影响，具体数值以 `task5.c` 当前值为准。

> **控制参数隔离规则**：可以跨任务复用算法实现、执行器和通信接口，但**禁止共享可调控制参数**。每个任务必须在自己的 `taskN.c` 顶部定义私有参数组和 profile；即使初值参考其他任务，也必须按值复制为新参数。调整一个任务的增益、死区、滤波、静摩擦、速度或保持条件，不得改变菜单或任何其他任务的行为。新增任务时同步在 `CLAUDE.md`、`AGENTS.md` 与 `docs/` 声明其独立参数组。

### 功能总开关（按需启用外设）

[empty/common/app_config.h](empty/common/app_config.h) 顶部有一组 `APP_FEATURE_*`（1=启用/0=禁用），`App_Init` 用它门控各任务创建。用开发板时把不需要的外设置 0 即可（不创建任务、不占 CPU、不刷串口；板级硬件初始化仍保留）：`APP_FEATURE_LED_HEARTBEAT / UART_ECHO / UI_MENU / PERIPH_OLED / SERVO / MOTOR / IMU / IMU_UART_LOG / LASER / BALL_VISION / LINE_TRACK / EMM42 / BALL_CONTROL / LIFT_HOMING`。**当前默认**：`UI_MENU=1`（题目菜单）；因 UI 独占 OLED 与按键，`PERIPH_OLED / SERVO / MOTOR` 默认置 0。`UART_ECHO=0`（关接收自检）、`IMU_UART_LOG=0`（IMU 不打印串口遥测——串口彻底静默；但 IMU 读取 + Yaw 快照照常运行供 OLED 状态栏）。调试时把这两个改回 1 即可恢复串口输出。`BALL_VISION=1`（UART0 RX 中断解析上位机 `$BALL` 报文，OLED 右侧文字面板显示）——它与 `UART_ECHO` 争用 UART0 RX，二者互斥（同时置 1 编译期 `#error` 拦截）。`LINE_TRACK=1`（7 路灰度循迹 PB17~PB23，OLED 菜单右半显示状态）。`EMM42=1`（张大头 Emm42_V5.0 闭环步进：UART1 板级初始化 + 注册回复接收中断，当前总线上 3 台设备（地址 1/2/3 = 摆杆高低调节/左轮/右轮），命令由题目二、题目四和题目五经 `module/emm42/emm42_robot.h` 的角色化接口 `Emm42Robot_*` 下发（内部再转发到 `emm42_v5.h` 的按地址协议接口）；它不创建任务，仅注册中断。方向/差速运动学尚未标定，见 `emm42_robot.h` 文件头说明）。`LIFT_HOMING=1`（2026-08-01 新增：开机 ID1 自动归零，依赖 `EMM42`，见下方「ID1 开机自动归零」小节；PA24 原继电器接口已改接归零限位开关，继电器功能与相关代码已整体移除）。**`IMU=0`、`LASER=0`、`NRF24_TX_TEST=0`（2026-07-29 临时关闭，减少 CPU/中断占用，需要时改回 1 即可，代码逻辑未删改）**：关闭后 `IMU100Hz`/`NRF24TX` 任务不创建、UART2 激光 RX 中断不使能，OLED 状态栏 Yaw/距离显示 `---`；题目四 `LINE 6S` 不依赖 IMU，可照常测试。

修改或新增任务后必须同步更新 [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md)。

`APP_FEATURE_BALL_CONTROL=1` 创建 `BALLCTRL` 线程但上电不使能 ID1；它依赖
`APP_FEATURE_VISION_LINK=1` 和 `APP_FEATURE_EMM42=1`，无效组合由编译期护栏拦截。

### EMM42 方向标定状态

- 角色层 `module/emm42/emm42_robot.c` 统一处理 Emm42 正方向：ID1（摆杆）正方向为**抬升摇臂**（2026-08 机构改直驱曲柄摇杆后经题目六 900 脉冲实测确认，取代旧丝杆机构"连杆向下"的说法），ID2（左轮）直通，ID3（右轮）取反。
- 题目五 `Five` **2026-08 起同时使用 ID1 与 ID2/ID3**：第二次 K3 后才使能 ID2 左轮、ID3 右轮，沿用题目二的节拍化使能时序，两轮各等待约 180ms 后进入交替速度控制；ID1 由 `BALLCTRL` 独立线程按题目五私有 `T5_BALL_*` profile 闭环，不经这套使能时序。机械安装变化时，只修改 ID1 的角色层标定表。
- `BALLCTRL` 独立线程只控制 ID1；Emm42 协议出口用互斥量保证它与 ID2/ID3 题目线程的 UART1 整帧不交叉，并统一保留至少 6ms 帧间隔。ID1 正 RPM 已标定到"抬升摇臂"，钢球 X 的最终控制极性（各题目 `T*_BALL_OUTPUT_SIGN`）仍需各自低速实机确认。
- 左右轮差速运动学尚未实现。

### ID1 开机自动归零（2026-08-01 新增）

P1 接口（原继电器接口，v1.1 接线文档标注 `RELAY ← PA24`）已改接一颗轻触开关，
一端接地，充当 ID1（摆杆曲柄摇臂）的归零限位开关；**继电器功能与 `bsp_relay.c/h`、
`app_relay_test_task.c/h`、`APP_FEATURE_RELAY`/`APP_FEATURE_RELAY_SELFTEST` 已整体
删除**，PA24 现为数字输入 + 内部上拉（`bsp/bsp_home_switch.h` 的 `BspHomeSwitch_IsPressed()`）。

`App_Init()` 在 `Emm42Robot_Init()` 之后、任何任务创建之前调用
`AppLiftHoming_RunAtBoot()`（[empty/app/app_lift_homing.c](empty/app/app_lift_homing.c)，
由 `APP_FEATURE_LIFT_HOMING` 门控），全程用 `Delay_ms` 忙等轮询开关（调度器尚未启动，
不会与任何任务竞争 ID1）：

**2026-08 机构改为电机直驱曲柄摇杆后已重写流程**（方向依据：题目六 900 脉冲实测
确认 ID1 正脉冲 = 抬升摇杆）：

1. **下降**（负方向）移动，直到压下限位开关——压下点即物理归零参考点；
2. 立即急停；
3. **抬升**（正方向）移动 `LIFT_HOMING_LEVEL_OFFSET_PULSES` 个脉冲到摆杆水平位置。
   换算 3200 脉冲 = 360°（1° ≈ 8.9 脉冲）；当前 `711`（≈80°）是目测估算，**待实测修正**：
   停得比水平低 → 调大，比水平高 → 调小。
4. **极端情况**：若开机时开关已经被压住，跳过第 1 步（不再继续往下顶死），直接抬升。

归零流程只移动 ID1、不清零位置（不调用 `Emm42Robot_ResetPosToZero()`）；`BALLCTRL`
（`hasZeroedSinceBoot`，见下节）和 `task6`（`T6_ZERO_ON_ENTER`）各自的清零逻辑不变，
只要 ID1 在开机归零结束到它们首次清零之间没有被移动过，清零点就等于归零终点——
**因此归零终点就是各题目 `LEVEL_TRIM_PULSE=0` 所假设的物理水平基准，offset 没调准
会让所有题目的水平点一起偏**。这也取代了此前"每次上电先人工把杆摆到目视水平"的步骤。
⚠️ 归零流程故意不加超时保护：若限位开关故障或没装好导致第 1 步永远读不到触发，
调度器不会启动（LED1 不闪），摆杆会持续往下顶到机械死点；这是找不到物理参考点就
不能继续的题中之义，不要为此加时间兜底，但首次上电请守在电源开关旁。

### EMM42 位置模式（摆杆控制的目标形态）

摆杆的正确控制量是**角度**而不是角速度：速度命令对摆杆角度是一次积分、球位置对摆杆角度又是二次积分，速度模式下整链三阶、纯 PID 极难镇定（`v2.2` 实测现象）。系统降为球杆系统标准的二阶 PD 外环：`targetPulse = LEVEL_TRIM_PULSE + SIGN*(Kx*error − Kv*velocity)`。

**2026-07-31 `BALLCTRL` 已切到位置模式**（题目六 `ID1 POS` 先验证过底层 API）：外环为无输出限幅、无 I 项的 PD。`BALL_CTRL_SETTLE_DEADBAND_PX` 是实际到位保持区：范围内回水平并停止驱动，避免已满足精度的钢球被再次推出目标区；只有超出该范围才重新驱动。**位置原点只在本次上电后第一次启动时清零**（函数级 `static bool hasZeroedSinceBoot`），之后反复进出各题目停/启调参沿用同一原点，重新上电才建立新原点——这是有意设计，避免调参中途摆杆停在某个倾角时被误当成新零点导致误差跨轮次累积。配合开机自动归零（见「ID1 开机自动归零」小节），本次上电后第一次启动 `BALLCTRL` 时清零的位置就是归零终点。**单帧视觉丢失不再触发任何停止命令**：位置模式下没有新目标时电机保持在原地（结构自带安全），此前速度模式为防止 RPM 失控而加的软降速/硬急停/方向发散保护已删除。唯一仍主动下发命令的保护是边缘保护（球真的快滚出摆杆，命令回水平并锁定）。**2026-08 机构改直驱曲柄摇臂后，`Kx/Kv/POS_RPM/POS_ACC` 等旧丝杆机构标定值已全部失效**，各题目独立 profile 需要现场重新整定，方法论见 [empty/docs/CONTROL_ALGORITHM.md](empty/docs/CONTROL_ALGORITHM.md) §10；具体数值以各 `taskN.c`/`app_ball_control_task.c` 当前源码为准，不要照抄本文档任何历史数字。完整设计见 [empty/docs/BALL_CONTROL.md](empty/docs/BALL_CONTROL.md)。

- 角色层已提供 `Emm42Robot_MoveAbsolute()`（绝对位置）、`Emm42Robot_ResetPosToZero()`（定原点）、`Emm42Robot_ClearClogProtection()`（解堵转保护），业务层不要直接调协议层 `Emm42_*`。
- **`MoveAbsolute` 不能像 `MoveRelative` 那样把 `pulses==0` 当"不动"提前返回**——绝对模式下 0 是最常用的目标（回原点）。
- **位置原点建立**：ID1 开机已经过 PA24 限位开关自动归零（见上方「ID1 开机自动归零」小节），不再需要人工把杆摆到目视水平；进题目六时仍会 `OnEnter` 自动发 `0x0A 0x6D` 清零一次（`T6_ZERO_ON_ENTER`），只要 ID1 自开机归零后没被移动过，清零点就等于归零终点。驱动器断电不保留多圈位置计数，不能沿用上次零点；失能不丢位置。题目六里的一切位置都锚在**按 K3 进入那一刻**，反复进出会重新置零、位移累积。
- ⚠️ **新的位置命令会覆盖尚未走完的上一条并重新规划**。连续下发时，两帧间隔必须覆盖整段运动时间（acc=0 时理论时间 = 圈数/转速 分钟），否则表现为"命令发了却几乎没走到位"。`task6.c` 用 `T6_SEGMENT_WAIT_MS` 编译期算出并留余量；彻底的解法是读 `0x3A` bit1 到位标志。这个"覆盖"特性本身对 15Hz 外环是**好事**（每帧刷新目标即可），只有做定量运动时才需要等到位。
- 脉冲单位随驱动器细分，出厂 16 细分 = 3200 脉冲/圈；`task6.c` 的 `T6_PULSES_PER_REV` 要与驱动器 `MStep` 菜单一致。
- **读实时位置 `0x36` 的单位不是脉冲**，而是编码器角度（65536 = 一圈），与位置命令的 3200 脉冲/圈差 20.48 倍，做反馈时必须换算。
- **堵转保护是丝杆类大静摩擦负载的首要嫌疑**：转速<40RPM 且电流>2400mA 且持续>4000ms 三条同时成立即触发，触发后位置命令返回 `E2` 且电机纹丝不动，须先发 `0x0E 0x52` 解除。出现"第一次能动、之后怎么发都不动"先查这个，别怀疑帧格式。
- 详细手册结论（到位标志 `0x3A` bit1、到位返回命令 `地址+FD+9F+6B`、`S_Vel_IS` 0.1RPM 模式、原点回零命令组）见 [empty/docs/BALL_CONTROL.md](empty/docs/BALL_CONTROL.md)。

### 循迹通道状态

- 灰度循迹已启用 8 路：H6 的 LINE1~LINE8 分别接 PB17~PB24；`bsp_line` 位图 bit0~bit7 对应 LINE1~LINE8。
- 硬件物理左→右为 LINE8→LINE1，OLED 菜单右半下部同步显示 `87654321` 及 bit7→bit0 的对应 8 位状态；当前模块识别到线为低电平，`BSP_LINE_ACTIVE_LOW=1` 归一化为 `1`=识别到线；PB17~PB24 均非 5V 容忍，必须使用 3.3V 信号电平。

### 题目二正式循迹（已上车测试通过，进入稳定迭代阶段）

- 题目二当前为 `LINE PID`：用 8 路灰度循迹经 UART1 控制 Emm42 的 ID2 左轮与 ID3 右轮前进，ID1 摆杆不参与；PID 增益、基础速度等具体数值由用户在实车上持续调参，以 `app/tasks/task2.c` 顶部当前值为准，不要以文档里的旧数值为准。
- **入场时序（状态机）**：`Task2_OnEnter()` 只复位变量、不直接下发 Emm42 命令；实际动作在 `OnLoop` 的状态机里节拍化完成：`T2_STATE_RESET_DISABLE`（先失能 ID2/ID3 清残留状态）→ `T2_STATE_RESET_WAIT`（等 `T2_RESET_SETTLE_TICKS`）→ 依次使能 ID2、ID3，各使能后等 `T2_ENABLE_SETTLE_TICKS` → `T2_STATE_RUN`。**`T2_ENABLE_SETTLE_TICKS` 已实测验证**：3 拍(90ms) 会有约一半概率使能不生效（驱动器来不及处理），6 拍(180ms) 才稳定，不要再往下调；`T2_RESET_SETTLE_TICKS` 没有类似的失败实测数据，是偏保守加的，需要压缩入场延时时优先调这个。
- 误差先做一阶低通滤波（`T2_ERROR_FILTER_ALPHA`）再叠加死区 `T2_ERROR_DEADBAND`（`|误差|` 小于此值直接当 0），抑制离散传感器命中路数在相邻两档（如 2 路/3 路）来回跳变导致的中心附近小幅摆动；转向输出越大基础速度按比例自动降低（`T2_CORNER_SLOWDOWN_GAIN`/`T2_MIN_BASE_RPM`），实现直道快、弯道稳。速度命令加速度档位 `T2_EMM_ACC` 用非 0 曲线档位（说明书 §6.3.1）——0 会立即生效但实测车身会突兀抖动，任务层不再叠加软件斜坡，平滑完全交给这个硬件曲线档位。
- 差速安全限幅 `Task2_ClampWheelPair()` 对左右轮目标 RPM 做"整体平移"而非各自独立 clamp：独立 clamp 会在外侧轮触顶时压扁差速、内侧轮却不受影响，导致弯道实际转向权限远小于 PID 算出的量（高速冲出弯道的典型根因）；`T2_MAX_WHEEL_RPM` 须 `>= T2_BASE_RPM` 且总跨度 `>= 2*T2_MAX_STEER_RPM`。
- 终点检测：先连续 `T2_FINISH_ARM_TICKS` 拍命中细线确认已离开出发线才"武装"终点检测，武装后再连续 `T2_FINISH_HIT_TICKS` 拍命中路数 `>= T2_FINISH_HIT_MIN`（当前 6，含 7/8 路）**且累计时间 `>= T2_FINISH_MIN_ELAPSED_MS`（当前 3000ms）**判定跑完一圈，硬停车锁定并等待 K4 手动退出。赛道是环形、起跑线与终点线是同一根线，只靠"武装+命中路数"防不住"赛道较小、发车不久又经过这根线"被误判成终点，因此 2026-08 加了这层时间下限；避免出发瞬间被误判为终点仍是"武装"这一步的职责，两者互补。
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
| ID1 归零限位开关 | PA24 | 按下接地，内部上拉；P1 接口原为继电器（v1.1 述 `RELAY ← PA24`），2026-08-01 改接轻触开关，用于开机自动归零，见 `bsp/bsp_home_switch.h` |
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
- **跨界跳变**：Yaw 在 ±180° 边界跳变（−179° ↔ +179°），计算角度差必须用最短路径差值算法（如 `angle_utils` 接口），不可直接减法。
- **漂移**：无磁力计/外部参考时，Yaw 长期会漂移；短时间（几秒到几十秒）内相对精度足够用于 90°/180° 转弯闭环。

### 跨模块通信

`BALLCTRL` 使用长度为 1 的 FreeRTOS 覆盖队列接收启停/目标 X/profile 命令，并通过临界区快照发布状态；`AppBallControl_RequestTargetWithProfile()` 会按值复制调用方 profile，供任务三与菜单调参隔离。定义见 [empty/docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md)。后续新增有先后语义的消息仍优先使用 queue / event group / stream buffer，应用之间不要通过裸全局变量传递业务数据。

## 修改代码后自查

1. 是否符合分层边界（app/module/algo/bsp/common）？
2. 是否遗漏 FreeRTOS 同步、中断安全、`FromISR` 版本？
3. 是否误用了核心板特殊功能引脚？
4. 是否新增了未记录的任务、消息、模式或硬件接线？
5. 是否需要更新 `docs/` 下的 `FREERTOS_TASKS.md`、`MESSAGE_LIST.md`、`HARDWARE_WIRING.md`？
6. 关键代码是否写了必要的中文注释？

## 题目六启动加速度抬升前馈

题目六在第二次 K3 后以 `T6_START_ACCEL_LIFT_*` 叠加独立的 ID1 水平点临时偏置：当前为
正方向 `+40` 脉冲，150 ms 平滑抬升，覆盖轮速爬升段，达到巡航后 300 ms 平滑撤回。偏置
必须经线程安全的 `AppBallControl_SetLevelTrimOffset()` 交给 `BALLCTRL`，不能直接下发相对
位置命令；它是抵消起步加速度扰球的前馈，不是任何 PID 增益。新建/停止闭环和边缘故障会
自动清零；任务六还会在丢线停车、终点和退出时主动清零，其他题目不得调用或复用该参数组。

## 维护文档索引

| 文档 | 用途 |
|---|---|
| [docs/PROJECT_CONTEXT.md](empty/docs/PROJECT_CONTEXT.md) | 当前架构、硬件、主要模块、关键约定 |
| [docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md) | 任务名、周期、优先级、栈大小、输入输出 |
| [docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md) | 跨任务消息定义（当前为空） |
| [docs/HARDWARE_WIRING.md](empty/docs/HARDWARE_WIRING.md) | 硬件接线、引脚、电气说明、排查备注 |
| [docs/UART_DEBUG_GUIDE.md](empty/docs/UART_DEBUG_GUIDE.md) | UART 配置原则与乱码排查流程 |
| [docs/CONTROL_ALGORITHM.md](empty/docs/CONTROL_ALGORITHM.md) | 小球闭环控制算法文档：位置模式 PD、α-β 滤波、静摩擦补偿、Emm42 协议 |
| [docs/AI_MEMORY.md](empty/docs/AI_MEMORY.md) | 用户固定要求、重要决策、踩坑记录 |
