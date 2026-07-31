# 仓库协作指南

## 首读文件与同步规则

每次阅读、分析或修改本仓库前，必须先阅读根目录的 `CLAUDE.md`，再阅读本文件。`CLAUDE.md` 保存详细工程约定，本文件提供简明执行指南；二者共同生效。

凡是修改编码、构建、硬件、任务、测试或协作流程规则，必须在同一次改动中同步更新 `CLAUDE.md` 和 `AGENTS.md`，确保内容不矛盾。新增或修改的代码注释、README、`docs/` 文档、协作指南及其他开发说明统一使用中文；命令、文件路径、代码标识符、协议字段和必要专有名词除外。

## 项目结构

工程主体在 `empty/`：`app/` 放任务、业务流程和题目状态机，题目 N 使用 `app/tasks/taskN.c`；当前 5 道题均为硬件/函数测试（非正式赛题）：题目一 `VIDEO 5S`（通过视觉协议录制 5 秒无叠加标注的正常画面）、题目二 `LINE PID`（8 路灰度 PID 循迹）、题目三 `Task 3`（**2026-08 由题目四整体移植，是 task4 框架的完整副本**，独立 `T3_*` 参数组，原第一阶段定位业务已废弃）、题目四 `LINE 6S`（**两段式 K3 启动**：第一次 K3 启动独立 `T4_BALL_*` profile 的钢珠位置闭环并等待就绪，第二次 K3 才使能 ID2/ID3 循迹前进、录像开始、秒表开始计时；累计前进 `T4_STOP_AFTER_MS` 后以速度模式 0 RPM 缓停，`T4_STATE_TIME_STOPPED`/`FINISHED` 是真正终止态，入口即 `return`，不会被后续逻辑误拨回 `RUN`）、题目五 `Five`（**2026-08 同样改为两段式 K3 启动**，加速度/顶速参考题目四当前值；武装后循迹命中 `T5_STOP_LINE_HIT_MIN`（当前 5/8 路）即判定到达终点，两轮改为同一转速直行，维持 `T5_AFTER_LINE_MS`（当前 1500ms）后转入缓停；蜂鸣器第二次响、结束录像不等直行走完，在其中更早的 `T5_BEEP_DELAY_MS`（当前 800ms）先触发，之后再用与起步同一根软件斜坡 `T5_RAMP_RPM_PER_SEC` 对称降速到 0；另有题目二/四没有的转向输出软件限速 `T5_STEER_SLEW_RPM_PER_SEC`，抑制离散灰度台阶式转向量一拍到位导致的过弯横摆带得钢珠晃动）、题目六 `ID1 POS`（仅对 Emm42 ID1 做位置模式 API 冒烟测试：进题目自动清零定原点，再按 `T6_RETURN_ENABLE`/`T6_ABS_TEST_ENABLE` 决定单程/往返/绝对验证），菜单任一按键短促嘀声（约2~3ms），题目二、四、五自动完成时使用同一短促提示音，M1/M2 已在 BSP 全局取反标定。题目四的缓停加速度由 `T4_STOP_EMM_ACC` 传给 `Emm42Robot_VelControl()`，该接口会保留 0 RPM 速度模式帧；K4 退出和终点保护仍急停。`algo/` 放可跨题复用的纯算法（PID 控制器、角度差值/归一化、α-β 滤波），任务层只保留可调参数宏。`bsp/` 放板级与外设驱动，手写 DriverLib 初始化位于 `bsp/board/ti_msp_dl_config.c`；`module/` 放可复用设备/协议模块；`common/` 放功能开关、FreeRTOS 配置和共享消息；`docs/` 放任务、接线、UART、控制算法和架构记录。

`source/ti/` 与 `empty/third_party/` 属于 SDK 或第三方代码，除非任务明确涉及 SDK 或 FreeRTOS 移植，否则不要修改。

钢球位置控制使用独立 `app/app_ball_control_task.c`（`BALLCTRL`）线程：上电默认
OFF，由各题目经 profile 接口按需启停（菜单已不提供 K4 启停入口，见下方
「ID1 开机自动归零」）；运行态 K4 仍退出当前题目。线程只在新视觉 `xSeq`
到达时用 α-β + 局部 PD（**位置模式，无 I 项**）更新 Emm42 ID1 的绝对目标脉冲；
误差进 `settleDeadbandPx` 死区【且】球基本停住才回真实水平点保持；唯一仍
主动出手的保护是边缘保护（球快滚出摆杆，命令回水平并锁定 `FAULT_EDGE`，
等题目自行发起"停止→等释放→重新请求"的恢复握手）。profile 另含
`maxPulseStepPerFrame`（软件限速，把目标突变摊到多帧，0=不限速，菜单/
题目三保持默认）。**临时调试**：`app_ball_control_task.c` 顶部
`BALL_CTRL_DEBUG_LOG_ENABLE`（默认 0）打开后把每次算出的目标/测量/滤波
位置/速度/命令脉冲/状态打到 UART0（TX 空闲不影响视觉 RX），排查完记得关闭。
后续题目可通过 `AppBallControl_RequestTarget()` 使用菜单默认参数，或通过按值
复制 profile 的 `AppBallControl_RequestTargetWithProfile()` 使用独立参数；题目三、
四各自持有独立 `T3_*`/`T4_BALL_*` profile（题目三现为题目四框架的完整副本），
互不影响。具体上车步骤和参数以 `empty/docs/BALL_CONTROL.md`、
`empty/docs/CONTROL_ALGORITHM.md` §10 为准。

### ID1 开机自动归零

P1 接口（原继电器接口，PA24）已改接一颗轻触开关，作为 ID1（摆杆曲柄摇臂）的
归零限位开关；继电器功能与 `bsp_relay.c/h`、`app_relay_test_task.c/h`、
`APP_FEATURE_RELAY`/`APP_FEATURE_RELAY_SELFTEST` 已整体删除。`App_Init()` 在
`Emm42Robot_Init()` 之后、任何任务创建前调用 `app/app_lift_homing.c` 的
`AppLiftHoming_RunAtBoot()`（`APP_FEATURE_LIFT_HOMING` 门控）。**2026-08 改为直驱
曲柄摇杆后已重写**（正脉冲 = 抬升，题目六实测）：下降（负方向）直到压下限位开关
（即归零参考点）→ 急停 → 抬升（正方向）`LIFT_HOMING_LEVEL_OFFSET_PULSES` 个脉冲
到摆杆水平位置（3200 脉冲 = 360°，当前 `711` ≈ 80° 为目测估算值，待实测修正）；若
开机时开关已被压住则跳过下降直接抬升。全程在调度器启动前忙等，不与任何任务
争抢 ID1；不清零位置，`BALLCTRL`/`task6` 各自的清零逻辑不变，因此归零终点就是各
题目 `LEVEL_TRIM_PULSE=0` 的物理水平基准，取代了此前"每次上电先人工把杆摆到目视
水平"的步骤。故意不加超时保护，开关故障会永久忙等、调度器不启动。

### 控制参数隔离

算法、执行器和通信接口可以跨任务复用，但可调控制参数不得共用。每个任务都必须在自身
`taskN.c` 定义私有参数宏和 profile；即使初值取自其他任务，也只允许按值复制后独立调节。
调整一个任务的增益、死区、滤波、静摩擦、速度或保持条件，不能影响菜单或其他任务。新增
控制任务时必须同步在 `CLAUDE.md`、本文件和对应 `docs/` 标明其独立参数组。

### EMM42 方向标定状态

角色方向由 `module/emm42/emm42_robot.c` 统一处理：ID1（摆杆）正方向为**抬升摇臂**（2026-08 机构改直驱曲柄摇杆后经题目六 900 脉冲实测确认）、ID2（左轮）直通、ID3（右轮）取反。题目五 `Five` **2026-08 起同时使用 ID1 与 ID2/ID3**：ID1 由 `BALLCTRL` 按题目五私有 `T5_BALL_*` profile 闭环，第二次 K3 后才使能 ID2/ID3 做循迹；每轮依次使能后均等待约 180ms，再开始交替下发速度帧。机械安装变化时仅更新角色层标定表。

`BALLCTRL` 与轮子题目会并发共用 UART1，`module/emm42/emm42_v5.c` 协议出口
必须保留整帧互斥和至少 6ms 帧间隔。ID1 的"抬升摇臂"标定不能推导相机 X 的最终
闭环极性，各题目独立 `T*_BALL_OUTPUT_SIGN` 必须各自低速实机确认，方向未确认前不得调增益。

### EMM42 位置模式（摆杆控制的目标形态）

摆杆的正确控制量是**角度**而非角速度：速度模式下"速度→角度→球位置"整链三阶，纯 PID 极难镇定（`v2.2` 实测）。外环退化成球杆系统标准的二阶 PD：`targetPulse = LEVEL_TRIM_PULSE + SIGN*(Kx*error − Kv*velocity)`。

**2026-07-31 `BALLCTRL` 已切到位置模式**（先由题目六 `ID1 POS` 验证过底层 API）：外环为无输出限幅、无 I 项的 PD。`BALL_CTRL_SETTLE_DEADBAND_PX` 是实际到位保持区：范围内回水平并停止驱动，避免已满足精度的钢球被再次推出目标区；只有超出该范围才重新驱动。**位置原点只在本次上电后第一次启动时清零**，之后反复 K4 停/启不重新清零，避免调参中途误差累积。**单帧视觉丢失不再触发任何停止命令**（位置模式下没有新目标电机保持原地，结构自带安全）；此前速度模式为防 RPM 失控加的软降速/硬急停/方向发散保护已删除。唯一仍主动出手的保护是边缘保护。**2026-08 机构改直驱曲柄摇臂后，旧丝杆机构的 `Kx/Kv/POS_RPM/POS_ACC` 标定值已全部失效**，各题目独立 profile 需要现场重新整定，方法论见 `empty/docs/CONTROL_ALGORITHM.md` §10，具体数值以各 `taskN.c`/`app_ball_control_task.c` 当前源码为准。完整设计见 `empty/docs/BALL_CONTROL.md`。

角色层已提供 `Emm42Robot_MoveAbsolute()` / `Emm42Robot_ResetPosToZero()` / `Emm42Robot_ClearClogProtection()`，业务层不要直接调协议层 `Emm42_*`。**`MoveAbsolute` 不能照抄 `MoveRelative` 的 `pulses==0` 早退**——绝对模式下 0 是最常用的目标（回原点）。

ID1 开机已经过 PA24 限位开关自动归零（见上方「ID1 开机自动归零」），不再需要人工把杆摆到目视水平；进题目六仍会 `OnEnter` 自动发 `0x0A 0x6D` 清零一次；驱动器断电不保留多圈位置计数，失能不丢位置。题目六里的一切位置都锚在按 K3 进入那一刻，反复进出会重新置零、位移累积。⚠️ **新的位置命令会覆盖尚未走完的上一条并重新规划**，连续下发时两帧间隔必须覆盖整段运动时间（acc=0 时理论时间 = 圈数/转速 分钟），否则表现为"命令发了却几乎没走到位"；这个覆盖特性对 15Hz 外环反而是好事，只有定量运动才需要等到位。脉冲单位随细分，出厂 16 细分 = 3200 脉冲/圈，`T6_PULSES_PER_REV` 必须与驱动器 `MStep` 一致。**读实时位置 `0x36` 返回的是编码器角度（65536=一圈）而不是脉冲**，差 20.48 倍。丝杆低速顶死会触发堵转保护（转速<40RPM 且电流>2400mA 且持续>4000ms），触发后位置命令返回 `E2` 且电机不动，须先发 `0x0E 0x52` 解除——"第一次能动、之后怎么发都不动"先查这个。完整手册结论见 `empty/docs/BALL_CONTROL.md`。

### 循迹通道状态

灰度循迹使用 H6 的全部 8 路：LINE1~LINE8=PB17~PB24；硬件物理左→右为 LINE8→LINE1，OLED 菜单右半同步显示 `87654321` 及 bit7→bit0 的对应 8 位状态。当前模块识别到线为低电平，`BSP_LINE_ACTIVE_LOW=1` 归一化为 OLED 的 `1`=识别到线。PB17~PB24 都不是 5V 容忍引脚，灰度模块信号必须为 3.3V。

### 题目二正式循迹（已上车测试通过）

题目二当前为 `LINE PID`：以 8 路灰度循迹经 UART1 控制 Emm42 的 ID2 左轮与 ID3 右轮前进，ID1 摆杆不参与；PID 增益、基础速度等具体数值由用户持续在实车上调参，以 `app/tasks/task2.c` 顶部当前值为准。**入场走节拍化状态机**：`OnEnter` 只复位变量，`OnLoop` 里先失能 ID2/ID3（`T2_STATE_RESET_DISABLE`→等 `T2_RESET_SETTLE_TICKS`）再依次使能两路（各等 `T2_ENABLE_SETTLE_TICKS`）才进入 `T2_STATE_RUN`；`T2_ENABLE_SETTLE_TICKS` 已实测 3 拍(90ms)会有约一半概率使能不生效、6 拍(180ms)才稳定，不要调低于此。左右轮命令每 30ms 交替下发一帧，速度命令加速度档位 `T2_EMM_ACC` 用非 0 曲线档位（0 会立即生效但车身突兀抖动），PID 目标 RPM 每拍直接下发、不叠加软件斜坡。误差先做一阶低通滤波、再过一道死区 `T2_ERROR_DEADBAND` 才喂给 PID，抑制命中路数在相邻两档间跳变引起的中心抖动；转向越大基础速度自动按比例降低。左右轮差速限幅 `Task2_ClampWheelPair()` 用"整体平移"而非独立 clamp，避免外侧轮触顶压扁转向差速（高速冲出弯道的典型根因），`T2_MAX_WHEEL_RPM` 须 `>= T2_BASE_RPM` 且总跨度 `>= 2*T2_MAX_STEER_RPM`。终点检测看命中路数是否 `>= T2_FINISH_HIT_MIN`（当前 6，含 7/8 路），且需先连续命中细线一段时间"武装"（避免出发瞬间在宽起始线上误判），才判定跑完一圈并硬停车锁定，等 K4 手动退出。K4 退出与终点急停都要给 ID2/ID3 背靠背下发多帧，帧间插 5ms 延时防共享总线互相干扰丢帧；ID1 摆杆不受影响。OLED 运行界面复用题目五的题名行显示秒表计时（`Task2_GetUiStatus()`，到终点自动定格），因主频/tick 精度疑点乘了一个实测校准系数 `T2_STOPWATCH_CAL_SCALE`。秒表时间达到 `T2_DECEL_START_MS`（固定 14500ms）后基础速度线性下降（梯度 `T2_DECEL_GRADIENT_RPM_PER_SEC`，下限 `T2_DECEL_MIN_RPM`），跟转弯减速取更小值生效。

## 构建、烧录与验证

用 Keil uVision 打开 `empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`，按 `Build (F7)` 编译，再用 `Download` 烧录。也可在 `empty/keil/` 目录执行：

```powershell
UV4.exe -b empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  -t empty_LP_MSPM0G3507_nortos_keil -o build.log
```

以 `build.log` 出现 `0 Error(s)` 为编译通过。SysConfig 已停用，`empty.syscfg` 仅作历史参考。仓库没有自动化测试框架；固件改动除编译外还应实机验证 UART0 的两条 `BOOT:` 信息、PB25 每 300 ms 心跳，以及受影响外设的实际行为。

## 代码与硬件约束

沿用现有 C 风格：四空格缩进、既有大括号格式；板级接口使用 `Bsp*`，应用入口使用 `App_*`，赛题状态机使用 `TaskN_OnEnter/OnLoop/OnExit`，常量和功能开关使用大写蛇形命名，例如 `APP_FEATURE_IMU`。关键硬件操作、中断、状态切换和安全保护应写必要的中文注释；中断保持短小，涉及 FreeRTOS 时使用 `FromISR` 接口。

修改引脚分配或外设复用前，必须查阅 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md` 及其网表；它们的优先级高于代码注释和 Git 历史。行为、接线、任务或报文变化时，同步更新对应的 `empty/docs/` 文档。

## 提交与合并请求

提交标题使用简短中文，可带版本或范围前缀，例如 `v1.9.3：修复电机急停状态`、`docs：更新 UART2 接线说明`。保持一次提交只处理一个主题。合并请求应说明目的、影响的硬件/模块、构建结果、实机验证结果和配置变更；界面或 OLED 行为变化时附上可见证据。
