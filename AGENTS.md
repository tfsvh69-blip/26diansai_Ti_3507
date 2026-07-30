# 仓库协作指南

## 首读文件与同步规则

每次阅读、分析或修改本仓库前，必须先阅读根目录的 `CLAUDE.md`，再阅读本文件。`CLAUDE.md` 保存详细工程约定，本文件提供简明执行指南；二者共同生效。

凡是修改编码、构建、硬件、任务、测试或协作流程规则，必须在同一次改动中同步更新 `CLAUDE.md` 和 `AGENTS.md`，确保内容不矛盾。新增或修改的代码注释、README、`docs/` 文档、协作指南及其他开发说明统一使用中文；命令、文件路径、代码标识符、协议字段和必要专有名词除外。

## 项目结构

工程主体在 `empty/`：`app/` 放任务、业务流程和题目状态机，题目 N 使用 `app/tasks/taskN.c`；当前 5 道题均为硬件/函数测试（非正式赛题）：题目一 `VIDEO 5S`（通过视觉协议录制 5 秒无叠加标注的正常画面）、题目二 `LINE PID`（8 路灰度 PID 循迹）、题目三 `Task 3`（经 UART1 仅控制 Emm42 ID1，低速正反各约 500ms）、题目四 `LINE 6S`（完整复用题目二循迹参数，累计前进 6.5 秒后以速度模式 0 RPM 缓停）、题目五 `Five`（低速横向停止线后 0.5 秒循迹并线性缓停，UART1 仅控制左/右轮 ID2/ID3），菜单任一按键短促嘀声（约2~3ms），题目二、五自动完成时使用同一短促提示音，M1/M2 已在 BSP 全局取反标定。题目四的缓停加速度由 `T4_STOP_EMM_ACC` 传给 `Emm42Robot_VelControl()`，该接口会保留 0 RPM 速度模式帧；K4 退出和终点保护仍急停。`algo/` 放可跨题复用的纯算法（PID 控制器、角度差值/归一化），任务层只保留可调参数宏。`bsp/` 放板级与外设驱动，手写 DriverLib 初始化位于 `bsp/board/ti_msp_dl_config.c`；`module/` 放可复用设备/协议模块；`common/` 放功能开关、FreeRTOS 配置和共享消息；`docs/` 放任务、接线、UART 和架构记录。

`source/ti/` 与 `empty/third_party/` 属于 SDK 或第三方代码，除非任务明确涉及 SDK 或 FreeRTOS 移植，否则不要修改。

钢球位置控制使用独立 `app/app_ball_control_task.c`（`BALLCTRL`）线程：上电默认
OFF，OLED 菜单态 K4 启停目标 `X=320`，运行态 K4 仍退出当前题目。线程只在约
15Hz 的新视觉 `xSeq` 上用 α-β + 局部 PID 更新 Emm42 ID1；I 项只在新有效帧、
原始误差 1~12px 且低速时按真实帧间隔累积，并单独限制为 ±2RPM；目标变化、
K4、DEG/LOST、误差过零、低误差、高速度或安全故障时必须清零 I。正常 RUN 状态不做
最大 RPM、软件斜率或驱动器加速度曲线限制。单次 `X,NA` 或
130ms 无有效 X 进入 `B:DEG`，每 20ms 把命令向 0 RPM 回退 2 RPM；连续两帧
NA 或 220ms 无有效 X 才进入 `B:LOST` 急停，恢复需连续两帧有效 X。方向发散或
画面边缘仍立即保护停车；不再使用软件估算行程限幅。后续题目通过
`AppBallControl_RequestTarget()` 传目标；题目三直接测试 ID1，进入前必须等待
后台闭环释放。具体上车步骤和参数以 `empty/docs/BALL_CONTROL.md` 为准。

### EMM42 方向标定状态

角色方向由 `module/emm42/emm42_robot.c` 统一处理：ID1（摆杆）正方向为连杆向下、ID2（左轮）直通、ID3（右轮）取反。题目五 `Five` 只使用 ID2/ID3 做循迹；每轮依次使能后均等待约 180ms，再开始交替下发速度帧。机械安装变化时仅更新角色层标定表。

`BALLCTRL` 与轮子题目会并发共用 UART1，`module/emm42/emm42_v5.c` 协议出口
必须保留整帧互斥和 5ms 帧间隔。ID1 的“连杆向下”标定不能推导相机 X 的最终
闭环极性，首次上车必须低速确认 `BALL_CTRL_OUTPUT_SIGN`，方向未确认前不得调 PID。

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
