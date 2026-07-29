# empty/app/ 阅读指南

本目录存放所有 FreeRTOS 应用层任务与题目业务。建议按下表从上往下读：从入口到细节、从"当前默认在跑的"到"默认禁用的验证任务"。

> **当前默认形态**（由 [`../common/app_config.h`](../common/app_config.h) 的 `APP_FEATURE_*` 门控）：
> 上电就跑的是 **LED1 心跳** + **UIMENU 题目菜单**（独占 OLED 与 4 按键）+ **IMU100Hz**（读姿态、静默不刷串口）；
> 另有两路**中断驱动的接收器（非任务）**：激光测距(UART2) 与 树莓派视觉通信(UART0)。
> **默认禁用**：`UART0TX`(串口回显自检)、`MOTORTEST`、`SERVOSWEEP`、`PERIPH`——它们要么与 UIMENU 抢按键/OLED，要么只是外设验证，需要时把对应开关置 1 即可。

---

## 阅读顺序

| 序号 | 文件 | 读什么 | 为什么这么排 |
|---|---|---|---|
| **1** | [`main.c`](main.c) | 程序入口：`BspBoard_Init()` → 串口启动提示 → `App_Init()` → `vTaskStartScheduler()` | 先理解整机启动流程 |
| **2** | [`app_main.c`](app_main.c) | `App_Init()` 里用 `APP_FEATURE_*` 门控创建了哪些任务、注册了哪些 RX 中断 | 拿到"任务清单 + 功能开关"全貌 |
| **3** | [`app_led_task.c`](app_led_task.c) | 最简单的任务：每 300ms 翻转 PB25 心跳灯 | 入门理解 FreeRTOS 任务的最简写法 |
| **4** | [`app_ui_task.c`](app_ui_task.c) | **当前主界面 UIMENU**：4 键选题/运行状态机、底部传感器状态栏(Yaw/激光)、右侧小球检测文字面板；事件驱动整屏刷 + 局部低频刷 | 现在上电看到的就是它；理解 OLED 独占与分区刷新 |
| **5** | [`app_robot_core.c`](app_robot_core.c) / [`.h`](app_robot_core.h) | 6 道题的 **dispatch 表**：登记每题 `OnEnter/OnLoop/OnExit` 钩子，被 UIMENU 在运行态调用 | 题目业务与 UI 解耦；**只登记不写业务** |
| **5.1** | [`tasks/app_tasks.h`](tasks/app_tasks.h) + [`tasks/task1.c`](tasks/task1.c) … [`task6.c`](tasks/task6.c) | **各题业务代码就在这里**：第 N 题 = `taskN.c`；task1 测试 M1~M4 正方向，task2 为 8 路灰度 PID 循迹，task3 经 UART1 仅控制 Emm42 ID1 正反方向各约 500ms，task4 完整复用任务二并在前进 6.5 秒后缓停，task5 为横线后 0.5 秒循迹并线性缓停且显示校准秒表，task6 为 demo 框架 | 写赛题状态机只改这里；`app_tasks.h` 顶部有“怎么填 + 能调哪些底层接口”说明 |
| **6** | [`app_imu_uart_task.c`](app_imu_uart_task.c) | IMU 姿态：GPIO 软件 I2C → LSM6DSV16X → SFLP 融合欧拉角；发布线程安全 **Yaw 快照** 供 OLED；串口遥测默认静默 | 最复杂的任务：I2C 超时/总线恢复、FIFO、打印节流、非阻塞重试 |
| **7** | [`app_uart_test_task.c`](app_uart_test_task.c) | UART0 接收回显自检（**默认禁用**，`APP_FEATURE_UART_ECHO=0`） | 理解串口多任务共享的递归互斥量模式；注意它与小球接收互斥 |
| **8** | [`app_motor_test_task.c`](app_motor_test_task.c) | 四电机定圈旋转（**默认禁用**，按键让给 UIMENU） | "任务发令→bsp+ISR 执行→自动停表"的分层模型 |
| **9** | [`app_servo_test_task.c`](app_servo_test_task.c) | 四舵机独立错相摆动（**默认禁用**） | TIMA0 四路 50Hz PWM，每 CCP 独立占空比 |
| **10** | [`app_periph_test_task.c`](app_periph_test_task.c) / [`app_motor_status.h`](app_motor_status.h) | 外设综合验证 + 电机诊断共享结构 `g_motorDiag`（**默认禁用**，OLED 已归 UIMENU） | 旧的跨任务数据流示例（MOTOR 写 → PERIPH 读 OLED） |

> **两路接收器不是任务**（在 `App_Init` 里注册中断，无 `Entry` 循环）：
> - **激光测距(UART2 RX 中断)** → `module/laser` LD14 解析 → 距离并入 IMU 遥测行、供 UIMENU 状态栏 `D:`。
> - **树莓派视觉通信(UART0 RX 中断)** → `module/vision` 解析 `$PONG/$ACK/$X`，`app_vision_link` 发送 PING/TASK 并供 UIMENU 显示 `NET/X/ACK`。
> 详见 [`../docs/FREERTOS_TASKS.md`](../docs/FREERTOS_TASKS.md) 的对应小节与 [`../docs/MESSAGE_LIST.md`](../docs/MESSAGE_LIST.md)。

---

## 阅读后应该能回答

1. **整机上电后发生了什么？** → `main.c` + `app_main.c`；默认进入 UIMENU 菜单界面。
2. **上电看到的 OLED 界面是谁画的？** → `app_ui_task.c`（UIMENU）：菜单/运行两态、底部状态栏、右侧视觉通信面板。
3. **题目业务逻辑写在哪？** → **`app/tasks/taskN.c`（第 N 题就在 taskN.c）**，每题一套状态机骨架；`app_robot_core.c` 只把它们登记进 dispatch 表，UIMENU 只管显示与按键。
4. **树莓派发来的 X 坐标怎么进来的、显示在哪？** → UART0 RX 中断喂 `module/vision` 解析器 → `app_vision_link` 做在线/超时判定 → UIMENU 右侧面板；开关 `APP_FEATURE_VISION_LINK`。
5. **陀螺仪 Yaw / 激光距离怎么显示到状态栏？** → `IMU100Hz` 临界区发布 Yaw 快照、`module/laser` 发布距离快照，UIMENU 局部低频刷底行。
6. **怎么加一个新任务？** → 参照 `app_led_task.c`：`.c` 写 `Entry`+`Init`、`.h` 声明 `Init`，在 `app_main.c` 的 `App_Init()` 里用 `#if APP_FEATURE_xxx` 门控调用。
7. **多个任务共用 UART0 不会冲突吗？** → `bsp_uart.c` 的递归互斥量，`BspUart0_Lock/Unlock` 保证整行原子（注意 UART0 RX 中断接收小球报文时，轮询自检 `UART_ECHO` 必须关，二者互斥有编译期护栏）。

---

## 依赖关系

```
app 层（本目录）
  ├── 依赖 bsp/   （板级外设：GPIO、UART0/2、电机 STEP/DIR/ENN、按键、舵机 PWM、蜂鸣器）
  ├── 依赖 module/（可复用模块：imu 姿态、oled 显示、laser 测距解析、vision 小球报文解析）
  ├── 依赖 common/（app_config.h：功能开关 + 任务栈/优先级/周期宏）
  └── 不依赖 algo/（算法层当前为空）

当前数据流（都是"最新值快照"，非事件队列）：
  IMU100Hz ──写──▶ Yaw 快照 ─────────┐
  激光 UART2中断 ─▶ module/laser 距离 ─┼─读─▶ UIMENU（OLED 状态栏 + 右侧小球面板）
  上位机 UART0中断 ▶ module/vision 小球 ┘        │
                                                └─▶ RobotCore（6 题 onEnter/onLoop/onExit）

  （默认禁用链路）MOTOR ──写──▶ g_motorDiag ──读──▶ PERIPH（OLED 显示）
```

---

## 小车开发速查（电赛控制题）

### 我要写第 N 题，代码加在哪？

1. 打开 [`tasks/taskN.c`](tasks/)（第 N 题就在这个文件），在文件头注释写上本题要求。
2. 按本题流程改 **状态枚举**（如 `直行→路口→转弯→…→完成`）。
3. 在 `OnLoop()` 的 `switch(state)` 里，每个 `case` 写"**做什么动作** + **什么条件切到下一个状态**"。
   `OnLoop` 每 30ms 被调一次，做机动级决策足够（电机命令会直接下发，定距计步由底层 ISR 后台完成）。
4. `OnEnter` 里做一次性准备（使能电机、舵机归中、清零），`OnExit` 里急停+失能保证安全。
5. 题名/题数在 [`app_robot_core.c`](app_robot_core.c) 的 `s_robotTasks[]` 里改。

> "寻迹到路口就转弯""走够距离就停"这类判断，直接在对应 `case` 里写——你比框架更清楚具体阈值。

### 四路步进电机接口（[`../bsp/bsp_motor.h`](../bsp/bsp_motor.h)，四路完全独立）

| 需求 | 调用 | 说明 |
|---|---|---|
| 指定速度+方向（连续转） | `BspMotor_SetSpeedRpm(id, rpm)` | **rpm 正=小车前进方向/负=反转/0=立即停**；单位 RPM；命令直接下发；M1/M2 已在 BSP 完成取反标定 |
| 四轮一起给速度（差速/整车） | `BspMotor_SetSpeedRpm4(r1,r2,r3,r4)` | 一次设四路，便于写差速 |
| 兼容的立即速度接口 | `BspMotor_SetSpeedRpm4Immediate(r1,r2,r3,r4)` | 与常规速度接口相同，所有速度命令均立即以目标 RPM 运行，`0` 为立即停 |
| 指定脉冲/走固定距离 | `BspMotor_MoveSteps(id, steps, rpm)` | **steps 符号=方向、\|steps\|=脉冲数**；走完自动停 |
| 四轮一起定距（整车前进/原地转） | `BspMotor_MoveSteps4(s1,s2,s3,s4,rpm)` | 直行=四个 steps 相同；原地转=左右反号 |
| 判断动作是否走完 | `BspMotor_IsStopped(id)` / `BspMotor_AllStopped()` / `GetRemainingSteps(id)` | 状态机切状态的常用条件 |
| 使能 / 停止 | `BspMotor_EnableAll()` / `BspMotor_StopAll()` / `BspMotor_EmergencyStop(id)` | 起转前须先 EnableAll |
| 圈↔脉冲换算 | `BspMotor_StepsPerRev()` | = 200×细分（1/32→6400） |
| 左右镜像标定 | `BspMotor_SetDirInvert(id, true)` | 让"正 rpm=前进"对四轮统一成立 |

`id` 取 `BSP_MOTOR_1..BSP_MOTOR_4`。四路各占独立定时器（TIMG0/8/12/6）与独立 ZERO 中断计步，
互不影响；全部接口**非阻塞**、直接下发（调用即返回，运动在后台跑）。1/32 细分下安全转速约 5~300 RPM；直接高速起转可能失步，应先低速实测。

### 张大头 Emm42_V5.0 闭环步进接口（UART1 串口命令式，两层 API）

与上面的 `bsp_motor`（TMC2209 开环 STEP/DIR）是**两套完全不同的电机**：这一套 MCU 只发命令帧，驱动器内部闭环执行，不占定时器、不产生脉冲。**当前 4 路 TMC2209 开环电机暂不用于本轮开发**，代码保留未改动。

当前总线上 3 台设备，地址 1/2/3 对应角色见下表；**业务代码优先用角色层 [`emm42_robot.h`](../module/emm42/emm42_robot.h)**，不要直接记地址数字：

| 角色枚举 | 协议地址 | 部件 |
|---|---|---|
| `EMM42_ROBOT_LIFT` | 1 | 摆杆高低调节 |
| `EMM42_ROBOT_WHEEL_L` | 2 | 左轮 |
| `EMM42_ROBOT_WHEEL_R` | 3 | 右轮 |

| 需求 | 调用（角色层，推荐） | 说明 |
|---|---|---|
| 初始化 | `Emm42Robot_Init()` | 内部转调 `Emm42_Init()`，注册 UART1 回复中断；`App_Init` 已调用一次 |
| 使能 / 失能 | `Emm42Robot_Enable(id, true/false)` | 起转前必须先使能 |
| 指定速度+方向 | `Emm42Robot_SetSpeedRpm(id, ±rpm, acc)` | 正/负会按角色方向标定后转换为协议层 CW/CCW；`0`=转急停帧；`acc` 0=立即变速 |
| 指定速度模式（含 0 RPM） | `Emm42Robot_VelControl(id, ±rpm, acc)` | 正/负会按角色方向标定；`0`仍是速度模式帧，控制器按 `acc` 曲线减速到零，适合平缓停车 |
| 相对位置模式 | `Emm42Robot_MoveRelative(id, ±pulses, rpm, acc)` | 正/负会按角色方向标定；以当前位置为起点，`acc=0` 时立即到达设定速度 |
| 立即停止（单路） | `Emm42Robot_Stop(id)` | 急停 |
| 立即停止（3 路） | `Emm42Robot_StopAll()` | 背靠背 3 帧，**仅限 OnExit 等一次性安全收尾场景**，不要放 OnLoop |
| 取协议地址 | `Emm42Robot_GetAddr(id)` | 诊断打印用 |

⚠️ **方向标定/差速运动学状态**：`Emm42Robot_SetSpeedRpm` 会在角色层补偿方向；ID1（摆杆）正方向为连杆向下，ID2（左轮）已确认直通，ID3（右轮）已确认取反。左右轮差速运动学仍未实现；机械安装变化时只需更新 `emm42_robot.c` 的标定表。

底层协议接口（`module/emm42/emm42_v5.h`，只认地址，角色层内部转调这些）：`Emm42_Enable/SetSpeedRpm/VelControl/PosControl/StopNow/SyncMotion/ResetClogProtection/ResetCurPosToZero/ReadSysParams`，诊断用 `Emm42_GetTxFrameCount/GetRxByteCount/GetRxFrameCount/GetLastReply`（`GetRxByteCount()>0` 说明总线上至少有驱动器回话）。地址常量 `EMM42_ADDR_MOTOR1/2/3` + `EMM42_ADDR_BROADCAST(0)`。单路相对位置模式优先使用角色层 `Emm42Robot_MoveRelative()`；仅需绝对位置或同步多机时才绕过角色层直接调底层接口。

所有接口（两层都一样）**非阻塞、不等回复、不做延时**；
⚠️ 驱动器处理一帧需要时间，**不要在同一个 `OnLoop` 里连发多帧**——题目二、四、五均采用每 30ms 轮询拍最多下发一帧，左右轮交替更新。

### 固定半径差速圆弧接口（[`../module/diff_drive/diff_drive.h`](../module/diff_drive/diff_drive.h)）

日常直接调用 `DiffDrive_RunRadiusTurn(半径, 中心RPM)`：正半径左转、负半径右转，内部立即映射为 M1/M2 左侧同速、M3/M4 右侧同速，不经过梯形加速，也不会私自缩放输入速度。默认几何为左右轮距 201 mm、前后轮距 201 mm、胎宽 27 mm。调用示例和侧滑校正方法见 [`../docs/DIFF_DRIVE.md`](../docs/DIFF_DRIVE.md)。
