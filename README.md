# 3507 小车控制工程（MSPM0G3507 + FreeRTOS）

面向 **2026 全国大学生电子设计竞赛控制题（小车）** 的下位机工程：以 TI MSPM0G3507（Cortex-M0+，80MHz）为主控，跑 FreeRTOS，驱动 **4 个步进电机 + 4 个舵机**，配合陀螺仪、激光测距、寻迹、上位机视觉等传感器完成寻迹、定距、转弯、抓取等赛题动作。

> 当前版本 **v1.9**：四路步进电机改为**完全独立**的 RPM 驱动；6 道赛题业务拆成**每题一个文件**的状态机骨架，方便快速定位与编写。详见下文与 [empty/docs/](empty/docs/)。

---

## 快速信息

| 项 | 值 |
|---|---|
| 主控 | TI MSPM0G3507（Cortex-M0+，MCLK 80MHz，外部 40MHz 晶振 HFXT 锁定） |
| RTOS | FreeRTOS V11.3.0，ARM_CM0 移植层，heap_4，tick 1000Hz(1ms) |
| 构建 | **Keil uVision**（工程见下），不依赖 TI SysConfig |
| 调试串口 | UART0 = PA10(TX)/PA11(RX)，115200 8N1 |

---

## 构建与烧录

工程文件：[empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx](empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx)

1. 用 Keil uVision 打开上面的 `.uvprojx`；
2. `Build (F7)` 编译，`Download` 烧录到板子；
3. `BeforeMake` 已关闭 `syscfg.bat`，**无需安装 TI SysConfig** 即可编译。

上电后打开串口助手（115200 8N1，无流控），应依次看到：

```
BOOT: board init ok
BOOT: MCLK 80MHz <- HFXT 40MHz OK
BOOT: start scheduler
```

随后 PB25(LED1) 每 300ms 闪烁（心跳灯，说明调度器在跑），OLED 进入题目菜单界面。

> 命令行编译（独立进程，不受 Keil GUI 影响）：
> `UV4.exe -b empty_LP_MSPM0G3507_nortos_keil.uvprojx -t empty_LP_MSPM0G3507_nortos_keil -o build.log`，看日志 `0 Error(s)` 即成功。

---

## 目录结构

```
3507mode_text_motor/            仓库根
├── README.md                   本文件（项目总入口）
├── CLAUDE.md                   给 AI 协作者的工程约定与踩坑记录
├── pcb引脚配置文档/            权威硬件接线图（v1.0 / v1.1，引脚以此为准）
├── syscfg.bat                  SysConfig 脚本（构建时已停用）
└── empty/                      工程主体
    ├── app/                    FreeRTOS 任务、业务流程、控制状态机
    │   └── tasks/              ★ 各赛题代码：第 N 题 = taskN.c（含状态机骨架）
    ├── bsp/                    板级外设封装（GPIO/UART/电机/舵机/按键/延时）
    │   └── board/              手写 DriverLib 板级初始化（ti_msp_dl_config.c/h）
    ├── module/                 可复用模块（imu / oled / laser / vision）
    ├── algo/                   纯算法（PID/滤波/解算，当前为空）
    ├── common/                 app_config.h（功能开关 + 任务栈/优先级/周期）
    ├── docs/                   维护文档（任务表/消息表/接线表/上下文/调试指南）
    └── third_party/            FreeRTOS 内核、TI DriverLib、ST 传感器驱动
```

启动流程：`main()` → `BspBoard_Init()`（裸机初始化外设）→ `App_Init()`（按开关建任务）→ `vTaskStartScheduler()`。
入口见 [empty/app/main.c](empty/app/main.c)、[empty/app/app_main.c](empty/app/app_main.c)。

---

## 两大核心子系统

### 1. 四路独立步进电机驱动（[empty/bsp/bsp_motor.h](empty/bsp/bsp_motor.h)）

四个电机各占一个独立定时器（TIMG0/8/12/6）+ 各自中断做梯形加减速与精确计步，**可各自不同速度/方向/距离**（小车差速/转弯前提）。接口用 **RPM 单位、正负号定方向**，全部非阻塞：

```c
BspMotor_EnableAll();                          // 起转前先使能四路驱动

BspMotor_SetSpeedRpm(BSP_MOTOR_1, 150);        // 连续转：正=正转/负=反转/0=平滑停
BspMotor_SetSpeedRpm4(150, -150, 150, -150);   // 四轮一次给速度（差速）
BspMotor_MoveSteps(BSP_MOTOR_1, 6400, 120);    // 定距：走 6400 脉冲(=1圈@1/32) 后自动停
BspMotor_MoveSteps4(s1, s2, s3, s4, 120);      // 四轮一起定距（直行/原地转）

BspMotor_IsStopped(BSP_MOTOR_1);               // 判单路走完
BspMotor_AllStopped();                         // 判四路都停（状态机切状态用）
BspMotor_StopAll();                            // 平滑停全部
```
`id` 取 `BSP_MOTOR_1..4`。1/32 细分下安全转速约 5~300 RPM。完整速查见 [empty/app/README.md](empty/app/README.md#小车开发速查电赛控制题)。

### 2. 赛题状态机框架（[empty/app/tasks/](empty/app/tasks/)）

**第 N 题的代码就在 `empty/app/tasks/taskN.c`**。每题实现三个钩子，由 OLED 题目菜单 UI 通过 [app_robot_core.c](empty/app/app_robot_core.c) 分发调用：

- `TaskN_OnEnter()`  进入本题一次性初始化（使能电机、清零、舵机归中）；
- `TaskN_OnLoop()`   进入后每 30ms 调用一次——在 `switch(state)` 里写状态机主体；
- `TaskN_OnExit()`   返回菜单时急停 + 失能，保证安全。

每个 `taskN.c` 已给好 **状态枚举 + OnLoop 的 switch 骨架**：按本题流程增删状态（直行→路口→转弯→…→完成），在每个 `case` 里写"做什么动作 + 什么条件切下一个状态"。task1 是可直接改的示例，task2~6 为待填骨架。开发说明与底层接口速查见 [empty/app/tasks/app_tasks.h](empty/app/tasks/app_tasks.h) 顶部注释。

---

## 任务与功能开关

各任务的启用/禁用由 [empty/common/app_config.h](empty/common/app_config.h) 顶部的 `APP_FEATURE_*` 门控。**当前默认**：LED1 心跳 + OLED 题目菜单（独占 OLED 与 4 按键）+ IMU 姿态读取（静默不刷串口）+ 两路中断接收器（激光测距、上位机小球报文）。默认禁用的电机/舵机/外设测试任务需要时置 1 即可。

| 任务/机制 | 文件 | 说明 |
|---|---|---|
| `LED1` 心跳 | [app_led_task.c](empty/app/app_led_task.c) | PB25 每 300ms 翻转，判断调度器是否在跑 |
| `UIMENU` 题目菜单 | [app_ui_task.c](empty/app/app_ui_task.c) | 4 键选题/运行，底部 Yaw/距离状态栏，右侧小球面板 |
| 赛题业务 | [app/tasks/taskN.c](empty/app/tasks/) | 6 题状态机（经 [app_robot_core.c](empty/app/app_robot_core.c) 分发） |
| `IMU100Hz` 姿态 | [app_imu_uart_task.c](empty/app/app_imu_uart_task.c) | 读六轴 + 发布 Yaw 快照供 OLED |
| 激光测距（UART2 中断） | [module/laser](empty/module/laser/) | LD14 解析，非任务 |
| 小球视觉（UART0 中断） | [module/vision](empty/module/vision/) | 解析上位机 `$BALL` 报文，非任务 |

详见 [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md)。

---

## 硬件接线（摘要）

> **铁律**：任何涉及引脚/外设复用的操作，必须先查权威接线图 [pcb引脚配置文档/v1.1/机器人控制板_接线说明.md](pcb引脚配置文档/v1.1/)，不得凭代码或记忆推断。

| 硬件 | 引脚 |
|---|---|
| LED1/LED2/LED3 | PB25 / PA7 / PB12 |
| 蜂鸣器 | PA15 |
| OLED（软件 I2C） | SCL=PB9 / SDA=PB8 |
| 步进电机 1~4 STEP | PB10 / PB6 / PB13 / PB26（TIMG0/8/12/6 CCP0） |
| 步进电机 1~4 DIR | PB11 / PB7 / PB14 / PB27 |
| TMC 共用 ENN/MS1/MS2 | PA13 / PB0 / PB1 |
| 舵机 1~4 PWM | PA8 / PA9 / PB4 / PA12（TIMA0 50Hz） |
| 按键 KEY1~4 | PA28 / PA31 / PA30 / PA29 |
| UART0 TX/RX | PA10 / PA11（115200） |
| 激光测距 UART2 RX | PB16（230400） |
| ATK-MS6DSV IMU（软件 I2C） | SCL=PB2 / SDA=PB3 |

完整接线、电气说明、排查备注见 [empty/docs/HARDWARE_WIRING.md](empty/docs/HARDWARE_WIRING.md)。

---

## 文档索引

| 文档 | 用途 |
|---|---|
| [empty/app/README.md](empty/app/README.md) | app 层阅读指南 + **小车开发速查**（电机 API、第 N 题加在哪） |
| [empty/bsp/README.md](empty/bsp/README.md) | bsp 层封装说明（UART/电机/舵机/IMU 端口） |
| [empty/docs/PROJECT_CONTEXT.md](empty/docs/PROJECT_CONTEXT.md) | 架构、硬件、主要模块、CPU 占用预算 |
| [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md) | 任务表、周期/优先级/栈、题目开发指南 |
| [empty/docs/HARDWARE_WIRING.md](empty/docs/HARDWARE_WIRING.md) | 硬件接线、引脚复用、排查备注 |
| [empty/docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md) | 跨任务/串口报文定义（含 `$BALL`） |
| [empty/docs/UART_DEBUG_GUIDE.md](empty/docs/UART_DEBUG_GUIDE.md) | UART 配置原则与乱码排查流程 |
| [CLAUDE.md](CLAUDE.md) | 工程编码约定、关键约束、AI 协作指引 |
