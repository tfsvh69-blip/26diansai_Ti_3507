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
| `empty/algo/` | 纯算法（PID、滤波、数学解算，当前为空） |
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
| `UIMENU` | `app/app_ui_task.c` | 30 ms 轮询 | **OLED 题目菜单 UI**：4 键(K1上/K2下/K3确认/K4返回)选题并进入运行界面，**独占 OLED 与 KEY1~4**；题目业务委托 `app_robot_core` → `app/tasks/taskN.c` 的 `OnEnter/OnLoop/OnExit`（task1 示例、task2~6 骨架待填） |
| `IMU100Hz` | `app/app_imu_uart_task.c` | 10 ms | 读取 ATK-MS6DSV 姿态 + 追加激光测距1(D1)，按 5Hz 整行输出 Roll/Pitch/Yaw/加减速度/D1 |
| `MOTORTEST` | `app/app_motor_test_task.c` | 20 ms 轮询 | **【默认禁用】** KEY1/KEY2 让 4 个电机（各自独立接口同时下发）正/反转 2 圈测试（按键已让给 UIMENU） |
| `SERVOSWEEP` | `app/app_servo_test_task.c` | 20 ms | **【默认禁用】4 个舵机各自独立错相摆动**（800~2200us，无按键）；单控用 `BspServo_SetPulseUs(id,us)` |
| `PERIPH` | `app/app_periph_test_task.c` | 500 ms | **【默认禁用】** OLED 已交给 UIMENU（互斥防抢屏）；原为外设验证：OLED/LED2/LED3/蜂鸣器 |

> 激光测距1（UART2）不是任务，而是 **UART2 RX 中断**逐字节喂 `module/laser` 的 LD14 解析器；距离由 `IMU100Hz` 任务读取并随整行输出。
> 小球检测（UART0）同样不是任务，而是 **UART0 RX 中断**逐字节喂 `module/vision` 的 `$BALL` 解析器（上位机→下位机，NMEA+XOR）；结果由 `UIMENU` 读取显示在 OLED 右侧文字面板。见 [empty/docs/MESSAGE_LIST.md](empty/docs/MESSAGE_LIST.md) 的 `$BALL` 报文小节。
> 题目菜单 UI 见 [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md) 的「OLED 题目菜单 UI」小节；**题目业务逻辑逐题填 [empty/app/tasks/](empty/app/tasks/) 的 `taskN.c`**（第 N 题 = `taskN.c`），题名/题数登记在 `app_robot_core.c` 的 `s_robotTasks[]`。四路步进电机为**各自独立**驱动（`bsp_motor.h`，RPM 单位、带符号定方向）。OLED 当前只能显示 ASCII（无中文字库）。

### 功能总开关（按需启用外设）

[empty/common/app_config.h](empty/common/app_config.h) 顶部有一组 `APP_FEATURE_*`（1=启用/0=禁用），`App_Init` 用它门控各任务创建。用开发板时把不需要的外设置 0 即可（不创建任务、不占 CPU、不刷串口；板级硬件初始化仍保留）：`APP_FEATURE_LED_HEARTBEAT / UART_ECHO / UI_MENU / PERIPH_OLED / SERVO / MOTOR / IMU / IMU_UART_LOG / LASER / BALL_VISION / LINE_TRACK / RELAY / RELAY_SELFTEST`。**当前默认**：`UI_MENU=1`（题目菜单）；因 UI 独占 OLED 与按键，`PERIPH_OLED / SERVO / MOTOR` 默认置 0。`UART_ECHO=0`（关接收自检）、`IMU_UART_LOG=0`（IMU 不打印串口遥测——串口彻底静默；但 IMU 读取 + Yaw 快照照常运行供 OLED 状态栏）。调试时把这两个改回 1 即可恢复串口输出。`BALL_VISION=1`（UART0 RX 中断解析上位机 `$BALL` 报文，OLED 右侧文字面板显示）——它与 `UART_ECHO` 争用 UART0 RX，二者互斥（同时置 1 编译期 `#error` 拦截）。`LINE_TRACK=1`（7 路灰度循迹 PB17~PB23，OLED 菜单右半显示状态）。继电器开关**已解耦**：`RELAY=1`（继电器 PA24 功能——板级初始化 + OLED 状态栏 `R:ON/OFF` 显示 + 对外接口 `BspRelay_*` 可直接调用），`RELAY_SELFTEST=0`（每 2s 自动切换的自检任务 `RELAYTEST` 默认关；正常运行继电器由业务代码经 `bsp_relay` 接口按需控制、不自动切换，上电自检时才置 1）。

修改或新增任务后必须同步更新 [empty/docs/FREERTOS_TASKS.md](empty/docs/FREERTOS_TASKS.md)。

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
| 电机1 STEP/DIR | PB10/PB11 | TIMG0_CCP0 / GPIO，低=正向（各路独立） |
| 电机2 STEP/DIR | PB6/PB7 | TIMG8_CCP0 / GPIO（独立调速调向） |
| 电机3 STEP/DIR | PB13/PB14 | TIMG12_CCP0 / GPIO（独立调速调向） |
| 电机4 STEP/DIR | PB26/PB27 | TIMG6_CCP0 / GPIO（独立调速调向） |
| TMC 使能 ENN | PA13 | 低有效，四路共用 |
| TMC 细分 MS1 | PB0 | 四路共用细分 |
| TMC 细分 MS2 | PB1 | 四路共用细分 |
| 按键 KEY1~4 | PA28/PA31/PA30/PA29 | 按下接地，内部上拉 |
| UART0 TX | PA10 | MFCLK 4MHz，115200 8N1 |
| UART0 RX | PA11 | — |
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
