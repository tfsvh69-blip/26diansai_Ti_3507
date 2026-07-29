# FreeRTOS 任务列表

任务周期、优先级、栈大小集中定义在 `common/app_config.h`。

| 任务名 | 所在文件 | 周期 | 优先级 | 栈大小 | 输入 | 输出 | 说明 |
|---|---|---:|---:|---:|---|---|---|
| `LED1` | `app/app_led_task.c` | 300 ms | `APP_LED_TASK_PRIORITY` | `APP_LED_TASK_STACK_WORDS` | 无 | LED1(PB25) 翻转 | 当前已启动，用作 FreeRTOS 调度心跳 |
| `UART0TX` | `app/app_uart_test_task.c` | 10 ms 接收轮询 | `APP_UART_TEST_TASK_PRIORITY` | `APP_UART_TEST_TASK_STACK_WORDS` | UART0 RX 任意非换行字符 | 返回 `UART RX OK` | **【默认禁用，`APP_FEATURE_UART_ECHO=0`】** 调试时需串口收发验证可改回 1 启用 |
| `UIMENU` | `app/app_ui_task.c` | 30ms 按键轮询 | `APP_UI_TASK_PRIORITY` | `APP_UI_TASK_STACK_WORDS` | KEY1~4(PA28/PA31/PA30/PA29) 按下沿；**IMU Yaw 快照**(`AppImuUartTask_GetYaw`，⚠️`APP_FEATURE_IMU=0` 时恒返回 false)、**激光距离**(`LaserLd14_GetLatest`，⚠️`APP_FEATURE_LASER=0` 时恒返回 false)、**小球检测**(`BallParser_GetLatest`) | OLED(PB8/PB9 软件I2C) 题目菜单/运行界面 + 蜂鸣器(PA15)短促嘀声(~2ms) + 底部传感器状态栏 + 右侧小球面板 | **OLED 题目菜单 UI**：任一按键按下沿均短促嘀一声（同周期内忙等约 2ms 后立即关断）；菜单态 K1上移/K2下移(循环)、K3确认进入运行界面、K4返回菜单。当前 5 道题均为硬件/函数测试（非正式赛题）：题目一 `DIR TEST` 四轮方向核对、题目二 `ARC TEST` 差速圆弧、题目三 `GYRO 90L` 陀螺仪闭环左转 90°（✅ 已调通，⚠️当前 IMU 关闭无法测试，见下）、题目四 `SERVO SWP` 四路舵机 2s 间隔 0°↔270° 翻转、题目五 `EMM VEL` 张大头 Emm42_V5.0 闭环步进速度模式（UART1）。底部常驻**传感器/系统状态栏** `[R:ON/OFF ]Y:<yaw> D:<dist>mm`(局部低频刷；启用 `APP_FEATURE_RELAY` 时最前显示继电器逻辑状态，便于对照实际动作核对极性；IMU/激光关闭时 Yaw/D 显示 `---`)；启用 `APP_FEATURE_BALL_VISION` 时菜单右半常驻**小球检测文字面板** `BALL`/`F/n/x/y`(局部低频刷)。**独占 OLED 与 4 按键**，界面整屏刷为事件驱动 |
| `IMU100Hz` | `app/app_imu_uart_task.c` | 10 ms | `APP_IMU_UART_TASK_PRIORITY` | `APP_IMU_UART_TASK_STACK_WORDS` | ATK-MS6DSV/LSM6DSV16X FIFO 融合姿态 + 加速度/角速度输出寄存器、PA16 INT 电平、**激光测距1(`LaserLd14_GetLatest`)** | Yaw 快照(`AppImuUartTask_GetYaw`，供 OLED 状态栏)；串口遥测（由 `APP_FEATURE_IMU_UART_LOG` 独立控制，**默认 0=静默**，调试时改 1 恢复 5Hz 打印） | **【2026-07-29 临时禁用，`APP_FEATURE_IMU=0`】** 减少 CPU 占用（软件 I2C 100Hz 读取开销最大），需要陀螺仪/Yaw 数据时改回 1 即可，代码逻辑未删改。启用时：欧拉角每 10ms 读 + 临界区发布 Yaw 快照；串口打印由 `APP_FEATURE_IMU_UART_LOG` 门控（默认关，不刷任何串口）；IR 读取 + Yaw 发布始终运行，OLED 状态栏不依赖串口 |
| `MOTORTEST` | `app/app_motor_test_task.c` | 20ms 按键轮询 | `APP_MOTOR_TEST_TASK_PRIORITY` | `APP_MOTOR_TEST_TASK_STACK_WORDS` | KEY1/KEY2(PA28/PA31) 按下沿 | 四路 STEP(PB10/PB6/PB13/PB26)+四路 DIR(PB11/PB7/PB14/PB27)，PA13 ENN、MS1/MS2 共用；UART0 打印状态，更新 `g_motorDiag` 供 OLED 显示 | **【默认禁用，`APP_FEATURE_MOTOR=0`】** 按键让给 UIMENU。4 电机一起转测试：K1 全部正转2圈、K2 全部反转2圈；四路按目标速度直接开始，各自由 TIMG0/8/12/6 中断计步；1/32细分6400脉冲/圈 |
| `SERVOSWEEP` | `app/app_servo_test_task.c` | 20ms | `APP_SERVO_TEST_TASK_PRIORITY` | `APP_SERVO_TEST_TASK_STACK_WORDS` | 无（自动） | 四路 SERVO PWM(PA8/PA9/PB4/PA12)，TIMA0 50Hz | **【默认禁用，`APP_FEATURE_SERVO=0`】** 4 舵机各自独立错相摆动（800↔2200us，不用按键），演示四路可完全独立控制；每秒串口打印 `SERVO us S1=.. S2=.. S3=.. S4=..`。脉宽经 `BspServo_SetPulseUs` 极性补偿；四路方向须一次 `setCCPDirection` 写全(见下) |
| `PERIPH` | `app/app_periph_test_task.c` | 500 ms | `APP_PERIPH_TEST_TASK_PRIORITY` | `APP_PERIPH_TEST_TASK_STACK_WORDS` | `g_motorDiag`（只读） | OLED(PB8/PB9 软件I2C) 刷屏、LED2(PA7)/LED3(PB12) 翻转、蜂鸣器(PA15) 通断 | **【默认禁用，`APP_FEATURE_PERIPH_OLED=0`】** OLED 已交给 UIMENU，两任务抢软件 I2C 会花屏故互斥。原功能：OLED 显示标题/运行秒+LED+BUZZ+电机状态；LED2/LED3 交替心跳；上电自检 |
| `RELAYTEST` | `app/app_relay_test_task.c` | 2000 ms | `APP_RELAY_TEST_TASK_PRIORITY` | `APP_RELAY_TEST_TASK_STACK_WORDS` | 无（自动） | 继电器 RELAY(PA24) 吸合/断开 | **【默认禁用，`APP_FEATURE_RELAY_SELFTEST=0`】** 仅上电自检用：置 1 后每 2 秒自动切换吸合/断开验证继电器及电磁铁负载。⚠️ 每次切换都真实通断电磁铁。**正常运行不跑此任务**，继电器由业务代码经 `bsp_relay` 接口(`BspRelay_On/Off/Set/Toggle/IsOn`)按需控制；OLED 状态栏(`APP_FEATURE_RELAY=1`)照常显示当前吸合/断开态。高电平吸合(极性宏可反相)，上电默认断开 |
| `NRF24TX` | `app/app_nrf24_tx_test_task.c` | 500 ms | `APP_NRF24_TX_TEST_TASK_PRIORITY` | `APP_NRF24_TX_TEST_TASK_STACK_WORDS` | NRF24L01+ 自动应答状态；固定对端地址 `15 52 33 54 55` | USB 无线串口收到 `TMX NRF24 TEST 000001` 形式的递增文本 | **【2026-07-29 临时禁用，`APP_FEATURE_NRF24_TX_TEST=0`】** 减少 CPU 占用，需要无线串口测试时改回 1 即可，代码逻辑未删改。启用时：上电等待 100ms 后初始化为实机验证参数：2.402GHz、2Mbps、0dBm、16位CRC、32字节固定载荷；IRQ 不接，软件轮询 STATUS 并带 12ms 超时；模块掉线后自动重试初始化；UART0 不输出 NRF 调试日志 |

## NRF24L01+ 连续发射测试（`NRF24TX` 任务）

**【2026-07-29 临时禁用，`APP_FEATURE_NRF24_TX_TEST=0`】** 需要无线串口发射测试时改回 1 即可，代码逻辑未删改。

- 任务每 500ms 发送一次 ASCII 文本，成功收到自动应答后序号递增：`TMX NRF24 TEST 000001`、`...000002`。
- USB 无线串口 V2.0 使用固定 32 字节无线载荷：`payload[0]` 为有效文本长度，`payload[1]` 起为正文，剩余字节清零；不能直接发送普通 C 字符串。
- 实机验证参数：地址 `{0x15,0x52,0x33,0x54,0x55}`、2.402GHz、2Mbps、0dBm、16位CRC、自动重传间隔500us/最多10次。TX 地址与 RX 通道0地址保持相同，以接收自动应答。
- 未接 IRQ；驱动用软件轮询 `STATUS.TX_DS/MAX_RT` 判断结果。轮询有 12ms 上限，模块拔掉不会永久卡死任务；SPI/超时异常会在下一个周期重新初始化。
- 电脑端打开 USB 模块串口（截图为 9600 8N1）即可持续看到文本。上位机的“本地地址”必须与板端发送地址一致，射频频率、空中速率和 CRC 也必须一致。

## NRF24L01+ 模块接口

头文件：`module/nrf24l01/nrf24l01.h`。

```c
bool Nrf24_Init(const Nrf24RadioConfig_t *radioConfig);
Nrf24TxResult_t Nrf24_SendPayload(const uint8_t payload[32]);
Nrf24TxResult_t Nrf24_SendUsbUartText(const uint8_t *text, uint8_t textLength);
```

- `g_nrf24UsbUartV20Config`：已验证可直接传给 `Nrf24_Init()` 的 USB 无线串口 V2.0 参数。
- `Nrf24_SendPayload()`：发送完整的 32 字节 NRF 固定载荷；调用方自行组织每个字节。
- `Nrf24_SendUsbUartText()`：发送 1~31 字节文本，内部自动组装 USB 无线串口协议的长度字节和 32 字节零填充载荷，后续业务优先使用此接口。
- 返回值 `NRF24_TX_OK` 表示对端已自动应答；`NRF24_TX_MAX_RETRY` 表示无线侧未收到应答；`NRF24_TX_TIMEOUT` 或 `NRF24_TX_IO_ERROR` 后应重新调用 `Nrf24_Init()`。
- 不需要串口日志；如需定位，可在 Keil Watch 查看只读快照 `g_nrf24Diag`，其中包含初始化结果、寄存器回读和收发计数。

## LED1(PB25) 心跳灯行为

- `App_Init()` 会启动 `LED1` 任务。
- LED1(PB25) 每 300ms 翻转一次；若持续闪烁，说明 FreeRTOS 调度至少已经运行。
- 串口任务不再控制 LED1，避免心跳判断被串口命令干扰。
- LED2/LED3/蜂鸣器/OLED 由 `PERIPH` 任务驱动，与 LED1 心跳互不干扰。

## OLED 题目菜单 UI（`UIMENU` 任务）

- 目的：上电即在 OLED 上做题目选择界面，用 4 个按键选题、确认、返回；取代旧的 KEY1/KEY2 电机测试与外设自检显示。
- **独占资源**：本任务独占板载 OLED（PB8/PB9 软件 I2C）与 KEY1~4；启用它时 `PERIPH`（也刷 OLED）、`MOTORTEST`（占 KEY1/KEY2）默认关闭，避免抢屏/抢键（见 `common/app_config.h` 的 `APP_FEATURE_*`）。
- **两态状态机**：
  - `MENU` 菜单态：标题 `== SELECT TASK ==` + 题目列表（`n.名称`），当前项整行**反色高亮**；底部提示 `K1/K2 K3=OK K4=BK`。题目多于一屏（6 项）时按选中项自动滚动。启用 `APP_FEATURE_BALL_VISION` 时，标题收窄为 `TASKS`、高亮与题目文字收窄到左半屏(宽 60)，右半让给小球面板（见下「右侧小球检测面板」）。
  - `RUN` 运行态：大字 `TASK n` + 题名 + `K4: back to menu`；周期调用该题 `onLoop` 钩子。题目五运行时题名行改显示 `ID1 EN/RUN/STOP`、`ID2 ...`、`ID3 ...` 或 `EMM DONE`，方便观察当前测试对象。
- **按键（30ms 轮询，去抖 + 按下沿）**：
  - `K1`(PA28) 上移（循环回绕）、`K2`(PA31) 下移（循环回绕）
  - `K3`(PA30) 确认：进入选中题目运行界面（先调 `onEnter`）
  - `K4`(PA29) 返回：从运行界面回菜单（先调 `onExit`）；菜单态无动作
- **题目表**：题名/题数登记在 `app_robot_core.c` 的 `s_robotTasks[]`（题目一 `DIR TEST`、题目二 `LINE PID`、题目三 `GYRO 90L`）；**各题业务钩子 `OnEnter/OnLoop/OnExit` 实现在 [`../app/tasks/`](../app/tasks/) 的 `taskN.c`**。题目一按 M1→M2→M3→M4 单轮正向各转两圈，核对安装位置和方向；题目二按 8 路灰度传感器计算偏差并 PID 调节 Emm42 的 ID2 左轮、ID3 右轮速度，进行正式循迹，ID1 摆杆不参与。题目三为陀螺仪闭环左转 90° 冒烟测试：进入后记录起始 Yaw，以 PID 原地转向使 Yaw 减小 90°，到位后停车保持。实车标定 M1/M2 已在 BSP 取反，故正 RPM/正 steps 对四路均代表小车前进。题目二速度命令加速度档位为 0（立即生效，不做曲线加减速），参数在 `task2.c` 顶部；UART1 速度帧左右轮交替下发。UIMENU 只负责显示与按键，经 robot_core 分发调用。
- **刷屏策略**：软件 I2C 整屏刷约 50ms，故只在选中项/状态变化时才重绘（事件驱动），平时仅轻量轮询按键，CPU 友好。
- **传感器状态栏**（`Y:<yaw> D:<dist>mm`）：常驻底部（菜单态 Y=56、运行态 Y=48），实时显示陀螺仪 Yaw（度，1 位小数）与激光测距（mm），方便一眼判断两个传感器是否在工作。
  - 数据来源：Yaw 取 `IMU100Hz` 任务发布的线程安全快照 `AppImuUartTask_GetYaw()`（IMU 未就绪显示 `---`）；距离取 `LaserLd14_GetLatest()`（无有效帧显示 `---`）。UI 任务**不直接访问软件 I2C/激光**，避免与 IMU 任务争用总线。
  - **不影响实时性的做法**：状态栏用 `OLED_UpdateArea` **只局部刷一行**(128×8≈整屏 1/8)，且每 `APP_UI_STATUS_DIVIDER`(默认 10 拍=300ms) 才刷一次；整屏刷仍只在菜单/运行切换时发生。故周期刷屏的软件 I2C 忙等极小，不拖累按键响应与其它任务。
- **OLED 只能显示 ASCII**：`OLED_Data.h` 的中文字库 `OLED_CHARSET_GB2312` 处于注释禁用状态且无 `OLED_ShowChinese` 接口，故题名用英文/编号；要中文需另做字模并启用字库。

### 右侧小球检测面板（`APP_FEATURE_BALL_VISION`）

- 目的：利用菜单空出的右半屏，常驻显示上位机经 UART0 下发的 `$BALL` 小球检测结果（见 `MESSAGE_LIST.md`）。
- 布局（128×64，6×8 字体；仅**菜单态**显示）：右半自 x=66 起，一条竖分隔线(x=63) + 顶部 `BALL` 头 + 四行字段：
  - `F:YES`/`F:no`（found；还没收到合法帧显 `F: ?`）
  - `n:<count>`（本帧球总数）
  - `x:<x>` / `y:<y>`（主目标球心像素；未检测到球或无有效帧显 `---`）
- 数据来源：`BallParser_GetLatest()`（线程安全快照），由 UART0 RX 中断解析（见下「小球检测 `$BALL`」小节）。
- 刷新：字段区(x66,y8,62×32)用 `OLED_UpdateArea` **只局部刷**，与底部状态栏同频（`APP_UI_STATUS_DIVIDER`，默认 300ms），面积小、频率低，不打断按键响应；`BALL` 头与分隔线只在整屏刷（进入菜单/切换）时画。
- 运行态(RUN)不显示该面板（整屏归题目自身）；关闭 `APP_FEATURE_BALL_VISION` 时菜单恢复整行高亮与整宽标题，右半留空。

### 右侧循迹面板（`APP_FEATURE_LINE_TRACK`）

> ✅ 2026-07-25 已上板实测通过，功能正常。

- 目的：利用菜单右半**下部第 6/7 行**（BALL 面板占第 1~5 行，两者错开不重叠），常驻显示 8 路灰度循迹（PB17~PB24=LINE1~LINE8）的实时高低电平状态。
- 布局（128×64，6×8 字体；仅**菜单态**显示）：右半自 x=66 起，共用同一条竖分隔线(x=63)：
  - 第 6 行(y=40)：通道号 `87654321`（左=8 号=小车左，右=1 号=小车右）
  - 第 7 行(y=48)：按 LINE8→LINE1 对应状态，如 `10011100`；与通道号逐位对齐——`1`=识别到线、`0`=未识别（屏上左右即小车物理左右）
- 数据来源：`BspLine_ReadAll()`（`bsp/bsp_line.c`，一次读全 8 路打包成位图 bit0=LINE1…bit7=LINE8）。OLED 显示时按 bit7→bit0 倒序，使硬件最左的 LINE8 显示在最左。当前模块极性「识别到线=低电平」，`BSP_LINE_ACTIVE_LOW=1` 已将其归一化为 OLED 上 `1`=识别到线；如换模块极性相反，改该宏即可整体反相。
- 刷新：两行区(x66,y40,62×16)用 `OLED_UpdateArea` **只局部刷**，与底部状态栏、BALL 面板同频（`APP_UI_STATUS_DIVIDER`，默认 300ms），面积小、频率低，不打断按键响应；分隔线只在整屏刷时画。
- 运行态(RUN)不显示该面板；关闭 `APP_FEATURE_LINE_TRACK`（且 BALL 也关）时菜单恢复整行高亮与整宽标题，右半留空。
- ⚠️ PB17~PB24 **非 5V 容忍**：灰度模块信号须 3.3V 电平，否则需分压/电平转换（见 `HARDWARE_WIRING.md` 与接线文档风险 R2）。

### 题目业务说明

各题的具体实现在 `app/tasks/taskN.c`，通过 `app_robot_core.c` 的 dispatch 表登记后被 UIMENU 调用。

**题目一 `DIR TEST`**（`task1.c`）：
- 进入后按 M1→M2→M3→M4 顺序单轮正向各转 2 圈（60 RPM），每一路完全停稳后才切换。
- 目的：核对四轮安装位置（M1=左前、M2=左后、M3=右前、M4=右后）和方向标定（M1/M2 已 BSP 取反）。
- 四轮走完后停车保持，K4 退出急停失能。

**题目二 `LINE PID`**（`task2.c`，8 路灰度循迹，✅ 已上车测试通过）：
- 输入：`BspLine_ReadAll()` 的 8 位状态，位图固定为 bit7=LINE8（物理最左）…bit0=LINE1（物理最右）。任务按物理左右做加权平均，误差范围为 -7（最左）到 +7（最右）；误差为正时左轮加速、右轮减速，使小车向右修正。
- PID 参数（`T2_KP`/`T2_KI`/`T2_KD`）、积分限幅 `T2_INTEGRAL_LIMIT`、转向输出限幅 `T2_MAX_STEER_RPM` 由用户持续在实车上调参，具体数值以 `task2.c` 顶部当前值为准，不要以文档里记的历史数字为准。
- **入场节拍化状态机**（`Task2_OnEnter()` 只复位变量，不直接下发 Emm42 命令；所有命令都在 `OnLoop` 里按拍发出）：
  1. `T2_STATE_RESET_DISABLE`：先失能 ID2/ID3（两帧间隔 `T2_EMM_CMD_GAP_MS`），清掉上一次残留的使能/速度状态；
  2. `T2_STATE_RESET_WAIT`：等待 `T2_RESET_SETTLE_TICKS` 拍；
  3. `T2_STATE_ENABLE_LEFT`/`_WAIT`：使能 ID2，等待 `T2_ENABLE_SETTLE_TICKS` 拍；
  4. `T2_STATE_ENABLE_RIGHT`/`_WAIT`：使能 ID3，同样等待 `T2_ENABLE_SETTLE_TICKS` 拍；
  5. 转入 `T2_STATE_RUN`。
  **`T2_ENABLE_SETTLE_TICKS` 有实测故障数据支撑，不要调到 3 拍(90ms) 以下**：3 拍时约有一半概率使能不生效（驱动器来不及处理刚发的使能帧），6 拍(180ms) 才验证稳定。`T2_RESET_SETTLE_TICKS` 没有类似的失败案例，是偏保守加上的，需要压缩入场总延时时优先调这个。
- 输出：经 `Emm42Robot_SetSpeedRpm()` 控制 UART1 总线上的 ID2=`EMM42_ROBOT_WHEEL_L` 与 ID3=`EMM42_ROBOT_WHEEL_R`；ID1 摆杆不下发任何命令。运行中每个 30ms 周期只发送一帧，左右轮交替更新，避免共享总线背靠背丢帧。
- 速度参数：基础速度 `T2_BASE_RPM`，单轮限幅 `[T2_MIN_WHEEL_RPM, T2_MAX_WHEEL_RPM]`——`T2_MAX_WHEEL_RPM` 必须 `>= T2_BASE_RPM` 且总跨度 `>= 2*T2_MAX_STEER_RPM`，否则会被 `Task2_ClampWheelPair` 或直接顶限压缩转向差速。速度命令加速度档位 `T2_EMM_ACC`（0~255，说明书 §6.3.1）不用 0——实测 0 会让车身突兀抖动，改用非 0 档位让驱动器内部把每次变速摊成短暂曲线；PID 算出的目标 RPM 每拍直接下发，任务层不再叠加软件斜坡。
- 差速安全限幅：`Task2_ClampWheelPair()` 对左右轮目标 RPM 做"整体平移"限幅而非各自独立 clamp——独立 clamp 会在外侧轮触顶时压扁差速、内侧轮却不受影响，导致弯道实际转向权限远小于 PID 算出的量，是"低速正常、高速冲出弯道"的典型根因；整体平移能在夹到上下限的同时保留左右轮差值，只有差速本身超过 `[T2_MIN_WHEEL_RPM,T2_MAX_WHEEL_RPM]` 总跨度才被迫压缩。
- 丢线保护：短时丢线维持上次转向；连续 `T2_LINE_LOST_TICKS` 拍仍无任何有效通道时，平滑减速停车。重新压线后 PID 自动复位并恢复循迹。
- 抗震荡：误差先做一阶低通滤波（`T2_ERROR_FILTER_ALPHA`，越小滤波越强）再喂给 PID，抑制离散传感器命中路数切换导致的台阶式误差跳变被微分项放大成尖峰转向；滤波后再过一道死区 `T2_ERROR_DEADBAND`，`|误差|` 小于此值直接当 0，专门抑制车身在赛道中心附近命中路数在相邻两档（如 2 路/3 路）来回跳变引起的小幅左右摆动。
- 转弯减速：基础速度按 `dynBase = T2_BASE_RPM − T2_CORNER_SLOWDOWN_GAIN×|转向输出|` 动态下调（下限 `T2_MIN_BASE_RPM`），直道保持全速、弯道自动放慢。
- 定时减速：秒表校准后时间（`Task2_GetElapsedMs()`，跟 OLED 显示同一套换算）达到 `T2_DECEL_START_MS`（固定 14500ms）后，基础速度按 `T2_BASE_RPM − T2_DECEL_GRADIENT_RPM_PER_SEC×(超过阈值的秒数)` 线性下降，下限 `T2_DECEL_MIN_RPM`；跟转弯减速算出的 `dynBase` 取更小值生效，两者不冲突。可调参数：`T2_DECEL_GRADIENT_RPM_PER_SEC`（梯度系数，越大降速越陡）、`T2_DECEL_MIN_RPM`（能降到的最低基础速度）。
- 终点检测：先连续 `T2_FINISH_ARM_TICKS` 拍命中路数 `≤T2_FINISH_ARM_HIT_MAX` 才"武装"终点检测（避免出发瞬间仍压在宽线上被立即误判为跑完一圈）；武装后再连续 `T2_FINISH_HIT_TICKS` 拍命中路数 `≥T2_FINISH_HIT_MIN`（6/7/8 路均算），判定到达终点，立即硬停车（`Emm42Robot_Stop`）并锁定在 `T2_STATE_FINISHED`，之后不再重新进入循迹，等待 K4 手动退出。
- 秒表计时：`s_elapsedTicks` 从 `Task2_OnEnter()` 起每拍（30ms）累加，到 `T2_STATE_FINISHED` 后停止累加（定格）；`Task2_GetUiStatus()` 输出 `"T:12.3s"`（终点后追加 `" DONE"`），在 `app_ui_task.c` 里复用 task5 的 `UI_RUN_NAME_Y` 显示位（`UI_TASK2_INDEX`），每 `APP_UI_STATUS_DIVIDER`（约300ms）刷新一次，运行界面首次绘制也会立即显示 `T:0.0s`。显示毫秒数会乘一个 `T2_STOPWATCH_CAL_SCALE` 校准系数——实测计时比真实时间偏快（怀疑跟 FreeRTOS tick 依赖的主频跟工程假设的 80MHz 有偏差有关，根因未定位，见 `AI_MEMORY.md`），先用系数硬补偿；如果实测偏差比例继续变化，按"新系数 = 当前系数 × (最新实测秒数/最新显示秒数)"滚动修正即可，不用每次从 1.0 重新推导。
- K4 退出时对 ID2/ID3 先急停再失能（轮子无重力负载，失能不会溜车，比保持力矩更省电安全）；ID1 摆杆不属于本题，不下发任何命令。K4 退出与终点急停这两处需要背靠背给两轮下发命令，帧间用 `vTaskDelay(T2_EMM_CMD_GAP_MS)` 隔开，避免共享 UART1 总线互相干扰丢帧（emm42_v5.h 协议层明确要求连续下发需自行留间隔）。

**题目三 `GYRO 90L`**（`task3.c`，✅ 冒烟测试已通过）：
- 进入后记录起始 Yaw，以 PID 闭环原地转向（左侧后退、右侧前进），使 Yaw 减小 90°（左转）。
- 到位后（误差 < 1° 连续 450ms）停车保持，K4 退出急停失能。
- 已验证 PID 参数（`task3.c` 顶部）：`T3_KP=1.5`、`T3_KI=0.1`、`T3_KD=0.2`；到位阈值 1.0°、最小 RPM=8、输出限幅 ±120 RPM、积分限幅 ±40 RPM。
- Yaw 取自 `AppImuUartTask_GetYaw()`（厘度 0.01°，范围 −180°~+180°）。角度差使用最短路径算法（`AngleDiffCd()`）处理 ±180° 跨界跳变。
- 复位初始 Yaw 不同不影响闭环——本题使用相对角度（目标 = 起始 − 90°）。
- 当前为单环 PID 冒烟测试版本：dt 固定 30ms、无前馈、到位后被动保持（步进使能自带保持力矩）。后续可按需要加入前馈/死区 PID/主动漂移补偿。

**题目四 `SERVO SWP`**（`task4.c`，舵机全行程翻转测试）：
- 四路舵机每 2 秒同时在 0°（500us）和 270°（2500us）之间翻转，验证舵机全行程覆盖和 PWM 独立性。
- 使用 `BspServo_SetPulseUsRange()` 临时放宽安全限幅到 500~2500us（正常业务仍走默认 800~2200us 安全区间）。
- 可调参数（`task4.c` 顶部）：`T4_PULSE_MIN_US=500`、`T4_PULSE_MAX_US=2500`、`T4_TOGGLE_TICKS=67`（67 拍 × 30ms ≈ 2s）。
- K4 退出时四路归中（1500us）后停 PWM。
- ⚠️ 若某舵机在极端脉宽有异响/堵转，说明机械行程不到 270°，收窄 `T4_PULSE_MIN_US`/`T4_PULSE_MAX_US` 即可。

**题目五 `EMM VEL`**（`task5.c`，张大头 Emm42_V5.0 闭环步进电机【方向标定测试】，2026-07-29 改为依次单路测试）：
- 硬件：UART1（PA17=TX → 驱动器 RX，PB5=RX ← 驱动器 TX），115200 8N1，v1.1 排针 H7；**当前 3 台驱动器挂同一总线**，靠设备地址区分：地址1=摆杆高低调节(LIFT)、地址2=左轮(WHEEL_L)、地址3=右轮(WHEEL_R)。
- **两层 API**：`module/emm42/emm42_v5.c` 是纯协议层（只认地址，不知道地址对应哪个部件）；`module/emm42/emm42_robot.c` 在协议层之上按本车实际用途包一层角色映射（`Emm42RobotId_t`: `EMM42_ROBOT_LIFT/WHEEL_L/WHEEL_R`），`task5.c` 只经 `Emm42Robot_*` 接口调用，不直接碰地址数字。
- ⚠️ **本题当前用途 = 方向标定观察，不是最终业务动作**：依次让 ID1→ID2→ID3【单独】以【正 RPM】转 3 秒再停，一次只转一路，便于逐个肉眼确认"正 RPM 对应哪个物理方向"（摆杆抬升还是下降、左/右轮朝小车前进还是后退方向转）。三路（或已启用的几路）测完后保持停车、**不循环**，等使用者观察记录后按 K4 退出。正 RPM 会先经过 `Emm42Robot_SetSpeedRpm()` 的角色方向标定：ID2 已确认直通，ID3 已确认取反，ID1 尚待确认且暂按直通；后续只需改 `emm42_robot` 层的标定表，不用改 `task5.c`。
- 动作序列（单路 4 小节）：使能当前角色 → 等待 `T5_ENABLE_SETTLE_TICKS`(≈300ms) 使驱动器处理使能帧 → 以 `T5_TEST_RPM` 正转 `T5_RUN_TICKS`(≈3s) → 停止 `T5_STOP_TICKS`(≈1s) 间歇 → 切下一角色，如此走完 ID1→ID2→ID3。首路 ID1 同样经过等待，避免其速度帧过早下发。K4 退出时 `Emm42Robot_StopAll()` 背靠背下发 3 帧急停（一次性安全收尾，不发失能帧，保持力矩防止摆杆下坠/轮子溜车）。
- OLED 反馈：运行界面题名行每 300ms 显示当前角色与阶段；进入后可依次看到 `ID1 EN`/`ID1 WAIT`/`ID1 RUN`/`ID1 STOP`、`ID2 ...`、`ID3 ...`，全部完成后显示 `EMM DONE`。因此 ID1 已纳入测试且其执行阶段可见。
- 一次只有一路在转，帧节奏很宽松（每路仅 3 帧：使能/转/停），不存在"一个 OnLoop 里连发多帧"的问题，不需要像并行测试那样按拍强制错开。
- 与 `bsp_motor`（TMC2209 开环 STEP/DIR）本质不同：本电机是**串口命令式闭环驱动**，MCU 不产生脉冲、不占定时器，发完命令即返回，速度/位置由驱动器内部执行。**当前 4 路 TMC2209 开环电机（bsp_motor，task1~3 用）暂不用于本轮开发，代码保留未改动。**
- 可调参数（`task5.c` 顶部）：`T5_TEST_RPM=60`、`T5_ACC=10`（0=立即变速）、`T5_ENABLE_SETTLE_TICKS=10`（使能后等待300ms）、`T5_RUN_TICKS=100`（3s）、`T5_STOP_TICKS=34`（1s）、`T5_ENABLE_LIFT/WHEEL_L/WHEEL_R=1`（未接全 3 台时把对应角色置 0 跳过，直接测下一路）。
- 协议模块 `module/emm42/emm42_v5.c`：命令帧 `[地址][功能码][参数(大端)][校验0x6B]`，含速度模式 `0xF6`、位置模式 `0xFD`、使能 `0xF3`、急停 `0xFE`、同步 `0xFF`、读参数等。
- ⚠️ 上板前先用 USB-TTL 分别把 3 台驱动器地址设成 1、2、3，并确认波特率为 115200（若设成 38400，改 `ti_msp_dl_config.h` 里 `SYSCFG_DL_UART_1_init` 使用的 IBRD/FBRD，两组值都已算好）。
- ⚠️ 电机不转的排查顺序：① 量 PA17 有无数据波形；② `Emm42_GetRxByteCount()` 是否 >0（>0 说明总线上至少有驱动器回话，链路双向通；恒为 0 查地址/波特率/TX-RX 接反）；③ 驱动器 4S 电源是否接入（Emm42_V5.0 需 12~36V，只给 3V3 逻辑电不会转）。
- ⚠️ 接线说明风险 R1：多台驱动器的 TX 并联在 PB5 上（现为 3 台，原文档述两台，风险等比放大），需外部肖特基（BAT54S）做线与 + 4.7kΩ 上拉后才可同时接；改造完成前调试期建议只接 1~2 台，配合 `T5_ENABLE_*` 跳过未接的角色。
- ⚠️ 第 3 台设备的物理接线点：v1.1 PCB 的 H7 排针按接线文档只设计了 Motor1/Motor2 两个物理连接头，第 3 台需要额外从同一 UART1 总线（PA17/PB5）引出，具体接法待确认后补充到 `pcb引脚配置文档/v1.1/机器人控制板_接线说明.md`（该文档是硬件权威来源，本仓库不代为修改）。

## PERIPH 外设测试行为

- 上电自检：LED2(PA7)、LED3(PB12) 同时点亮 + 蜂鸣器短响 200ms，随后熄灭。
- 稳态循环（500ms）：
  - LED2/LED3 每拍交替翻转（同相），直观表明任务在正常调度。
  - 蜂鸣器每 4 拍（2s）通一拍再断，做通断控制测试。
  - OLED 刷屏显示三行：
    - 第1行：`3507 MOTOR1 v1.1`（标题）
    - 第2行：`T:XXXXXs L:2/3/- B:~/_`（运行秒 + LED2/LED3 状态 + 蜂鸣器状态）
    - 第3行：`M1:RUN  FWD L3` / `M1:STOP REV L1` 等（电机1 运行/方向/速度档）
  - 电机状态通过只读 `g_motorDiag`（`running`/`dirForward`/`param`）获取，由 MOTOR1 任务写入。
- OLED 为板载软件 I2C（PB8/PB9），与 IMU 软件 I2C（PB2/PB3）、TMC 细分（PB0/PB1）均无引脚冲突。

## 步进电机驱动（`bsp_motor.c`，四路完全独立，v1.9）

- 硬件：四路 STEP 各占独立定时器 CCP0（M1=TIMG0/PB10、M2=TIMG8/PB6、M3=TIMG12/PB13、M4=TIMG6/PB26），四路 DIR=PB11/PB7/PB14/PB27；ENN/MS1/MS2 四路共用。
- **控制模型（四路独立）**：每路各开**自己的 ZERO 中断**（`TIMG0/8/12/6_IRQHandler`）完成定距计步，互不影响 → 可各自不同速度/方向/距离（小车差速/转弯的前提）。全部接口非阻塞且直接下发，不做加减速。
  - 定时器周期限制为 125~8000（32kHz~500Hz）；1/32 细分下安全转速约 5~300 RPM。直接高速起转可能失步，先用低 RPM 实机验证。
  - CPU：四路同时高速时中断量约为单路 4 倍（各路 = 步频次/秒），常规巡航速度下开销很小。
- **对外接口（RPM 单位，正负号定方向）**：
  - `BspMotor_SetSpeedRpm(id, rpm)` 连续转（rpm 正=正/负=反/0=立即停）；`BspMotor_SetSpeedRpm4(...)` 一次设四路
  - `BspMotor_MoveSteps(id, steps, rpm)` 定距（steps 符号=方向、|steps|=脉冲数，走完自动停）
  - `BspMotor_IsStopped(id)` / `GetRemainingSteps(id)` 判完成；`BspMotor_EnableAll/StopAll/EmergencyStop`；`BspMotor_SetDirInvert(id,inv)` 左右镜像标定；`BspMotor_StepsPerRev()` 圈↔脉冲
  - `id` = `BSP_MOTOR_1..4`。速查表见 [`../app/README.md`](../app/README.md)。

### 电机测试任务 `MOTORTEST`（默认禁用，按键已让给 UIMENU）

- 目的：单独验证四路电机都正常。用新独立接口对四路同时下发相同的 `BspMotor_MoveSteps`。
- 按键（20ms 轮询按下沿，移动期间忽略按键）：`KEY1`=四电机各自正转 2 圈、`KEY2`=各自反转 2 圈（120 RPM）。
- 运行状态发布到 `g_motorDiag`（取电机1为代表），供 `PERIPH` 任务在 OLED 只读显示。
- 串口：上电 `MOTOR test: K1=...`；触发 `MOTORx4 key start`，完成 `MOTORx4 done`；每秒 `MOTORx4 DIAG m1_run=x m1_left=y`。

## 舵机测试（4 舵机各自独立摆动，`SERVOSWEEP` 任务）

- 目的：验证四路舵机都正常，并演示"4 个舵机可完全独立控制"。四路共用 TIMA0 50Hz，各通道有独立比较值(CCP0/1/2/3)、脉宽互不影响。
- 行为：不用按键，四路以**不同起始相位**在 800↔2200us 之间各自三角波来回摆动（约 2.8s 单程），所以任意时刻四个舵机处于不同角度，直观体现独立。
- **独立控制**：单独控制某一路直接 `BspServo_SetPulseUs(BSP_SERVO_x, us)`，只动那一路、其它不变。
- 串口：每秒打印 `SERVO us S1=.. S2=.. S3=.. S4=..`（四值互不相同即证明独立）。
- **脉宽范围**：安全范围收窄到 **800~2200us**（中心 1500，见 bsp_servo.h），给舵机机械行程留余量；极值不灵是舵机机械限位所致，非 MCU 问题（补偿在任何脉宽都精确）。
- **两个避坑（都已修）**：
  1. 脉宽只经 `BspServo_SetPulseUs()`（内部 `period-pulseUs` 极性补偿，EDGE_ALIGN 下高电平=period-CC；直接写 CC=pulseUs 会反相、舵机不动）。
  2. 四路 CCP 方向必须**一次** `DL_Timer_setCCPDirection(TIMA0, CC0|CC1|CC2|CC3_OUTPUT)` 写全——该函数整体覆盖 CCPD 寄存器，分 4 次单独调用只有最后一次(CC3)生效，会导致只有舵机4能动、舵机1/2/3 信号线浮 0.3V（2026-07-16 实测根因）。

## 激光测距1（UART2 RX 中断，无独立任务）

**【2026-07-29 临时禁用，`APP_FEATURE_LASER=0`】** 减少 CPU/中断占用（230400bps 持续流是当前中断触发最频繁的一路），需要激光测距数据时改回 1 即可，代码逻辑未删改；关闭后 `App_Init` 不再调用 `BspUart2_Init`，UART2 RX 中断不会被使能，`LaserLd14_GetLatest()` 恒返回 false（OLED 状态栏显示 `D:---`）。

- 不是 FreeRTOS 任务，而是 **UART2 接收中断**：`UART2_IRQHandler`（`bsp_uart.c`）把 RX FIFO 取空并逐字节喂给 `LaserLd14_FeedByte()`（`module/laser/laser_ld14.c`）。
- 选中断而非任务轮询的原因：激光 230400 波特率**连续外发**，一帧 195 字节约 8.5ms 内字节连续到达，RX FIFO 仅几字节深，任务轮询必然溢出丢字节；中断按字节及时取走才不丢。
- `App_Init()` 中先 `LaserLd14_Reset()` 复位解析器，再 `BspUart2_Init(LaserLd14_FeedByte)` 注册回调并使能 RX 中断+NVIC（UART2 外设已在 `BspBoard_Init` 的 `SYSCFG_DL_UART_2_init` 里初始化）。
- 解析器攒满一整帧(195B)校验通过后，取 12 点非零距离平均为单值 `distanceMm`，连同 `valid`/`frameOkCnt`/`crcErrCnt`/`rxBytes` 供上层 `LaserLd14_GetLatest()` 读取。
- **输出与陀螺仪合并**：距离由 `IMU100Hz` 任务在打印整行时追加 `D1=<mm>mm`，与欧拉角/加减速度同一行、5Hz 刷新，满足"跟陀螺仪一起发、频率不高、便于阅读"。
- ISR 内不调用任何非 FromISR 的 FreeRTOS API（符合 CLAUDE.md FreeRTOS 规则）。

## 小球检测 `$BALL`（UART0 RX 中断，无独立任务）

- 不是 FreeRTOS 任务，而是 **UART0 接收中断**：`UART0_IRQHandler`（`bsp_uart.c`）把 RX FIFO 取空并逐字节喂给 `BallParser_FeedByte()`（`module/vision/ball_parser.c`）。
- 选中断而非任务轮询的原因：上位机 `$BALL` 报文约 **17 帧/秒连续下发**，每帧约 25 字节（约 425 B/s），RX FIFO 仅 4 字节深，10~30ms 任务轮询节拍必然溢出丢字节；中断按字节及时取走才不丢。
- `App_Init()` 中先 `BallParser_Reset()` 复位解析器，再 `BspUart0_SetRxHandler(BallParser_FeedByte)` 注册回调并使能 RX+溢出中断+NVIC（UART0 外设已在 `BspBoard_Init` 的 `SYSCFG_DL_UART_0_init` 里初始化，RX FIFO 阈值设为 1 字节）。
- 解析器行缓冲：`$` 起始、`\r`/`\n` 结束；成行后校验 `$`..`*` 间字符逐字节 XOR 与两位十六进制一致，再解析前缀 `BALL,` 与 `found,x,y,n` 四字段，通过则更新快照，失步/校验失败自动等待下一个 `$` 重同步。
- 输出：由 `UIMENU` 任务读取 `BallParser_GetLatest()` 在 OLED 右侧文字面板显示（见「右侧小球检测面板」）。
- 由 `APP_FEATURE_BALL_VISION` 门控；与 `APP_FEATURE_UART_ECHO`（UART0 轮询自检）**互斥**——二者都占 UART0 RX，同时置 1 会被 `app_config.h` 的编译期 `#error` 拦截。
- ISR 内不调用任何非 FromISR 的 FreeRTOS API（符合 CLAUDE.md FreeRTOS 规则）。

## UART0 多任务共享（递归互斥量）

- 旧实现用 `vTaskSuspendAll` 挂起整个调度器保证整行日志原子，一行 IMU 日志约 10ms 会冻结全部任务，造成按键/舵机/OLED 卡顿。
- 现改为递归互斥量（`bsp_uart.c`，`BspUart0_Init` 在 `BspBoard_Init` 中于调度器启动前创建）：
  - 发送方仅独占 UART0，不再冻结调度器，其余任务照常按时间片轮转，从根上消除全局卡顿。
  - `BspUart0_SendString` 单次调用自动加锁即原子；多段拼接由 `BspUart0_Lock/Unlock` 包裹，递归类型允许嵌套不自锁。

## 当前串口测试行为

- `main()` 在 `BspBoard_Init()` 后立即输出 `BOOT: board init ok`，此时还未创建任务、未进入 FreeRTOS 调度。
- `main()` 在 `App_Init()` 返回后输出 `BOOT: start scheduler`，随后才启动 FreeRTOS 调度器。
- 上电后输出 `UART0 RX READY, LED1 heartbeat active`。
- 收到任意非换行字符后返回 `UART RX OK`。
- 换行和回车会被忽略，避免串口助手自动追加换行造成重复提示。

## IMU100Hz 姿态输出

**【2026-07-29 临时禁用，`APP_FEATURE_IMU=0`】** 减少 CPU 占用（软件 I2C 100Hz 读取是当前任务里开销最大的一个），需要陀螺仪/Yaw 数据（含依赖它的题目三 `GYRO 90L`）时改回 1 即可，代码逻辑未删改；关闭后 `IMU100Hz` 任务不会被创建，`AppImuUartTask_GetYaw()` 恒返回 false（OLED 状态栏显示 `Y:---`）。

- 上电后任务先输出 `IMU UART 100Hz START, SWI2C addr=0x6A`。
- 初始化成功输出 `IMU INIT OK`，紧接着输出一行单位说明 `IMU FORMAT: R/P/Y=deg AX/AY/AZ=mg GX/GY/GZ=mdps D1=mm(激光测距1)`。
- 随后按 **5Hz(每 20 个读取周期)** 输出整行 `IMU R=... P=... Y=... AX=... AY=... AZ=... GX=... GY=... GZ=... FIFO=... D1=<mm>mm INT=...`。
- 其中 `D1` 为激光测距1(UART2)，单位 mm；尚未收到有效激光帧时显示 `D1=---`（排查见 `HARDWARE_WIRING.md` 激光测距1 小节）。
- 其中 `R/P/Y` 为融合欧拉角(度，2 位小数)；`AX/AY/AZ` 为三轴加速度(mg，±2g 量程直读)；`GX/GY/GZ` 为三轴角速度(mdps，±125dps 量程直读)。加速度/角速度由输出寄存器直读(不经 FIFO)。
- **欧拉角数值范围**：`P`(pitch)、`Y`(yaw) 为 **-180 ~ +180 度**；`R`(roll) 由 `asin` 解算，范围只有 **-90 ~ +90 度**。三者都会在 ±180 处环绕跳变(如 +179°→-179°)。注意 `GX/GY/GZ` 是角速度(mdps)不是角度，范围 ±125000，不在 -180..180。
- ⚠️ 后续用 yaw 做累计转角/PID/积分时，必须对角度做 unwrap 或取最短差值(把 `target-current` 规范到 ±180)，否则 ±180 跳变点会让控制量瞬间打满。
- 【方案A CPU 优化】加速度/角速度当前只用于串口显示、无 100Hz 消费者，故只在「要打印那一拍」(20Hz)才读；软件 I2C 一次 6 字节读≈1ms 且忙等，两组≈2ms/周期，改到 20Hz 后读取开销从 ~20% CPU 降到 ~4%。将来算法需要 100Hz 原始数据时，把 `ReadImuRaw` 移回每周期并优先恢复硬件 I2C 提速。
- 初始化失败每 1s 输出 `IMU INIT FAIL:n STEP=...` 并自动重试初始化，其中 `1` 为 ID 不匹配，`2` 为 I2C 通信失败，`3` 为配置或复位超时；若初始化阶段成功读到 WHO_AM_I，会追加 `LAST_ID=0x..`。
- 当前初始化默认跳过 `RESTORE_CTRL_REGS`/boot reset，`STEP=RESET_SKIP` 属于预期步骤；原因是实测 `RESET_SET` 会导致 SDA 被拉低。
- 初始化失败时同时输出 `IMU WHOAMI 0x6A=... 0x6B=...`；正常 LSM6DSV16X 应读到 `0x70`。
- 初始化失败时还会切到 GPIO 软件 I2C，并给 SCL 输出 18 个恢复脉冲，再输出 `IMU BUS SCL=... SDA=... STAT=...`；恢复后 SCL/SDA 应都为 `1`，否则优先检查短路、接反、模块供电或上拉。
- 初始化首次失败和之后每 5 次失败会输出 `IMU SCAN: ...`，当前只通过写入 `WHO_AM_I` 寄存器地址探测 `0x6A/0x6B`，避免异常状态下全地址扫描刷出假 ACK。
- 若输出 `IMU SCAN: bus stuck ...`，表示 SCL/SDA 没有释放到空闲高电平，此时不会继续扫描，避免 SDA 低电平造成全地址假 ACK。
- 融合欧拉角走 FIFO，无新 SFLP 数据时保留上一次角度值；加速度/角速度走寄存器直读，不受 FIFO 影响。若角度长时间冻结但 `AX/GX` 仍在变化，说明 SFLP 融合链路异常，而非整条 I2C 断掉。
- `FIFO=0/1/2` 间歇变化正常，原因是芯片内部 SFLP 为 120Hz，而任务按 100Hz 读取；若 FIFO 长时间持续增大或长时间为 0 且角度不更新，才需要继续排查。
- `APP_IMU_I2C_PIN_TEST_ENABLE` 当前为 `0`，`IMU100Hz` 正常访问 IMU；若临时置为 `1`，任务不访问 IMU，改为每 500ms 用开漏模拟方式交替翻转 PB2/PB3，并直接读取 GPIO DIN 输出 `SET`/`READ` 电平。

## 机器人题目核心模块（`app_robot_core.c/h`）

- **不是独立 FreeRTOS 任务**——是一组同步钩子函数，被 `UIMENU` 任务在 RUN 态调用。
- **只做登记 + 分发**：`app_robot_core.c` 里的 **dispatch 表 `s_robotTasks[]`** 登记 6 道题的 name + `OnEnter/OnLoop/OnExit` 函数指针；**各题业务代码在 [`../app/tasks/`](../app/tasks/) 的 `taskN.c`**（第 N 题 = `taskN.c`，每题一套 `状态枚举 + OnLoop switch 状态机骨架`；当前 task1、task2 仅为硬件/函数测试，分别测试 M1→M4 单轮正方向和固定半径差速圆弧，task3~6 为待填骨架）。
- **按键反馈**：`UIMENU` 对 KEY1~KEY4 的每一次按下沿统一短响 30ms；题目核心不再单独鸣叫，K3 不会双响。
- **接口**：
  - `RobotCore_GetTaskCount()` — 题目总数（菜单滚动循环用）
  - `RobotCore_GetTaskName(idx)` — 题目显示名（OLED 菜单/运行界面显示）
  - `RobotCore_EnterTask(idx)` — 进入题目（调 `onEnter`）
  - `RobotCore_LoopTask(idx)` — 每周期循环（由 UIMENU 30ms 节拍驱动，调 `onLoop`）
  - `RobotCore_ExitTask(idx)` — 退出题目（调 `onExit`）
  - `RobotMaster_Start()` — 机器人总任务入口（占位，后续做按顺序自动执行全部 6 题）
- **调试日志**：`EnterTask`/`ExitTask`/`RobotMaster_Start` 仅在 `APP_FEATURE_IMU_UART_LOG=1` 时向 UART0 打印（默认静默模式不刷串口）。

### 按键反馈

`UIMENU` 对 KEY1~KEY4 的每个按下沿统一给一次**蜂鸣器短响 30ms**反馈；鸣叫不阻塞按键轮询。旧版按题错开的 LED2/LED3 组合已取消，改由各题 `taskN.c` 自行按需驱动 LED/OLED。

### 题目开发指南（v1.9：每题一个文件）

1. 打开 [`../app/tasks/`](../app/tasks/) 下对应题号的 **`taskN.c`**（第 N 题就在这个文件），在文件头写本题要求；
2. 按本题流程改**状态枚举**（如 直行→路口→转弯→…→完成）；
3. 在 `OnLoop()` 的 `switch(state)` 里逐状态写"**动作 + 切换条件**"（`OnLoop` 每 30ms 调一次，做机动级决策足够；电机命令直接下发，定距计步由 ISR 后台完成）；
4. `OnEnter` 做一次性准备（`BspMotor_EnableAll`、舵机归中、清零），`OnExit` 急停+失能（`BspMotor_StopAll`/`DisableAll`）保证安全；
5. 需要高频控制环的题目，在 `OnEnter` 里 `xTaskCreate` 自己的任务、`OnExit` 里 `vTaskDelete` 销毁；
6. 改题名/题数只改 `app_robot_core.c` 的 `s_robotTasks[]`，无需动 UI。
7. 底层接口速查（电机/舵机/传感器）见 `app/tasks/app_tasks.h` 顶部注释与 [`../app/README.md`](../app/README.md)。
