# FreeRTOS 任务列表

任务周期、优先级、栈大小集中定义在 `common/app_config.h`。

| 任务名 | 所在文件 | 周期 | 优先级 | 栈大小 | 输入 | 输出 | 说明 |
|---|---|---:|---:|---:|---|---|---|
| `LED1` | `app/app_led_task.c` | 300 ms | `APP_LED_TASK_PRIORITY` | `APP_LED_TASK_STACK_WORDS` | 无 | LED1(PB25) 翻转 | 当前已启动，用作 FreeRTOS 调度心跳 |
| `UART0TX` | `app/app_uart_test_task.c` | 10 ms 接收轮询 | `APP_UART_TEST_TASK_PRIORITY` | `APP_UART_TEST_TASK_STACK_WORDS` | UART0 RX 任意非换行字符 | 返回 `UART RX OK` | **【默认禁用，`APP_FEATURE_UART_ECHO=0`】** 调试时需串口收发验证可改回 1 启用 |
| `UIMENU` | `app/app_ui_task.c` | 30ms 按键轮询 | `APP_UI_TASK_PRIORITY` | `APP_UI_TASK_STACK_WORDS` | KEY1~4(PA28/PA31/PA30/PA29) 按下沿；**IMU Yaw 快照**(`AppImuUartTask_GetYaw`，⚠️`APP_FEATURE_IMU=0` 时恒返回 false)、**激光距离**(`LaserLd14_GetLatest`，⚠️`APP_FEATURE_LASER=0` 时恒返回 false)、**小球检测**(`BallParser_GetLatest`) | OLED(PB8/PB9 软件I2C) 题目菜单/运行界面 + 蜂鸣器(PA15)短促嘀声(~2~3ms) + 底部传感器状态栏 + 右侧小球面板 | **OLED 题目菜单 UI**：任一按键按下沿、题目二到达终点或题目五自动停车完成，均调用同一个短促提示音；菜单态 K1上移/K2下移(循环)、K3确认进入运行界面，K4 在菜单态无功能（原"启停钢球居中到 X=320"的调试入口已于 2026-08-01 移除，PA24 引脚已改接 ID1 归零限位开关）；运行态 K4 返回菜单。当前 5 道题均为硬件/函数测试（非正式赛题）：题目一 `VIDEO 5S` 通过视觉协议录制 5 秒无叠加标注的正常画面、题目二 `LINE PID` 正式循迹、题目三 `Task 3` 经 UART1 仅控制 Emm42 ID1 正反各约 500ms 低速测试、题目四 `LINE 6S` 复用题目二循迹并在累计前进 6.5 秒后缓停、题目五 `Five` 横向停止线后 0.5 秒循迹并线性缓停、题目六 `ID1 POS` 对 Emm42 ID1 做位置模式 API 冒烟测试。底部常驻**传感器/系统状态栏** `Y:<yaw> D:<dist>mm`(局部低频刷；IMU/激光关闭时 Yaw/D 显示 `---`)；启用 `APP_FEATURE_BALL_VISION` 时菜单右半常驻**小球检测文字面板** `BALL`/`F/n/x/y`(局部低频刷)。**独占 OLED 与 4 按键**，界面整屏刷为事件驱动 |
| `BALLCTRL` | `app/app_ball_control_task.c` | 10ms 轮询；约 30fps 新 X 更新控制 | `APP_BALL_CONTROL_TASK_PRIORITY` | `APP_BALL_CONTROL_TASK_STACK_WORDS` | 长度 1 覆盖队列的启停/目标 X；视觉 `xSeq/xPixel` | Emm42 ID1 **绝对位置模式**；闭环状态快照（`commandPulse`） | **默认启用但上电 OFF**。由题目三/四经 `AppBallControl_RequestTargetWithProfile()` 按需启停各自目标（菜单已不提供 K4 启停入口，见「OLED 题目菜单 UI」小节）。**2026-07-31 由速度模式切到位置模式**：`pulseDelta = Kx*error − Kv*v_filter` 纯 PD（无 I 项），每帧直接下发绝对目标脉冲 `targetPulse = LEVEL_TRIM_PULSE + SIGN*pulseDelta`。同日追加**静摩擦夹紧**：题目六实测钢球静摩擦阈值约 3200~4000 脉冲（`BALL_CTRL_STICTION_PULSE`），中等误差下 `Kx*error` 常够不到这个阈值导致"抬到一半僵住"，现在球基本没动（`|velocity|≤BALL_CTRL_STUCK_VELOCITY_PXPS`）且量级不足时直接把 `pulseDelta` 顶到阈值（保留方向）。`BALL_CTRL_SETTLE_DEADBAND_PX=3px` 是到位保持区：范围内回水平且禁止静摩擦夹紧，只有超出该精度才重新驱动，避免近端来回抖动。使能完成后进入 `ZERO` 状态，**只在本次上电后第一次启动时**发位置清零（`hasZeroedSinceBoot` 函数级 static），之后反复进出题目三/四调参沿用同一原点，重新上电才建立新原点（配合 2026-08-01 新增的 ID1 开机自动归零，第一次清零的位置就是归零终点，见「ID1 开机自动归零」章节）。单帧 NA 不再改变状态；连续两帧 NA 或 220ms 无有效 X 才判定 `LOST`，**仅影响 OLED 显示，不下发任何停止命令**（摆杆保持在最后一次有效目标，位置模式本身安全）；唯一仍主动下发命令的保护是画面边缘（命令回水平并锁定 `FAULT_EDGE`，等 K4 处理）。详见 `BALL_CONTROL.md` |
| `IMU100Hz` | `app/app_imu_uart_task.c` | 10 ms | `APP_IMU_UART_TASK_PRIORITY` | `APP_IMU_UART_TASK_STACK_WORDS` | ATK-MS6DSV/LSM6DSV16X FIFO 融合姿态 + 加速度/角速度输出寄存器、PA16 INT 电平、**激光测距1(`LaserLd14_GetLatest`)** | Yaw 快照(`AppImuUartTask_GetYaw`，供 OLED 状态栏)；串口遥测（由 `APP_FEATURE_IMU_UART_LOG` 独立控制，**默认 0=静默**，调试时改 1 恢复 5Hz 打印） | **【2026-07-29 临时禁用，`APP_FEATURE_IMU=0`】** 减少 CPU 占用（软件 I2C 100Hz 读取开销最大），需要陀螺仪/Yaw 数据时改回 1 即可，代码逻辑未删改。启用时：欧拉角每 10ms 读 + 临界区发布 Yaw 快照；串口打印由 `APP_FEATURE_IMU_UART_LOG` 门控（默认关，不刷任何串口）；IR 读取 + Yaw 发布始终运行，OLED 状态栏不依赖串口 |
| `MOTORTEST` | `app/app_motor_test_task.c` | 20ms 按键轮询 | `APP_MOTOR_TEST_TASK_PRIORITY` | `APP_MOTOR_TEST_TASK_STACK_WORDS` | KEY1/KEY2(PA28/PA31) 按下沿 | 四路 STEP(PB10/PB6/PB13/PB26)+四路 DIR(PB11/PB7/PB14/PB27)，PA13 ENN、MS1/MS2 共用；UART0 打印状态，更新 `g_motorDiag` 供 OLED 显示 | **【默认禁用，`APP_FEATURE_MOTOR=0`】** 按键让给 UIMENU。4 电机一起转测试：K1 全部正转2圈、K2 全部反转2圈；四路按目标速度直接开始，各自由 TIMG0/8/12/6 中断计步；1/32细分6400脉冲/圈 |
| `SERVOSWEEP` | `app/app_servo_test_task.c` | 20ms | `APP_SERVO_TEST_TASK_PRIORITY` | `APP_SERVO_TEST_TASK_STACK_WORDS` | 无（自动） | 四路 SERVO PWM(PA8/PA9/PB4/PA12)，TIMA0 50Hz | **【默认禁用，`APP_FEATURE_SERVO=0`】** 4 舵机各自独立错相摆动（800↔2200us，不用按键），演示四路可完全独立控制；每秒串口打印 `SERVO us S1=.. S2=.. S3=.. S4=..`。脉宽经 `BspServo_SetPulseUs` 极性补偿；四路方向须一次 `setCCPDirection` 写全(见下) |
| `PERIPH` | `app/app_periph_test_task.c` | 500 ms | `APP_PERIPH_TEST_TASK_PRIORITY` | `APP_PERIPH_TEST_TASK_STACK_WORDS` | `g_motorDiag`（只读） | OLED(PB8/PB9 软件I2C) 刷屏、LED2(PA7)/LED3(PB12) 翻转、蜂鸣器(PA15) 通断 | **【默认禁用，`APP_FEATURE_PERIPH_OLED=0`】** OLED 已交给 UIMENU，两任务抢软件 I2C 会花屏故互斥。原功能：OLED 显示标题/运行秒+LED+BUZZ+电机状态；LED2/LED3 交替心跳；上电自检 |
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
  - `MENU` 菜单态：标题 `== SELECT TASK ==` + 题目列表（`n.名称`），当前项整行**反色高亮**；底部提示 `K1/K2 K3=OK`。题目多于一屏（6 项）时按选中项自动滚动。启用 `APP_FEATURE_VISION_LINK` 时，标题收窄为 `TASKS`、高亮与题目文字收窄到左半屏(宽 60)，右半让给视觉通信面板（见下）。
  - `RUN` 运行态：大字 `TASK n` + 题名 + `K4: back to menu`；周期调用该题 `onLoop` 钩子。题目五运行时题名行显示 `ARM`、`LINE`、`HOLD`、`GO:<剩余秒数>`、`DEC:<剩余秒数>` 或 `DONE`，便于观察横线后的停车流程。
- **按键（30ms 轮询，去抖 + 按下沿）**：
  - `K1`(PA28) 上移（循环回绕）、`K2`(PA31) 下移（循环回绕）
  - `K3`(PA30) 确认：进入选中题目运行界面（先调 `onEnter`）
  - `K4`(PA29)：运行态返回菜单（先调 `onExit`）；菜单态无功能（原"启停后台钢球居中闭环，目标 `X=320`"的调试入口已于 2026-08-01 移除，PA24 引脚已改接 ID1 归零限位开关，见「ID1 开机自动归零」章节）
- **题目表**：题名/题数登记在 `app_robot_core.c` 的 `s_robotTasks[]`（题目一 `VIDEO 5S`、题目二 `LINE PID`、题目三 `Task 3`、题目四 `LINE 6S`、题目五 `Five`、题目六 `ID1 POS`）；**各题业务钩子 `OnEnter/OnLoop/OnExit` 实现在 [`../app/tasks/`](../app/tasks/) 的 `taskN.c`**。题目一不驱动电机，向视觉端发起一次录像并在约 5 秒后自动停止，保存无叠加标注的正常画面；题目二、四均按 8 路灰度传感器计算偏差并 PID 调节 Emm42 的 ID2 左轮、ID3 右轮速度；题目四还会同时通过 ID1 平衡钢珠至 `X=320`。题目三当前仅进行 ID1 位置模式的第一阶段钢珠定位：`X=325→415±10px`；题目四在正常循迹累计前进 6.5 秒后执行带加速度的速度模式 0 RPM 缓停。UIMENU 只负责显示与按键，经 robot_core 分发调用。
- **参数隔离**：同一算法可以在不同任务复用，但每个任务必须拥有独立可调参数宏和 profile，禁止跨任务直接共用增益、死区、滤波、静摩擦、速度或保持条件。任务三使用 `T3_STAGE1_*`，任务四使用 `T4_BALL_*`；新增任务必须在自身 `taskN.c` 另建参数组。
- **刷屏策略**：软件 I2C 整屏刷约 50ms，故只在选中项/状态变化时才重绘（事件驱动），平时仅轻量轮询按键，CPU 友好。
- **传感器状态栏**（`Y:<yaw> D:<dist>mm`）：常驻底部（菜单态 Y=56、运行态 Y=48），实时显示陀螺仪 Yaw（度，1 位小数）与激光测距（mm），方便一眼判断两个传感器是否在工作。
  - 数据来源：Yaw 取 `IMU100Hz` 任务发布的线程安全快照 `AppImuUartTask_GetYaw()`（IMU 未就绪显示 `---`）；距离取 `LaserLd14_GetLatest()`（无有效帧显示 `---`）。UI 任务**不直接访问软件 I2C/激光**，避免与 IMU 任务争用总线。
  - **不影响实时性的做法**：状态栏用 `OLED_UpdateArea` **只局部刷一行**(128×8≈整屏 1/8)，且每 `APP_UI_STATUS_DIVIDER`(默认 10 拍=300ms) 才刷一次；整屏刷仍只在菜单/运行切换时发生。故周期刷屏的软件 I2C 忙等极小，不拖累按键响应与其它任务。
- **OLED 只能显示 ASCII**：`OLED_Data.h` 的中文字库 `OLED_CHARSET_GB2312` 处于注释禁用状态且无 `OLED_ShowChinese` 接口，故题名用英文/编号；要中文需另做字模并启用字库。

### 右侧视觉通信面板（`APP_FEATURE_VISION_LINK`）

- 目的：利用菜单空出的右半屏，常驻反馈树莓派视觉端在线、相机发送的钢珠 X 位置及最近一次题目 ACK。
- 布局（128×64，6×8 字体；仅**菜单态**显示）：右半自 x=66 起，一条竖分隔线(x=63) + 顶部 `VISION` 头 + 四行字段：
  - `NET:ON`/`NET:OFF`：匹配 `PONG` 后的 3 秒在线判定；
  - `X:<原始字段>` 或 `X:---`：只要接收到 `$X,` 帧就显示其原始字段（最多 8 个 ASCII 字符），不先因格式、范围或校验失败而隐藏；用于确认相机链路和实际发送格式。正式有效坐标仍按协议校验和 `0～640` 的无符号整数像素范围判定；
  - `ACK:ST`/`ACK:SP`/`ACK:--`：当前或最近一次 `TASK START/STOP` 的匹配确认。
  - `B:OFF/START/WAIT/RUN/HOLD/DEG/LOST/...`：后台钢球闭环状态与故障提示；`DEG` 表示单次 NA 或短时断帧期间正在软降速。
- 数据来源：`AppVisionLink_GetStatus()`；UART0 RX 中断解析 `$PONG/$ACK/$X`，UIMENU 任务发送 PING 并做时效判断（见 `MESSAGE_LIST.md`）。
- 刷新：字段区(x66,y8,62×32)用 `OLED_UpdateArea` **只局部刷**，与底部状态栏同频（`APP_UI_STATUS_DIVIDER`，默认 300ms），面积小、频率低，不打断按键响应；`VISION` 头与分隔线只在整屏刷（进入菜单/切换）时画。
- 运行态(RUN)不显示该面板（整屏归题目自身）；关闭 `APP_FEATURE_VISION_LINK` 时菜单恢复整行高亮与整宽标题，右半留空。

### 右侧循迹面板（`APP_FEATURE_LINE_TRACK`）

> ✅ 2026-07-25 已上板实测通过，功能正常。

- 目的：利用菜单右半**下部第 6/7 行**（视觉通信面板占第 1~4 行，两者错开不重叠），常驻显示 8 路灰度循迹（PB17~PB24=LINE1~LINE8）的实时高低电平状态。
- 布局（128×64，6×8 字体；仅**菜单态**显示）：右半自 x=66 起，共用同一条竖分隔线(x=63)：
  - 第 6 行(y=40)：通道号 `87654321`（左=8 号=小车左，右=1 号=小车右）
  - 第 7 行(y=48)：按 LINE8→LINE1 对应状态，如 `10011100`；与通道号逐位对齐——`1`=识别到线、`0`=未识别（屏上左右即小车物理左右）
- 数据来源：`BspLine_ReadAll()`（`bsp/bsp_line.c`，一次读全 8 路打包成位图 bit0=LINE1…bit7=LINE8）。OLED 显示时按 bit7→bit0 倒序，使硬件最左的 LINE8 显示在最左。当前模块极性「识别到线=低电平」，`BSP_LINE_ACTIVE_LOW=1` 已将其归一化为 OLED 上 `1`=识别到线；如换模块极性相反，改该宏即可整体反相。
- 刷新：两行区(x66,y40,62×16)用 `OLED_UpdateArea` **只局部刷**，与底部状态栏、视觉通信面板同频（`APP_UI_STATUS_DIVIDER`，默认 300ms），面积小、频率低，不打断按键响应；分隔线只在整屏刷时画。
- 运行态(RUN)不显示该面板；关闭 `APP_FEATURE_LINE_TRACK`（且视觉通信也关）时菜单恢复整行高亮与整宽标题，右半留空。
- ⚠️ PB17~PB24 **非 5V 容忍**：灰度模块信号须 3.3V 电平，否则需分压/电平转换（见 `HARDWARE_WIRING.md` 与接线文档风险 R2）。

## ID1 开机自动归零（`app/app_lift_homing.c`，无独立任务）

**2026-08-01 新增**，由 `APP_FEATURE_LIFT_HOMING`（默认 1，依赖 `APP_FEATURE_EMM42`）门控。

- 不是 FreeRTOS 任务，而是 `App_Init()` 里的一段阻塞流程：在 `Emm42Robot_Init()` +
  `App_Emm42BootDisableAll()` 之后、任何任务创建之前调用 `AppLiftHoming_RunAtBoot()`，
  全程用 `Delay_ms` 忙等轮询限位开关（调度器尚未启动，不会与任何任务竞争 ID1）。
- 硬件：PA24（P1 接口，原继电器接口）已改接一颗轻触开关，一端接地，按下时引脚被拉低；
  GPIO 配为数字输入 + 内部上拉，读取接口 `bsp/bsp_home_switch.h` 的 `BspHomeSwitch_IsPressed()`。
- 流程：
  1. 正方向移动，直到压下限位开关；
  2. 压下的瞬间反向（负方向）退让，直到开关释放——释放点即物理归零参考点；
  3. 继续往负方向移动 `LIFT_HOMING_TARGET_OFFSET_PULSES`（`app_lift_homing.c` 顶部宏，
     当前占位为 `0`，**待实测后填入实际值**）个脉冲，到达指定的相对工作位置。
  4. **极端情况**：若开机时开关已经被压住，跳过第 1 步（不再继续往正方向顶死），直接进入
     第 2 步的退让。
- 只移动 ID1、不清零位置（不调用 `Emm42Robot_ResetPosToZero()`）；`BALLCTRL` 的
  `hasZeroedSinceBoot` 和 `task6` 的 `T6_ZERO_ON_ENTER` 各自的清零逻辑不变——只要 ID1
  在开机归零结束到它们首次清零之间没有被移动过，清零点就等于归零终点。这就**取代了此前
  "每次上电先人工把杆摆到目视水平"的步骤**。
- ⚠️ 故意不加超时保护：若限位开关故障导致第 1 步永远读不到触发，本流程会一直忙等，
  调度器不会启动（LED1 不闪、OLED 不亮）——这是找不到物理参考点就不能继续的题中之义，
  不要为此加时间兜底。
- 串口输出 `HOMING: ID1 start` / `HOMING: seeking switch` / `HOMING: switch pre-pressed,
  backing off` / `HOMING: ID1 done`，在 `BOOT: board init ok` 之后、`BOOT: start scheduler`
  之前出现。
- **原继电器功能已随此改动整体移除**：`bsp_relay.c/h`、`app_relay_test_task.c/h`、
  `APP_FEATURE_RELAY`/`APP_FEATURE_RELAY_SELFTEST`、`RELAYTEST` 任务、OLED 状态栏
  `R:ON/OFF` 显示均已删除。

### 题目业务说明

各题的具体实现在 `app/tasks/taskN.c`，通过 `app_robot_core.c` 的 dispatch 表登记后被 UIMENU 调用。

**题目一 `VIDEO 5S`**（`task1.c`）：
- `RobotCore_EnterTask()` 先向视觉端发送 `$TASK,<run_id>,1,START`；任务随即停住并失能四路 STEP/DIR 电机，避免录像期间产生车辆动作。
- 任务使用 FreeRTOS tick 计时约 5 秒后，只发送一次对应的 `$TASK,<run_id>,1,STOP`；视觉端保存该段无文字、检测框、坐标或其他调试图层的正常相机画面。
- K4 提前退出也会立即安全失能，并由 `RobotCore_ExitTask()` 发送 `STOP`，视觉端保存当前已录片段。

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
- 终点检测：先连续 `T2_FINISH_ARM_TICKS` 拍命中路数 `≤T2_FINISH_ARM_HIT_MAX` 才"武装"终点检测（避免出发瞬间仍压在宽线上被立即误判为跑完一圈）；武装后再连续 `T2_FINISH_HIT_TICKS` 拍命中路数 `≥T2_FINISH_HIT_MIN`（6/7/8 路均算），判定到达终点，立即硬停车（`Emm42Robot_Stop`）并锁定在 `T2_STATE_FINISHED`，之后不再重新进入循迹，等待 K4 手动退出；停车后调用 `BspBuzzer_BeepShort()`，响铃长度与按键反馈一致。
- 秒表计时：`s_elapsedTicks` 从 `Task2_OnEnter()` 起每拍（30ms）累加，到 `T2_STATE_FINISHED` 后停止累加（定格）；`Task2_GetUiStatus()` 输出 `"T:12.3s"`（终点后追加 `" DONE"`），在 `app_ui_task.c` 里复用 task5 的 `UI_RUN_NAME_Y` 显示位（`UI_TASK2_INDEX`），每 `APP_UI_STATUS_DIVIDER`（约300ms）刷新一次，运行界面首次绘制也会立即显示 `T:0.0s`。显示毫秒数会乘一个 `T2_STOPWATCH_CAL_SCALE` 校准系数——实测计时比真实时间偏快（怀疑跟 FreeRTOS tick 依赖的主频跟工程假设的 80MHz 有偏差有关，根因未定位，见 `AI_MEMORY.md`），先用系数硬补偿；如果实测偏差比例继续变化，按"新系数 = 当前系数 × (最新实测秒数/最新显示秒数)"滚动修正即可，不用每次从 1.0 重新推导。
- K4 退出时对 ID2/ID3 先急停再失能（轮子无重力负载，失能不会溜车，比保持力矩更省电安全）；ID1 摆杆不属于本题，不下发任何命令。K4 退出与终点急停这两处需要背靠背给两轮下发命令，帧间用 `vTaskDelay(T2_EMM_CMD_GAP_MS)` 隔开，避免共享 UART1 总线互相干扰丢帧（emm42_v5.h 协议层明确要求连续下发需自行留间隔）。

**题目三 `Task 3` 第一阶段**（`task3.c`）：
- 只经 `BALLCTRL` 使用 UART1 的 `EMM42_ROBOT_LIFT`（地址 1）；不会向 ID2、ID3 或四路 STEP/DIR 电机发送命令，也不直接调用协议层。
- ID1 的位置原点由开机自动归零流程建立（见「ID1 开机自动归零」章节），不再需要人工在菜单页按 K4。进题后先请求 `BALLCTRL` 停止并等待其释放 ID1，再以自身 profile 重新启动；任务三本身不发送位置清零命令，清零仍由 `BALLCTRL` 首次启动时完成。
- 状态机以独立 `T3_STAGE1_*` profile 下发 `X=415`，滤波位置进入 `415±10px` 且低速连续 200ms 即完成。到位死区同为 10px，进入验收范围即停止微调，避免在目标附近反复推球。
- 总时长从后台首条实际位置控制帧开始计，超过 5 秒或触发边缘故障即停止、失能并显示 `T3 TIMEOUT`；成功时保持 ID1 使能及 `X=415` 目标，K4 退出才急停失能。后续第二阶段另建 `T3_STAGE2_*` 参数组。

**题目四 `LINE 6S`**（`task4.c`）：
- 循迹 PID、入场状态机、丢线保护、终点保护、转弯减速、定时减速和所有同名参数均照搬题目二，仅使用独立的 `T4_*` 参数，便于单独调试。
- 进入后先通过 `AppBallControl_RequestTargetWithProfile()` 投递独立 `T4_BALL_*` profile，目标固定 `X=320`；后台闭环发布 `RUNNING/HOLDING` 且已确认目标后，任务四才开始 ID2/ID3 的使能时序。因此车辆移动期间 ID1 始终由后台位置模式闭环平衡钢珠。`T4_BALL_*` 初值来自任务三第一阶段，后续只调这一组，不影响 `T3_STAGE1_*` 或菜单参数。为消除车辆扰动刚出现时的无补偿区，任务四单独将 `T4_BALL_SETTLE_DEADBAND_PX` 设为 `3px`；任务三仍保留自身的 `10px`。
- 仅有业务差异：正常循迹累计前进 `T4_STOP_AFTER_MS=6500ms` 后，状态机分两拍给 ID2、ID3 发送速度模式 `0 RPM`，每拍只发一帧以避免共享 UART1 总线丢帧。
- 缓停使用 `Emm42Robot_VelControl(..., 0, T4_STOP_EMM_ACC)`，它会保留速度模式帧并把 `T4_STOP_EMM_ACC` 交给控制器；与 `Emm42Robot_SetSpeedRpm(..., 0, ...)` 的急停语义不同。`T4_STOP_EMM_ACC` 是本题可调的停车加速度参数，数值越小越平缓、越大越接近立即停。
- 6.5 秒只统计 `T4_STATE_RUN` 的正常循迹时间，丢线停车期间不计时；两轮收到 0 RPM 帧后，状态机仍持续读取灰度、滤波并更新 PID、丢线保护和终点保护，OLED 秒表追加 `DONE`。为保持驱动器正在执行的 0 RPM 曲线，此阶段不会再发送非零速度帧；ID1 在缓停和自动完成后仍保持 `X=320`，K4 退出与终点保护继续急停并失能轮子、并请求后台停止 ID1，属于安全收尾，不使用缓停。
- **2026-08-01 新增钢珠故障自动恢复**：`BALLCTRL` 的 `FAULT_EDGE`（球触边、命令回水平并锁定）不会自行恢复；`Task4_OnLoop()` 每拍检测该状态，一旦出现就自动走一遍"请求停止 → 等 `BALLCTRL` 回到 `OFF` → 重新请求"的握手（新增 `T4_STATE_BALL_RECOVER_WAIT`，OLED 显示 `T4 B RECOV`），不需要用户手动退出重进任务四。若轮子已经使能起步过（`s_wheelsStarted`），恢复完成后直接回到 `T4_STATE_RUN` 循迹，不重新走一遍轮子使能时序；只要仍在题目四内就持续保证钢珠被伺服到 `X=320`，直至 K4 退出。
- **手动调参开关 `T4_MOTORS_DISABLED_MANUAL_TEST`**（`task4.c` 顶部，当前置 `1`）：置 1 时轮子不使能、不驱动，只让后台 `BALLCTRL` 按 `T4_BALL_*` profile 保持钢球在 `X=320`，供用手推小车模拟前进/加减速扰动、专门调 `T4_BALL_*` 参数（OLED 显示 `T4 MANUAL`）；调参完成后改回 `0` 才会执行完整循迹+缓停流程。

**题目五 `Five`**（`task5.c`，横向停止线后 0.5 秒循迹并线性缓停）：
- 硬件：UART1（PA17=TX → 驱动器 RX，PB5=RX ← 驱动器 TX），仅控制 ID2 左轮与 ID3 右轮；ID1 摆杆不参与。
- 入场、PID 滤波、死区、丢线保护、使能等待和左右轮交替下发均复用题目二。行驶 RPM 参数按 62% 缩放：`T5_BASE_RPM=80.6`、`T5_MAX_WHEEL_RPM=142.6`、`T5_MAX_STEER_RPM=62`、`T5_MIN_BASE_RPM=6.2`；`T5_MIN_WHEEL_RPM` 保持 5 RPM，避免低于驱动器的稳定低速范围。PID 增益和驱动器加速度档位保持题目二当前值。
- 连续 `T5_ARM_TICKS=15` 拍命中 ≤3 路细线后才武装。武装后的首次 ≥6 路黑线开始 `T5_AFTER_LINE_MS=500ms` 计时；横带仍在传感器下方时，不更新 PID 或发送新轮速帧，保持触发前最后有效轮速。命中数回落到 <6 后恢复正常 PID，0.5 秒计时不中断。
- 0.5 秒结束后用 `T5_DECEL_MS=2000ms` 软件线性降低基础速度到 0 RPM，同时保持 PID 差速；差速幅度随当前基础速度收窄，避免内侧轮倒转并保证最终归零。减速期再次命中 ≥6 路时，冻结轮速并暂停减速计时，离开横带后继续剩余减速。
- 题目五秒表从 `Task5_OnEnter()` 起每 30ms 累加，停车完成后定格；`Task5_GetUiStatus()` 在 OLED 显示 `T:12.3s` 加当前阶段（如 `GO:0.5s`、`DEC:1.8s` 或 `DONE`）。显示时间乘 `T5_STOPWATCH_CAL_SCALE=0.897`，与任务二使用同一实测初值但可独立按“新系数 = 当前系数 × 实测时长 / 显示时长”继续校正。
- 到 0 RPM 后，左右轮分两拍发送速度模式 0 RPM 并锁定等待 K4，随后调用 `BspBuzzer_BeepShort()`；K4 退出仍急停后失能。自动完成提示音与任一按键按下的提示音为同一接口、同一时长。

**题目六 `ID1 POS`**（`task6.c`，Emm42 ID1 位置模式 API 冒烟测试）：
- 目的：验证摆杆能用**位置模式**（下发"该停在哪个脉冲位置"）驱动，为把 `BALLCTRL` 从速度模式切过去做准备。速度模式下"RPM→摆杆角度→球位置"整链三阶，纯 PID 极难镇定，这是 `v2.2` 难调参的根因。
- 只操作 `EMM42_ROBOT_LIFT`（地址 1），不触碰 ID2/ID3；与 `BALLCTRL` 争用 ID1，`OnEnter` 先 `AppBallControl_RequestStop()`，`OnLoop` 等其状态回到 OFF 才开始（与题目三同一套做法）。
- 状态机（每 30ms 轮询拍最多发一帧）：`WAIT_BALL_RELEASE → RESET_DISABLE → RESET_WAIT(60ms) → CLEAR_CLOG → ENABLE → ENABLE_WAIT(180ms) → ZERO → REL(+N圈) → [REL(−N圈)] → [ABS(+MOVE) → ABS(0) → ABS(−MOVE) → ABS(0)] → CYCLE_END`。方括号两段分别由 `T6_RETURN_ENABLE`、`T6_ABS_TEST_ENABLE` 控制；`T6_LOOP_FOREVER=0` 时 `CYCLE_END` 直接进 `DONE`，**不发急停帧**，保持使能让驱动器用保持力矩停在目标位置，方便拿尺子量行程（K4 退出时才急停失能）。
- **⚠️ 运动段的等待时间必须覆盖整段运动**：位置模式下新的位置命令会**覆盖**尚未走完的上一条并重新规划，等待太短就表现为"命令发了却几乎没走到位"。`T6_SEGMENT_WAIT_MS` 由 `T6_TEST_REVS / T6_POS_RPM` 编译期算出理论运动时间（acc=0 无加减速曲线，时间 = 圈数/转速 分钟），乘 `T6_MOVE_MARGIN_PCT` 余量后再叠加 `T6_DWELL_MS` 观察时间。清零这类瞬时动作仍只等 `T6_DWELL_MS`。转速越低等待越长：10 圈 @30RPM 理论就要 20 秒。
- **所有位置都锚在按 K3 进入本题那一刻的电机位置**：相对模式命令本就以当前实际位置为起点，绝对模式的原点也是进入时才建立的，不存在跨上电或跨一次进出的持久绝对坐标。反复退出再进入会以当时位置重新置零，单程模式下位移一路累积，量完要手摇回行程中部再测下一次。
- **当前默认配置**：`T6_TEST_REVS=10`、`T6_RETURN_ENABLE=0`、`T6_ABS_TEST_ENABLE=0`、`T6_LOOP_FOREVER=0`，即进题目后正转 10 圈就停住，用于标定丝杆导程（量行进了多少 mm）。**已实测：10 圈 = 约 80mm 行程（导程约 8mm/圈，约 400 脉冲/mm）**。
- `ZERO` 调 `Emm42Robot_ResetPosToZero()`（协议 `0x0A 0x6D`）把当前机械位置定义为绝对位置 0。ID1 开机已经过 PA24 限位开关自动归零（见「ID1 开机自动归零」章节），不再需要人工把杆摆到目视水平再按 K3 进本题；驱动器断电不保留多圈位置计数，每次上电都要重做。
- `CLEAR_CLOG` 调 `Emm42Robot_ClearClogProtection()`（`0x0E 0x52`）。丝杆低速顶死会触发堵转保护（转速<40RPM 且电流>2400mA 且持续>4000ms），触发后位置命令返回 `E2` 且电机纹丝不动，不清掉后面所有命令都会被拒。
- **`T6_STEP_TEST_ENABLE=1` 时完全替代相对/绝对测试，改为阶梯步长测试**：清零后从水平位置依次下发 `s_stepTestPulses[]={1600,3200,4800,6400,9600,12800}`（0.5~4 圈）的绝对目标，每步等 `Task6_StepWaitMs()` 按该步脉冲数动态算出的理论运动时间走完后，再停留 `T6_STEP_TEST_DWELL_MS=3000ms` 观察钢球是否开始可见地滚动，再回到水平测下一步（回程按同一步长重新计算等待时间）。每步测试前都先解一次堵转保护（`T6_STATE_STEP_CLEAR_CLOG`），避免某一步顶到限位触发堵转后把后续所有步骤锁死、被误判成摩擦力问题。用于标定"钢球在凹槽里的静摩擦阈值"（区别于前三项测的"丝杆本身能否被位置模式驱动"）——这个阈值决定了 `BALLCTRL` 的 `Kx` 下限，误差算出的脉冲量级长期小于此阈值会导致球在目标附近"死区"里出不来。只测 `T6_FIRST_DIR` 方向，测另一方向需把该宏改成 `-1` 重新跑。**2026-07-31 实测 200~1200 脉冲全程无反应**，已确认摆杆确实水平（排除零点问题），已把步长整体上调到 0.5~4 圈继续测。
- 可调宏集中在 `task6.c` 顶部：`T6_PULSES_PER_REV`(3200，须与驱动器 `MStep` 一致)、`T6_TEST_REVS`、`T6_POS_RPM`、`T6_POS_ACC`、`T6_FIRST_DIR`、`T6_DWELL_MS`、`T6_MOVE_MARGIN_PCT`、`T6_LOOP_FOREVER`、`T6_ZERO_ON_ENTER`、`T6_CLEAR_CLOG_ON_ENTER`、`T6_RETURN_ENABLE`、`T6_ABS_TEST_ENABLE`、`T6_STEP_TEST_ENABLE`、`T6_STEP_TEST_DWELL_MS`。**故意不加软件行程限位和软件斜坡**。
- `Task6_GetUiStatus()` 在 OLED 题名行显示当前阶段与脉冲数（如 `ABS +9600`、`REL -9600`、`ZERO +0`、`DONE`）；串口静默时这是唯一反馈通道。
- 上板前置检查、判据和要顺带标定的物理量见 [BALL_CONTROL.md](BALL_CONTROL.md) §9。

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

## 视觉端通信（UART0 RX 中断 + UIMENU 状态机）

- UART0 RX 中断把 FIFO 取空并逐字节喂给 `VisionParser_FeedByte()`（`module/vision/ball_parser.c`），解析 `$PONG`、`$ACK`、`$X` 三类协议帧；接收端没有独立 FreeRTOS 任务。
- `App_Init()` 调用 `AppVisionLink_Init()` 复位解析器、注册 UART0 回调并使能 RX+溢出中断；`UIMENU` 每 30ms 调 `AppVisionLink_Service()`，负责 PING 周期、PONG 在线超时和 X 时效。
- MCU 每次复位后只以 500ms 间隔发送 3 次 `PING`，第 3 次后停止发送直到下次复位；任一 PING 收到匹配 PONG 即在线，最后一次匹配 PONG 超过 3s 切为离线。当前约 30fps 的 X 正式有效坐标超过 220ms 自动失效；`BALLCTRL` 收到单次 `X,NA` 不再继续使用旧坐标计算新控制量，但也不主动下发任何停止命令（位置模式下摆杆保持在最后一次有效目标即安全），连续两帧 NA 才把 OLED 状态标记为 `B:LOST`。但 OLED 会保留并显示最近收到的原始 X 字段，包含格式或校验失败的帧，作为链路诊断信息。
- 进入/退出题目时 `RobotCore` 分别发送 `TASK START/STOP`；OLED 菜单视觉面板显示 `NET`、`X` 和匹配的 `ACK`。由 `APP_FEATURE_VISION_LINK` 门控，并与 `APP_FEATURE_UART_ECHO` 互斥。
- ISR 内不调用任何非 FromISR 的 FreeRTOS API（符合 CLAUDE.md FreeRTOS 规则）。

## UART0 多任务共享（递归互斥量）

- 旧实现用 `vTaskSuspendAll` 挂起整个调度器保证整行日志原子，一行 IMU 日志约 10ms 会冻结全部任务，造成按键/舵机/OLED 卡顿。
- 现改为递归互斥量（`bsp_uart.c`，`BspUart0_Init` 在 `BspBoard_Init` 中于调度器启动前创建）：
  - 发送方仅独占 UART0，不再冻结调度器，其余任务照常按时间片轮转，从根上消除全局卡顿。
  - `BspUart0_SendString` 单次调用自动加锁即原子；多段拼接由 `BspUart0_Lock/Unlock` 包裹，递归类型允许嵌套不自锁。

## 题目三：第一阶段钢珠定位

- 前置标定：ID1 位置原点由开机自动归零流程建立（见「ID1 开机自动归零」章节），不再需要人工在菜单页按 K4；任务三本身绝不再清零。
- 状态机：`OnEnter` 先请求 `BALLCTRL` 停止并等待其释放 ID1；随后以独立 `T3_STAGE1_*` profile 下发 `X=415`（这是本次上电后第一次调用 `BALLCTRL` 时才会真正清零）。后台下发首条有效位置控制帧开始 5 秒计时，滤波位置进入 `415±10px` 且低速后开始稳定计时。
- 完成条件：滤波位置保持 `415±10px` 且速度不超过 `T3_STAGE1_HOLD_VELOCITY_PXPS` 连续 200ms；完成后保持 `X=415` 绝对位置目标，K4 退出才急停失能。超时或边缘故障会请求安全停止并显示 `T3 TIMEOUT`。
- 参数隔离：当前第一阶段只调整 `task3.c` 的 `T3_STAGE1_*`。后续第二阶段另建 `T3_STAGE2_*`，profile 经 `AppBallControl_RequestTargetWithProfile()` 按值复制到后台队列，不共享可写调参变量。
- OLED：运行行每 300ms 刷新 `T3 TO415`、`T3 STABLE`、`T3 DONE` 或 `T3 TIMEOUT` 及运行时间。

## 当前串口测试行为

- `main()` 在 `BspBoard_Init()` 后立即输出 `BOOT: board init ok`，此时还未创建任务、未进入 FreeRTOS 调度。
- `main()` 在 `App_Init()` 返回后输出 `BOOT: start scheduler`，随后才启动 FreeRTOS 调度器。
- 上电后输出 `UART0 RX READY, LED1 heartbeat active`。
- 收到任意非换行字符后返回 `UART RX OK`。
- 换行和回车会被忽略，避免串口助手自动追加换行造成重复提示。

## IMU100Hz 姿态输出

**【2026-07-29 临时禁用，`APP_FEATURE_IMU=0`】** 减少 CPU 占用（软件 I2C 100Hz 读取是当前任务里开销最大的一个），需要陀螺仪/Yaw 数据时改回 1 即可，代码逻辑未删改；关闭后 `IMU100Hz` 任务不会被创建，`AppImuUartTask_GetYaw()` 恒返回 false（OLED 状态栏显示 `Y:---`）。

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
- **只做登记 + 分发**：`app_robot_core.c` 里的 **dispatch 表 `s_robotTasks[]`** 登记 6 道题的 name + `OnEnter/OnLoop/OnExit` 函数指针；**各题业务代码在 [`../app/tasks/`](../app/tasks/) 的 `taskN.c`**（第 N 题 = `taskN.c`，每题一套状态机；当前 task1 为约 5 秒正常画面录像，task2 为正式循迹，task3 为 UART1 控制 ID1 正反各约 500ms 的低速测试，task4 为 6.5 秒循迹缓停，task5 按说明实现，task6 `ID1 POS` 为 Emm42 ID1 位置模式 API 冒烟测试）。
- **按键反馈**：`UIMENU` 对 KEY1~KEY4 的每一次按下沿统一调用 `BspBuzzer_BeepShort()`，短响约 2~3ms；任务二到达终点、任务五自动停车完成复用同一接口，K3 不会双响。
- **接口**：
  - `RobotCore_GetTaskCount()` — 题目总数（菜单滚动循环用）
  - `RobotCore_GetTaskName(idx)` — 题目显示名（OLED 菜单/运行界面显示）
  - `RobotCore_EnterTask(idx)` — 进入题目（调 `onEnter`）
  - `RobotCore_LoopTask(idx)` — 每周期循环（由 UIMENU 30ms 节拍驱动，调 `onLoop`）
  - `RobotCore_ExitTask(idx)` — 退出题目（调 `onExit`）
  - `RobotMaster_Start()` — 机器人总任务入口（占位，后续做按顺序自动执行全部 6 题）
- **调试日志**：`EnterTask`/`ExitTask`/`RobotMaster_Start` 仅在 `APP_FEATURE_IMU_UART_LOG=1` 时向 UART0 打印（默认静默模式不刷串口）。

### 按键反馈

`UIMENU` 对 KEY1~KEY4 的每个按下沿统一调用 `BspBuzzer_BeepShort()`，短响约 2~3ms；任务二到达终点、任务五自动停车完成也调用同一接口。鸣叫仅做短暂忙等，随后立即关断。旧版按题错开的 LED2/LED3 组合已取消，改由各题 `taskN.c` 自行按需驱动 LED/OLED。

### 题目开发指南（v1.9：每题一个文件）

1. 打开 [`../app/tasks/`](../app/tasks/) 下对应题号的 **`taskN.c`**（第 N 题就在这个文件），在文件头写本题要求；
2. 按本题流程改**状态枚举**（如 直行→路口→转弯→…→完成）；
3. 在 `OnLoop()` 的 `switch(state)` 里逐状态写"**动作 + 切换条件**"（`OnLoop` 每 30ms 调一次，做机动级决策足够；电机命令直接下发，定距计步由 ISR 后台完成）；
4. `OnEnter` 做一次性准备（`BspMotor_EnableAll`、舵机归中、清零），`OnExit` 急停+失能（`BspMotor_StopAll`/`DisableAll`）保证安全；
5. 需要高频控制环的题目，在 `OnEnter` 里 `xTaskCreate` 自己的任务、`OnExit` 里 `vTaskDelete` 销毁；
6. 改题名/题数只改 `app_robot_core.c` 的 `s_robotTasks[]`，无需动 UI。
7. 底层接口速查（电机/舵机/传感器）见 `app/tasks/app_tasks.h` 顶部注释与 [`../app/README.md`](../app/README.md)。
