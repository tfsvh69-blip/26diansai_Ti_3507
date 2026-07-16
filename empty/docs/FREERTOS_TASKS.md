# FreeRTOS 任务列表

任务周期、优先级、栈大小集中定义在 `common/app_config.h`。

| 任务名 | 所在文件 | 周期 | 优先级 | 栈大小 | 输入 | 输出 | 说明 |
|---|---|---:|---:|---:|---|---|---|
| `LED1` | `app/app_led_task.c` | 300 ms | `APP_LED_TASK_PRIORITY` | `APP_LED_TASK_STACK_WORDS` | 无 | LED1(PB25) 翻转 | 当前已启动，用作 FreeRTOS 调度心跳 |
| `UART0TX` | `app/app_uart_test_task.c` | 10 ms 接收轮询 | `APP_UART_TEST_TASK_PRIORITY` | `APP_UART_TEST_TASK_STACK_WORDS` | UART0 RX 任意非换行字符 | 返回 `UART RX OK` | UART0 使用 MFCLK 115200，PA10=TX，PA11=RX |
| `IMU100Hz` | `app/app_imu_uart_task.c` | 10 ms | `APP_IMU_UART_TASK_PRIORITY` | `APP_IMU_UART_TASK_STACK_WORDS` | ATK-MS6DSV/LSM6DSV16X FIFO 融合姿态 + 加速度/角速度输出寄存器、PA16 INT 电平、**激光测距1(`LaserLd14_GetLatest`)** | UART0 输出 Roll/Pitch/Yaw、三轴加速度(mg)、三轴角速度(mdps)、FIFO 深度、**激光测距1(D1,mm)**、INT 电平 | 欧拉角每 10ms 读；串口整行按 `APP_IMU_PRINT_DIVIDER`(默认 **20**) 节流到 **5Hz** 打印（含激光测距，刷新慢便于阅读）；方案A：加速度/角速度只在打印那拍寄存器直读；芯片内部 ODR 配为 120Hz |
| `MOTOR1` | `app/app_motor_test_task.c` | 20ms 按键轮询 | `APP_MOTOR_TEST_TASK_PRIORITY` | `APP_MOTOR_TEST_TASK_STACK_WORDS` | KEY1~4(PA28/PA31/PA30/PA29) 按下沿 | TIMG0_CCP0(PB10) STEP 脉冲+梯形加减速、PB11 DIR、PA13 ENN、MS1=PB0/MS2=PB1，UART0 打印状态，更新 `g_motorDiag` 供 OLED 显示 | 电机1**四按键定圈旋转**：K1 正转1圈、K2 反转1圈、K3 正转3圈、K4 正转5圈；1/32细分 6400脉冲/圈，巡航周期500(≈1.25圈/秒)；起转/巡航/停止全程梯形加减速(+500Hz起步+RAMP_DELTA=4)，移动期间忽略按键
| `PERIPH` | `app/app_periph_test_task.c` | 500 ms | `APP_PERIPH_TEST_TASK_PRIORITY` | `APP_PERIPH_TEST_TASK_STACK_WORDS` | `g_motorDiag`（只读） | OLED(PB8/PB9 软件I2C) 刷屏、LED2(PA7)/LED3(PB12) 翻转、蜂鸣器(PA15) 通断 | 外设功能验证+电机状态显示：OLED 显示标题/运行秒+LED+BUZZ状态/电机1运行·方向·速度档；LED2/LED3 交替心跳；蜂鸣器保持静音；上电自检点亮 LED2/LED3 并短响 |

## LED1(PB25) 心跳灯行为

- `App_Init()` 会启动 `LED1` 任务。
- LED1(PB25) 每 300ms 翻转一次；若持续闪烁，说明 FreeRTOS 调度至少已经运行。
- 串口任务不再控制 LED1，避免心跳判断被串口命令干扰。
- LED2/LED3/蜂鸣器/OLED 由 `PERIPH` 任务驱动，与 LED1 心跳互不干扰。

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

## 电机1 四按键定圈旋转

- 控制模型：`bsp_motor` 提供定步数位置移动接口 `BspMotor1_MoveSteps(steps, cruisePeriod)`，
  TIMG0 ZERO 中断每脉冲计步，自动判断加速段→巡航段→减速段，走完指定步数且回到起步速度后自动停表。
  同时保留 `RunContinuous`/`RequestStop` 连续旋转接口供后续扩展。
- 1/32 细分下 1 圈 = 200 × 32 = **6400 脉冲**。
- 按键（`APP_MOTOR_KEY_POLL_TICKS`=20ms 轮询，检测按下沿）：
  - `KEY1`(PA28)：**正转 1 圈**（6400 脉冲）
  - `KEY2`(PA31)：**反转 1 圈**（6400 脉冲）
  - `KEY3`(PA30)：**正转 3 圈**（19200 脉冲）
  - `KEY4`(PA29)：**正转 5 圈**（32000 脉冲）
- 巡航周期 500（≈1.25 圈/秒），统一用于所有移动；若总步数不足以加速到巡航速度，自动退化为三角形速度曲线。
- 移动期间按键被忽略；移动完成（停稳）后才能触发下一次移动。
- 起步速度 500Hz（`MOTOR_STEP_PERIOD_START=8000`），斜坡增量 `MOTOR_RAMP_DELTA=4`（原为 8，加倍平滑以消除换驱动芯片后的轻微抖动）。
- 每秒 UART0 诊断：`MOTOR DIAG pos=1 run=0/1 left=<剩余步数> per=<当前周期>`。
- 移动开始/完成时各输出一条日志供排查。

## 激光测距1（UART2 RX 中断，无独立任务）

- 不是 FreeRTOS 任务，而是 **UART2 接收中断**：`UART2_IRQHandler`（`bsp_uart.c`）把 RX FIFO 取空并逐字节喂给 `LaserLd14_FeedByte()`（`module/laser/laser_ld14.c`）。
- 选中断而非任务轮询的原因：激光 230400 波特率**连续外发**，一帧 195 字节约 8.5ms 内字节连续到达，RX FIFO 仅几字节深，任务轮询必然溢出丢字节；中断按字节及时取走才不丢。
- `App_Init()` 中先 `LaserLd14_Reset()` 复位解析器，再 `BspUart2_Init(LaserLd14_FeedByte)` 注册回调并使能 RX 中断+NVIC（UART2 外设已在 `BspBoard_Init` 的 `SYSCFG_DL_UART_2_init` 里初始化）。
- 解析器攒满一整帧(195B)校验通过后，取 12 点非零距离平均为单值 `distanceMm`，连同 `valid`/`frameOkCnt`/`crcErrCnt`/`rxBytes` 供上层 `LaserLd14_GetLatest()` 读取。
- **输出与陀螺仪合并**：距离由 `IMU100Hz` 任务在打印整行时追加 `D1=<mm>mm`，与欧拉角/加减速度同一行、5Hz 刷新，满足"跟陀螺仪一起发、频率不高、便于阅读"。
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
