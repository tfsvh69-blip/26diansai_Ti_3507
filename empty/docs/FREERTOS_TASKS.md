# FreeRTOS 任务列表

任务周期、优先级、栈大小集中定义在 `common/app_config.h`。

| 任务名 | 所在文件 | 周期 | 优先级 | 栈大小 | 输入 | 输出 | 说明 |
|---|---|---:|---:|---:|---|---|---|
| `LED1` | `app/app_led_task.c` | 300 ms | `APP_LED_TASK_PRIORITY` | `APP_LED_TASK_STACK_WORDS` | 无 | PB22 LED 翻转 | 当前已启动，用作 FreeRTOS 调度心跳 |
| `UART0TX` | `app/app_uart_test_task.c` | 10 ms 接收轮询 | `APP_UART_TEST_TASK_PRIORITY` | `APP_UART_TEST_TASK_STACK_WORDS` | UART0 RX 任意非换行字符 | 返回 `UART RX OK` | UART0 使用 MFCLK 115200，PA10=TX，PA11=RX |
| `IMU100Hz` | `app/app_imu_uart_task.c` | 10 ms | `APP_IMU_UART_TASK_PRIORITY` | `APP_IMU_UART_TASK_STACK_WORDS` | ATK-MS6DSV/LSM6DSV16X FIFO 姿态数据、PA16 INT 电平 | UART0 输出 Roll/Pitch/Yaw、FIFO 深度、INT 电平 | 对外读取/输出 100Hz；芯片内部 ODR 配为 120Hz，因为无精确 100Hz 档位 |
| `MOTOR1` | `app/app_motor_test_task.c` | 1000 ms 状态打印 | `APP_MOTOR_TEST_TASK_PRIORITY` | `APP_MOTOR_TEST_TASK_STACK_WORDS` | 无 | TIMG0_CCP0(PB10) 持续 4kHz STEP、PB11 DIR、PA13 ENN、PB8/PB9 MS1/MS2，UART0 打印状态 | 电机1驱动测试：线序+VREF 电流已标定，实测可正常旋转；当前 1/8 细分、4kHz、约 2.5 转/秒 |

## PB22 心跳灯行为

- `App_Init()` 会启动 `LED1` 任务。
- PB22 每 300ms 翻转一次；若持续闪烁，说明 FreeRTOS 调度至少已经运行。
- 串口任务不再控制 PB22，避免心跳判断被串口命令干扰。

## 当前串口测试行为

- `main()` 在 `BspBoard_Init()` 后立即输出 `BOOT: board init ok`，此时还未创建任务、未进入 FreeRTOS 调度。
- `main()` 在 `App_Init()` 返回后输出 `BOOT: start scheduler`，随后才启动 FreeRTOS 调度器。
- 上电后输出 `UART0 RX READY, PB22 heartbeat active`。
- 收到任意非换行字符后返回 `UART RX OK`。
- 换行和回车会被忽略，避免串口助手自动追加换行造成重复提示。

## IMU100Hz 姿态输出

- 上电后任务先输出 `IMU UART 100Hz START, SWI2C addr=0x6A`。
- 初始化成功输出 `IMU INIT OK`，随后按 10ms 周期输出 `IMU R=... P=... Y=... FIFO=... INT=...`。
- 初始化失败每 1s 输出 `IMU INIT FAIL:n STEP=...` 并自动重试初始化，其中 `1` 为 ID 不匹配，`2` 为 I2C 通信失败，`3` 为配置或复位超时；若初始化阶段成功读到 WHO_AM_I，会追加 `LAST_ID=0x..`。
- 当前初始化默认跳过 `RESTORE_CTRL_REGS`/boot reset，`STEP=RESET_SKIP` 属于预期步骤；原因是实测 `RESET_SET` 会导致 SDA 被拉低。
- 初始化失败时同时输出 `IMU WHOAMI 0x6A=... 0x6B=...`；正常 LSM6DSV16X 应读到 `0x70`。
- 初始化失败时还会切到 GPIO 软件 I2C，并给 SCL 输出 18 个恢复脉冲，再输出 `IMU BUS SCL=... SDA=... STAT=...`；恢复后 SCL/SDA 应都为 `1`，否则优先检查短路、接反、模块供电或上拉。
- 初始化首次失败和之后每 5 次失败会输出 `IMU SCAN: ...`，当前只通过写入 `WHO_AM_I` 寄存器地址探测 `0x6A/0x6B`，避免异常状态下全地址扫描刷出假 ACK。
- 若输出 `IMU SCAN: bus stuck ...`，表示 SCL/SDA 没有释放到空闲高电平，此时不会继续扫描，避免 SDA 低电平造成全地址假 ACK。
- 初始化后短时间没有 FIFO 数据时，每 1s 节流输出 `IMU WAIT DATA`。
- 当前已实测能连续输出 `IMU R=... P=... Y=... FIFO=... INT=...`。`FIFO=0/1/2` 间歇变化正常，原因是芯片内部 SFLP 为 120Hz，而任务按 100Hz 读取；若 FIFO 长时间持续增大或长时间为 0 且角度不更新，才需要继续排查。
- `APP_IMU_I2C_PIN_TEST_ENABLE` 当前为 `0`，`IMU100Hz` 正常访问 IMU；若临时置为 `1`，任务不访问 IMU，改为每 500ms 用开漏模拟方式交替翻转 PB2/PB3，并直接读取 GPIO DIN 输出 `SET`/`READ` 电平。
