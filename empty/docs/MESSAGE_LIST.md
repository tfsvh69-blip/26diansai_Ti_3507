# 消息列表

当前工程使用一条长度为 1 的 FreeRTOS 覆盖队列传递钢球目标命令，并用线程安全
快照共享传感器/控制状态：

| 快照 | 接口 | 发布者 | 订阅者 | 用途 |
|---|---|---|---|---|
| IMU Yaw | `AppImuUartTask_GetYaw(int16_t *centideg)` → bool | `IMU100Hz`（100Hz 更新，临界区发布） | `UIMENU`（状态栏显示） | 偏航角（厘度，0.01°），返回 false 表示 IMU 未就绪 |
| 激光距离 | `LaserLd14_GetLatest(LaserLd14Data_t *)` → bool | UART2 RX 中断解析（`module/laser`） | `IMU100Hz`（并入遥测行）、`UIMENU`（状态栏显示） | 单点距离(mm)+置信度+统计，返回 false 表示未收到有效帧 |
| 视觉通信 | `VisionParser_GetLatest(VisionData_t *)` → bool | UART0 RX 中断解析（`module/vision`） | `app_vision_link`、`UIMENU` | `$PONG/$ACK/$X` 的最新快照；应用层据此判定在线、ACK 和 X 时效 |
| 电机诊断 | `g_motorDiag`（`app_motor_status.h`，临界区读写） | `MOTORTEST`（默认禁用时不更新） | `PERIPH`（默认禁用） | 电机运行/方向/圈数，仅当这两个任务启用时有效 |
| 钢球闭环状态 | `AppBallControl_GetStatus(AppBallControlStatus_t *)` | `BALLCTRL`（临界区发布） | `UIMENU`、题目层 | 状态、目标/实测/滤波 X、速度和 ID1 命令 RPM |

`AppBallControl_RequestTarget()` / `AppBallControl_RequestStop()` 通过长度为 1 的覆盖
队列向 `BALLCTRL` 传命令，只保留最新目标，避免题目线程直接操作闭环内部变量。

## 视觉端与单片机通信（UART0，115200 8N1）

协议以 [`../../通信协议/单片机树莓派通信协议.md`](../../通信协议/单片机树莓派通信协议.md) 为准，所有帧均为 `$TYPE,DATA...*CHK\r\n`，`CHK` 是 `$` 与 `*` 之间 ASCII 字节的 XOR。

- `UIMENU` 每 30ms 调用 `AppVisionLink_Service()`：每次 MCU 复位后仅以 500ms 间隔发送 3 次 `$PING,<id>`，随后停止发送直到下一次复位；收到**同 id** 的 `$PONG,<id>` 后判在线，最后一次匹配 PONG 超过 3s 即离线。
- `RobotCore_EnterTask()` 为新的一次进入分配递增 `run_id` 并发送 `$TASK,<run_id>,<task_id>,START`；`RobotCore_ExitTask()` 完成安全收尾后发送对应 `STOP`。视觉端对题目 1～6 都创建/保存录像：题目 1 不驱动电机，仅录制约 5 秒无叠加标注的正常相机画面；题目 2～6 录制题目过程。接收的 `$ACK,<run_id>,<task_id>,START|STOP` 会在 OLED 右侧显示 `ACK:ST` 或 `ACK:SP`。
- 视觉端每读取到一条有效 X 像素坐标就立即发送一帧，当前实测约 15 帧/秒（名义间隔 66.7ms），不积压历史帧；收到任意以 `$X,` 开头并以换行结束的帧，OLED 都会优先显示其原始数据字段（最多 8 个 ASCII 字符），包括小数、`+` 号、超范围值、缺失/错误校验和的帧，便于确认链路；只有校验正确且为 `0～640` 的无符号整数像素坐标才作为正式有效 X（有效期 220ms）。未收到过 X 才显示 `X:---`。
- UART0 RX 中断逐字节喂 `VisionParser_FeedByte()`；解析器只做行缓冲、XOR 校验与快照发布，ISR 内不调用 FreeRTOS API。由 `APP_FEATURE_VISION_LINK` 门控，且与 `APP_FEATURE_UART_ECHO` 互斥。

## 下位机 → 张大头驱动器命令帧（UART1 下行，115200 8N1）

MCU 经 UART1（PA17=TX → 驱动器 RX，PB5=RX ← 驱动器 TX）向张大头 Emm42_V5.0 闭环步进驱动器下发二进制命令帧，当前 3 台驱动器挂同一总线靠**设备地址**区分（0x01=摆杆高低调节、0x02=左轮、0x03=右轮，0x00 为广播）。协议层（`module/emm42/emm42_v5.c`）只认地址；地址与部件的角色映射在 `module/emm42/emm42_robot.c`（`Emm42RobotId_t`），业务代码应优先经这层调用：

```
[设备地址][功能码][参数...(大端)][校验字节 0x6B]
```

| 命令 | 功能码 | 帧长 | 参数 |
|---|---|---|---|
| 使能/失能 | `0xF3` | 6 | `0xAB`、使能状态、多机同步 |
| 速度模式 | `0xF6` | 8 | 方向(0=CW/1=CCW)、转速16位(RPM)、加速度档、多机同步 |
| 位置模式 | `0xFD` | 13 | 方向、转速16位、加速度档、脉冲数32位、相对/绝对、多机同步 |
| 立即停止 | `0xFE` | 5 | `0x98`、多机同步 |
| 同步运动（广播） | `0xFF` | 4 | `0x66` |
| 当前位置清零 | `0x0A` | 4 | `0x6D` |
| 解除堵转保护 | `0x0E` | 4 | `0x52` |
| 读系统参数 | `0x1F/0x24/0x35/0x36/0x37/0x3A/0x43` | 4~5 | 见 `emm42_v5.c` 的 `Emm42_ReadSysParams` |

校验字节按驱动器出厂默认的**固定 0x6B**；若驱动器改成 XOR/CRC 校验，需同步改 `EMM42_CHECK_BYTE` 与组帧末字节。

发送路径：题目状态机或 `BALLCTRL` → `module/emm42/emm42_robot.c`（角色→地址映射）→
`emm42_v5.c` 组帧 → `BspUart1_SendBytes()` 整帧阻塞发送。协议出口用 FreeRTOS
互斥量保证多线程整帧不交叉，并在调度器运行后每帧统一留至少 6ms 处理间隔；接口不等待回复。

接收路径：UART1 RX 中断（`bsp_uart.c` 的 `UART1_IRQHandler`）逐字节喂 `Emm42_OnRxByte`，以校验字节 `0x6B` 分帧，**仅用于诊断统计**（`Emm42_GetTxFrameCount/GetRxByteCount/GetRxFrameCount/GetLastReply`），不解析字段。上板时 `Emm42_GetRxByteCount() > 0` 即可判定总线双向通。由 `APP_FEATURE_EMM42` 开关门控。
