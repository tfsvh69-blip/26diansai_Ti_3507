# 消息列表

当前工程尚未使用 FreeRTOS queue / event group / stream buffer 做业务消息传递，但有两处**线程安全的只读快照**被跨任务共享（发布者在临界区写、订阅者在临界区/模块内取一致快照，属于「最新值」语义，不需要队列的生产者-消费者排队）：

| 快照 | 接口 | 发布者 | 订阅者 | 用途 |
|---|---|---|---|---|
| IMU Yaw | `AppImuUartTask_GetYaw(int16_t *centideg)` → bool | `IMU100Hz`（100Hz 更新，临界区发布） | `UIMENU`（状态栏显示） | 偏航角（厘度，0.01°），返回 false 表示 IMU 未就绪 |
| 激光距离 | `LaserLd14_GetLatest(LaserLd14Data_t *)` → bool | UART2 RX 中断解析（`module/laser`） | `IMU100Hz`（并入遥测行）、`UIMENU`（状态栏显示） | 单点距离(mm)+置信度+统计，返回 false 表示未收到有效帧 |
| 小球检测 | `BallParser_GetLatest(BallData_t *)` → bool | UART0 RX 中断解析（`module/vision`） | `UIMENU`（右侧文字面板显示） | 上位机 `$BALL` 报文解析结果：found/x/y/count+统计，返回 false 表示未收到校验通过的帧 |
| 电机诊断 | `g_motorDiag`（`app_motor_status.h`，临界区读写） | `MOTORTEST`（默认禁用时不更新） | `PERIPH`（默认禁用） | 电机运行/方向/圈数，仅当这两个任务启用时有效 |

> 说明：以上都是「最新值快照」而非事件消息。若后续要做**有先后语义**的跨任务通信（命令、事件、数据流），仍应优先用 FreeRTOS queue / event group / stream buffer，并在本表追加记录。

## 上位机 → 下位机报文：`$BALL`（UART0 下行，115200 8N1）

上位机（视觉主机）经 UART0（PA10=TX/PA11=RX）向下位机下发小球检测结果，NMEA 风格带 XOR 校验的 ASCII，一帧一行，约 17 帧/秒：

```
$BALL,<found>,<x>,<y>,<n>*<CHK>\r\n
```

| 字段 | 含义 |
|---|---|
| `found` | 1=检测到球，0=没检测到 |
| `x` `y` | 主目标（面积最大）球心像素坐标（整数）；去畸变后 640×480，原点左上，x∈[0,639]、y∈[0,479]；`found=0` 时为 0,0 |
| `n` | 本帧检测到的球总数 |
| `CHK` | `$` 与 `*` 之间所有字符逐字节 XOR，两位大写十六进制 |

样例（校验值实测）：`$BALL,1,321,240,3*07`、`$BALL,0,0,0,0*03`。

接收路径：UART0 RX 中断（`bsp_uart.c` 的 `UART0_IRQHandler`）逐字节喂 `module/vision` 的 `BallParser_FeedByte` 行解析器 → 校验通过更新快照 → `UIMENU` 任务读取并在 OLED 右侧文字面板显示。由 `APP_FEATURE_BALL_VISION` 开关门控；与 `APP_FEATURE_UART_ECHO`（轮询自检）互斥（编译期护栏拦截）。
