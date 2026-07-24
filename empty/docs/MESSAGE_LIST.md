# 消息列表

当前工程尚未使用 FreeRTOS queue / event group / stream buffer 做业务消息传递，但有两处**线程安全的只读快照**被跨任务共享（发布者在临界区写、订阅者在临界区/模块内取一致快照，属于「最新值」语义，不需要队列的生产者-消费者排队）：

| 快照 | 接口 | 发布者 | 订阅者 | 用途 |
|---|---|---|---|---|
| IMU Yaw | `AppImuUartTask_GetYaw(int16_t *centideg)` → bool | `IMU100Hz`（100Hz 更新，临界区发布） | `UIMENU`（状态栏显示） | 偏航角（厘度，0.01°），返回 false 表示 IMU 未就绪 |
| 激光距离 | `LaserLd14_GetLatest(LaserLd14Data_t *)` → bool | UART2 RX 中断解析（`module/laser`） | `IMU100Hz`（并入遥测行）、`UIMENU`（状态栏显示） | 单点距离(mm)+置信度+统计，返回 false 表示未收到有效帧 |
| 电机诊断 | `g_motorDiag`（`app_motor_status.h`，临界区读写） | `MOTORTEST`（默认禁用时不更新） | `PERIPH`（默认禁用） | 电机运行/方向/圈数，仅当这两个任务启用时有效 |

> 说明：以上都是「最新值快照」而非事件消息。若后续要做**有先后语义**的跨任务通信（命令、事件、数据流），仍应优先用 FreeRTOS queue / event group / stream buffer，并在本表追加记录。
