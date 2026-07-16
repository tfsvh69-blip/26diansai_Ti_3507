# empty/app/ 阅读指南

本目录存放所有 FreeRTOS 应用层任务。建议按以下顺序从上往下读，从入口到细节、从简单到复杂。

---

## 阅读顺序

| 序号 | 文件 | 读什么 | 为什么先读 |
|---|---|---|---|
| **1** | [`main.c`](main.c) | 程序入口：`BspBoard_Init()` → 串口启动提示 → `App_Init()` → `vTaskStartScheduler()` | 先理解整机启动流程 |
| **2** | [`app_main.c`](app_main.c) | `App_Init()` 里创建了哪些任务、按什么顺序 | 拿到"任务清单"，知道有哪些角色在跑 |
| **3** | [`app_motor_status.h`](app_motor_status.h) | `AppMotorDiag_t` 结构体——任务间唯一的共享数据 | 理解任务间怎么传递诊断信息（单写多读，无需锁） |
| **4** | [`app_led_task.c`](app_led_task.c) | 最简单的任务：每 300ms 翻转一次 PB25 | 入门理解 FreeRTOS 任务的最简写法 |
| **5** | [`app_uart_test_task.c`](app_uart_test_task.c) | UART0 接收回显：收到字符→回答 `UART RX OK` | 理解串口多任务共享的递归互斥量模式 |
| **6** | [`app_motor_test_task.c`](app_motor_test_task.c) | **电机1 核心任务**：四按键定圈旋转，按键扫描→`BspMotor1_MoveSteps()`→ISR 后台梯形加减速 | 理解"任务发令→bsp 层+ISR 执行→自动停表"的分层模型 |
| **7** | [`app_periph_test_task.c`](app_periph_test_task.c) | 外设验证：OLED 刷屏 + LED2/LED3 翻转 + 蜂鸣器 + 读 `g_motorDiag` 显示电机状态 | 理解跨任务数据流向（MOTOR1 写 → PERIPH 读 OLED） |
| **8** | [`app_imu_uart_task.c`](app_imu_uart_task.c) | IMU 姿态读取：GPIO 软件 I2C → LSM6DSV16X → SFLP 融合欧拉角 + 加速度/角速度寄存器直读 → UART0 打印 | 最复杂的任务：I2C 超时处理、FIFO、打印节流、总线恢复 |
| **9** | [`app_servo_test_task.c`](app_servo_test_task.c) | 舵机 PWM 慢速平滑摆动测试 | TIMA0 四路 50Hz PWM，每个 CCP 独立占空比 |

---

## 阅读后应该能回答

1. **整机上电后发生了什么？** → 看 `main.c` + `app_main.c`
2. **"MOTOR1"任务怎么控制电机的？** → 看 `app_motor_test_task.c`，关注 `MotorKey_StartMove()` → `BspMotor1_MoveSteps()` 调用链
3. **怎么加一个新任务？** → 参照 `app_led_task.c` 的最简模式：`.c` 写 `Entry` + `Init`，`.h` 声明 `Init`，在 `app_main.c` 的 `App_Init()` 里调用
4. **电机状态怎么显示到 OLED 上的？** → MOTOR1 写 `g_motorDiag`（[app_motor_status.h](app_motor_status.h)）→ PERIPH 读并显示（[app_periph_test_task.c](app_periph_test_task.c)）
5. **多个任务共用 UART0 不会冲突吗？** → `bsp_uart.c` 里的递归互斥量，`BspUart0_Lock/Unlock` 保证整行原子

---

## 依赖关系

```
app 层（本目录）
  ├── 依赖 bsp/   （板级外设：GPIO、UART、电机 STEP/DIR/ENN、按键、舵机 PWM）
  ├── 依赖 module/（可复用模块：IMU 驱动、OLED 驱动）
  ├── 依赖 common/（app_config.h：任务栈/优先级/周期宏）
  └── 不依赖 algo/（算法层当前为空）

任务间数据流：
  MOTOR1 ──写──▶ g_motorDiag ──读──▶ PERIPH（OLED 显示）
    │                                    │
    └── UART0 ───────────────────────────┘  （各自打日志，递归互斥量保护）
```
