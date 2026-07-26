#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "FreeRTOS.h"

/*
 * ==================== 功能总开关 ====================
 * 用开发板时按需勾选：把不用的外设任务置 0 即可（1=启用，0=禁用）。
 * 说明：
 *   - 只门控"任务/功能是否启动"，各外设的板级初始化(BspBoard_Init 里)始终保留，
 *     即引脚/定时器/串口都已配置好、随时可用，只是对应任务不跑、不占 CPU、不刷串口。
 *   - 想彻底不初始化某外设硬件时，再去 bsp_board.c / ti_msp_dl_config.c 里注释对应 init。
 *   - 激光测距(D1)是随 IMU 遥测整行一起输出的：APP_FEATURE_IMU=0 时该行不打印，
 *     此时即使 APP_FEATURE_LASER=1，激光仍在后台接收但没有打印出口（详见 app_imu_uart_task.c）。
 */
#define APP_FEATURE_LED_HEARTBEAT   (1U)  /* LED1(PB25) 心跳灯 */
#define APP_FEATURE_UART_ECHO       (0U)  /* UART0 接收回显自检（默认关闭：串口静默，需调试时改回 1 即可） */
#define APP_FEATURE_UI_MENU         (1U)  /* OLED 题目菜单 UI（4 键：K1上/K2下/K3确认/K4返回，独占 OLED） */
#define APP_FEATURE_PERIPH_OLED     (0U)  /* 旧外设测试任务（OLED已交给UI_MENU，两者抢屏，故互斥禁用） */
#define APP_FEATURE_SERVO           (0U)  /* 4 路舵机（SERVOSWEEP 任务，已取消：按键改做题目菜单） */
#define APP_FEATURE_MOTOR           (0U)  /* 4 路步进电机（MOTORTEST 任务，已取消：KEY1/KEY2 让给题目菜单） */
#define APP_FEATURE_IMU             (1U)  /* 六轴 IMU 读取 + Yaw 快照发布（供 OLED 状态栏显示，不依赖串口） */
#define APP_FEATURE_LASER           (1U)  /* UART2 激光测距1（RX 中断接收，供 OLED 状态栏显示，不依赖串口） */
#define APP_FEATURE_BALL_VISION     (1U)  /* UART0 接收上位机 $BALL 小球检测报文（RX 中断解析，OLED 右侧文字面板显示） */
#define APP_FEATURE_LINE_TRACK      (1U)  /* 7 路灰度循迹 PB17~PB23（直接读高低电平，OLED 菜单右半第6/7行显示状态） */
#define APP_FEATURE_RELAY           (1U)  /* 继电器 PA24 功能：板级初始化 + OLED 状态栏 R:ON/OFF 显示 + 对外接口 BspRelay_*(On/Off/Set/Toggle/IsOn) 可直接调用；不含自动切换 */
#define APP_FEATURE_RELAY_SELFTEST  (0U)  /* 继电器自检任务(RELAYTEST)：每 2 秒自动切换吸合/断开，仅上电验证用；默认关，置 1 恢复自检 */
#define APP_FEATURE_NRF24_TX_TEST   (1U)  /* NRF24L01+ 发射测试：每 500ms 向 USB 无线串口发送递增文本 */
#define APP_NRF24_DIAG_UART_LOG     (1U)  /* NRF24 排查日志：UART0 115200 每秒输出寄存器和发包计数，定位后改回 0 */
#define APP_NRF24_DIAG_COMPAT_SCAN  (1U)  /* NRF24 兼容扫描：轮换地址顺序与 CRC，命中 ACK 后自动锁定，定位后改回 0 */
/*
 * IMU 串口遥测日志独立开关：控制 IMU 任务是否向 UART0 打印启动信息、初始化诊断
 * 和 5Hz 姿态/激光遥测行。置 0 时 IMU 读取与 Yaw 快照发布照常运行（OLED 状态栏
 * 仍能显示 Yaw/距离），只是串口彻底静默。调试需要看遥测行时改回 1 即可。
 * 注意：此开关与 APP_FEATURE_IMU 独立，IMU=1+LOG=0 是正常组合（静默传感器模式）。
 */
#define APP_FEATURE_IMU_UART_LOG    (0U)  /* IMU 串口调试日志（0=静默/1=打开） */
/*
 * 说明：题目菜单 UI 接管 OLED 与 4 个按键，故本版默认把电机/舵机测试任务、旧外设
 * 测试任务(PERIPH_OLED)一并关闭——它们与 UI 争用按键或 OLED。底层驱动仍在
 * BspBoard_Init 中初始化，题目业务(onEnter/onLoop)里可直接调 bsp_motor/bsp_servo。
 * UART_ECHO 与 IMU_UART_LOG 默认关闭：串口保持静默，减少对调试/通信的干扰。
 */
/*
 * 互斥护栏：$BALL 接收中断会取空 UART0 RX FIFO，与 UART_ECHO 的轮询自检抢字节，
 * 二者不可同时启用。需要串口收发自检时先把 APP_FEATURE_BALL_VISION 置 0。
 */
#if (APP_FEATURE_BALL_VISION != 0U) && (APP_FEATURE_UART_ECHO != 0U)
#error "APP_FEATURE_BALL_VISION 与 APP_FEATURE_UART_ECHO 争用 UART0 RX，不能同时为 1"
#endif
/* =================================================== */

/* 任务栈单位为 word，不是 byte。 */
#define APP_LED_TASK_STACK_WORDS        (configMINIMAL_STACK_SIZE)
#define APP_LED_TASK_PRIORITY           (1U)

/* UART0 测试发送任务只发送短字符串，栈保持最小配置。 */
#define APP_UART_TEST_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE)
#define APP_UART_TEST_TASK_PRIORITY     (1U)

/* IMU 姿态读取包含 I2C、FIFO 解析和三角函数计算，栈空间单独放大。 */
#define APP_IMU_UART_TASK_STACK_WORDS   (configMINIMAL_STACK_SIZE * 4U)
#define APP_IMU_UART_TASK_PRIORITY      (1U)

/* 电机1按键控制任务：轮询按键并触发定长旋转，含串口输出，栈适当放大。 */
#define APP_MOTOR_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 2U)
#define APP_MOTOR_TEST_TASK_PRIORITY    (1U)

/* 外设测试任务：驱动 OLED 软件 I2C、LED2/LED3、蜂鸣器；OLED 刷屏占栈，放大到 3 倍。 */
#define APP_PERIPH_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 3U)
#define APP_PERIPH_TEST_TASK_PRIORITY    (1U)

/* PB22 心跳灯翻转周期。 */
#define APP_LED1_PERIOD_TICKS           pdMS_TO_TICKS(300U)

/* UART0 串口测试信息发送周期，历史发送测试保留。 */
#define APP_UART_TEST_PERIOD_TICKS      pdMS_TO_TICKS(1000U)

/* UART0 接收命令轮询周期。 */
#define APP_UART_RX_POLL_PERIOD_TICKS   pdMS_TO_TICKS(10U)

/* IMU 对外读取周期：10ms，即 100Hz。 */
#define APP_IMU_UART_PERIOD_TICKS       pdMS_TO_TICKS(10U)

/*
 * IMU + 激光测距1 串口打印节流：每 N 个读取周期打印一整行。
 * 读取保持 100Hz（持续排空 IMU FIFO）；整行含欧拉角+三轴加速度+三轴角速度+激光测距约 120 字节。
 * 打印降到 100Hz/N=5Hz（N=20），刷新不快、便于在串口助手里阅读，同时减轻单串口负载。
 * 若想更慢，把 N 调大（如 50→2Hz）。
 */
#define APP_IMU_PRINT_DIVIDER           (20U)

/*
 * IMU 初始化失败后的非阻塞重试节流：每 N 个读取周期(10ms)尝试一次重新初始化。
 * 100 → 每 1s 重试一次。改为非阻塞后，IMU 缺失/损坏时任务不再死等，
 * 主循环照常运行，激光 D1 等遥测正常输出（解决"IMU 坏了连激光也发不出"的耦合）。
 */
#define APP_IMU_REINIT_DIVIDER          (100U)

/* 电机1按键扫描周期：20ms，兼作简单去抖。 */
#define APP_MOTOR_KEY_POLL_TICKS        pdMS_TO_TICKS(20U)

/* 外设测试任务刷新周期：500ms。OLED 全屏软件 I2C 刷屏约几十 ms，周期不宜过短。 */
#define APP_PERIPH_TEST_PERIOD_TICKS    pdMS_TO_TICKS(500U)

/* 舵机1 测试任务：20ms 周期，逐 us 调整脉宽实现慢速平滑摆动。 */
#define APP_SERVO_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 2U)
#define APP_SERVO_TEST_TASK_PRIORITY    (1U)
#define APP_SERVO_TEST_PERIOD_TICKS     pdMS_TO_TICKS(20U)

/* 继电器通断测试任务：只翻转一个 GPIO，栈保持最小配置；每 2 秒切换一次状态。 */
#define APP_RELAY_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE)
#define APP_RELAY_TEST_TASK_PRIORITY    (1U)
#define APP_RELAY_TEST_PERIOD_TICKS     pdMS_TO_TICKS(2000U)

/*
 * NRF24L01+ 发射测试：上电等待模块完成复位后，每 500ms 发送一包。
 * 单次发送含自动重传和 12ms 软件超时，栈只需容纳 32 字节固定载荷。
 */
#define APP_NRF24_TX_TEST_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 2U)
#define APP_NRF24_TX_TEST_TASK_PRIORITY    (1U)
#define APP_NRF24_TX_STARTUP_TICKS         pdMS_TO_TICKS(100U)
#define APP_NRF24_TX_TEST_PERIOD_TICKS     pdMS_TO_TICKS(500U)
#define APP_NRF24_DIAG_LOG_DIVIDER         (2U)
#define APP_NRF24_DIAG_PROFILE_TRY_COUNT   (8U)

/*
 * OLED 题目菜单 UI 任务：轮询 4 键驱动菜单/运行状态机。
 * 栈放大到 3 倍（OLED 全屏软件 I2C 刷屏占栈，与旧 PERIPH 任务同量级）。
 * 30ms 轮询既是按键去抖窗口也是响应节拍；刷屏为事件驱动，非每周期刷。
 */
#define APP_UI_TASK_STACK_WORDS         (configMINIMAL_STACK_SIZE * 3U)
#define APP_UI_TASK_PRIORITY            (1U)
#define APP_UI_POLL_TICKS               pdMS_TO_TICKS(30U)

/*
 * 传感器状态栏（Yaw + 激光距离）刷新节流：每 N 个轮询周期局部刷一次。
 * 10 → 30ms×10=300ms(约3Hz)，人眼看数字够用。状态栏用 OLED_UpdateArea 只推送
 * 一行(128×8≈整屏1/8)，配合低频，对 CPU/实时性几乎无影响（整屏刷仍只在界面切换时）。
 */
#define APP_UI_STATUS_DIVIDER           (10U)

/*
 * IMU I2C 引脚物理测试开关。
 * 置 1 后 IMU 任务不初始化传感器，而是用开漏模拟方式低频翻转 PB2/PB3，
 * 并直接读取 GPIO 输入寄存器回报实际电平，用于排查 MCU 端口和外部拉低问题。
 */
#define APP_IMU_I2C_PIN_TEST_ENABLE     (0U)
#define APP_IMU_I2C_PIN_TEST_PERIOD_TICKS \
                                        pdMS_TO_TICKS(500U)

#endif
