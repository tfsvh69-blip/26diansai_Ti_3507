#include "app_main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_imu_uart_task.h"
#include "app_led_task.h"
#include "app_motor_test_task.h"
#include "app_nrf24_tx_test_task.h"
#include "app_periph_test_task.h"
#include "app_relay_test_task.h"
#include "app_robot_core.h"
#include "app_servo_test_task.h"
#include "app_uart_test_task.h"
#include "app_ui_task.h"
#include "ball_parser.h"
#include "bsp_uart.h"
#include "emm42_robot.h"
#include "laser_ld14.h"

/*
 * ============================ 应用层顶层编排 ============================
 *
 * App_Init() 是整个系统的"总装线"：硬件已由 BspBoard_Init() 就绪后，
 * 这里把各功能拆成独立的 FreeRTOS 任务分别创建，任务之间不互相调用，
 * 各自按周期跑，靠 BSP/模块层的线程安全接口共享硬件。启动关系：
 *
 *   main() → BspBoard_Init()(裸机初始化外设) → App_Init()(建任务) → vTaskStartScheduler()
 *
 * 【任务清单】（周期/优先级/栈集中在 common/app_config.h；由 APP_FEATURE_* 门控）
 *   LED1      心跳灯，300ms 翻转 PB25，用来一眼确认调度器活着
 *   UIMENU    OLED 题目菜单：4 键选题/运行 + 底部传感器状态栏(Yaw/距离)，独占 OLED
 *             + 题目业务委托 app_robot_core 模块（6 道题 dispatch 表，钩子占位待填）
 *   IMU100Hz  100Hz 读六轴姿态 + 发布 Yaw 快照（OLED状态栏用），默认不打印串口
 *   NRF24TX   每 500ms 向 USB 无线串口发送带递增序号的测试文本
 *   （默认关闭：UART0TX/PERIPH/SERVOSWEEP/MOTORTEST —— 串口静默、按键/OLED 让给 UIMENU）
 *
 * 【中断驱动（非任务）】
 *   激光测距1：UART2 RX 中断逐字节喂 module/laser 解析器，见下方注释。
 *   电机 STEP：TIMG0_IRQHandler 做定距计步（bsp_motor.c）。
 *
 * 【数据流】
 *   IMU(软件I2C) ──► IMU100Hz 任务每 100Hz 读融合角 ──► 临界区快照 s_yawCentideg
 *                                       │
 *     激光(UART2中断) ──► module/laser ─┤（AppImuUartTask_GetYaw / LaserLd14_GetLatest）
 *                                       │
 *                       UIMENU 任务 ────┘  状态栏每 300ms 局部刷 Yaw + 距离
 *   串口遥测(APP_FEATURE_IMU_UART_LOG=1时)：姿态/激光按 5Hz 发到 UART0（默认关闭）
 *
 * 设计约定：任务间不共享业务全局变量；跨层只通过 bsp 层、module 层的接口或
 * 线程安全快照 getter 访问。OLED 不直接访问软件 I2C/激光，避免总线争用。
 *
 * 硬件备注：v1.1 板 OLED 在板载 PB8/PB9（软件 I2C），TMC 细分改用 PB0/PB1，两者不再冲突。
 * ======================================================================
 */

void App_Init(void)
{
    /*
     * 各功能由 common/app_config.h 的 APP_FEATURE_* 开关按需启用/禁用。
     * 置 0 的功能不创建任务、不占 CPU、不刷串口；外设的板级初始化仍保留，随时可再启用。
     */

#if (APP_FEATURE_LED_HEARTBEAT != 0U)
    /* 心跳灯：每 300ms 翻转 LED1(PB25)，一眼确认 FreeRTOS 调度在跑。 */
    AppLedTask_Init();
#endif

#if (APP_FEATURE_UART_ECHO != 0U)
    /* UART0 接收自检：收到非换行字符回 "UART RX OK"，验证调试串口收发。 */
    AppUartTestTask_Init();
#endif

#if (APP_FEATURE_UI_MENU != 0U)
    /* OLED 题目菜单：4 键选题/确认/返回，独占 OLED 与 KEY1~4（取代旧的电机/舵机按键测试）。 */
    AppUiTask_Init();
#endif

#if (APP_FEATURE_PERIPH_OLED != 0U)
    /* 外设综合验证：每 500ms 刷 OLED + LED2/LED3 心跳 + 蜂鸣器，并显示电机测试状态。 */
    AppPeriphTestTask_Init();
#endif

#if (APP_FEATURE_SERVO != 0U)
    /* 舵机测试：4 个舵机各自独立错相摆动(800↔2200us，不用按键)，演示四路可完全独立控制。 */
    AppServoTestTask_Init();
#endif

#if (APP_FEATURE_MOTOR != 0U)
    /* 电机测试：KEY1/KEY2 让 4 个电机一起正/反转 2 圈，验证四路步进电机是否都正常。 */
    AppMotorTestTask_Init();
#endif

#if (APP_FEATURE_RELAY_SELFTEST != 0U)
    /*
     * 继电器自检任务(默认关，APP_FEATURE_RELAY_SELFTEST=0)：每 2 秒自动切换继电器(PA24)
     * 吸合/断开，仅上电验证继电器及其电磁铁负载用。正常运行不需要自动切换，继电器由业务
     * 代码经 bsp_relay 接口(BspRelay_On/Off/Set)按需控制；OLED 状态栏仍显示当前吸合/断开态。
     */
    AppRelayTestTask_Init();
#endif

#if (APP_FEATURE_NRF24_TX_TEST != 0U)
    /* NRF24L01+ 发射测试：2.402GHz/2Mbps/16位CRC，每 500ms 发送递增文本。 */
    AppNrf24TxTestTask_Init();
#endif

#if (APP_FEATURE_LASER != 0U)
    /*
     * 激光测距1（UART2/PB15/PB16，230400 8N1）：不是任务，走 UART2 RX 中断，
     * 逐字节喂给 LaserLd14 解析器；解析出的距离由 IMU 任务在打印整行时一并输出。
     * 先复位解析器，再注册回调并放开中断。
     */
    LaserLd14_Reset();
    BspUart2_Init(LaserLd14_FeedByte);
#endif

#if (APP_FEATURE_BALL_VISION != 0U)
    /*
     * 上位机小球检测报文（UART0/PA10/PA11，115200 8N1）：不是任务，走 UART0 RX 中断，
     * 逐字节喂给 BallParser 解析 $BALL 帧；解析结果由 UIMENU 任务在 OLED 右侧文字面板显示。
     * 先复位解析器，再注册回调并放开中断。与 UART_ECHO 轮询自检互斥（见 app_config.h 护栏）。
     */
    BallParser_Reset();
    BspUart0_SetRxHandler(BallParser_FeedByte);
#endif

#if (APP_FEATURE_EMM42 != 0U)
    /*
     * 张大头 Emm42_V5.0 闭环步进驱动（UART1/PA17/PB5，115200 8N1，当前 3 路：
     * 摆杆/左轮/右轮，见 module/emm42/emm42_robot.h）：不是任务，命令由题目
     * 状态机（app/tasks/task5.c）经 Emm42Robot_* 接口按需下发，驱动器回复走
     * UART1 RX 中断逐字节喂给 module/emm42 的诊断统计。此处只注册回调 + 放开中断。
     */
    Emm42Robot_Init();
#endif

#if (APP_FEATURE_IMU != 0U)
    /* IMU 姿态：100Hz 读六轴 + 发布 Yaw 快照（OLED 状态栏用）。串口遥测由 APP_FEATURE_IMU_UART_LOG 独立控制（默认关）。 */
    AppImuUartTask_Init();
#endif
}
