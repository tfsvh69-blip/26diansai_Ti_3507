#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ================== 电赛题目代码总入口（app/tasks/）==================
 *
 * 【第 N 题的代码在哪？】就在本目录的 taskN.c 里。
 *   task1.c → 第 1 题   task2.c → 第 2 题   ...   task6.c → 第 6 题
 *   当前 task1.c、task2.c 仅用于函数和硬件测试，尚不是真正赛题。
 *
 * 每道题只需实现三个钩子（下方声明），由题目菜单 UI 通过 app_robot_core 调用：
 *   TaskN_OnEnter()  进入本题时调用一次——初始化状态机、清零变量、使能电机等。
 *   TaskN_OnLoop()   进入后每 30ms(UIMENU 节拍 APP_UI_POLL_TICKS) 调用一次——
 *                    在这里写你的状态机主体（switch(state)），做动作、判条件、切状态。
 *   TaskN_OnExit()   按 K4 返回菜单时调用一次——急停/失能电机、复位状态，保证安全。
 *
 * 【怎么写状态机】每个 taskN.c 已给好骨架：一个状态枚举 + 一个静态 state 变量 +
 *   OnLoop 里的 switch。你只需：
 *     1. 按本题流程增删状态枚举（如 直行→路口→转弯→...→完成）；
 *     2. 在每个 case 里调用底层接口做动作、读传感器判条件、满足就切到下一个状态。
 *   "寻迹到路口就转弯""走够距离就停"这类判断，直接在对应 case 里写即可（你比我更快）。
 *
 * 【能调用的底层接口】（题目里直接 include 对应头文件即可）
 *   电机 bsp_motor.h ：所有命令均直接下发；BspMotor_SetSpeedRpm(id, rpm) 连续转（±方向）、
 *                      BspMotor_MoveSteps(id, steps, rpm) 定距、
 *                      BspMotor_SetSpeedRpm4(...) / BspMotor_MoveSteps4(...) 四轮一起、
 *                      BspMotor_StopAll()、BspMotor_IsStopped(id)/AllStopped()/GetRemainingSteps(id) 判完成。
 *   差速圆弧 diff_drive.h：DiffDrive_RunRadiusTurn(半径,中心RPM) 立即执行圆弧，
 *                          自动下发 M1/M2 左侧、M3/M4 右侧。
 *   舵机 bsp_servo.h ：BspServo_SetPulseUs(id, us)。
 *   姿态 app_imu_uart_task.h：AppImuUartTask_GetYaw(&cd) 取 Yaw(0.01°)。
 *   激光 laser_ld14.h：LaserLd14_GetLatest(&d) 取距离(mm)。
 *   小球 ball_parser.h：BallParser_GetLatest(&b) 取上位机视觉结果。
 *   蜂鸣器/LED bsp_buzzer.h / bsp_led.h。
 *
 * 【要更高频的控制环?】30ms 够做机动级决策；若某题要更快的闭环(如高频寻迹PID)，
 *   在该题 OnEnter 里 xTaskCreate 自己的高频任务、OnExit 里 vTaskDelete 销毁即可。
 * ==================================================================== */

/* 第 1 题 */
void Task1_OnEnter(void);
void Task1_OnLoop(void);
void Task1_OnExit(void);

/* 第 2 题 */
void Task2_OnEnter(void);
void Task2_OnLoop(void);
void Task2_OnExit(void);

/* 第 3 题 */
void Task3_OnEnter(void);
void Task3_OnLoop(void);
void Task3_OnExit(void);

/* 第 4 题 */
void Task4_OnEnter(void);
void Task4_OnLoop(void);
void Task4_OnExit(void);

/* 第 5 题 */
void Task5_OnEnter(void);
void Task5_OnLoop(void);
void Task5_OnExit(void);

/* 第 6 题 */
void Task6_OnEnter(void);
void Task6_OnLoop(void);
void Task6_OnExit(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
