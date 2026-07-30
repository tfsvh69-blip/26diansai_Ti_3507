#ifndef APP_LIFT_HOMING_H
#define APP_LIFT_HOMING_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 开机 ID1（摆杆升降丝杆）自动归零。
 *
 * 必须在 Emm42Robot_Init() 之后、调度器启动前（App_Init 内）调用一次：
 * 此时还没有其它任务会抢占 ID1，全程用 Delay_ms 忙等轮询限位开关，
 * 与 App_Emm42BootDisableAll() 的写法一致。
 */
void AppLiftHoming_RunAtBoot(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LIFT_HOMING_H */
