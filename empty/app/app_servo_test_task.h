#ifndef APP_SERVO_TEST_TASK_H
#define APP_SERVO_TEST_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 舵机1 慢速来回摆动测试任务（v1.1）。
 * 20ms 周期，脉宽在 600~2400us 间往复，每步变 10us，单程约 3.6 秒。
 */
void AppServoTestTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SERVO_TEST_TASK_H */
