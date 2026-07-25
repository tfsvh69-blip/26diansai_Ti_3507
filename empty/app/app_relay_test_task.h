#ifndef APP_RELAY_TEST_TASK_H
#define APP_RELAY_TEST_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 继电器通断测试任务（v1.1，PA24）。
 * 每 2 秒切换一次吸合/断开状态，用于验证继电器及其驱动的电磁铁负载。
 * 由 APP_FEATURE_RELAY 门控（见 common/app_config.h）。
 */
void AppRelayTestTask_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RELAY_TEST_TASK_H */
