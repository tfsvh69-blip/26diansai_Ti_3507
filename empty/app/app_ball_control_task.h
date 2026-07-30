#ifndef APP_BALL_CONTROL_TASK_H
#define APP_BALL_CONTROL_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 上电后默认居中的相机水平像素坐标。 */
#define APP_BALL_CONTROL_CENTER_X_PX (320)

typedef enum {
    APP_BALL_CONTROL_OFF = 0,
    APP_BALL_CONTROL_STARTING,
    APP_BALL_CONTROL_WAIT_VISION,
    APP_BALL_CONTROL_RUNNING,
    APP_BALL_CONTROL_HOLDING,
    APP_BALL_CONTROL_LOST,
    APP_BALL_CONTROL_FAULT_EDGE
} AppBallControlState_t;

typedef struct {
    AppBallControlState_t state;
    int16_t targetPx;
    int16_t measuredPx;
    float filteredPx;
    float velocityPxPerSec;
    int32_t commandPulse;   /* 位置模式当前下发的绝对目标脉冲（原速度模式的 commandRpm/integralRpm 已废弃） */
    uint32_t sampleSeq;
} AppBallControlStatus_t;

/* 创建钢球位置控制线程和命令队列；调度器启动前调用一次。 */
void AppBallControlTask_Init(void);

/*
 * 启动或更新目标位置。接口只投递命令，不在调用者上下文直接操作电机。
 * targetPx 超出安全范围时返回 false。
 */
bool AppBallControl_RequestTarget(int16_t targetPx);

/* 请求停止闭环；控制线程会先急停 ID1，再失能并回到 OFF。 */
void AppBallControl_RequestStop(void);

/* 读取控制线程发布的最新状态快照。 */
void AppBallControl_GetStatus(AppBallControlStatus_t *out);

/* 返回当前是否占用 ID1；STARTING 到故障保持态都视为占用。 */
bool AppBallControl_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_CONTROL_TASK_H */
