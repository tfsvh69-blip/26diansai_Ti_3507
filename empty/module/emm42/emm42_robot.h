#ifndef EMM42_ROBOT_H
#define EMM42_ROBOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============ 本车 3 路张大头 Emm42_V5.0 闭环步进电机——角色映射层 ============
 *
 * emm42_v5.c/h 是纯协议层，只认【设备地址】，不知道地址对应小车上哪个部件。
 * 本文件在协议层之上按【当前小车的实际用途】再包一层，让上层业务代码用
 * "摆杆/左轮/右轮"取代协议地址，不用到处记地址数字对应哪个部件。
 *
 * 当前 3 路设备地址分配（2026-07-29 确定，需已用上位机 USB-TTL 分别设好地址，
 * 见 emm42_v5.h 顶部说明；同一 UART1 总线，靠地址区分，接线不变仍是 H7）：
 *   地址1 = 摆杆高低调节电机（升降）
 *   地址2 = 左轮
 *   地址3 = 右轮
 *
 * ⚠️ 方向标定状态（2026-07-29）：
 *   本层集中维护各角色的方向补偿，使上层业务代码不必感知协议层 CW/CCW。
 *   地址2（左轮）已确认正方向直通，地址3（右轮）已确认需取反；地址1（摆杆）
 *   尚待实机确认，当前暂按直通处理。确认地址1方向后，只需修改本层标定表，
 *   不用改任务代码。左右轮差速运动学仍未实现。
 *
 * 【重要】UART1 总线电气风险（继承自协议层 §emm42_v5.h，现在是 3 台并联）：
 *   3 台驱动器的 TX 都推挽输出并联在同一根 PB5 上，被寻址方拉低时未被寻址的
 *   驱动器仍可能驱动高电平，属于总线冲突（对应接线文档风险 R1，原文只提两台，
 *   现为三台，风险等比放大）。硬件上必须加肖特基线与（如 BAT54S）+ 上拉才能
 *   3 台同时接；改造完成前调试期建议一次只接 1~2 台，用下方 Enable/Init 接口
 *   按需跳过未接的角色。
 * ================================================================================
 */

/* 小车上 3 路 Emm42 电机的角色（顺序固定，新增角色需同时改 .c 里的地址表）。 */
typedef enum {
    EMM42_ROBOT_LIFT = 0,   /* 摆杆高低调节，协议地址 EMM42_ADDR_MOTOR1 */
    EMM42_ROBOT_WHEEL_L,    /* 左轮，协议地址 EMM42_ADDR_MOTOR2 */
    EMM42_ROBOT_WHEEL_R,    /* 右轮，协议地址 EMM42_ADDR_MOTOR3 */
    EMM42_ROBOT_COUNT
} Emm42RobotId_t;

/* ==================== 初始化 ==================== */

/*
 * 初始化本层（内部直接调用 Emm42_Init() 注册 UART1 RX 中断 + 清诊断计数）。
 * 需在 SYSCFG_DL_UART_1_init() 之后、调度器启动前调用一次（当前在 App_Init 中）。
 * 只需调这一个接口，不需要额外再调 Emm42_Init()。
 */
void Emm42Robot_Init(void);

/* ==================== 按角色控制（单帧，OnLoop 安全） ==================== */

/* 单路使能/失能。enable=false 后该电机失力（可手动转动/推动）。 */
void Emm42Robot_Enable(Emm42RobotId_t id, bool enable);

/*
 * 单路速度模式：rpm 的正负会先经过本层角色方向标定，再转换为协议层 CW/CCW；
 * rpm==0 转成急停帧；acc 为加速度档位，0=不使用曲线立即变速。
 * 与底层 Emm42_* 系列一致：单帧下发、非阻塞、不等回复，调用方自行控制帧间隔
 * （不要在同一个 OnLoop 里连续调用多个角色，参考 app/tasks/task5.c 的按拍错开写法）。
 */
void Emm42Robot_SetSpeedRpm(Emm42RobotId_t id, int16_t rpm, uint8_t acc);

/*
 * 单路速度模式原样下发：与 Emm42Robot_SetSpeedRpm 不同，rpm==0 仍发送速度
 * 模式帧，因此控制器会按 acc 指定的曲线减速到 0 RPM，而不是立即急停。
 */
void Emm42Robot_VelControl(Emm42RobotId_t id, int16_t rpm, uint8_t acc);

/* 单路立即停止（急停）。 */
void Emm42Robot_Stop(Emm42RobotId_t id);

/* ==================== 一次性/安全场景便捷接口（多帧背靠背，仅限 OnEnter/OnExit 用） ==================== */

/*
 * 依次急停 3 路（背靠背 3 帧，无延时）。
 * 仅用于 OnExit 这类"一次性、安全收尾"场景——急停帧短且是幂等指令，
 * 背靠背下发的可靠性问题远小于运动类指令；不要在 OnLoop 里调用。
 * 参考 app/tasks/task5.c 的 Task5_OnExit 用法。
 */
void Emm42Robot_StopAll(void);

/* 取该角色对应的协议地址（1/2/3），供诊断打印或直接调用 emm42_v5 的地址接口。 */
uint8_t Emm42Robot_GetAddr(Emm42RobotId_t id);

#ifdef __cplusplus
}
#endif

#endif /* EMM42_ROBOT_H */
