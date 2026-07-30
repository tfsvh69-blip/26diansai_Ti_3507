#ifndef EMM42_V5_H
#define EMM42_V5_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============ 张大头 Emm42_V5.0 闭环步进驱动器 UART 协议模块 ============
 *
 * 硬件：v1.1 排针 H7，UART1（MCU PA17=TX → 驱动器 RX，PB5=RX ← 驱动器 TX），115200 8N1。
 *       当前总线上挂 3 台驱动器（地址 1/2/3），用【设备地址】区分（出厂需先用 USB-TTL 各设一次）。
 *       本文件只是【协议层】，不知道地址 1/2/3 分别是小车上的哪个部件；
 *       地址 → 部件（摆杆/左轮/右轮）的角色映射见同目录 emm42_robot.h，上层业务优先用那一层。
 *
 * 与 bsp_motor（TMC2209 开环 STEP/DIR）的区别：
 *   - 本模块是【串口命令式】驱动，MCU 只发一帧命令，驱动器内部闭环执行，不占用定时器；
 *   - 速度/位置由驱动器自身维护，MCU 不产生脉冲，因此没有 ISR 计步这一套。
 *
 * 协议要点（Emm_V5 命令帧）：
 *   [地址][功能码][参数...][校验字节]
 *   本模块按驱动器出厂默认的【固定校验 0x6B】组帧；若驱动器改成 XOR/CRC 校验，
 *   需同步修改 EMM42_CHECK_BYTE 及组帧末字节。
 *
 * 用法（速度模式，见 app/tasks/task5.c）：
 *   Emm42_Init();                                     // 注册 UART1 RX 回调（上电一次）
 *   Emm42_Enable(EMM42_ADDR_MOTOR1, true, false);     // 使能电机
 *   Emm42_SetSpeedRpm(EMM42_ADDR_MOTOR1, 60, 10);     // +60RPM 起转（负数反转，0 停）
 *   Emm42_StopNow(EMM42_ADDR_MOTOR1, false);          // 立即停止
 *
 * 重要：所有接口只组帧下发、不等待回复；调度器运行后，协议出口会用互斥量
 * 保证多线程整帧原子，并在每帧后 vTaskDelay 6ms 给驱动器处理/回复。
 * 调度器启动前的上电失能流程仍由 App_Init 在调用间显式留 10ms。
 * ======================================================================
 */

/* 设备地址：0x00 为广播（所有驱动器都执行，且不回复）。 */
#define EMM42_ADDR_BROADCAST   (0x00U)
#define EMM42_ADDR_MOTOR1      (0x01U)
#define EMM42_ADDR_MOTOR2      (0x02U)
#define EMM42_ADDR_MOTOR3      (0x03U)

/* 命令帧固定校验字节（驱动器出厂默认校验方式）。 */
#define EMM42_CHECK_BYTE       (0x6BU)

/* 速度上限保护：协议本身支持到 5000RPM，测试阶段限到 3000RPM 更安全。 */
#define EMM42_MAX_RPM          (3000U)

/* 旋转方向（驱动器视角；实际转向还取决于电机接线相序）。 */
typedef enum {
    EMM42_DIR_CW  = 0,   /* 顺时针 */
    EMM42_DIR_CCW = 1    /* 逆时针 */
} Emm42Dir_t;

/* 可读取的系统参数（功能码见 emm42_v5.c 的 switch）。 */
typedef enum {
    EMM42_PARAM_VERSION  = 0,    /* 固件版本 */
    EMM42_PARAM_VBUS,            /* 总线电压 */
    EMM42_PARAM_VEL,             /* 实时转速 */
    EMM42_PARAM_CPOS,            /* 实时位置 */
    EMM42_PARAM_PERR,            /* 位置误差 */
    EMM42_PARAM_FLAG,            /* 使能/到位/堵转标志 */
    EMM42_PARAM_STATE            /* 驱动器状态 */
} Emm42SysParam_t;

/* ==================== 初始化 ==================== */

/*
 * 注册 UART1 接收回调并清空诊断统计。
 * 需在 SYSCFG_DL_UART_1_init() 之后、调度器启动前调用一次（当前在 App_Init 中）。
 */
void Emm42_Init(void);

/* ==================== 运动控制 ==================== */

/*
 * 使能/失能电机。enable=false 后电机失力（可手动转动）。
 * sync=true 时命令暂存、等 Emm42_SyncMotion() 才生效（多机同步用）。
 */
void Emm42_Enable(uint8_t addr, bool enable, bool sync);

/*
 * 速度模式（恒速连续旋转）。
 *   dir  旋转方向；rpm 目标转速（0~EMM42_MAX_RPM，超出自动限幅）；
 *   acc  加速度档位，0=不使用曲线、立即到目标速度，数值越大加速越快；
 *   sync 是否等待同步信号。
 */
void Emm42_VelControl(uint8_t addr, Emm42Dir_t dir, uint16_t rpm, uint8_t acc, bool sync);

/*
 * 速度模式便捷封装：用【带符号 RPM】表达方向与转速。
 *   rpm > 0 → CW，rpm < 0 → CCW，rpm == 0 → 立即停止（转发到 Emm42_StopNow）。
 */
void Emm42_SetSpeedRpm(uint8_t addr, int16_t rpm, uint8_t acc);

/*
 * 位置模式（定量运动）。
 *   clk      目标脉冲数（驱动器默认 16 细分时 3200 脉冲 = 1 圈）；
 *   absolute true=绝对位置，false=相对当前位置。
 */
void Emm42_PosControl(uint8_t addr, Emm42Dir_t dir, uint16_t rpm, uint8_t acc,
                      uint32_t clk, bool absolute, bool sync);

/* 立即停止（急停）。 */
void Emm42_StopNow(uint8_t addr, bool sync);

/* 触发同步运动：让此前所有 sync=true 暂存的命令同时生效（广播帧）。 */
void Emm42_SyncMotion(void);

/* ==================== 辅助命令 ==================== */

/* 把当前位置清零（作为新的位置原点）。 */
void Emm42_ResetCurPosToZero(uint8_t addr);

/* 解除堵转保护（驱动器报堵转后需要此命令才能重新运行）。 */
void Emm42_ResetClogProtection(uint8_t addr);

/* 请求读取系统参数；回复经 UART1 中断收下，用下方诊断接口查看原始字节。 */
void Emm42_ReadSysParams(uint8_t addr, Emm42SysParam_t param);

/* ==================== 诊断（判断总线通不通） ==================== */

/* 已下发的命令帧数（累计）。 */
uint32_t Emm42_GetTxFrameCount(void);

/* 已收到的回复字节数（累计）。为 0 说明驱动器没回话：查接线/波特率/地址。 */
uint32_t Emm42_GetRxByteCount(void);

/* 已收到的完整回复帧数（以校验字节 0x6B 结尾计一帧）。 */
uint32_t Emm42_GetRxFrameCount(void);

/*
 * 取最后一帧回复的原始字节（最长 EMM42_REPLY_MAX_LEN）。
 * 返回 false 表示尚未收到任何完整回复帧。
 */
#define EMM42_REPLY_MAX_LEN    (16U)
bool Emm42_GetLastReply(uint8_t *buf, uint8_t *len);

#ifdef __cplusplus
}
#endif

#endif /* EMM42_V5_H */
