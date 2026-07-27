#include "app_tasks.h"

#include <stdbool.h>
#include <stdint.h>

#include "angle_utils.h"
#include "app_imu_uart_task.h"
#include "bsp_motor.h"
#include "diff_drive.h"
#include "pid.h"

/* ==================================================================
 * 第 3 题：陀螺仪闭环左转 90° 并保持
 *
 * 进入后记录起始 Yaw，以 PID 闭环原地转向（左侧后退、右侧前进），
 * 使小车左转 90°（Yaw 减小 90°），到达目标角度后停车保持。
 *
 * PID 控制器来自 algo/pid.h，角度工具来自 algo/angle_utils.h；
 * 本题只保留可调参数宏，方便实测时快速改值。
 *
 * Yaw 行为约定（已实测）：
 *   - 范围 -180° ~ +180°（内部单位：厘度 0.01°）
 *   - 小车左转 → Yaw 递减（如 180° → 150° → 0° → -179°）
 *   - 每次复位后初始 Yaw 不同（IMU SFLP 融合收敛过程），
 *     但本题使用相对角度（目标 = 起始 − 90°），不受初始值影响。
 *   - 当前约 5° 稳态误差，怀疑是低速死区/摩擦导致积分不够；
 *     解决思路见本文件末尾注释及 algo/pid.h。
 *
 * 状态机：START(录起始角) → TURNING(PID闭环) → HOLD(到位停车)
 * OnLoop 每 30ms 被 UIMENU 调一次；PID 以固定 dt=30ms 计算。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 可调参数（实测按需微调，改完重新编译烧录即可）
 * ------------------------------------------------------------------ */

/* PID 增益。当前 Kp=1.5/Ki=0.02/Kd=0.2 已通过冒烟测试，转 90° 较平稳。 */
#define T3_KP                  (1.5F)
#define T3_KI                  (0.1F)
#define T3_KD                  (0.2F)

/* PID 输出限幅（RPM 绝对值）。原地转向不宜太快。 */
#define T3_MAX_TURN_RPM        (120)

/* 积分抗饱和限幅（RPM）。防止长期未到位时积分项无限累积。 */
#define T3_INTEGRAL_LIMIT_RPM  (40.0F)

/* PID 计算周期（秒），与 UIMENU 调用周期一致。 */
#define T3_DT_SEC              (0.03F)

/* 到位判定：误差绝对值 < 此阈值（厘度）× 连续周期数。 */
#define T3_SETTLE_THRESHOLD_CD (100)    /* 1.0° */
#define T3_SETTLE_CYCLES       (15)     /* 15 拍 × 30ms = 450ms 稳定窗口 */

/* 最小转向 RPM：PID 输出绝对值低于此值时钳到最低，避免低速丢步。
 * ⚠️ 此值与 5° 稳态误差有关：若电机在 <10 RPM 时也能稳定转动，
 *    降低此值有助于消除小角度残余误差；详见文件末尾注释。 */
#define T3_MIN_TURN_RPM        (8.0F)

/* ------------------------------------------------------------------
 * 状态机
 * ------------------------------------------------------------------ */

typedef enum {
    T3_STATE_START = 0,   /* 记录起始 Yaw，初始化 PID */
    T3_STATE_TURNING,     /* PID 闭环转向中 */
    T3_STATE_HOLD,        /* 已到位，停车保持 */
    T3_STATE_DONE         /* 完成（IMU 未就绪等异常） */
} Task3State_t;

static Task3State_t s_state;
static Pid_t        s_pid;
static int16_t      s_targetYawCd;    /* 目标 Yaw（厘度） */
static uint32_t      s_settleCount;    /* 连续到位周期计数 */

void Task3_OnEnter(void)
{
    int16_t startYawCd = 0;
    bool    yawValid;

    /* 急停上一题残留命令，使能驱动。 */
    BspMotor_EmergencyStop(BSP_MOTOR_1);
    BspMotor_EmergencyStop(BSP_MOTOR_2);
    BspMotor_EmergencyStop(BSP_MOTOR_3);
    BspMotor_EmergencyStop(BSP_MOTOR_4);
    BspMotor_EnableAll();

    /* 读取起始 Yaw，计算目标 = 起始 − 90°（左转）。 */
    yawValid = AppImuUartTask_GetYaw(&startYawCd);
    if (yawValid) {
        s_targetYawCd = Angle_NormalizeCd((int32_t)startYawCd - 9000);
    } else {
        /* IMU 未就绪（极少发生），不瞎转。 */
        s_targetYawCd = 0;
        s_state = T3_STATE_DONE;
        return;
    }

    /* 用 algo/pid 模块初始化 PID：传入增益 + 限幅。 */
    Pid_Init(&s_pid, T3_KP, T3_KI, T3_KD,
             T3_INTEGRAL_LIMIT_RPM, (float)T3_MAX_TURN_RPM);

    s_settleCount = 0U;
    s_state       = T3_STATE_TURNING;
}

void Task3_OnLoop(void)
{
    int16_t currentYawCd;
    int32_t errorCd;
    float   errorDeg;
    float   turnRpm;
    bool    yawValid;

    switch (s_state) {
    case T3_STATE_START:
        break;

    case T3_STATE_TURNING:
        yawValid = AppImuUartTask_GetYaw(&currentYawCd);
        if (!yawValid) {
            BspMotor_StopAll();
            break;
        }

        /* 最短路径角度差（algo/angle_utils）。 */
        errorCd  = Angle_DiffCd(s_targetYawCd, currentYawCd);
        errorDeg = (float)errorCd * 0.01F;

        /* PID 更新 → 转向 RPM（algo/pid）。
         * errorDeg < 0（需左转）→ turnRpm < 0 → 左转；
         * errorDeg > 0（需右转）→ turnRpm > 0 → 右转。 */
        turnRpm = Pid_Update(&s_pid, errorDeg, T3_DT_SEC);

        /* 死区：误差和输出都很小时停车，避免低速嗡嗡响。 */
        if ((errorCd < T3_SETTLE_THRESHOLD_CD) && (errorCd > -T3_SETTLE_THRESHOLD_CD) &&
            (turnRpm < (float)T3_MIN_TURN_RPM) && (turnRpm > -(float)T3_MIN_TURN_RPM)) {
            BspMotor_StopAll();
            s_settleCount++;
            if (s_settleCount >= T3_SETTLE_CYCLES) {
                s_state = T3_STATE_HOLD;
            }
        } else {
            DiffDriveWheelRpm_t cmd;
            int32_t              rpmInt;

            s_settleCount = 0U;

            /* RPM 四舍五入取整。 */
            rpmInt = (turnRpm >= 0.0F) ? (int32_t)(turnRpm + 0.5F)
                                        : (int32_t)(turnRpm - 0.5F);

            /* 最小 RPM 钳位：低于 T3_MIN_TURN_RPM 时抬到最低值，避免低速丢步。 */
            if (rpmInt > 0 && rpmInt < (int32_t)T3_MIN_TURN_RPM) {
                rpmInt = (int32_t)T3_MIN_TURN_RPM;
            } else if (rpmInt < 0 && rpmInt > -((int32_t)T3_MIN_TURN_RPM)) {
                rpmInt = -((int32_t)T3_MIN_TURN_RPM);
            }

            /* 原地转向：左转 = 左侧后退(M1/M2负) + 右侧前进(M3/M4正)。 */
            DiffDrive_CalcPivotTurn((turnRpm < 0.0F), (int32_t)(rpmInt < 0 ? -rpmInt : rpmInt), &cmd);
            DiffDrive_ApplyWheelRpmImmediate(&cmd);
        }
        break;

    case T3_STATE_HOLD:
        BspMotor_StopAll();
        break;

    case T3_STATE_DONE:
        BspMotor_StopAll();
        break;

    default:
        break;
    }
}

void Task3_OnExit(void)
{
    BspMotor_EmergencyStop(BSP_MOTOR_1);
    BspMotor_EmergencyStop(BSP_MOTOR_2);
    BspMotor_EmergencyStop(BSP_MOTOR_3);
    BspMotor_EmergencyStop(BSP_MOTOR_4);
    BspMotor_DisableAll();

    Pid_Reset(&s_pid);
    s_settleCount = 0U;
    s_state       = T3_STATE_START;
}

/* ==================================================================
 * 5° 稳态误差分析与解决思路
 *
 * 现象：转 90° 实际只转了约 85°，有约 5° 残余误差。
 *
 * 根因分析（按可能性排序）：
 *
 * 1.【最可能】最小 RPM 钳位造成的"死区"。
 *    T3_MIN_TURN_RPM=10 意味着 PID 输出 0~9 RPM 都会变成 0 或 10 RPM。
 *    当误差降到 5° 时，P 项只贡献 1.5×5=7.5 RPM，低于 10 RPM 最小阈值，
 *    PID 输出被钳到 0（死区条件触发）或 10 RPM（钳位后）。
 *    → 如果是死区触发：电机根本不转，误差永远消除不了。
 *    → 如果是钳到 10 RPM：10 RPM 对 5° 来说偏大，可能导致过冲震荡。
 *
 * 2.【次要】积分项太弱，来不及补。
 *    Ki=0.02，5° 误差每拍积累 5×0.03=0.15，100 拍(3s) 积到 15，
 *    I 项贡献仅 15×0.02=0.3 RPM——几乎可以忽略。
 *
 * 3.【底层】步进电机存在静摩擦/死区，低于某个 RPM 确实不能稳定转动。
 *    这个 RPM 阈值需要实测确认。当前用的 10 RPM 只是保守估计。
 *
 * 推荐的解决路径（按优先级）：
 *
 * A) 先测电机最低稳定 RPM——单独写一个低速测试，逐步降低 RPM，
 *    找到能稳定转动的最小值（假设实测可到 5 RPM，就把 T3_MIN_TURN_RPM 改成 5）。
 *
 * B) 提高积分增益——Ki 从 0.02 提到 0.1~0.3，让积分在小误差时更快积累，
 *    产生足够输出推动电机跨越摩擦。同时把积分限幅从 40 提到 60，
 *    确保积分能贡献足够多 RPM。这是最简单的参数调整。
 *
 * C) 采用"变速积分"策略——误差大时弱积分（防超调），误差小时强积分（消静差）。
 *    在 algo/pid.c 的 Pid_Update 里加一个误差→ki 的映射即可。
 *
 * D) 分层最小 RPM——误差 > 3° 时 minRPM=10，误差 1~3° 时 minRPM=5（如果电机能走）。
 *    或者：在 PID 输出之外加一个"dither"脉冲——当 |error| 在 1~5° 且持续一段时间，
 *    短暂给一个最小 RPM 脉冲来"挤"过摩擦。
 *
 * E) 使用串级控制（参考算法文档 §1.5）——角度外环输出目标角速度，
 *    角速度内环直接用陀螺仪角速度做反馈，从根本上消除静差。
 *
 * 推荐先试 A+B：测最小 RPM + 提 Ki。改动最小，通常能解决大部分问题。
 * ================================================================== */
