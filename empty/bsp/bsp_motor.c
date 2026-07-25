#include "bsp_motor.h"

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * ================== 四路独立步进电机驱动实现（v1.9）==================
 *
 * 每路电机 = 一个独立定时器 + 一套梯形加减速状态机，互不影响：
 *   - 该路定时器的 ZERO 中断每个 STEP 脉冲触发一次，在 ISR 里逐脉冲把当前周期
 *     朝目标周期逼近一个 RAMP_DELTA，实现平滑加减速与在线调速；
 *   - 连续模式（positionMove=0）：手动启停，stopRequest 触发减速到起步速度后停表；
 *   - 定距模式（positionMove=1）：按剩余步数自动加速→巡航→减速，走完自动停表。
 *
 * 定时器时钟 = MFCLK 4MHz、÷1；步频 = 4MHz / period。period 越小步频越高、转越快。
 * ISR 中不调用任何 FreeRTOS API（符合 CLAUDE.md FreeRTOS 规则）。
 * ==================================================================== */

/* 定时器时钟频率（Hz）：MFCLK 4MHz、prescale=0、÷1。 */
#define MOTOR_TIMER_CLK_HZ        (4000000UL)

/*
 * 梯形加减速参数：
 *   START  起步/停止周期，8000 → 500Hz。直接高速起转会失步，从此慢速起步。
 *   MIN    最快巡航周期（步频上限），125 → 32kHz。硬件安全上限，勿再调小。
 *   RAMP_DELTA 每个脉冲的周期增/减量，决定加减速斜率（越小越平缓）。
 */
#define MOTOR_PERIOD_START        (8000U)
#define MOTOR_PERIOD_MIN          (125U)
#define MOTOR_RAMP_DELTA          (4U)

/* 内部逻辑方向：正向 = DIR 低电平（实际正反以装配后实测为准，可用 invert 统一）。 */
#define MOTOR_DIR_FWD             (0U)
#define MOTOR_DIR_REV             (1U)

/* 单路电机的完整运行状态（每个字段的并发访问都在 ISR 与任务之间，故用 volatile）。 */
typedef struct {
    GPTIMER_Regs *timer;      /* STEP 定时器实例 */
    GPIO_Regs    *dirPort;    /* DIR 引脚端口 */
    uint32_t      dirPin;     /* DIR 引脚掩码 */
    IRQn_Type     irqn;       /* 该定时器的中断号 */
    uint8_t       invert;     /* 方向取反标定：1=把 DIR 电平反过来 */

    volatile uint32_t curPeriod;     /* 当前定时器周期（加减速中动态变化） */
    volatile uint32_t targetPeriod;  /* 目标巡航周期 */
    volatile uint8_t  running;       /* 1=转动中或正在减速停止 */
    volatile uint8_t  curDir;        /* 当前逻辑方向 MOTOR_DIR_FWD/REV */
    volatile uint8_t  stopRequest;   /* 连续模式：1=请求减速停 */
    volatile uint8_t  positionMove;  /* 1=定距模式，0=连续模式 */

    volatile uint32_t stepsTotal;    /* 定距模式：本次总脉冲数 */
    volatile uint32_t stepsDone;     /* 定距模式：已发脉冲数 */
    volatile uint32_t accelSteps;    /* 定距模式：加/减速段脉冲数阈值 */

    /* 反向换向挂起：运行中被命令反向时，先减速停稳，ISR 停稳瞬间按此重启。 */
    volatile uint8_t  pendingValid;
    volatile uint8_t  pendingDir;
    volatile uint32_t pendingPeriod;
} MotorCtl_t;

static MotorCtl_t s_mot[BSP_MOTOR_COUNT];

/* 当前四路共用细分档位，用于 RPM↔脉冲频率换算。 */
static BspTmcMicrostep_t s_microstep = TMC_MICROSTEP_32;

/* ------------------------------------------------------------------
 * 细分 / RPM 换算
 * ------------------------------------------------------------------ */

/* 细分枚举 → 倍数（8/16/32/64）。 */
static uint32_t Motor_MicrostepFactor(void)
{
    switch (s_microstep) {
    case TMC_MICROSTEP_16: return 16U;
    case TMC_MICROSTEP_32: return 32U;
    case TMC_MICROSTEP_64: return 64U;
    case TMC_MICROSTEP_8:
    default:               return 8U;
    }
}

uint32_t BspMotor_StepsPerRev(void)
{
    return BSP_MOTOR_FULL_STEPS_PER_REV * Motor_MicrostepFactor();
}

/* 巡航周期限幅：不快于安全上限、不慢于起步速度。 */
static uint32_t Motor_ClampPeriod(uint32_t period)
{
    if (period < MOTOR_PERIOD_MIN) {
        period = MOTOR_PERIOD_MIN;
    }
    if (period > MOTOR_PERIOD_START) {
        period = MOTOR_PERIOD_START;
    }
    return period;
}

/*
 * 转速大小(RPM) → 定时器周期。
 *   步频(Hz) = rpm/60 × 每圈脉冲数；period = 定时器时钟 / 步频。
 * rpm 太小(步频→0)时贴到最慢起步周期；结果统一限幅到安全范围。
 * 用 64 位中间量避免 rpm×每圈脉冲数 溢出。
 */
static uint32_t Motor_RpmToPeriod(uint32_t rpm)
{
    uint64_t stepFreq = (uint64_t)rpm * (uint64_t)BspMotor_StepsPerRev() / 60ULL;

    if (stepFreq == 0ULL) {
        return MOTOR_PERIOD_START;   /* 太慢：贴到最慢档 */
    }
    return Motor_ClampPeriod((uint32_t)(MOTOR_TIMER_CLK_HZ / stepFreq));
}

/* ------------------------------------------------------------------
 * 底层寄存器操作
 * ------------------------------------------------------------------ */

/* 写 DIR 引脚（考虑 invert 取反），并记录逻辑方向。 */
static void Motor_ApplyDir(MotorCtl_t *m, uint8_t dir)
{
    uint8_t level = dir;                 /* FWD=0(低) / REV=1(高) */
    if (m->invert != 0U) {
        level ^= 1U;
    }
    if (level != 0U) {
        DL_GPIO_setPins(m->dirPort, m->dirPin);
    } else {
        DL_GPIO_clearPins(m->dirPort, m->dirPin);
    }
    m->curDir = dir;
}

/* 把周期写入该路定时器 LOAD 与 50% 占空 CC，下个脉冲生效。 */
static void Motor_ApplyPeriod(MotorCtl_t *m, uint32_t period)
{
    DL_TimerG_setLoadValue(m->timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(m->timer, period / 2U, DL_TIMER_CC_0_INDEX);
}

/* 停掉该路定时器硬件并清 running（ISR 停表与急停共用的最小动作）。 */
static void Motor_StopTimerHw(MotorCtl_t *m)
{
    DL_TimerG_stopCounter(m->timer);
    NVIC_DisableIRQ(m->irqn);
    m->running = 0U;
}

/* 立即停表并收敛该路软件状态（急停/复位用）。 */
static void Motor_HardStop(MotorCtl_t *m)
{
    Motor_StopTimerHw(m);
    m->stopRequest   = 1U;
    m->positionMove  = 0U;
    m->pendingValid  = 0U;
    m->curPeriod     = MOTOR_PERIOD_START;
}

/* 使能该路定时器 ZERO 中断并从慢速起步启动计数（连续/定距共用的启动尾段）。 */
static void Motor_KickStart(MotorCtl_t *m)
{
    m->curPeriod = MOTOR_PERIOD_START;
    Motor_ApplyPeriod(m, MOTOR_PERIOD_START);

    DL_TimerG_clearInterruptStatus(m->timer, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(m->timer, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(m->irqn);

    m->running = 1U;
    DL_TimerG_startCounter(m->timer);
}

/* ------------------------------------------------------------------
 * 初始化 / 全局控制
 * ------------------------------------------------------------------ */

/* 填充一路电机的静态硬件信息。 */
static void Motor_ConfigOne(BspMotorId_t id, GPTIMER_Regs *timer,
                            GPIO_Regs *dirPort, uint32_t dirPin, IRQn_Type irqn)
{
    MotorCtl_t *m = &s_mot[id];
    m->timer        = timer;
    m->dirPort      = dirPort;
    m->dirPin       = dirPin;
    m->irqn         = irqn;
    m->invert       = 0U;
    m->curPeriod    = MOTOR_PERIOD_START;
    m->targetPeriod = MOTOR_PERIOD_START;
    m->running      = 0U;
    m->curDir       = MOTOR_DIR_FWD;
    m->stopRequest  = 1U;
    m->positionMove = 0U;
    m->stepsTotal   = 0U;
    m->stepsDone    = 0U;
    m->accelSteps   = 0U;
    m->pendingValid = 0U;
}

void BspMotor_Init(void)
{
    /* 绑定每路的定时器 / DIR 引脚 / 中断号（引脚宏见 ti_msp_dl_config.h）。 */
    Motor_ConfigOne(BSP_MOTOR_1, MOTOR1_STEP_TIMER_INST, MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, MOTOR1_STEP_TIMER_IRQn);
    Motor_ConfigOne(BSP_MOTOR_2, MOTOR2_STEP_TIMER_INST, MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, MOTOR2_STEP_TIMER_IRQn);
    Motor_ConfigOne(BSP_MOTOR_3, MOTOR3_STEP_TIMER_INST, MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, MOTOR3_STEP_TIMER_IRQn);
    Motor_ConfigOne(BSP_MOTOR_4, MOTOR4_STEP_TIMER_INST, MOTOR4_DIR_PORT, MOTOR4_DIR_PIN, MOTOR4_STEP_TIMER_IRQn);

    /* 上电安全态：先禁用驱动，再统一方向和默认细分，最后停掉四路 STEP。 */
    BspMotor_DisableAll();
    BspMotor_SetMicrostep(TMC_MICROSTEP_32);
    for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; i++) {
        Motor_ApplyDir(&s_mot[i], MOTOR_DIR_FWD);
        Motor_HardStop(&s_mot[i]);
    }
}

/* 设置 MS1/MS2 电平（四路共用）。 */
static void Motor_ApplyMs(uint8_t ms1, uint8_t ms2)
{
    if (ms1) { DL_GPIO_setPins(TMC_MS1_PORT, TMC_MS1_PIN); }
    else     { DL_GPIO_clearPins(TMC_MS1_PORT, TMC_MS1_PIN); }
    if (ms2) { DL_GPIO_setPins(TMC_MS2_PORT, TMC_MS2_PIN); }
    else     { DL_GPIO_clearPins(TMC_MS2_PORT, TMC_MS2_PIN); }
}

void BspMotor_SetMicrostep(BspTmcMicrostep_t microstep)
{
    s_microstep = microstep;
    switch (microstep) {
    case TMC_MICROSTEP_16: Motor_ApplyMs(1U, 1U); break;
    case TMC_MICROSTEP_32: Motor_ApplyMs(1U, 0U); break;
    case TMC_MICROSTEP_64: Motor_ApplyMs(0U, 1U); break;
    case TMC_MICROSTEP_8:
    default:               Motor_ApplyMs(0U, 0U); break;
    }
}

void BspMotor_EnableAll(void)
{
    /* ENN 低有效，拉低使能四路驱动。 */
    DL_GPIO_clearPins(TMC_ENN_PORT, TMC_ENN_PIN);
}

void BspMotor_DisableAll(void)
{
    /* ENN 拉高禁用四路驱动，电机失力。 */
    DL_GPIO_setPins(TMC_ENN_PORT, TMC_ENN_PIN);
}

void BspMotor_SetDirInvert(BspMotorId_t id, bool invert)
{
    if (id < BSP_MOTOR_COUNT) {
        s_mot[id].invert = invert ? 1U : 0U;
    }
}

/* ------------------------------------------------------------------
 * 核心：速度 / 脉冲
 * ------------------------------------------------------------------ */

void BspMotor_SetSpeedRpm(BspMotorId_t id, int32_t rpm)
{
    MotorCtl_t *m;
    uint8_t     newDir;
    uint32_t    mag;
    uint32_t    period;

    if (id >= BSP_MOTOR_COUNT) {
        return;
    }
    m = &s_mot[id];

    /* rpm=0：请求平滑停止。 */
    if (rpm == 0) {
        if (m->running != 0U) {
            m->stopRequest  = 1U;
            m->positionMove = 0U;
            m->pendingValid = 0U;
        }
        return;
    }

    newDir = (rpm > 0) ? MOTOR_DIR_FWD : MOTOR_DIR_REV;
    mag    = (rpm > 0) ? (uint32_t)rpm : (uint32_t)(-rpm);
    period = Motor_RpmToPeriod(mag);

    if (m->running == 0U) {
        /* 已停：设向后从慢速起步。 */
        Motor_ApplyDir(m, newDir);
        m->targetPeriod = period;
        m->stopRequest  = 0U;
        m->positionMove = 0U;
        m->pendingValid = 0U;
        Motor_KickStart(m);
    } else if ((newDir == m->curDir) && (m->positionMove == 0U)) {
        /* 同向连续运行：在线调速。 */
        m->targetPeriod = period;
        m->stopRequest  = 0U;
        m->pendingValid = 0U;
    } else {
        /* 反向或原为定距：先平滑减速停，ISR 停稳时按 pending 翻向重启。 */
        m->pendingDir    = newDir;
        m->pendingPeriod = period;
        m->pendingValid  = 1U;
        m->stopRequest   = 1U;
        m->positionMove  = 0U;
    }
}

void BspMotor_MoveSteps(BspMotorId_t id, int32_t steps, uint32_t rpm)
{
    MotorCtl_t *m;
    uint8_t     dir;
    uint32_t    n;
    uint32_t    period;
    uint32_t    accel;

    if (id >= BSP_MOTOR_COUNT) {
        return;
    }
    m = &s_mot[id];

    if (steps == 0) {
        BspMotor_Stop(id);
        return;
    }

    /* 定距要求精确计步，若正在运行先急停，从停止态干净起步。 */
    if (m->running != 0U) {
        Motor_HardStop(m);
    }

    dir    = (steps > 0) ? MOTOR_DIR_FWD : MOTOR_DIR_REV;
    n      = (steps > 0) ? (uint32_t)steps : (uint32_t)(-steps);
    period = Motor_RpmToPeriod(rpm);

    /* 从起步速度加速到巡航速度所需脉冲数；步数不够则退化为三角形速度曲线。 */
    accel = (MOTOR_PERIOD_START - period) / MOTOR_RAMP_DELTA;
    m->accelSteps = (n <= (2U * accel)) ? (n / 2U) : accel;

    m->stepsTotal    = n;
    m->stepsDone     = 0U;
    m->targetPeriod  = period;
    m->stopRequest   = 0U;
    m->positionMove  = 1U;
    m->pendingValid  = 0U;

    Motor_ApplyDir(m, dir);
    Motor_KickStart(m);
}

void BspMotor_SetSpeedRpm4(int32_t rpm1, int32_t rpm2, int32_t rpm3, int32_t rpm4)
{
    BspMotor_SetSpeedRpm(BSP_MOTOR_1, rpm1);
    BspMotor_SetSpeedRpm(BSP_MOTOR_2, rpm2);
    BspMotor_SetSpeedRpm(BSP_MOTOR_3, rpm3);
    BspMotor_SetSpeedRpm(BSP_MOTOR_4, rpm4);
}

void BspMotor_MoveSteps4(int32_t steps1, int32_t steps2, int32_t steps3, int32_t steps4,
                         uint32_t rpm)
{
    BspMotor_MoveSteps(BSP_MOTOR_1, steps1, rpm);
    BspMotor_MoveSteps(BSP_MOTOR_2, steps2, rpm);
    BspMotor_MoveSteps(BSP_MOTOR_3, steps3, rpm);
    BspMotor_MoveSteps(BSP_MOTOR_4, steps4, rpm);
}

/* ------------------------------------------------------------------
 * 停止 / 状态
 * ------------------------------------------------------------------ */

void BspMotor_Stop(BspMotorId_t id)
{
    BspMotor_SetSpeedRpm(id, 0);
}

void BspMotor_StopAll(void)
{
    for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; i++) {
        BspMotor_Stop((BspMotorId_t)i);
    }
}

void BspMotor_EmergencyStop(BspMotorId_t id)
{
    if (id < BSP_MOTOR_COUNT) {
        Motor_HardStop(&s_mot[id]);
    }
}

bool BspMotor_IsStopped(BspMotorId_t id)
{
    if (id >= BSP_MOTOR_COUNT) {
        return true;
    }
    return (s_mot[id].running == 0U);
}

bool BspMotor_AllStopped(void)
{
    for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; i++) {
        if (s_mot[i].running != 0U) {
            return false;
        }
    }
    return true;
}

uint32_t BspMotor_GetRemainingSteps(BspMotorId_t id)
{
    MotorCtl_t *m;

    if (id >= BSP_MOTOR_COUNT) {
        return 0U;
    }
    m = &s_mot[id];
    if ((m->positionMove == 0U) || (m->stepsDone >= m->stepsTotal)) {
        return 0U;
    }
    return m->stepsTotal - m->stepsDone;
}

/* ------------------------------------------------------------------
 * 共用中断处理：单路电机的梯形加减速 + 计步 + 停/换向
 * ------------------------------------------------------------------ */

static void Motor_Isr(MotorCtl_t *m)
{
    uint32_t target;
    uint32_t remaining = 0U;   /* 仅定距模式有意义；步骤1算出后复用到步骤3 */
    uint32_t cur;

    DL_TimerG_clearInterruptStatus(m->timer, DL_TIMERG_INTERRUPT_ZERO_EVENT);

    if (m->running == 0U) {
        return;
    }

    /* 1) 确定本脉冲的目标周期。 */
    if (m->positionMove != 0U) {
        if (m->stepsDone < m->stepsTotal) {
            m->stepsDone++;
        }
        remaining = m->stepsTotal - m->stepsDone;
        /* remaining<=accelSteps 为收尾/减速段，否则加速/巡航段。 */
        target = (remaining <= m->accelSteps) ? MOTOR_PERIOD_START : m->targetPeriod;
    } else {
        target = (m->stopRequest != 0U) ? MOTOR_PERIOD_START : m->targetPeriod;
    }

    /* 2) 逐脉冲朝目标逼近一个 RAMP_DELTA（梯形斜坡）；curPeriod 读一次、变了才回写并刷周期。 */
    cur = m->curPeriod;
    if (cur < target) {
        cur += MOTOR_RAMP_DELTA;
        if (cur > target) { cur = target; }
    } else if (cur > target) {
        cur -= MOTOR_RAMP_DELTA;
        if (cur < target) { cur = target; }
    }
    if (cur != m->curPeriod) {
        m->curPeriod = cur;
        Motor_ApplyPeriod(m, cur);
    }

    /* 3) 停止 / 换向判定：只有减速回到起步速度才处理。 */
    if (cur < MOTOR_PERIOD_START) {
        return;
    }
    if (m->positionMove != 0U) {
        /* 定距：所有步走完 → 停表。 */
        if (remaining == 0U) {
            Motor_StopTimerHw(m);
            m->positionMove = 0U;
        }
    } else if (m->stopRequest != 0U) {
        if (m->pendingValid != 0U) {
            /* 反向换向：此刻电机处于最慢速，翻 DIR 后重新加速。 */
            Motor_ApplyDir(m, m->pendingDir);
            m->targetPeriod = m->pendingPeriod;
            m->stopRequest  = 0U;
            m->pendingValid = 0U;
        } else {
            /* 连续：已请求停止且减速到起步速度 → 停表。 */
            Motor_StopTimerHw(m);
        }
    }
}

/* 四路各自的中断入口（向量名见启动文件；一一映射到对应电机）。 */
void TIMG0_IRQHandler(void)  { Motor_Isr(&s_mot[BSP_MOTOR_1]); }
void TIMG8_IRQHandler(void)  { Motor_Isr(&s_mot[BSP_MOTOR_2]); }
void TIMG12_IRQHandler(void) { Motor_Isr(&s_mot[BSP_MOTOR_3]); }
void TIMG6_IRQHandler(void)  { Motor_Isr(&s_mot[BSP_MOTOR_4]); }
