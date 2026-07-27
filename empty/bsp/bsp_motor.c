#include "bsp_motor.h"

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * ================== 四路独立步进电机驱动实现（v1.9）==================
 *
 * 每路电机 = 一个独立定时器 + 一套计步状态，互不影响：
 *   - STEP 方波由该路硬件定时器直接以目标周期输出，命令不做加减速；
 *   - 连续模式（positionMove=0）：持续输出，收到停止命令后立即停表；
 *   - 定距模式（positionMove=1）：按目标周期计步，走完最后一步立即停表。
 *
 * 定时器时钟 = MFCLK 4MHz、÷1；步频 = 4MHz / period。period 越小步频越高、转越快。
 * ISR 中不调用任何 FreeRTOS API（符合 CLAUDE.md FreeRTOS 规则）。
 * ==================================================================== */

/* 定时器时钟频率（Hz）：MFCLK 4MHz、prescale=0、÷1。 */
#define MOTOR_TIMER_CLK_HZ        (4000000UL)

/*
 * 周期限幅参数：
 *   MAX：最低允许步频对应的周期，8000 → 500Hz。
 *   MIN：最高允许步频对应的周期，125 → 32kHz。
 */
#define MOTOR_PERIOD_MAX          (8000U)
#define MOTOR_PERIOD_MIN          (125U)

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

    volatile uint32_t curPeriod;     /* 当前定时器周期 */
    volatile uint8_t  running;       /* 1=定时器正在输出 STEP */
    volatile uint8_t  curDir;        /* 当前逻辑方向 MOTOR_DIR_FWD/REV */
    volatile uint8_t  positionMove;  /* 1=定距模式，0=连续模式 */

    volatile uint32_t stepsTotal;    /* 定距模式：本次总脉冲数 */
    volatile uint32_t stepsDone;     /* 定距模式：已发脉冲数 */
} MotorCtl_t;

static MotorCtl_t s_mot[BSP_MOTOR_COUNT];

/* 当前四路共用细分档位，用于 RPM↔脉冲频率换算。 */
static BspTmcMicrostep_t s_microstep = TMC_MICROSTEP_32;

/*
 * 实车方向标定：M1(左前)、M2(左后)与 M3/M4 的机械安装方向相反。
 * 因此逻辑正向须翻转 M1/M2 的 DIR 电平，保证四路正 RPM/正 steps
 * 都表示小车前进方向。若更换电机线序或机械安装，重新实测后只改这里。
 */
#define MOTOR1_DIR_INVERT_DEFAULT    (true)
#define MOTOR2_DIR_INVERT_DEFAULT    (true)
#define MOTOR3_DIR_INVERT_DEFAULT    (false)
#define MOTOR4_DIR_INVERT_DEFAULT    (false)

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

/* 目标周期限幅：不快于安全上限、不慢于最低允许步频。 */
static uint32_t Motor_ClampPeriod(uint32_t period)
{
    if (period < MOTOR_PERIOD_MIN) {
        period = MOTOR_PERIOD_MIN;
    }
    if (period > MOTOR_PERIOD_MAX) {
        period = MOTOR_PERIOD_MAX;
    }
    return period;
}

/*
 * 转速大小(RPM) → 定时器周期。
 *   步频(Hz) = rpm/60 × 每圈脉冲数；period = 定时器时钟 / 步频。
 * rpm 太小(步频→0)时贴到最低允许步频对应的周期；结果统一限幅到安全范围。
 * 用 64 位中间量避免 rpm×每圈脉冲数 溢出。
 */
static uint32_t Motor_RpmToPeriod(uint32_t rpm)
{
    uint64_t stepFreq = (uint64_t)rpm * (uint64_t)BspMotor_StepsPerRev() / 60ULL;

    if (stepFreq == 0ULL) {
        return MOTOR_PERIOD_MAX;   /* 太慢：贴到最慢档 */
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
    m->positionMove  = 0U;
    m->curPeriod     = MOTOR_PERIOD_MAX;
}

/*
 * 以目标周期启动。所有电机命令均经过此函数，统一直接下发，不做加减速。
 * 直接高速起转可能失步，调用方应先从较低 RPM 实机验证。
 */
static void Motor_Start(MotorCtl_t *m, uint32_t period)
{
    m->curPeriod = period;
    Motor_ApplyPeriod(m, period);

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
    m->curPeriod    = MOTOR_PERIOD_MAX;
    m->running      = 0U;
    m->curDir       = MOTOR_DIR_FWD;
    m->positionMove = 0U;
    m->stepsTotal   = 0U;
    m->stepsDone    = 0U;
}

void BspMotor_Init(void)
{
    /* 绑定每路的定时器 / DIR 引脚 / 中断号（引脚宏见 ti_msp_dl_config.h）。 */
    Motor_ConfigOne(BSP_MOTOR_1, MOTOR1_STEP_TIMER_INST, MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, MOTOR1_STEP_TIMER_IRQn);
    Motor_ConfigOne(BSP_MOTOR_2, MOTOR2_STEP_TIMER_INST, MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, MOTOR2_STEP_TIMER_IRQn);
    Motor_ConfigOne(BSP_MOTOR_3, MOTOR3_STEP_TIMER_INST, MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, MOTOR3_STEP_TIMER_IRQn);
    Motor_ConfigOne(BSP_MOTOR_4, MOTOR4_STEP_TIMER_INST, MOTOR4_DIR_PORT, MOTOR4_DIR_PIN, MOTOR4_STEP_TIMER_IRQn);

    /* 在写初始 DIR 前装载实车标定，避免上电安全态与逻辑正向不一致。 */
    BspMotor_SetDirInvert(BSP_MOTOR_1, MOTOR1_DIR_INVERT_DEFAULT);
    BspMotor_SetDirInvert(BSP_MOTOR_2, MOTOR2_DIR_INVERT_DEFAULT);
    BspMotor_SetDirInvert(BSP_MOTOR_3, MOTOR3_DIR_INVERT_DEFAULT);
    BspMotor_SetDirInvert(BSP_MOTOR_4, MOTOR4_DIR_INVERT_DEFAULT);

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
    /* 常规接口也采用直接下发，保留原函数名以兼容已有任务代码。 */
    BspMotor_SetSpeedRpmImmediate(id, rpm);
}

void BspMotor_SetSpeedRpmImmediate(BspMotorId_t id, int32_t rpm)
{
    MotorCtl_t *m;
    uint8_t     newDir;
    uint32_t    mag;
    uint32_t    period;

    if (id >= BSP_MOTOR_COUNT) {
        return;
    }
    m = &s_mot[id];

    /* rpm=0 就是直接停表，不保留减速过程。 */
    if (rpm == 0) {
        Motor_HardStop(m);
        return;
    }

    newDir = (rpm > 0) ? MOTOR_DIR_FWD : MOTOR_DIR_REV;
    mag    = (rpm > 0) ? (uint32_t)rpm : (uint32_t)(-rpm);
    period = Motor_RpmToPeriod(mag);

    /* 无论原来是否在转，都先直接停表，再用新方向和新周期重启。 */
    if (m->running != 0U) {
        Motor_HardStop(m);
    }
    Motor_ApplyDir(m, newDir);
    m->positionMove = 0U;
    Motor_Start(m, period);
}

void BspMotor_MoveSteps(BspMotorId_t id, int32_t steps, uint32_t rpm)
{
    MotorCtl_t *m;
    uint8_t     dir;
    uint32_t    n;
    uint32_t    period;

    if (id >= BSP_MOTOR_COUNT) {
        return;
    }
    m = &s_mot[id];

    if (steps == 0) {
        BspMotor_Stop(id);
        return;
    }

    /* 定距要求精确计步，若正在运行先停表，再从新方向和目标周期开始。 */
    if (m->running != 0U) {
        Motor_HardStop(m);
    }

    dir    = (steps > 0) ? MOTOR_DIR_FWD : MOTOR_DIR_REV;
    n      = (steps > 0) ? (uint32_t)steps : (uint32_t)(-steps);
    period = Motor_RpmToPeriod(rpm);

    m->stepsTotal    = n;
    m->stepsDone     = 0U;
    m->positionMove  = 1U;

    Motor_ApplyDir(m, dir);
    Motor_Start(m, period);
}

void BspMotor_SetSpeedRpm4(int32_t rpm1, int32_t rpm2, int32_t rpm3, int32_t rpm4)
{
    BspMotor_SetSpeedRpm(BSP_MOTOR_1, rpm1);
    BspMotor_SetSpeedRpm(BSP_MOTOR_2, rpm2);
    BspMotor_SetSpeedRpm(BSP_MOTOR_3, rpm3);
    BspMotor_SetSpeedRpm(BSP_MOTOR_4, rpm4);
}

void BspMotor_SetSpeedRpm4Immediate(int32_t rpm1, int32_t rpm2,
                                    int32_t rpm3, int32_t rpm4)
{
    BspMotor_SetSpeedRpmImmediate(BSP_MOTOR_1, rpm1);
    BspMotor_SetSpeedRpmImmediate(BSP_MOTOR_2, rpm2);
    BspMotor_SetSpeedRpmImmediate(BSP_MOTOR_3, rpm3);
    BspMotor_SetSpeedRpmImmediate(BSP_MOTOR_4, rpm4);
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
    BspMotor_SetSpeedRpmImmediate(id, 0);
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
 * 共用中断处理：单路电机的定距计步。
 * ------------------------------------------------------------------ */

static void Motor_Isr(MotorCtl_t *m)
{
    DL_TimerG_clearInterruptStatus(m->timer, DL_TIMERG_INTERRUPT_ZERO_EVENT);

    if (m->running == 0U) {
        return;
    }

    /* 连续模式无需在中断中调整速度；定距模式只计步并在完成时立即停表。 */
    if (m->positionMove != 0U) {
        if (m->stepsDone < m->stepsTotal) {
            m->stepsDone++;
        }
        if (m->stepsDone >= m->stepsTotal) {
            Motor_StopTimerHw(m);
            m->positionMove = 0U;
        }
    }
}

/* 四路各自的中断入口（向量名见启动文件；一一映射到对应电机）。 */
void TIMG0_IRQHandler(void)  { Motor_Isr(&s_mot[BSP_MOTOR_1]); }
void TIMG8_IRQHandler(void)  { Motor_Isr(&s_mot[BSP_MOTOR_2]); }
void TIMG12_IRQHandler(void) { Motor_Isr(&s_mot[BSP_MOTOR_3]); }
void TIMG6_IRQHandler(void)  { Motor_Isr(&s_mot[BSP_MOTOR_4]); }
