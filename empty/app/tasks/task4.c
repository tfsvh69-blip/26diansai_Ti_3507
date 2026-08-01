#include "app_tasks.h"
#include "app_robot_core.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "app_ball_control_task.h"
#include "bsp_buzzer.h"
#include "bsp_line.h"
#include "emm42_robot.h"

/* ==================================================================
 * 第 4 题：循迹 PID 缓停（当前 T4_STOP_AFTER_MS=7.5 秒）
 *
 * 本题循迹、入场、丢线保护、终点保护和定时减速逻辑均照搬第 2 题，参数也
 * 保持同值。唯一业务区别是：进入正常循迹后累计前进 T4_STOP_AFTER_MS，分别给
 * 左右轮发送速度模式 0 RPM 帧；该帧带 T4_STOP_EMM_ACC 加速度档位，驱动器按
 * 曲线缓停。车辆行驶期间通过 BALLCTRL 同时控制 ID1，使钢珠持续保持在
 * T4_BALL_TARGET_X_PX。具体秒数以 T4_STOP_AFTER_MS 当前值为准，不要以本注释
 * 里的数字为准（历史上改过好几次）。
 * ================================================================== */

/* PID 增益、积分限幅与转向输出限幅。 */
#define T4_KP                              (2.4F)
#define T4_KI                              (0.2F)
#define T4_KD                              (0.7F)
#define T4_INTEGRAL_LIMIT                  (20.0F)
#define T4_MAX_STEER_RPM                   (100.0F)

/*
 * 基础速度和单轮安全范围。
 * 2026-08 "整体速度提高但加速度别太快"：80→150。这个值必须跟下面
 * T4_START_RAMP_RPM_PER_SEC 一起看，爬满全速时间=本值÷斜坡值，见那边说明。
 */
#define T4_BASE_RPM                        (95.0F)
#define T4_MIN_WHEEL_RPM                   (5.0F)
#define T4_MAX_WHEEL_RPM                   (230.0F)

/*
 * 正常循迹速度模式加速度档位（驱动器内部曲线）。
 *
 * ⚠️ 方向容易搞反：协议公式（手册§6.3.1）每升 1RPM 需要 (256−acc)×50us，
 * 【acc 数值越大爬升越快】，所以"降低加速度"= 把这个数【调小】。
 * acc=0 是特例（不走曲线、瞬间到目标速度，最抖），不是最慢。
 *
 * ⚠️ 2026-08 实测确认的重要限制：**这个参数在低端几乎没有区分度**——
 *   acc=1  → 每 1RPM 耗时 (256−1)×50us  = 12.75ms，爬到 110RPM 约 1.40s（硬件最慢）
 *   acc=5  → 12.55ms/RPM，爬到 110RPM 约 1.38s
 *   acc=30 → 11.3ms/RPM，爬到 110RPM 约 1.24s
 * acc 从 1 调到 30，分子只从 255 变到 226，差别不到 12%，所以"从 0 往上加
 * 感觉没怎么变"是必然的。而且 acc=1 已经是驱动器能给的最慢曲线，**想要
 * 比 1.4 秒更缓的起步，驱动器层面无解**，必须靠下面的软件斜坡。
 * 本参数保持一个较小值即可，真正调起步快慢请改 T4_START_RAMP_RPM_PER_SEC。
 */
#define T4_EMM_ACC                         (5U)

/*
 * 【起步软件速度斜坡】——真正能调到"很低很低"的起步加速度旋钮。
 *
 * 驱动器 acc 曲线最慢也只能 1.4 秒爬到 110RPM（见上），要更缓就不能一上来
 * 就把目标速度甩给驱动器，而是任务层每拍（30ms）只把目标速度往上抬一点点，
 * 让驱动器始终在追一个缓慢上升的目标。斜坡多慢都行，不受协议限制。
 *
 * 单位 RPM/秒，含义直白 = 每秒钟基础速度允许增加多少；爬满 T4_BASE_RPM 所需
 * 时间 = T4_BASE_RPM ÷ 本值。想更缓就把这个数继续往下调，没有下限限制。
 *
 * ⚠️ 必须和 T4_STOP_AFTER_MS 一起看，两者是同一个权衡的两面：
 *   爬满时间 = T4_BASE_RPM ÷ 本值，若这个值 ≥ T4_STOP_AFTER_MS（换算成秒），
 *   说明小车全程都在加速、从没跑到过全速，观感就是"没怎么动就停了"。
 *   想要有一段稳定全速巡航，要么本值调大（爬更快），要么 T4_STOP_AFTER_MS
 *   调大（留够爬升时间），当前两个宏定义处互相都有算好的换算提醒。
 * 只限制【上升】，不限制转弯/定时减速的下降；爬到 T4_BASE_RPM 后自动失效，
 * 之后的转弯减速恢复不受影响。
 *
 * 2026-08 配合 T4_BASE_RPM 80→150 一起调："整体速度提高、加速度不能太快、
 * 总时长仍是 7.5 秒、要求整车非常平稳"——这四点必须联立着算，不能只改一个：
 *   若斜坡仍用 10，爬满 150RPM 要 15 秒，比 7.5 秒还长一倍，全程都在加速，
 *   跟"整体速度提高"的诉求矛盾（根本到不了新速度，只是把旧问题挪到更高的
 *   目标值上重演）。改成 40：爬满时间 = 150÷40 = 3.75 秒，7.5 秒里前一半
 *   爬坡、后一半能稳定巡航在满速，且 40RPM/s 仍只是驱动器自身最快曲线
 *   （acc=5，约 1.4 秒到 110RPM，等效约 78RPM/s）的一半左右，比阶跃平缓得多。
 *   还要更平稳可以继续调小（比如 25~30），代价是巡航时间进一步缩短。
 */
#define T4_START_RAMP_RPM_PER_SEC          (30.0F)  /* 40RPM 约需 8 秒爬满，优先保证起步平稳。 */

/*
 * 缓停参数：T4_STOP_AFTER_MS 固定本题开始缓停的时间；
 * T4_STOP_EMM_ACC 由使用者按实车需要传给速度模式 0 RPM 帧。
 * 数值越小减速越平缓，越大越接近立即停；可直接修改后重新烧录。
 *
 * 2026-08 在当前状态基础上多跑 800ms：7500→8300。当前
 * T4_BASE_RPM(110)÷T4_START_RAMP_RPM_PER_SEC(25)=4.4 秒爬满，小于 8.3 秒，
 * 留了约 3.9 秒稳定全速巡航，延长这 800ms 期间小车仍是正常循迹（这段时间
 * 只是把 RUN 状态多留一会，PID/循迹逻辑本身不区分"早一点停"还是"晚一点停"）。
 * 改上面 T4_BASE_RPM 或 T4_START_RAMP_RPM_PER_SEC 任意一个都要回来重新核对
 * 这个关系，避免又变回爬不满的状态。
 */
#define T4_STOP_AFTER_MS                   (8300U)
#define T4_STOP_EMM_ACC                    (80U)

/* UIMENU 固定控制周期。 */
#define T4_DT_SEC                          (0.03F)
#define T4_TICK_MS                         (30U)

/* 丢线、入场和共享 UART1 总线时序参数。 */
#define T4_LINE_LOST_TICKS                 (20U)
#define T4_RESET_SETTLE_TICKS              (2U)
#define T4_ENABLE_SETTLE_TICKS             (6U)
#define T4_EMM_CMD_GAP_MS                  (6U)

/* 误差滤波、死区和转弯减速参数。 */
#define T4_ERROR_FILTER_ALPHA              (0.5F)
#define T4_ERROR_DEADBAND                  (2.0F)
#define T4_CORNER_SLOWDOWN_GAIN            (0.65F)
#define T4_MIN_BASE_RPM                    (10.0F)

/* 转向输出限速：将离散灰度带来的 PID 阶跃摊成渐变，减小过弯横摆。 */
#define T4_STEER_SLEW_RPM_PER_SEC          (250.0F)

/* 终点保护参数，与任务二一致。 */
#define T4_FINISH_ARM_HIT_MAX              (3U)
#define T4_FINISH_ARM_TICKS                (15U)
#define T4_FINISH_HIT_MIN                  (6U)
#define T4_FINISH_HIT_TICKS                (1U)

/* 秒表与定时减速参数，与任务二一致。 */
#define T4_STOPWATCH_CAL_SCALE             (0.897F)
#define T4_DECEL_START_MS                  (14500U)
#define T4_DECEL_GRADIENT_RPM_PER_SEC      (60.0F)
#define T4_DECEL_MIN_RPM                   (10.0F)

/*
 * 任务四钢珠平衡参数。参数只归任务四所有，后续调车时不会影响菜单或任务三的摆球效果。
 * 车辆必须等后台闭环已经按此 profile 进入实际控制后才起步。
 */
#define T4_BALL_TARGET_X_PX                 (350)      /* 2026-08 新曲柄摇杆机构实测中心点，替换旧值 320 */

/* ---- A. 机械/视觉实测常量：由标定得出，不是调参旋钮，换硬件才重测 ---- */
/*
 * 2026-08-01 现场标定结果（视觉：菜单页读 X；机械：题目六 T6_RAMP_TEST 斜坡），
 * 对应旧"丝杆升降+连杆"机构：
 *   视觉比例  520px / 213mm = 2.441 px/mm（球心可达范围 X∈[66, 586]）
 *   静止噪声  ±2px（±0.8mm）→ 决定死区下限，也说明不需要重滤波
 *   脱离阈值  B = 1810 脉冲（4.53mm 升程 / 1.04° 倾角）
 *   真实水平  L = -70 脉冲（≈0.04°，机械基本是正的）
 *
 * 2026-08 机构变更为"电机直驱摇臂（±90°内旋转）"后的重新标定哲学：
 *   旧丝杆+螺母本身摩擦极大，占满了整个可用命令范围，必须靠 B 硬顶过去
 *   （见 docs/BALL_CONTROL.md §5）；直驱摇臂去掉了丝杆螺母这一大摩擦源，
 *   剩下的只是轴承转动摩擦和钢珠滚动摩擦，预期小得多。因此 B/L 先不做
 *   独立标定脚本测量，直接从 0 开始跑纯 PD（B=0 时下面的库仑摩擦前馈项
 *   在公式里自动归零，退化为纯 PD，不用改任何控制逻辑）：
 *   - 若实测出现"球离目标一截距离就完全僵住不动"，现场把 B 从 0 增量
 *     试大，观察消失即可，不再靠独立斜坡测试固定一个阈值——静摩擦本身
 *     随接触点/磨损/装配变化，固定常数覆盖全程不如现场调参鲁棒；
 *   - L 同理从 0 开始，靠"球停在偏目标固定像素、误差稳定不变"这个现象
 *     现场微调，不需要专门标定步骤。
 * 视觉比例、静止噪声与传动机构无关，继续沿用旧值。
 */
/*
 * 2026-08 隔离测试：50 时目标点附近持续等幅抖动，疑似前馈踹一脚→速度超阈值→
 * 踹一脚的自激循环，退回 0 验证过是它。
 *
 * 2026-08 二次复现：5 这个小值同样会导致"球已到位、电机仍来回颤抖"——球静止
 * 在目标附近时，pd=Kx×err−Kv×v 的符号完全由视觉噪声决定，每帧随机翻正负，
 * ffDir 在 |v| 很小时几乎等于 sign(pd)，于是前馈跟着每帧在 +5/-5 脉冲之间跳，
 * 球位移很小（肉眼看着"到位"）但电机是被直接驱动的，抖动很明显。
 * 现在 Kx=1.0 且 LEVEL_TRIM 已反推收敛，纯 PD 大概率已经够用，不再需要前馈
 * 顶过静摩擦。退回 0；如果又出现"球离目标一截距离完全僵住不动"才重新按
 * 现象表小步试大，不要直接抄旧值。
 */
#define T4_BALL_FRICTION_FF_PULSE           (0.0F)
/*
 * 2026-08 振荡（α/β 问题）解决后，实测稳态误差稳定卡在 err≈37（球停在
 * meas≈313，够不到目标 350），说明水平点假设有偏差。按公式反推：
 *   LEVEL_TRIM_new = LEVEL_TRIM_current + SIGN×Kx×err = 0 + (-1)×1.0×37 = -37
 * 若改完这个误差没有消失反而变大/变号，说明符号搞反了，改回 +37 试。
 *
 * 2026-08 acc 提到 240 响应变好后，用同一公式再反推一次：改完 -37 残差没有
 * 消失，只是从 37 缩小到平均约 17（说明方向和量级都对，只是当时修正量不够，
 * 不是又出现了新问题）。继续按同一公式收敛：
 *   LEVEL_TRIM_new = -37 + (-1)×1.0×17 = -54
 * 这套反推是线性、可迭代的，一次不够就再来一次，2~3 次内会收敛到很小。
 */
#define T4_BALL_LEVEL_TRIM_PULSE            (-5)
/*
 * 2026-08 实测教训：α=1.0（预测完全不用，每帧直接采信测量）配合 β=0.80
 * 导致剧烈振荡——β 越大，"测量-预测残差"里的噪声被放大进速度估计的比例越高：
 * 单帧速度修正量 = β×残差/dt，取 β=0.8、残差仅 5px（噪声量级）、dt=20ms，
 * 算出单帧就能凭空跳 200px/s，这个假速度乘上 Kv 直接灌进命令，把球真的
 * 推起来形成自激振荡（UART0 调试日志实测 v 峰值达 ±800px/s）。
 * 改回菜单默认闭环已用实测噪声（±2px）验证过的组合，不是随便给的数字。
 */
#define T4_BALL_FILTER_ALPHA                (0.9F)
#define T4_BALL_FILTER_BETA                 (0.2F)
/*
 * 2026-08 新曲柄摇杆机构方向实测（任务六 900 脉冲方向测试）：
 *   正脉冲 = 抬升摇杆；摇杆抬得越高，钢珠越往 X 变小方向移动。
 * 控制律 output = LEVEL_TRIM + SIGN×(Kx×error − Kv×velocity)，error = targetPx − 球位置。
 * error>0（目标在球右边）需要球向 +X 移动 → 必须降低摇杆 → output 须为负；
 * Kx>0 时 error>0 令 Kx×error>0，要让 output 为负必须 SIGN=-1。这是本次实测
 * 结论，不是猜测；如果实车验证方向反了，只改这一个数，不要改 emm42_robot.c
 * 的全局标定表。
 */
#define T4_BALL_OUTPUT_SIGN                 (-1.0F)

/* ---- B. 真正的调参旋钮 ---- */
/*
 * 旧丝杆机构曾有物理依据的系数 0.171（= (5/7)g / 400脉冲每mm / 250mm摆杆 *
 * 2.441px每mm）依赖丝杆导程换算，直驱摇臂后完全失效，需要新摇臂/连杆的实际
 * 几何尺寸才能重新推导，本次不臆测新系数。
 *
 * 2026-08 新机构第一次真实闭环测试，按 docs/CONTROL_ALGORITHM.md §10.2 方法论
 * 从纯 P 开始：Kv（抑制/阻尼项）先关掉、Kx 从保守小值起步，不做任何旋钮之外的
 * 限幅。调参顺序照 §10.3 现象表：
 *   球对误差反应很弱/很慢 → 加大 Kx；
 *   目标附近来回轻微振荡、幅度不变或缓慢衰减 → 停止加 Kx，固定住，转去从 0
 *     开始加 Kv 压振荡；
 *   球剧烈振荡/越振越猛/摆杆动作剧烈有异响 → Kx 过大，退回更小值重新找临界点；
 *   Kv 加到很大仍压不住振荡 → 大概率是 Kx 选大了，回去减小 Kx 而不是无限加 Kv。
 */
/*
 * 2026-08 稳态误差（LEVEL_TRIM）解决后，实测现象是"贴近目标但在附近持续振荡、
 * 不衰减、也不发散"——对应上面表格第二条："目标附近来回振荡→固定住 Kx，
 * 加 Kv 压振荡"。Kx=1.0 先不动，Kv 从 0.5 加到 1.0（先翻倍试，观察振荡幅度是
 * 缩小还是没变化）：
 *   继续缩小 → 方向对了，还需要接着加；
 *   没什么变化甚至更猛 → 说明 Kv 已经在放大速度噪声而不是提供阻尼，按现象表
 *     第四条回去减小 Kx 重新找临界点，不要无限加 Kv。
 */
/*
 * 2026-08 题目五（同一新曲柄摇杆机构、同一 ID1）在实车上把 Kx/Kv 从上面的
 * 保守起点（1.0/0.55）继续调到 1.066/0.504 并验证效果良好，现按值复制回
 * 题目四作为新起点——两题参数仍是独立副本，符合仓库【控制参数隔离规则】，
 * 后续任一题继续调参不会影响另一题。
 */
#define T4_BALL_KX_PULSE_PER_PX             (1.066F)
#define T4_BALL_KV_PULSE_PER_PXPS           (0.504F)
/*
 * 到位死区，同时就是静态精度上限：4px ≈ 1.6mm，取实测噪声 ±2px 的两倍裕度。
 * 误差进此范围【且球基本停住】才回真实水平点、停止驱动。
 */
#define T4_BALL_SETTLE_DEADBAND_PX          (4.0F)
/*
 * 判定钢珠"在运动"的速度门限：高于它按 sign(v) 补动摩擦，低于它过渡到按
 * sign(pd) 补静摩擦；同时是上面"到位保持"的速度条件。
 * 15px/s ≈ 6mm/s，明显高于速度估计噪声量级(±10px/s 的一半)。
 */
#define T4_BALL_FF_VEL_BLEND_PXPS           (15.0F)

/* ---- C. 执行器与显示 ---- */
/*
 * 摆杆每帧（20ms）行程能力 = POS_RPM * 3200/60 * 0.02 = POS_RPM * 1.067 脉冲。
 * 400RPM → 427 脉冲/帧，能容纳 Kv=18 带来的 180 脉冲/帧噪声并留出控制余量。
 * 这是上面 Kv 上限的来源，两者必须一起看。
 * ⚠️ 本机构尚未实测过 400RPM 的上限，上车先听有无异响/失步。
 *
 * 2026-08 机构变更为直驱摇臂后：同样 RPM 对应的角速度远大于丝杆机构
 * （旧机构靠丝杆导程+连杆大幅折算，直驱没有这层折算），预期需要调小，
 * 具体数值按现象判断——命令发出但摆杆没走到位/响应打折扣→调大，
 * 过冲很猛或有异响/失步→调小。
 */
/*
 * 2026-08 排查"反应特别慢/迟钝"：先把这个从 10 调到 200，实测【没有效果】——
 * 回头算才发现调错了旋钮：acc=200 时 20ms 内理论最多只能爬到 20÷2.8≈7.1RPM，
 * 本来就比旧的 10 更小，说明 acc 的爬升曲线早就先于 RPM 上限把每帧行程截断了，
 * RPM 从 10 提到 200 根本碰不到瓶颈。真正的限制因素是下面的 T4_BALL_POS_ACC，
 * 见那边的最新实测记录。这个值保持 200（明显高于 acc 能喂到的转速）即可，
 * 不用再往上调，也不用改回小值。
 */
#define T4_BALL_POS_RPM                     (200U)
/*
 * 2026-08 新机构实测：acc=0（瞬时到设定速度）在直驱摇臂上抖动很明显——BALLCTRL
 * 外环每约 17~25ms（视觉帧间隔）就下发一条全新绝对目标，acc=0 时每次都是速度
 * 阶跃，旧丝杆机构靠大幅减速比把这个阶跃吸收掉了，直驱摇臂没有这层缓冲，阶跃
 * 直接体现成机械抖动。现在改成可调旋钮，由使用者按下面方向自己试。
 *
 * 协议公式（手册§6.3.1，emm42_v5.h）：每升 1RPM 需要 (256−acc)×50us，
 * 【acc 越大爬升越快】，acc=0 是特例——不走曲线、直接瞬间给到目标速度（最快，
 * 也最抖），不是"最慢"，这一点容易搞反。例如 acc=180 时每 1RPM 耗时
 * (256−180)×50us=3.8ms，爬满 200RPM 约 760ms。
 *
 * ⚠️ 关键约束：外环约 17~25ms 就刷新一条全新目标，曲线基本不可能走完就被
 * 打断重新规划，实际能达到的转速 ≈ 帧间隔 / 每RPM耗时，远低于 POS_RPM。这是
 * 有意的效果（正是要靠"来不及冲到全速"来消除阶跃抖动），不是 bug；只是意味着
 * acc 越小（越接近但不等于 0），每帧实际能走的脉冲数越少、响应越绵软，acc 越
 * 大（越接近 255）越接近原来 acc=0 的阶跃手感。
 * 调参方向：还抖 → 减小 acc；感觉跟不上球、响应发软 → 增大 acc（增大后如果又
 * 开始抖，说明已经接近临界，退回上一档）。180 是题目二轮子已验证能跑的档位，
 * 仅作起点参考，不代表适合本机构，需要现场重新试。
 *
 * 2026-08 实测反馈"响应慢"：换算成 20ms 内实际能走的脉冲数就看出来了——
 * acc=120 时 20ms 只能爬到约 2.9RPM，实际走约 1.6 脉冲/帧，跟 Kx×error 算出来
 * 要求走多远完全无关，是 acc 卡死了行程上限，加 Kx 救不了。acc=230 时约
 * 15RPM、8.2 脉冲/帧，先提到这个值试；感觉还发软可以继续往 240~250 冲，
 * 代价是重新接近 acc=0 的阶跃手感，抖动可能回来，找中间平衡点。
 *
 * 2026-08 二次实测：acc=200（约 7.1RPM、7.6 脉冲/帧）仍然"特别迟钝"，说明
 * 需要的行程能力比之前估的更大，直接跳到 240 试：
 *   acc=240 → 每1RPM耗时(256-240)×50us=0.8ms → 20ms内约25RPM → 约26.7脉冲/帧，
 *   是 acc=200 时的 3.5 倍。如果这样还不够快，继续往 250 冲（约 71 脉冲/帧）；
 *   一旦目标附近开始重新出现抖动，就是抖动和迟钝的临界点，退回上一档定住。
 */
#define T4_BALL_POS_ACC                     (240U)
/*
 * 软件平滑：每帧实际下发的绝对目标相对上一帧最多变化多少脉冲，0=不限速。
 * 把 PD 输出的目标突变摊到连续多帧上，抑制目标跳变造成的机械冲击；小车行驶
 * 中球被持续扰动时尤其有用。调小 → 摆杆动作更柔和但跟踪更迟钝；调大 →
 * 越接近不限速的原始手感；0 = 完全关闭。
 *
 * ⚠️ 它【解决不了】"摆杆走一下停一下"的分段感：那是驱动器把每条绝对位置命令
 * 都规划成"加速→减速→精确停住"的点到点运动，帧间隔内走得完就会停一下；限速
 * 只会让每帧增量更小、更容易走完，反而加重。分段感要查静摩擦（FRICTION_FF）
 * 和 Kx，不要指望这个参数。
 */
#define T4_BALL_MAX_PULSE_STEP              (0U)
#define T4_BALL_HOLD_POSITION_PX            (6.0F)   /* 以下三项仅影响 OLED 的 B:HOLD 显示 */
#define T4_BALL_HOLD_VELOCITY_PXPS          (10.0F)
#define T4_BALL_HOLD_TIME_MS                (500U)

/*
 * 手动调参开关：置 1 时轮子完全不使能，第二次 K3 也不会发车，只让 BALLCTRL
 * 保持钢球平衡（用手推拉小车模拟加减速扰动，专调 T4_BALL_* 参数）。
 * 2026-08 改回 0 以启用完整的"两段式启动 + 循迹定时缓停"流程。
 */
#define T4_MOTORS_DISABLED_MANUAL_TEST      (0U)

typedef enum {
    T4_STATE_IDLE = 0,          /* 刚进题目：什么都不动，等第一次 K3 启动球杆平衡 */
    T4_STATE_WAIT_BALL_CONTROL, /* 已请求球杆闭环，等后台真正接管 ID1 */
    T4_STATE_BALL_READY,        /* 球杆闭环已工作、小车待发，等第二次 K3 */
    T4_STATE_BALL_RECOVER_WAIT, /* 钢珠闭环触发 FAULT_EDGE 后，等待其停止/释放 ID1 再重新请求 */
    T4_STATE_MANUAL_BALANCE,   /* 电机不启动，仅后台 BALLCTRL 保持钢球平衡 */
    T4_STATE_RESET_DISABLE,
    T4_STATE_RESET_WAIT,
    T4_STATE_ENABLE_LEFT,
    T4_STATE_ENABLE_LEFT_WAIT,
    T4_STATE_ENABLE_RIGHT,
    T4_STATE_ENABLE_RIGHT_WAIT,
    T4_STATE_RUN,
    T4_STATE_STOP,
    T4_STATE_TIME_STOP_LEFT,
    T4_STATE_TIME_STOP_RIGHT,
    T4_STATE_TIME_STOPPED,
    T4_STATE_FINISHED
} Task4State_t;

static const AppBallControlProfile_t s_task4BallProfile = {
    T4_BALL_FILTER_ALPHA,
    T4_BALL_FILTER_BETA,
    T4_BALL_OUTPUT_SIGN,
    T4_BALL_KX_PULSE_PER_PX,
    T4_BALL_KV_PULSE_PER_PXPS,
    T4_BALL_LEVEL_TRIM_PULSE,
    T4_BALL_SETTLE_DEADBAND_PX,
    T4_BALL_FRICTION_FF_PULSE,
    T4_BALL_FF_VEL_BLEND_PXPS,
    T4_BALL_POS_RPM,
    T4_BALL_POS_ACC,
    T4_BALL_MAX_PULSE_STEP,
    T4_BALL_HOLD_POSITION_PX,
    T4_BALL_HOLD_VELOCITY_PXPS,
    T4_BALL_HOLD_TIME_MS
};

static Task4State_t s_state;
static Pid_t        s_pid;
static float        s_lastSteerRpm;
static float        s_appliedSteerRpm; /* 经转向限速后实际参与左右轮差速的转向量 */
static float        s_leftRpm;
static float        s_rightRpm;
static float        s_filteredError;
static int32_t      s_sentLeftRpm;
static int32_t      s_sentRightRpm;
static uint32_t     s_lineLostTicks;
static uint32_t     s_enableSettleTicks;
static bool         s_sendLeftNext;
static bool         s_finishArmed;
static uint32_t     s_armTicks;
static uint32_t     s_finishHitTicks;
static uint32_t     s_elapsedTicks;
static uint32_t     s_runTicks;
static bool         s_ballControlRequested;
static bool         s_wheelsStarted;   /* 轮子是否已经完成过一次使能起步（钢珠故障恢复后据此跳过重复使能） */
/*
 * 本次任务是否已经跑完（到达 TIME_STOPPED 或 FINISHED 至少一次）。
 * 一旦置位就不再复位，直到 OnEnter/OnExit——防止跑完之后 BALLCTRL 仍在
 * 后台伺服钢珠、若此时球触边触发 FAULT_EDGE 故障恢复，恢复握手完成后单看
 * s_wheelsStarted 会把状态机误判回 T4_STATE_RUN，导致秒表重新计时、轮子
 * 收到一帧残留的非零速度命令。见 WAIT_BALL_CONTROL 分支的判断顺序。
 */
static bool         s_runCompleted;
/*
 * 两段式启动标志，由 Task4_OnConfirm()（运行态 K3 按下沿）置位、OnLoop 消费：
 *   s_startBallRequested —— 第一次 K3，启动球杆平衡；
 *   s_startCarRequested  —— 第二次 K3，小车开始循迹前进。
 * K3 在 UIMENU 上下文触发、在 UI 任务同一线程内消费，两者是同一个任务，
 * 不存在跨线程竞争，用普通 bool 即可。
 */
static bool         s_startBallRequested;
static bool         s_startCarRequested;
static bool         s_finishBeeped;    /* 缓停完成提示音只响一次 */
static float        s_rampBaseRpm;     /* 起步软件斜坡当前允许的基础速度上限，只增不减 */
static char         s_uiStatusBuf[16];

static float Task4_Clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

/* 整体平移限幅，保留左右轮差速，避免独立限幅压扁转向量。 */
static void Task4_ClampWheelPair(float *leftRpm, float *rightRpm)
{
    float hi = (*leftRpm > *rightRpm) ? *leftRpm : *rightRpm;
    float lo;
    float shift;

    if (hi > T4_MAX_WHEEL_RPM) {
        shift      = hi - T4_MAX_WHEEL_RPM;
        *leftRpm  -= shift;
        *rightRpm -= shift;
    }

    lo = (*leftRpm < *rightRpm) ? *leftRpm : *rightRpm;
    if (lo < T4_MIN_WHEEL_RPM) {
        shift      = T4_MIN_WHEEL_RPM - lo;
        *leftRpm  += shift;
        *rightRpm += shift;
    }

    *leftRpm  = Task4_Clamp(*leftRpm, T4_MIN_WHEEL_RPM, T4_MAX_WHEEL_RPM);
    *rightRpm = Task4_Clamp(*rightRpm, T4_MIN_WHEEL_RPM, T4_MAX_WHEEL_RPM);
}

/* 正常循迹每拍只发一帧，左右轮交替更新，避免共享总线背靠背丢帧。 */
static void Task4_ApplyWheelRpm(float leftRpm, float rightRpm)
{
    int32_t leftInt = (leftRpm >= 0.0F) ? (int32_t)(leftRpm + 0.5F)
                                        : (int32_t)(leftRpm - 0.5F);
    int32_t rightInt = (rightRpm >= 0.0F) ? (int32_t)(rightRpm + 0.5F)
                                          : (int32_t)(rightRpm - 0.5F);

    if (s_sendLeftNext) {
        if (leftInt != s_sentLeftRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_L, (int16_t)leftInt, T4_EMM_ACC);
            s_sentLeftRpm = leftInt;
        }
    } else {
        if (rightInt != s_sentRightRpm) {
            Emm42Robot_SetSpeedRpm(EMM42_ROBOT_WHEEL_R, (int16_t)rightInt, T4_EMM_ACC);
            s_sentRightRpm = rightInt;
        }
    }
    s_sendLeftNext = !s_sendLeftNext;
}

/* 按物理左→右的 LINE8→LINE1 顺序计算加权位置误差。 */
static bool Task4_GetLineError(float *error, uint32_t *hitCountOut)
{
    uint8_t  bitmap = BspLine_ReadAll();
    int32_t  weightedSum = 0;
    uint32_t hitCount = 0U;
    uint32_t physicalIdx;

    for (physicalIdx = 0U; physicalIdx < (uint32_t)BSP_LINE_COUNT; physicalIdx++) {
        uint32_t channel = (uint32_t)BSP_LINE_COUNT - 1U - physicalIdx;
        if ((bitmap & (uint8_t)(1U << channel)) != 0U) {
            weightedSum += (int32_t)(physicalIdx * 2U) - 7;
            hitCount++;
        }
    }

    *hitCountOut = hitCount;
    if (hitCount == 0U) {
        return false;
    }
    *error = (float)weightedSum / (float)hitCount;
    return true;
}

/* 采用任务五同款循迹控制：有效线数据更新 PID，转向输出再经过每拍限速。 */
static void Task4_UpdateTracking(bool lineFound, float rawError)
{
    if (lineFound) {
        if (s_state == T4_STATE_STOP) {
            s_filteredError = rawError;
            Pid_Reset(&s_pid);
        } else {
            s_filteredError += T4_ERROR_FILTER_ALPHA * (rawError - s_filteredError);
        }

        s_lineLostTicks = 0U;
        s_state = T4_STATE_RUN;
        {
            float pidError = s_filteredError;
            if ((pidError > -T4_ERROR_DEADBAND) && (pidError < T4_ERROR_DEADBAND)) {
                pidError = 0.0F;
            }
            s_lastSteerRpm = Pid_Update(&s_pid, pidError, T4_DT_SEC);
        }
    } else {
        s_lineLostTicks++;
        if (s_lineLostTicks >= T4_LINE_LOST_TICKS) {
            s_state = T4_STATE_STOP;
        }
    }

    {
        float maxDelta = T4_STEER_SLEW_RPM_PER_SEC * T4_DT_SEC;
        float delta = s_lastSteerRpm - s_appliedSteerRpm;

        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }
        s_appliedSteerRpm += delta;
    }
}

static uint32_t Task4_GetElapsedMs(void)
{
    return (uint32_t)((float)(s_elapsedTicks * T4_TICK_MS) * T4_STOPWATCH_CAL_SCALE);
}

const char *Task4_GetUiStatus(void)
{
    uint32_t totalMs = Task4_GetElapsedMs();
    uint32_t secWhole = totalMs / 1000U;
    uint32_t tenths = (totalMs / 100U) % 10U;
    uint32_t idx = 0U;
    char digits[10];
    uint32_t n = 0U;
    uint32_t value = secWhole;

    /* 两段式启动的三个等待态各给一行提示，告诉用户现在该按什么。 */
    if (s_state == T4_STATE_IDLE) {
        return "T4 K3=BALL";
    }
    if (s_state == T4_STATE_WAIT_BALL_CONTROL) {
        return "T4 B WAIT";
    }
    if (s_state == T4_STATE_BALL_READY) {
        return "T4 K3=GO";
    }
    if (s_state == T4_STATE_BALL_RECOVER_WAIT) {
        return "T4 B RECOV";
    }
#if T4_MOTORS_DISABLED_MANUAL_TEST
    if (s_state == T4_STATE_MANUAL_BALANCE) {
        return "T4 MANUAL";
    }
#endif

    s_uiStatusBuf[idx++] = 'T';
    s_uiStatusBuf[idx++] = ':';
    if (value == 0U) {
        s_uiStatusBuf[idx++] = '0';
    } else {
        while (value > 0U) {
            digits[n++] = (char)('0' + (value % 10U));
            value /= 10U;
        }
        while (n > 0U) {
            s_uiStatusBuf[idx++] = digits[--n];
        }
    }
    s_uiStatusBuf[idx++] = '.';
    s_uiStatusBuf[idx++] = (char)('0' + tenths);
    s_uiStatusBuf[idx++] = 's';
    if ((s_state == T4_STATE_FINISHED) || (s_state == T4_STATE_TIME_STOPPED)) {
        s_uiStatusBuf[idx++] = ' ';
        s_uiStatusBuf[idx++] = 'D';
        s_uiStatusBuf[idx++] = 'O';
        s_uiStatusBuf[idx++] = 'N';
        s_uiStatusBuf[idx++] = 'E';
    }
    s_uiStatusBuf[idx] = '\0';
    return s_uiStatusBuf;
}

/*
 * 运行态 K3 按下沿（UIMENU → RobotCore_ConfirmTask 转发）。
 * 只置标志、不在这里操作电机：本函数在 UI 的按键分支里被调用，实际动作统一
 * 交给同一拍随后运行的 OnLoop 状态机，避免两处都发 Emm42 命令。
 * 第一次按启动球杆平衡，第二次按发车；其余状态下按 K3 无效（忽略）。
 */
void Task4_OnConfirm(void)
{
    if (s_state == T4_STATE_IDLE) {
        s_startBallRequested = true;
    } else if (s_state == T4_STATE_BALL_READY) {
        s_startCarRequested = true;
    }
}

void Task4_OnEnter(void)
{
    Pid_Init(&s_pid, T4_KP, T4_KI, T4_KD, T4_INTEGRAL_LIMIT, T4_MAX_STEER_RPM);
    /* 进题目只做复位，什么都不动，等第一次 K3。 */
    s_state             = T4_STATE_IDLE;
    s_lastSteerRpm      = 0.0F;
    s_appliedSteerRpm   = 0.0F;
    s_leftRpm           = 0.0F;
    s_rightRpm          = 0.0F;
    s_filteredError     = 0.0F;
    s_sentLeftRpm       = 0;
    s_sentRightRpm      = 0;
    s_lineLostTicks     = 0U;
    s_enableSettleTicks = 0U;
    s_sendLeftNext      = true;
    s_finishArmed       = false;
    s_armTicks          = 0U;
    s_finishHitTicks    = 0U;
    s_elapsedTicks      = 0U;
    s_runTicks          = 0U;
    s_ballControlRequested = false;
    s_wheelsStarted     = false;
    s_startBallRequested = false;
    s_startCarRequested  = false;
    s_finishBeeped      = false;
    s_rampBaseRpm       = 0.0F;
    s_runCompleted      = false;
}

void Task4_OnLoop(void)
{
    float rawError;
    bool lineFound;
    float targetLeftRpm;
    float targetRightRpm;
    uint32_t hitCount;
    AppBallControlStatus_t ballStatus;

    AppBallControl_GetStatus(&ballStatus);

    /*
     * 秒表只统计"小车已发车之后"的时间：IDLE/等待球杆/待发车这三个等待态，
     * 以及故障恢复、手动模式、已完成态都不计时。
     */
    if ((s_state != T4_STATE_IDLE) &&
        (s_state != T4_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T4_STATE_BALL_READY) &&
        (s_state != T4_STATE_BALL_RECOVER_WAIT) &&
        (s_state != T4_STATE_MANUAL_BALANCE) &&
        (s_state != T4_STATE_FINISHED) && (s_state != T4_STATE_TIME_STOPPED)) {
        s_elapsedTicks++;
    }

    /* 第一次 K3 之前什么都不做：ID1 不闭环、轮子不使能，摆杆保持归零后的水平位置。 */
    if (s_state == T4_STATE_IDLE) {
        if (s_startBallRequested) {
            s_startBallRequested = false;
            s_state = T4_STATE_WAIT_BALL_CONTROL;
        }
        return;
    }

    /*
     * 钢珠闭环触发 FAULT_EDGE（球触边）后会一直锁在回水平的位置，不会自己恢复，
     * 必须重新走一遍"停止→等待释放→重新请求"的握手才能恢复到目标位置；否则表现
     * 就是"进了任务四但钢珠不再被伺服"。这里主动检测并恢复，不需要用户手动
     * 退出重进。只要不是正在做这套握手本身，任何时候（含 RUN/STOP/已完成等待
     * K4 退出期间）检测到故障都立即触发恢复。
     */
    if ((s_state != T4_STATE_WAIT_BALL_CONTROL) &&
        (s_state != T4_STATE_BALL_RECOVER_WAIT) &&
        (ballStatus.state == APP_BALL_CONTROL_FAULT_EDGE)) {
        AppBallControl_RequestStop();
        s_ballControlRequested = false;
        s_state = T4_STATE_BALL_RECOVER_WAIT;
    }

    /* 等 BALLCTRL 真正回到 OFF 才能重新请求，避免与刚发出的停止命令产生竞态。 */
    if (s_state == T4_STATE_BALL_RECOVER_WAIT) {
        if (ballStatus.state == APP_BALL_CONTROL_OFF) {
            s_state = T4_STATE_WAIT_BALL_CONTROL;
        }
        return;
    }

    /* 第一次 K3 后：请求球杆闭环，等后台真正接管 ID1。 */
    if (s_state == T4_STATE_WAIT_BALL_CONTROL) {
        if (!s_ballControlRequested) {
            s_ballControlRequested = AppBallControl_RequestTargetWithProfile(
                T4_BALL_TARGET_X_PX, &s_task4BallProfile);
        }

        if (s_ballControlRequested &&
            (ballStatus.targetPx == T4_BALL_TARGET_X_PX) &&
            ((ballStatus.state == APP_BALL_CONTROL_RUNNING) ||
             (ballStatus.state == APP_BALL_CONTROL_HOLDING))) {
            if (s_runCompleted) {
                /*
                 * 本次任务本来就已经跑完了（缓停或终点保护都已发生过）——
                 * BALLCTRL 停车后仍在后台伺服钢珠，球触边引发的 FAULT_EDGE
                 * 恢复握手不能把状态机复活成 RUN，否则秒表会重新计时、还会
                 * 给轮子发一帧残留的非零速度命令。直接停在"已完成"，不碰轮子。
                 */
                s_state = T4_STATE_TIME_STOPPED;
            } else if (s_wheelsStarted) {
                /* 钢珠故障恢复场景：轮子早已在跑，直接回到循迹，不重新走使能时序。 */
                s_state = T4_STATE_RUN;
            } else {
#if T4_MOTORS_DISABLED_MANUAL_TEST
                s_state = T4_STATE_MANUAL_BALANCE;
#else
                /* 球杆已在伺服，停在这里等第二次 K3 发车。 */
                s_state = T4_STATE_BALL_READY;
#endif
            }
        }
        return;
    }

    /*
     * 球杆平衡已工作、小车待发：持续保持钢珠伺服（BALLCTRL 后台自己在跑），
     * 等第二次 K3 才走轮子使能时序。这段时间用户可以目视确认小球是否稳住。
     */
    if (s_state == T4_STATE_BALL_READY) {
        if (s_startCarRequested) {
            s_startCarRequested = false;
            s_wheelsStarted = true;
            /* 起步斜坡从 0 重新爬，保证每次发车都是缓慢加速。 */
            s_rampBaseRpm = 0.0F;
            /*
             * 本题 deferVideoStart=true（app_robot_core.c 题目表），进题目时
             * 不会自动开始录像；真正发车（小车即将开始动作）的这一刻才通知
             * 视觉端开始，对应"第二次按键才开始录制"。结束仍由
             * RobotCore_NotifyTaskFinished(3U) 在缓停/终点保护触发蜂鸣器时调用，
             * 未受本次改动影响。
             */
            RobotCore_NotifyTaskStarted(3U);
            s_state = T4_STATE_RESET_DISABLE;
        }
        return;
    }

#if T4_MOTORS_DISABLED_MANUAL_TEST
    /* 手动调参模式：电机不使能、不驱动，仅后台 BALLCTRL 保持钢球 X=320。*/
    if (s_state == T4_STATE_MANUAL_BALANCE) {
        return;
    }
#endif

    if (s_state == T4_STATE_RESET_DISABLE) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
        vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);
        s_enableSettleTicks = T4_RESET_SETTLE_TICKS;
        s_state = T4_STATE_RESET_WAIT;
        return;
    }
    if (s_state == T4_STATE_RESET_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T4_STATE_ENABLE_LEFT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_LEFT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, true);
        s_enableSettleTicks = T4_ENABLE_SETTLE_TICKS;
        s_state = T4_STATE_ENABLE_LEFT_WAIT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_LEFT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T4_STATE_ENABLE_RIGHT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_RIGHT) {
        Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, true);
        s_enableSettleTicks = T4_ENABLE_SETTLE_TICKS;
        s_state = T4_STATE_ENABLE_RIGHT_WAIT;
        return;
    }
    if (s_state == T4_STATE_ENABLE_RIGHT_WAIT) {
        if (s_enableSettleTicks > 0U) {
            s_enableSettleTicks--;
            return;
        }
        s_state = T4_STATE_RUN;
    }

    /* 计时到：两拍分别给左右轮发送带加速度的速度模式 0 RPM，不能走急停接口。 */
    if (s_state == T4_STATE_TIME_STOP_LEFT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_L, 0, T4_STOP_EMM_ACC);
        s_sentLeftRpm = 0;
        s_state = T4_STATE_TIME_STOP_RIGHT;
        return;
    }
    if (s_state == T4_STATE_TIME_STOP_RIGHT) {
        Emm42Robot_VelControl(EMM42_ROBOT_WHEEL_R, 0, T4_STOP_EMM_ACC);
        s_sentRightRpm = 0;
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        s_state = T4_STATE_TIME_STOPPED;
        s_runCompleted = true;
        /* 到点缓停：与按键共用同一短促提示音，只响一次（本状态每拍都会进）。 */
        if (!s_finishBeeped) {
            s_finishBeeped = true;
            BspBuzzer_BeepShort();
        }
        RobotCore_NotifyTaskFinished(3U);
        return;
    }
    if (s_state == T4_STATE_FINISHED) {
        return;
    }
    /*
     * 2026-08 修复"OLED 计时器不停"：TIME_STOPPED 之前没有像 FINISHED 这样在
     * 入口直接 return，会继续往下穿过循迹读数、终点判定这些逻辑——虽然靠零散
     * 的 if 挡住了轮子和 RUN，但挡得不彻底（比如终点判定完全没检查当前状态，
     * 停车后如果车身正好压在满足终点条件的线型上，会把 TIME_STOPPED 悄悄
     * 拨成 FINISHED）。跟 FINISHED 一样在这里直接截断，彻底停止响应，是最
     * 可靠的写法，不用逐条枚举后面每个分支该不该管 TIME_STOPPED。
     */
    if (s_state == T4_STATE_TIME_STOPPED) {
        return;
    }

    /* 只统计正常循迹状态的前进时间；丢线停车期间不计入缓停计时。 */
    if (s_state == T4_STATE_RUN) {
        s_runTicks++;
        if ((s_runTicks * T4_TICK_MS) >= T4_STOP_AFTER_MS) {
            s_state = T4_STATE_TIME_STOP_LEFT;
            return;
        }
    }

    lineFound = Task4_GetLineError(&rawError, &hitCount);
    Task4_UpdateTracking(lineFound, rawError);

    if (!s_finishArmed) {
        if (hitCount <= T4_FINISH_ARM_HIT_MAX) {
            s_armTicks++;
            if (s_armTicks >= T4_FINISH_ARM_TICKS) {
                s_finishArmed = true;
            }
        } else {
            s_armTicks = 0U;
        }
    } else if (s_state != T4_STATE_FINISHED) {
        if (hitCount >= T4_FINISH_HIT_MIN) {
            s_finishHitTicks++;
            if (s_finishHitTicks >= T4_FINISH_HIT_TICKS) {
                s_state = T4_STATE_FINISHED;
                s_runCompleted = true;
            }
        } else {
            s_finishHitTicks = 0U;
        }
    }

    if (s_state == T4_STATE_FINISHED) {
        /* 终点保护沿用任务二：这是异常/终点保护，仍需要立即急停。 */
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
        vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
        Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
        s_leftRpm = 0.0F;
        s_rightRpm = 0.0F;
        /* 与定时缓停共用同一个"只响一次"标志，避免两条路径都触发时叫两声。 */
        if (!s_finishBeeped) {
            s_finishBeeped = true;
            BspBuzzer_BeepShort();
        }
        RobotCore_NotifyTaskFinished(3U);
        return;
    }

    if (s_state == T4_STATE_RUN) {
        float steerAbs = (s_appliedSteerRpm >= 0.0F) ? s_appliedSteerRpm : -s_appliedSteerRpm;
        float dynBaseRpm = T4_BASE_RPM - T4_CORNER_SLOWDOWN_GAIN * steerAbs;
        dynBaseRpm = Task4_Clamp(dynBaseRpm, T4_MIN_BASE_RPM, T4_BASE_RPM);
        {
            uint32_t elapsedMs = Task4_GetElapsedMs();
            if (elapsedMs >= T4_DECEL_START_MS) {
                float decelSec = (float)(elapsedMs - T4_DECEL_START_MS) * 0.001F;
                float decelBaseRpm = T4_BASE_RPM - T4_DECEL_GRADIENT_RPM_PER_SEC * decelSec;
                decelBaseRpm = Task4_Clamp(decelBaseRpm, T4_DECEL_MIN_RPM, T4_BASE_RPM);
                if (decelBaseRpm < dynBaseRpm) {
                    dynBaseRpm = decelBaseRpm;
                }
            }
        }
        /*
         * 起步软件斜坡：每拍把允许的基础速度上限抬高 (RPM/秒 × 本拍秒数)，
         * 只压【上升】不干预下降。驱动器 acc 曲线最慢只能 1.4 秒到 110RPM，
         * 想要更缓的起步必须靠这里。爬满 T4_BASE_RPM 后本限制自然失效，
         * 转弯减速后的恢复不受影响。
         */
        s_rampBaseRpm += T4_START_RAMP_RPM_PER_SEC * T4_DT_SEC;
        if (s_rampBaseRpm > T4_BASE_RPM) {
            s_rampBaseRpm = T4_BASE_RPM;
        }
        if (dynBaseRpm > s_rampBaseRpm) {
            dynBaseRpm = s_rampBaseRpm;
        }
        targetLeftRpm = dynBaseRpm + s_appliedSteerRpm;
        targetRightRpm = dynBaseRpm - s_appliedSteerRpm;
        Task4_ClampWheelPair(&targetLeftRpm, &targetRightRpm);
    } else {
        targetLeftRpm = 0.0F;
        targetRightRpm = 0.0F;
    }

    s_leftRpm = targetLeftRpm;
    s_rightRpm = targetRightRpm;
    if (s_state != T4_STATE_TIME_STOPPED) {
        Task4_ApplyWheelRpm(s_leftRpm, s_rightRpm);
    }
}

void Task4_OnExit(void)
{
    /* K4 退出保留任务二的硬安全收尾：急停后失能左右轮。 */
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_L);
    vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
    Emm42Robot_Stop(EMM42_ROBOT_WHEEL_R);
    vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_L, false);
    vTaskDelay(pdMS_TO_TICKS(T4_EMM_CMD_GAP_MS));
    Emm42Robot_Enable(EMM42_ROBOT_WHEEL_R, false);

    /* 任务四结束后才释放 ID1，运行和缓停期间保持钢珠 X=320 的位置目标。 */
    AppBallControl_RequestStop();

    Pid_Reset(&s_pid);
    /* 回到未启动态：下次进题目仍需重新按两次 K3。 */
    s_state = T4_STATE_IDLE;
    s_lastSteerRpm = 0.0F;
    s_appliedSteerRpm = 0.0F;
    s_finishArmed = false;
    s_armTicks = 0U;
    s_finishHitTicks = 0U;
    s_runTicks = 0U;
    s_elapsedTicks = 0U;
    s_ballControlRequested = false;
    s_wheelsStarted = false;
    s_startBallRequested = false;
    s_startCarRequested = false;
    s_finishBeeped = false;
    s_rampBaseRpm = 0.0F;
    s_runCompleted = false;
}
