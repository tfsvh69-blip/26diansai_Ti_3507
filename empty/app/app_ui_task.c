#include "app_ui_task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ball_control_task.h"
#include "app_config.h"
#include "app_imu_uart_task.h"
#include "app_robot_core.h"
#include "app_tasks.h"
#include "app_vision_link.h"
#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_line.h"
#include "laser_ld14.h"

#include "OLED.h"

/* ==================================================================
 * OLED 题目菜单 UI 任务
 *
 * 一个两态状态机，独占板载 OLED（PB8/PB9 软件 I2C）+ 四个按键：
 *   MENU 菜单态：一屏列出全部题目，反色高亮当前项；
 *   RUN  运行态：进入所选题目的运行界面，周期调用该题业务钩子。
 *
 * 按键（KEY1~4 = PA28/PA31/PA30/PA29，按下接地）：
 *   K1 上移   K2 下移   K3 确认进入   K4 运行态返回/菜单态启停 X=350 单点验证
 *
 * 刷屏策略：软件 I2C 整屏刷约 50ms，故采用「事件驱动」——只有按键改变了
 * 选中项或状态时才重绘 + OLED_Update；平时任务只是轻量轮询按键，不刷屏，
 * 对 CPU 友好（不像旧 PERIPH 任务那样固定 500ms 无条件全屏刷）。
 *
 * 题目业务：全部委托给 app_robot_core 模块——UI 只负责显示与按键，题目具体做什幺
 * 由 robot_core 的 dispatch 表分发。后续逐题填充时只改 robot_core，UI 框架不动。
 *
 * 文件结构：1.布局常量 → 2.OLED 绘制 → 3.按键/状态机任务入口。
 * ================================================================== */

/* ------------------------------------------------------------------
 * 1. 布局常量（128x64，菜单用 6x8 小字体：21 列 × 8 行）
 * ------------------------------------------------------------------ */

/* ------------------------------------------------------------------
 * 2. 布局常量（128x64，菜单用 6x8 小字体：21 列 × 8 行）
 * ------------------------------------------------------------------ */

#define UI_MENU_TITLE_Y   (0)     /* 标题行 */
#define UI_MENU_FIRST_Y   (8)     /* 第一个题目行的 Y */
#define UI_MENU_LINE_H    (8)     /* 行高（6x8 字体） */
#define UI_MENU_VISIBLE   (6U)    /* 一屏最多显示 6 项（Y=8..48），多于此自动滚动 */
#define UI_MENU_STATUS_Y  (56)    /* 菜单态：底行传感器状态栏（Yaw + 激光距离） */

/*
 * 右半面板（利用菜单空出的右半屏）。视觉通信面板 / 循迹面板 任一启用时：
 *   - 菜单文字与选中项高亮收窄到左半（宽 UI_MENU_LEFT_W），不侵入右侧面板；
 *   - 右半画一条竖分隔线（UI_RIGHT_DIV_X），面板自 x=UI_RIGHT_X 起。
 * 两块面板在右半分行错开：视觉通信占第1~4行(y=0..31)，循迹占第6/7行(y=40/48)，互不重叠。
 * 两者都关闭时菜单恢复整行(128)高亮与整宽标题，右半留空。
 */
#if (APP_FEATURE_VISION_LINK != 0U) || (APP_FEATURE_LINE_TRACK != 0U)
#define UI_MENU_LEFT_W    (60)     /* 菜单占左半宽度（右半让给面板） */
#define UI_RIGHT_X        (66)     /* 右半面板左边界 x */
#define UI_RIGHT_DIV_X    (63)     /* 左右分隔竖线 x */
#else
#define UI_MENU_LEFT_W    (128)    /* 无右半面板：菜单高亮整行 */
#endif

#if (APP_FEATURE_VISION_LINK != 0U)
#define UI_VISION_X         (UI_RIGHT_X) /* 右侧视觉通信面板左边界 x */
#define UI_VISION_W         (62)         /* 右侧视觉通信面板宽度（128-66） */
#define UI_VISION_FIELD_Y   (8)          /* 字段区起始 y */
#define UI_VISION_FIELD_H   (32)         /* 四行：网络、X、ACK、钢球闭环 */
#endif

#if (APP_FEATURE_LINE_TRACK != 0U)
#define UI_LINE_X         (UI_RIGHT_X) /* 右侧循迹面板左边界 x */
#define UI_LINE_W         (62)     /* 循迹面板宽度（128-66） */
#define UI_LINE_CH_Y      (40)     /* 第6行：通道号 "87654321" */
#define UI_LINE_ST_Y      (48)     /* 第7行：LINE8→LINE1 的对应状态 */
#define UI_LINE_H         (16)     /* 两行高度 y=40..55 */
#endif

/* 运行态布局：标题(0)、大字题号(16)、题名(40)、状态栏(48)、返回提示(56)。 */
#define UI_RUN_NAME_Y     (40)
#define UI_RUN_STATUS_Y   (48)    /* 运行态：传感器状态栏 */
#define UI_RUN_HINT_Y     (56)

/* task5 在题目表中的固定下标，用于显示其循迹与停车流程状态。 */
#define UI_TASK5_INDEX    (4U)

/* task2 在题目表中的固定下标，用于显示其秒表计时。 */
#define UI_TASK2_INDEX    (1U)

/* task3 在题目表中的固定下标，用于显示对称左右摆球阶段。 */
#define UI_TASK3_INDEX    (2U)

/* task4 在题目表中的固定下标，用于显示其秒表计时。 */
#define UI_TASK4_INDEX    (3U)

/* task6 在题目表中的固定下标，用于显示 ID1 位置模式测试的当前阶段。 */
#define UI_TASK6_INDEX    (5U)

/* 菜单当前选中项与可见窗口首项（题目多于一屏时滚动）。 */
static uint32_t s_sel     = 0U;
static uint32_t s_menuTop = 0U;

/* 两态状态机。 */
typedef enum {
    UI_STATE_MENU = 0,
    UI_STATE_RUN
} UiState_t;

/* ------------------------------------------------------------------
 * 3. OLED 绘制
 * ------------------------------------------------------------------ */

/* 无符号整数转十进制字符串（不含结尾 '\0'），返回写入的字符数。 */
static uint32_t Ui_U32ToStr(char *dst, uint32_t value)
{
    char     tmp[10];
    uint32_t n = 0U;
    uint32_t i;

    if (value == 0U) {
        dst[0] = '0';
        return 1U;
    }
    while (value > 0U) {
        tmp[n++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    /* tmp 里是反序，倒着拷回。 */
    for (i = 0U; i < n; i++) {
        dst[i] = tmp[n - 1U - i];
    }
    return n;
}

/*
 * 组装传感器状态栏文本到 buf（需 >= 32 字节）："Y:<yaw> D:<dist>mm"，
 * 用于一眼判断陀螺仪/激光测距的当前状态：
 *   Yaw  IMU 就绪显示带 1 位小数的度数(如 -179.9)，未就绪显示 "---"；
 *   Dist 激光收到有效帧显示 mm 数，否则显示 "---"。
 * 数据都取线程安全快照（Yaw 走 IMU 任务 getter，距离走 laser GetLatest）。
 */
static void Ui_FormatStatus(char *buf)
{
    uint32_t        idx = 0U;
    int16_t         yawCd = 0;
    LaserLd14Data_t laser;

    buf[idx++] = 'Y';
    buf[idx++] = ':';
    if (AppImuUartTask_GetYaw(&yawCd)) {
        int32_t cd = yawCd;
        if (cd < 0) {
            buf[idx++] = '-';
            cd = -cd;
        }
        idx += Ui_U32ToStr(&buf[idx], (uint32_t)(cd / 100));  /* 整数度 */
        buf[idx++] = '.';
        buf[idx++] = (char)('0' + ((cd % 100) / 10));         /* 1 位小数 */
    } else {
        buf[idx++] = '-';
        buf[idx++] = '-';
        buf[idx++] = '-';
    }

    buf[idx++] = ' ';
    buf[idx++] = 'D';
    buf[idx++] = ':';
    if (LaserLd14_GetLatest(&laser)) {
        idx += Ui_U32ToStr(&buf[idx], (uint32_t)laser.distanceMm);
        buf[idx++] = 'm';
        buf[idx++] = 'm';
    } else {
        buf[idx++] = '-';
        buf[idx++] = '-';
        buf[idx++] = '-';
    }

    buf[idx] = '\0';
}

/*
 * 局部刷新状态栏那一行：清行显存 → 写新文本 → 只把该行(128×8)推送到屏。
 * 只动这一行、面积小、频率低（APP_UI_STATUS_DIVIDER 控制），几乎不占 CPU，
 * 不打断按键响应，也不动菜单/运行界面的其它内容。
 */
static void Ui_DrawStatusBar(int16_t y)
{
    char buf[32];

    Ui_FormatStatus(buf);
    OLED_ClearArea(0, y, 128, UI_MENU_LINE_H);
    OLED_ShowString(0, y, buf, OLED_6X8);
    OLED_UpdateArea(0, y, 128, UI_MENU_LINE_H);
}

#if (APP_FEATURE_VISION_LINK != 0U)
#if (APP_FEATURE_BALL_CONTROL != 0U)
static const char *Ui_GetBallControlText(void)
{
    AppBallControlStatus_t status;

    AppBallControl_GetStatus(&status);
    switch (status.state) {
    case APP_BALL_CONTROL_OFF:             return "B:OFF";
    case APP_BALL_CONTROL_STARTING:        return "B:START";
    case APP_BALL_CONTROL_WAIT_VISION:     return "B:WAIT";
    case APP_BALL_CONTROL_RUNNING:         return "B:RUN";
    case APP_BALL_CONTROL_HOLDING:         return "B:HOLD";
    case APP_BALL_CONTROL_LOST:            return "B:LOST";
    case APP_BALL_CONTROL_FAULT_EDGE:      return "B:EDGE!";
    default:                               return "B:???";
    }
}
#endif

/*
 * 绘制右侧视觉通信字段（不做刷屏推送，交给调用方）：
 *   NET:ON/OFF 为匹配 PONG 后的 3 秒在线判定；
 *   X 为相机发送的原始水平像素字段，便于直接确认 UART0 链路；
 *   P 为最近发出的 PING 编号，便于串口联调时核对 PONG 是否回显相同 id。
 */
static void Ui_DrawVisionFields(void)
{
    AppVisionLinkStatus_t vision;
    char       buf[APP_VISION_X_TEXT_MAX + 3U];
    uint32_t   n;

    AppVisionLink_GetStatus(&vision);
    OLED_ShowString(UI_VISION_X, UI_VISION_FIELD_Y,
        vision.online ? "NET:ON" : "NET:OFF", OLED_6X8);

    buf[0] = 'X'; buf[1] = ':';
    if (vision.xReceived) {
        n = 2U;
        while ((n < (sizeof(buf) - 1U)) && vision.xText[n - 2U] != '\0') {
            buf[n] = vision.xText[n - 2U];
            n++;
        }
        if (n == 2U) {
            buf[n++] = '?';
        }
    } else {
        buf[2] = '-'; buf[3] = '-'; buf[4] = '-'; n = 5U;
    }
    buf[n] = '\0';
    OLED_ShowString(UI_VISION_X, UI_VISION_FIELD_Y + 8, buf, OLED_6X8);

    OLED_ShowString(UI_VISION_X, UI_VISION_FIELD_Y + 16,
        vision.ackMatched ? (vision.ackStart ? "ACK:ST" : "ACK:SP") : "ACK:--", OLED_6X8);

#if (APP_FEATURE_BALL_CONTROL != 0U)
    OLED_ShowString(UI_VISION_X, UI_VISION_FIELD_Y + 24,
                    (char *)Ui_GetBallControlText(), OLED_6X8);
#endif
}

/*
 * 局部刷新右侧视觉通信字段区：清区 → 重画网络、位置和 ACK → 只推送该矩形。
 * 与底部状态栏同频（APP_UI_STATUS_DIVIDER），面积小、频率低，几乎不占 CPU，
 * 不打断按键响应，也不动菜单左半与 "VISION" 头。仅菜单态调用。
 */
static void Ui_DrawVisionPanel(void)
{
    OLED_ClearArea(UI_VISION_X, UI_VISION_FIELD_Y, UI_VISION_W, UI_VISION_FIELD_H);
    Ui_DrawVisionFields();
    OLED_UpdateArea(UI_VISION_X, UI_VISION_FIELD_Y, UI_VISION_W, UI_VISION_FIELD_H);
}
#endif /* APP_FEATURE_VISION_LINK */

#if (APP_FEATURE_LINE_TRACK != 0U)
/*
 * 绘制右半循迹两行（不做刷屏推送，交给调用方）：
 *   第6行(y=40)：通道号 "87654321"（左=8号=小车左，右=1号=小车右）；
 *   第7行(y=48)：对应状态按 bit7…bit0 倒序显示，与上行逐位对齐，1=识别到线、0=未识别。
 * 状态取 BspLine_ReadAll 位图：bit7=LINE8 放最左，bit0=LINE1 放最右，屏上左右即小车左右。
 */
static void Ui_DrawLineFields(void)
{
    uint8_t  bitmap = BspLine_ReadAll();
    char     st[BSP_LINE_COUNT + 1U];
    uint32_t i;

    OLED_ShowString(UI_LINE_X, UI_LINE_CH_Y, "87654321", OLED_6X8);
    for (i = 0U; i < (uint32_t)BSP_LINE_COUNT; i++) {
        /* OLED 左侧从物理最左的 LINE8 开始显示，因此按位图高位到低位取值。 */
        st[i] = (char)('0' + ((bitmap >> ((uint32_t)BSP_LINE_COUNT - 1U - i)) & 0x1U));
    }
    st[BSP_LINE_COUNT] = '\0';
    OLED_ShowString(UI_LINE_X, UI_LINE_ST_Y, st, OLED_6X8);
}

/*
 * 局部刷新右侧循迹两行：清区 → 重画通道号+状态 → 只推送该矩形。
 * 与底部状态栏同频（APP_UI_STATUS_DIVIDER），面积小、频率低，几乎不占 CPU，
 * 不打断按键响应，也不动菜单左半与右半的视觉通信区。仅菜单态调用。
 */
static void Ui_DrawLinePanel(void)
{
    OLED_ClearArea(UI_LINE_X, UI_LINE_CH_Y, UI_LINE_W, UI_LINE_H);
    Ui_DrawLineFields();
    OLED_UpdateArea(UI_LINE_X, UI_LINE_CH_Y, UI_LINE_W, UI_LINE_H);
}
#endif /* APP_FEATURE_LINE_TRACK */

/* 根据当前选中项调整可见窗口首项，保证高亮项始终在屏内。 */
static void Ui_MenuScroll(void)
{
    if (s_sel < s_menuTop) {
        s_menuTop = s_sel;
    } else if (s_sel >= s_menuTop + UI_MENU_VISIBLE) {
        s_menuTop = s_sel - (UI_MENU_VISIBLE - 1U);
    }
}

/* 绘制菜单：标题 + 可见题目列表（反色高亮当前项）+ 底部操作提示，整屏刷新。 */
static void Ui_DrawMenu(void)
{
    uint32_t i;

    OLED_Clear();
#if (APP_FEATURE_VISION_LINK != 0U)
    /* 标题收窄到左半，给右侧视觉通信面板让位。 */
#if (APP_FEATURE_BALL_CONTROL != 0U)
    OLED_ShowString(0, UI_MENU_TITLE_Y, "K4:X350", OLED_6X8);
#else
    OLED_ShowString(0, UI_MENU_TITLE_Y, "TASKS", OLED_6X8);
#endif
#else
    OLED_ShowString(13, UI_MENU_TITLE_Y, "== SELECT TASK ==", OLED_6X8);
#endif

    for (i = 0U; i < UI_MENU_VISIBLE && (s_menuTop + i) < RobotCore_GetTaskCount(); i++) {
        uint32_t idx = s_menuTop + i;
        int16_t  y   = (int16_t)(UI_MENU_FIRST_Y + (int)i * UI_MENU_LINE_H);

        /* 行文本："n.名称"，编号从 1 开始。 */
        OLED_ShowNum(0, y, idx + 1U, 1, OLED_6X8);
        OLED_ShowString(6, y, ".", OLED_6X8);
        OLED_ShowString(12, y, (char *)RobotCore_GetTaskName(idx), OLED_6X8);

        /* 当前选中项高亮：启用球面板时只反色左半(UI_MENU_LEFT_W)，不侵入右侧面板。 */
        if (idx == s_sel) {
            OLED_ReverseArea(0, y, UI_MENU_LEFT_W, UI_MENU_LINE_H);
        }
    }

#if (APP_FEATURE_VISION_LINK != 0U) || (APP_FEATURE_LINE_TRACK != 0U)
    /* 右半竖分隔线（视觉通信与循迹面板共用）。 */
    OLED_DrawLine(UI_RIGHT_DIV_X, UI_MENU_FIRST_Y, UI_RIGHT_DIV_X, 54);
#endif
#if (APP_FEATURE_VISION_LINK != 0U)
    /* 右半上部视觉通信面板，字段之后由主循环低频局部刷更新。 */
    OLED_ShowString(UI_VISION_X, UI_MENU_TITLE_Y, "VISION", OLED_6X8);
    Ui_DrawVisionFields();
#endif
#if (APP_FEATURE_LINE_TRACK != 0U)
    /* 右半下部(第6/7行)循迹面板：通道号 + 状态（之后由主循环低频局部刷更新）。 */
    Ui_DrawLineFields();
#endif

    /* 底行：传感器状态栏（Yaw + 激光距离）。整屏刷时写入当前值，之后由主循环低频局部刷更新。 */
    {
        char buf[32];
        Ui_FormatStatus(buf);
        OLED_ShowString(0, UI_MENU_STATUS_Y, buf, OLED_6X8);
    }
    OLED_Update();
}

/* 绘制运行界面：大字题号 + 题名 + 返回提示，整屏刷新。 */
static void Ui_DrawRun(void)
{
    OLED_Clear();
    OLED_ShowString(13, 0, "==== RUNNING ====", OLED_6X8);

    /* 大字题号：TASK n（8x16 字体，字宽 8px）。 */
    OLED_ShowString(0, 16, "TASK", OLED_8X16);
    OLED_ShowNum(40, 16, s_sel + 1U, 1, OLED_8X16);

    /*
     * 题名；任务二、四显示秒表计时，任务五显示循迹与停车流程状态，
     * 任务六显示 ID1 位置模式测试的当前阶段与目标脉冲。
     */
    if (s_sel == UI_TASK5_INDEX) {
        OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task5_GetUiStatus(), OLED_6X8);
    } else if (s_sel == UI_TASK2_INDEX) {
        OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task2_GetUiStatus(), OLED_6X8);
    } else if (s_sel == UI_TASK3_INDEX) {
        OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task3_GetUiStatus(), OLED_6X8);
    } else if (s_sel == UI_TASK4_INDEX) {
        OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task4_GetUiStatus(), OLED_6X8);
    } else if (s_sel == UI_TASK6_INDEX) {
        OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task6_GetUiStatus(), OLED_6X8);
    } else {
        OLED_ShowString(0, UI_RUN_NAME_Y, (char *)RobotCore_GetTaskName(s_sel), OLED_6X8);
    }

    /* 传感器状态栏（写入当前值，之后由主循环低频局部刷更新）。 */
    {
        char buf[32];
        Ui_FormatStatus(buf);
        OLED_ShowString(0, UI_RUN_STATUS_Y, buf, OLED_6X8);
    }

    /* 返回提示。 */
    OLED_ShowString(0, UI_RUN_HINT_Y, "K4: back to menu", OLED_6X8);
    OLED_Update();
}

/* ------------------------------------------------------------------
 * 4. 任务入口：轮询 4 键（去抖 + 按下沿），驱动菜单/运行状态机。
 * ------------------------------------------------------------------ */

static TaskHandle_t s_uiTaskHandle = NULL;

static void AppUiTask_Entry(void *argument)
{
    UiState_t  state = UI_STATE_MENU;
    bool       prev[BSP_KEY_COUNT];
    uint32_t   i;
    uint32_t   statusTick = 0U;
    TickType_t lastWakeTime;

    (void)argument;

    /* OLED 软件 I2C 初始化（GPIO 已在板级初始化配为推挽输出）。 */
    OLED_Init();
    OLED_Clear();

    /* 用真实电平初始化按键基线，避免上电把"未松开"误判成一次按下沿。 */
    for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
        prev[i] = BspKey_IsPressed((BspKeyId_t)i);
    }

    /* 首屏菜单。 */
    s_sel     = 0U;
    s_menuTop = 0U;
    Ui_DrawMenu();

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        bool edge[BSP_KEY_COUNT];
        bool anyKeyPressed = false;

        /* 每拍先推进视觉通信状态机，保证 PING、在线超时和 X 超时按协议执行。 */
#if (APP_FEATURE_VISION_LINK != 0U)
        AppVisionLink_Service();
#endif

        /* 每键做一次去抖后的按下沿检测（30ms 轮询本身即去抖窗口）。 */
        for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
            bool now = BspKey_IsPressed((BspKeyId_t)i);
            edge[i]  = (now && !prev[i]);
            prev[i]  = now;
            anyKeyPressed = anyKeyPressed || edge[i];
        }

        /* 菜单态和运行态的任一按键都短促响一声；K3 不再额外双响。 */
        if (anyKeyPressed) {
            BspBuzzer_BeepShort();
        }

        if (state == UI_STATE_MENU) {
            bool dirty = false;

            /* K1 上移、K2 下移，都循环回绕（首↔尾）。 */
            if (edge[BSP_KEY_1]) {
                s_sel = (s_sel == 0U) ? (RobotCore_GetTaskCount() - 1U) : (s_sel - 1U);
                Ui_MenuScroll();
                dirty = true;
            }
            if (edge[BSP_KEY_2]) {
                s_sel = (s_sel + 1U >= RobotCore_GetTaskCount()) ? 0U : (s_sel + 1U);
                Ui_MenuScroll();
                dirty = true;
            }

#if (APP_FEATURE_BALL_CONTROL != 0U)
            /*
             * 菜单态 K4 是独立的单点验证：首次请求 X=350，闭环运行时再次按下
             * 则安全停止。状态与实测 X 由右侧 VISION 面板的 B:/X: 字段反馈。
             */
            if (edge[BSP_KEY_4]) {
                AppBallControlStatus_t ballStatus;

                AppBallControl_GetStatus(&ballStatus);
                if (ballStatus.state == APP_BALL_CONTROL_OFF) {
                    (void)AppBallControl_RequestTarget(APP_BALL_CONTROL_CENTER_X_PX);
                } else {
                    AppBallControl_RequestStop();
                }
                dirty = true;
            }
#endif

            /* K3 进入题目。 */
            if (edge[BSP_KEY_3]) {
                state = UI_STATE_RUN;
                RobotCore_EnterTask(s_sel);
                Ui_DrawRun();
            } else if (dirty) {
                Ui_DrawMenu();
            }
        } else { /* UI_STATE_RUN */
            /* K4 返回菜单：调 RobotCore_ExitTask 后回菜单界面；否则周期驱动 onLoop。 */
            if (edge[BSP_KEY_4]) {
                RobotCore_ExitTask(s_sel);
                state = UI_STATE_MENU;
                Ui_DrawMenu();
            } else {
                /*
                 * 运行态 K3 转发给题目（题目四用它分两步启动球杆平衡与小车）。
                 * 先派发确认再跑 onLoop，本拍就能看到新状态；未登记 onConfirm
                 * 的题目内部直接忽略。
                 */
                if (edge[BSP_KEY_3]) {
                    RobotCore_ConfirmTask(s_sel);
                }
                RobotCore_LoopTask(s_sel);
            }
        }

        /*
         * 传感器状态栏低频局部刷新（Yaw + 激光距离）：只刷底部一行(128×8)，
         * 每 APP_UI_STATUS_DIVIDER 拍一次，不整屏刷、不阻塞按键响应。
         * 菜单态刷 Y=56，运行态刷 Y=48。
         */
        if (++statusTick >= APP_UI_STATUS_DIVIDER) {
            statusTick = 0U;
            Ui_DrawStatusBar((state == UI_STATE_MENU) ? UI_MENU_STATUS_Y
                                                      : UI_RUN_STATUS_Y);
            if ((state == UI_STATE_RUN) && (s_sel == UI_TASK5_INDEX)) {
                /* 任务五每 300ms 刷新一次循迹与停车流程状态。 */
                OLED_ClearArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
                OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task5_GetUiStatus(), OLED_6X8);
                OLED_UpdateArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
            }
            if ((state == UI_STATE_RUN) && (s_sel == UI_TASK2_INDEX)) {
                /* 任务二每 300ms 刷新一次秒表计时。 */
                OLED_ClearArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
                OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task2_GetUiStatus(), OLED_6X8);
                OLED_UpdateArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
            }
            if ((state == UI_STATE_RUN) && (s_sel == UI_TASK3_INDEX)) {
                /* 任务三每 300ms 刷新回零等待与左右摆球阶段。 */
                OLED_ClearArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
                OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task3_GetUiStatus(), OLED_6X8);
                OLED_UpdateArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
            }
            if ((state == UI_STATE_RUN) && (s_sel == UI_TASK4_INDEX)) {
                /* 任务四每 300ms 刷新一次秒表计时。 */
                OLED_ClearArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
                OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task4_GetUiStatus(), OLED_6X8);
                OLED_UpdateArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
            }
            if ((state == UI_STATE_RUN) && (s_sel == UI_TASK6_INDEX)) {
                /* 任务六每 300ms 刷新一次秒表与循迹/停车流程状态。 */
                OLED_ClearArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
                OLED_ShowString(0, UI_RUN_NAME_Y, (char *)Task6_GetUiStatus(), OLED_6X8);
                OLED_UpdateArea(0, UI_RUN_NAME_Y, 128, UI_MENU_LINE_H);
            }
#if (APP_FEATURE_VISION_LINK != 0U)
            /* 右侧视觉通信面板只在菜单态显示/刷新（运行态整屏归题目自身用）。 */
            if (state == UI_STATE_MENU) {
                Ui_DrawVisionPanel();
            }
#endif
#if (APP_FEATURE_LINE_TRACK != 0U)
            /* 右侧循迹面板同样只在菜单态刷新。 */
            if (state == UI_STATE_MENU) {
                Ui_DrawLinePanel();
            }
#endif
        }

        vTaskDelayUntil(&lastWakeTime, APP_UI_POLL_TICKS);
    }
}

void AppUiTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppUiTask_Entry,
                      "UIMENU",
                      APP_UI_TASK_STACK_WORDS,
                      NULL,
                      APP_UI_TASK_PRIORITY,
                      &s_uiTaskHandle);
    configASSERT(ret == pdPASS);
}
