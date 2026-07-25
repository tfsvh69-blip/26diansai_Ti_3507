#include "app_ui_task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_imu_uart_task.h"
#include "app_robot_core.h"
#include "ball_parser.h"
#include "bsp_key.h"
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
 *   K1 上移   K2 下移   K3 确认进入   K4 返回菜单
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
 * 右侧小球检测面板（利用菜单空出的右半屏）。启用 APP_FEATURE_BALL_VISION 时：
 *   - 菜单文字与选中项高亮收窄到左半（宽 UI_MENU_LEFT_W），不侵入右侧面板；
 *   - 右半自 x=UI_BALL_X 起显示 "BALL" 头 + F/n/x/y 四行文字，字段区低频局部刷。
 * 关闭该功能时菜单恢复整行(128)高亮与整宽标题，右半留空。
 */
#if (APP_FEATURE_BALL_VISION != 0U)
#define UI_MENU_LEFT_W    (60)     /* 菜单占左半宽度（右半让给球面板） */
#define UI_BALL_X         (66)     /* 右侧球面板左边界 x */
#define UI_BALL_W         (62)     /* 右侧球面板宽度（128-66） */
#define UI_BALL_FIELD_Y   (8)      /* 字段区起始 y（F/n/x/y 四行） */
#define UI_BALL_FIELD_H   (32)     /* 字段区高度：y=8..39，四行 */
#else
#define UI_MENU_LEFT_W    (128)    /* 未启用球面板：菜单高亮整行 */
#endif

/* 运行态布局：标题(0)、大字题号(16)、题名(40)、状态栏(48)、返回提示(56)。 */
#define UI_RUN_NAME_Y     (40)
#define UI_RUN_STATUS_Y   (48)    /* 运行态：传感器状态栏 */
#define UI_RUN_HINT_Y     (56)

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
 * 组装传感器状态栏文本到 buf（需 >= 24 字节）："Y:<yaw> D:<dist>mm"，用于一眼判断
 * 陀螺仪与激光测距是否在工作：
 *   Yaw  IMU 就绪显示带 1 位小数的度数(如 -179.9)，未就绪显示 "---"；
 *   Dist 激光收到有效帧显示 mm 数，否则显示 "---"。
 * 数据都取线程安全快照（Yaw 走 IMU 任务 getter，距离走 laser 模块 GetLatest）。
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
    char buf[24];

    Ui_FormatStatus(buf);
    OLED_ClearArea(0, y, 128, UI_MENU_LINE_H);
    OLED_ShowString(0, y, buf, OLED_6X8);
    OLED_UpdateArea(0, y, 128, UI_MENU_LINE_H);
}

#if (APP_FEATURE_BALL_VISION != 0U)
/*
 * 绘制右侧球检测字段 F/n/x/y（不做刷屏推送，交给调用方）。
 * 数据取 BallParser 线程安全快照：
 *   valid=false（还没收到任何合法帧）时四行全显 "?/---"；
 *   found=0（收到帧但没检测到球）时坐标显 "---"，n 仍显真实计数（可能为 0）。
 */
static void Ui_DrawBallFields(void)
{
    BallData_t ball;
    char       buf[12];
    uint32_t   n;
    bool       valid = BallParser_GetLatest(&ball);

    /* 行1：检测标志。 */
    OLED_ShowString(UI_BALL_X, UI_BALL_FIELD_Y,
        valid ? (ball.found ? "F:YES" : "F:no ") : "F: ? ", OLED_6X8);

    /* 行2：球总数 n。 */
    buf[0] = 'n'; buf[1] = ':';
    if (valid) {
        n = 2U + Ui_U32ToStr(&buf[2], (uint32_t)ball.count);
    } else {
        buf[2] = '-'; buf[3] = '-'; buf[4] = '-'; n = 5U;
    }
    buf[n] = '\0';
    OLED_ShowString(UI_BALL_X, UI_BALL_FIELD_Y + 8, buf, OLED_6X8);

    /* 行3：主目标 x 像素（未检测到球时坐标无意义，显 ---）。 */
    buf[0] = 'x'; buf[1] = ':';
    if (valid && ball.found) {
        n = 2U + Ui_U32ToStr(&buf[2], (uint32_t)ball.x);
    } else {
        buf[2] = '-'; buf[3] = '-'; buf[4] = '-'; n = 5U;
    }
    buf[n] = '\0';
    OLED_ShowString(UI_BALL_X, UI_BALL_FIELD_Y + 16, buf, OLED_6X8);

    /* 行4：主目标 y 像素。 */
    buf[0] = 'y'; buf[1] = ':';
    if (valid && ball.found) {
        n = 2U + Ui_U32ToStr(&buf[2], (uint32_t)ball.y);
    } else {
        buf[2] = '-'; buf[3] = '-'; buf[4] = '-'; n = 5U;
    }
    buf[n] = '\0';
    OLED_ShowString(UI_BALL_X, UI_BALL_FIELD_Y + 24, buf, OLED_6X8);
}

/*
 * 局部刷新右侧球字段区：清区 → 重画 F/n/x/y → 只推送该矩形。
 * 与底部状态栏同频（APP_UI_STATUS_DIVIDER），面积小、频率低，几乎不占 CPU，
 * 不打断按键响应，也不动菜单左半与 "BALL" 头。仅菜单态调用。
 */
static void Ui_DrawBallPanel(void)
{
    OLED_ClearArea(UI_BALL_X, UI_BALL_FIELD_Y, UI_BALL_W, UI_BALL_FIELD_H);
    Ui_DrawBallFields();
    OLED_UpdateArea(UI_BALL_X, UI_BALL_FIELD_Y, UI_BALL_W, UI_BALL_FIELD_H);
}
#endif /* APP_FEATURE_BALL_VISION */

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
#if (APP_FEATURE_BALL_VISION != 0U)
    /* 标题收窄到左半，给右侧球面板让位。 */
    OLED_ShowString(0, UI_MENU_TITLE_Y, "TASKS", OLED_6X8);
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

#if (APP_FEATURE_BALL_VISION != 0U)
    /* 右侧球面板：竖分隔线 + "BALL" 头 + 字段（字段之后由主循环低频局部刷更新）。 */
    OLED_DrawLine(UI_BALL_X - 3, UI_MENU_FIRST_Y, UI_BALL_X - 3, 54);
    OLED_ShowString(UI_BALL_X, UI_MENU_TITLE_Y, "BALL", OLED_6X8);
    Ui_DrawBallFields();
#endif

    /* 底行：传感器状态栏（Yaw + 激光距离）。整屏刷时写入当前值，之后由主循环低频局部刷更新。 */
    {
        char buf[24];
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

    /* 题名。 */
    OLED_ShowString(0, UI_RUN_NAME_Y, (char *)RobotCore_GetTaskName(s_sel), OLED_6X8);

    /* 传感器状态栏（写入当前值，之后由主循环低频局部刷更新）。 */
    {
        char buf[24];
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

        /* 每键做一次去抖后的按下沿检测（30ms 轮询本身即去抖窗口）。 */
        for (i = 0U; i < (uint32_t)BSP_KEY_COUNT; i++) {
            bool now = BspKey_IsPressed((BspKeyId_t)i);
            edge[i]  = (now && !prev[i]);
            prev[i]  = now;
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

            /* K3 确认进入运行态：调 RobotCore_EnterTask 后切运行界面。K4 菜单态无动作。 */
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
#if (APP_FEATURE_BALL_VISION != 0U)
            /* 右侧球面板只在菜单态显示/刷新（运行态整屏归题目自身用）。 */
            if (state == UI_STATE_MENU) {
                Ui_DrawBallPanel();
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
