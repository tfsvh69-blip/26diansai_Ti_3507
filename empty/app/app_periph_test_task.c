#include "app_periph_test_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_motor_status.h"
#include "bsp_led.h"

#include "OLED.h"

/*
 * 外设综合验证任务（500ms 周期）：一处驱动多个"次要"外设，避免每个都单开一个任务。
 *   - OLED：整屏刷新，显示运行秒、LED 状态、电机1 运行/方向/速度档（只读 g_motorDiag 快照）。
 *   - LED2/LED3：交替翻转，直观表明本任务在调度。
 *   - 蜂鸣器：当前保持静音（已验证，仅上电自检短响）。
 * 注意：OLED_Update 整屏是软件 I2C 忙等大户（每次 ~45~57ms），是本工程第二大 CPU 消耗，
 * 若要降 CPU 优先改这里（局部刷新/降频），详见 docs/PROJECT_CONTEXT.md 的 CPU 占用预算。
 */

static TaskHandle_t s_periphTestTaskHandle = NULL;

/*
 * 上电外设自检：LED2/LED3 全亮 200ms 后熄灭。
 * 蜂鸣器已验证正常，后续不再触发，保持静音。
 */
static void Periph_SelfTest(void)
{
    BspLed_On(BSP_LED_2);
    BspLed_On(BSP_LED_3);
    vTaskDelay(pdMS_TO_TICKS(200U));

    BspLed_Off(BSP_LED_2);
    BspLed_Off(BSP_LED_3);
}

/*
 * 在 OLED 上刷新一屏综合调试数据，使用 6x8 小字体（每字符 6×8px）。
 * 全屏 128×64 → 每行 21 字符、共 8 行（Y=0/8/16/24/32/40/48/56）。
 * 当前只用前 4 行显示核心信息，后 4 行留空供后续扩展。
 */
static void Periph_DrawScreen(uint32_t sec, uint8_t buzzOn,
                               uint8_t led1On, uint8_t led2On, uint8_t led3On)
{
    /* 第1行(Y=0)：标题 */
    OLED_ShowString(0, 0, "3507 MOTOR1 v1.1", OLED_6X8);

    /*
     * 第2行(Y=8)：运行时间 + LED 状态 + 蜂鸣器（紧凑排版）。
     * T:12345s(42px) L:123(30px) B:~(18px) 总计约 90px。
     * L: 三位依次对应 LED1/LED2/LED3，亮显数字、灭显"-"。
     */
    OLED_ShowString(0, 8, "T:", OLED_6X8);
    OLED_ShowNum(12, 8, sec, 5, OLED_6X8);
    OLED_ShowString(42, 8, "s L:", OLED_6X8);
    OLED_ShowString(66, 8, led1On ? "1" : "-", OLED_6X8);
    OLED_ShowString(72, 8, led2On ? "2" : "-", OLED_6X8);
    OLED_ShowString(78, 8, led3On ? "3" : "-", OLED_6X8);
    OLED_ShowString(84, 8, " B:", OLED_6X8);
    OLED_ShowString(102, 8, buzzOn ? "~" : "_", OLED_6X8);

    /*
     * 第3行(Y=16)：电机1 运行/方向/速度档。
     * 形如 "M1:RUN  FWD L3" 或 "M1:STOP REV L1"。
     */
    OLED_ShowString(0, 16, "M1:", OLED_6X8);
    OLED_ShowString(18, 16, g_motorDiag.running ? "RUN " : "STOP", OLED_6X8);
    OLED_ShowString(48, 16, g_motorDiag.dirForward ? "FWD" : "REV", OLED_6X8);
    OLED_ShowString(72, 16, "R", OLED_6X8);
    OLED_ShowNum(78, 16, g_motorDiag.param, 1, OLED_6X8);

    OLED_Update();
}

static void AppPeriphTestTask_Entry(void *argument)
{
    uint32_t cycle = 0U;
    TickType_t lastWakeTime;

    (void)argument;

    /* OLED 软件 I2C 初始化（GPIO 已在板级初始化配为推挽输出）。 */
    OLED_Init();
    OLED_Clear();

    /* 上电自检：三外设各点一下。 */
    Periph_SelfTest();

    lastWakeTime = xTaskGetTickCount();
    for (;;) {
        uint8_t led1On, led2On, led3On;

        /* LED2/LED3 交替心跳，直观表明本任务在正常调度。 */
        BspLed_Toggle(BSP_LED_2);
        BspLed_Toggle(BSP_LED_3);

        /*
         * LED1 由 LED1 任务独立翻转（300ms），本任务直接读 GPIO DOUT
         * 输出寄存器获取当前电平（DL_GPIO_readPins 读 DIN 不可靠）。
         * DOUT bit25=1 → LED1 亮。
         */
        led1On = ((LED_LED1_PORT->DOUT31_0 & LED_LED1_PIN) != 0U) ? 1U : 0U;

        /*
         * LED2/LED3：Toggle 后 cycle 奇偶与 GPIO 实际电平相位相反
         * （cycle=0 时 Toggle 使 LED 亮，但 (0&1)=0→显示"-"），取反修正。
         */
        led2On = (cycle & 1U) ? 0U : 1U;
        led3On = led2On;

        /* 蜂鸣器已验证正常，保持静音，不再周期通断。 */

        {
            uint32_t tickMs = (uint32_t)xTaskGetTickCount();
            Periph_DrawScreen(tickMs / 1000U, 0U, led1On, led2On, led3On);
        }

        cycle++;
        vTaskDelayUntil(&lastWakeTime, APP_PERIPH_TEST_PERIOD_TICKS);
    }
}

void AppPeriphTestTask_Init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(AppPeriphTestTask_Entry,
                      "PERIPH",
                      APP_PERIPH_TEST_TASK_STACK_WORDS,
                      NULL,
                      APP_PERIPH_TEST_TASK_PRIORITY,
                      &s_periphTestTaskHandle);
    configASSERT(ret == pdPASS);
}
