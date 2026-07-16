#ifndef BSP_LED_H
#define BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_LED_1 = 0,   /* PB25 */
    BSP_LED_2,       /* PA7  */
    BSP_LED_3,       /* PB12 */
    BSP_LED_COUNT,
} BspLedId_t;

void BspLed_Init(void);
void BspLed_On(BspLedId_t led);
void BspLed_Off(BspLedId_t led);
void BspLed_Toggle(BspLedId_t led);

#ifdef __cplusplus
}
#endif

#endif
