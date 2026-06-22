#ifndef __DELAY_H
#define __DELAY_H

#include "ti_msp_dl_config.h"

void Delay_init(void);

void Delay_us(unsigned long __us);
void Delay_ms(unsigned long ms);
void Delay_1us(unsigned long __us);
void Delay_1ms(unsigned long ms);

#endif
