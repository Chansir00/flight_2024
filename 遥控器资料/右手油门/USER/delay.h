#ifndef _DELAY_H
#define _DELAY_H 


#include "stdint.h"

#define	IRQ_1MS	1000
#define	IRQ_2MS	2000
#define	IRQ_3MS	3000
#define	IRQ_4MS	4000

#define	DEFAULT_PERIOD	IRQ_1MS

#define	ITS_PER_MS		1000/DEFAULT_PERIOD   // 每毫秒中断的次数


extern volatile uint32_t TIMIRQCNT;

void CycleCounterInit(void);
void delay_ms(uint16_t nms);
uint32_t GetSysTime_us(void); 
void delay_us(unsigned int i);

#endif
