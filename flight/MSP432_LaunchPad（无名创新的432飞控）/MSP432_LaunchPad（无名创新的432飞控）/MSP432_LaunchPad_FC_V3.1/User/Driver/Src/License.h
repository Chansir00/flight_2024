#ifndef __LICENSE_H_
#define __LICENSE_H_

void System_Init(void);

void Timer32_init(void);
void Test_Period(Testime *Time_Lab);
void Get_Systime(systime *sys);
void Resume_Factory_Setting(void);

extern uint32_t TIME_ISR_CNT;
extern int32_t extern_remain_time;
#endif


