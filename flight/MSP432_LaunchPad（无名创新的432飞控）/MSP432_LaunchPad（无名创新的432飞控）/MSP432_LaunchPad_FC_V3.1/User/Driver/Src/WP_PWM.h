#ifndef __WP_PWM_H__
#define __WP_PWM_H__

#define  MAX_PWM 2500 //4.8Khz    


void Init_PWM(void);
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4);

void EPWM_GPIO_Init(void);
void Init_EPWM(void);	
void EPWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4);
#endif


