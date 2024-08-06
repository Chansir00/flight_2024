#include "MyCommon.h"
#include "LED.h"

//-------------------------------------------------
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
                             //default 300ms flash the status
/**************************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	AFIO->MAPR = 0X02000000; //使能4线烧写 释放某些与烧写相关的引脚
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB  , ENABLE);
	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_8 | GPIO_Pin_9;		     //LED12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		     //LED12
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;

	if(SysTick_count - LastTime < LED.FlashTime) return;
	else LastTime = SysTick_count;
	
	switch(LED.status)
	{
		case AlwaysOff:      //常暗   
			bLED_H();
			fLED_H();
			bLED_H();
			hLED_H();
			break;
		case AllFlashLight:				  //全部同时闪烁
			fLED_Toggle();			
			bLED_Toggle();
			hLED_Toggle();			
			aLED_Toggle();		
		  break;
		case AlwaysOn:  //常亮
			
		  bLED_L();
			fLED_L();
			aLED_L();
			hLED_L();
		  break;
		case AlternateFlash: //交替闪烁
			bLED_H();
			fLED_L();
			aLED_H();
			hLED_L();
			LED.status = AllFlashLight;
			break;
		case WARNING:
			fLED_L();
			hLED_L();
			bLED_Toggle();
			LED.FlashTime = 100;
			break;
		case DANGEROURS:
			bLED_L();
			aLED_L();
			fLED_Toggle();
			hLED_Toggle();
			LED.FlashTime = 70;
			break;
		default: LED.status = AlwaysOff;
			break;
	}
}

/**************************END OF FILE*********************************/



