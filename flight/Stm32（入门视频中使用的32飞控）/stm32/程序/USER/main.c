
#include "MyCommon.h"

void ALL_Init(void);
	
//特别声明：请在无人空旷的地带或者室内进行飞行。遇到非常紧急的情况，可紧急关闭遥控。
int main(void)
{		
	CycleCounterInit();  								//得到系统每个us的系统CLK个数，为以后延时函数，和得到精准的当前执行时间使用
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//2个bit的抢占优先级，2个bit的子优先级
	SysTick_Config(SystemCoreClock / 1000);				//系统滴答时钟

	ALL_Init();				//系统所有硬件的初始化

	while(1)
	{
		ANTO_polling();		//匿名上位机
		PilotLED();			//LED刷新
		if(ALL_flag.unlock == 1)
		{
			if((fabs(Angle.pitch) > 70)||(fabs(Angle.roll) > 70))		//飞行器自讨过度倾斜就停止运行
			{
				RemoteConnect.RemoteSignalLost	= 0;
				RemoteConnect.RemSigLostTimeCnt = 0;
				RemoteConnect.ConnectTimeCnt	= 0;
				RemoteConnect.ConnectState		= 0;
				ALL_flag.unlock = 0; 
				LED.FlashTime = 100;
				LED.status = AllFlashLight;
				NRF24L01_init();	
				TIM2_PWM_Config();		//4路PWM初始化
				TIM3_PWM_Config();		//4路PWM初始化				
			}
		}
		if((RemoteConnect.ConnectTimeCnt > 200) && (RemoteConnect.ConnectState == 1))		//连接状态下，如果遥控失联超300ms
		{
			//如果3秒没收到遥控数据，则判断遥控信号丢失，飞控在任何时候停止飞行，避免伤人。
			//意外情况，使用者可紧急关闭遥控电源，飞行器会在3秒后立即关闭，避免伤人。
			//立即关闭遥控，如果在飞行中会直接掉落，可能会损坏飞行器。
			RemoteConnect.RemoteSignalLost	= 1;
			if(Remote.thr >= 1350) Remote.thr = 1350;
			else Remote.thr = 1000;			
		}		
	}
}

void TIM4_IRQHandler(void)   //TIM4中断3ms
{	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		static uint8_t cnt_3ms = 0;
		static uint8_t cnt_6ms = 0;	 
		SysTick_count++;
		cnt_3ms++;
		cnt_6ms++;
		if(RemoteConnect.RemoteSignalLost == 1) 
		{
			RemoteConnect.RemSigLostTimeCnt++;		//信号丢失，计丢失时间
			if(RemoteConnect.RemSigLostTimeCnt > 1200)
			{
				RemoteConnect.RemoteSignalLost	= 0;
				RemoteConnect.RemSigLostTimeCnt = 0;
				RemoteConnect.ConnectTimeCnt	= 0;
				RemoteConnect.ConnectState		= 0;
				ALL_flag.unlock = 0;
				LED.FlashTime = 300;
				LED.status = AllFlashLight;
				NRF24L01_init();	
				TIM2_PWM_Config();		//4路PWM初始化
				TIM3_PWM_Config();		//4路PWM初始化	
			}	
		}
		else RemoteConnect.RemSigLostTimeCnt = 0;
		if(cnt_3ms == 1)  //3ms更新一次
		{
			cnt_3ms = 0;
			MpuGetData();
			RC_Analy();			
			FlightPidControl(0.003f);
			MotorControl();
		}		
		if(cnt_6ms == 2) //6ms更新一次
		{
			cnt_6ms = 0;
			GetAngle(&MPU6050,&Angle,0.00626f);
		}		

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update );  //清除TIMx的中断待处理位:TIM 中断源 
	}
}








