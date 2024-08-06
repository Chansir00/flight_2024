#include "MyCommon.h"
#include "Remote.h"
/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
	
uint8_t RC_rxData[32];
void remote_unlock(void);	

strRemoteConnect RemoteConnect;
void RC_Analy(void)  
{
	static uint16_t cnt;
	if(RemoteConnect.ConnectState == 1) RemoteConnect.ConnectTimeCnt++;
	
	//Receive  and check RC data 
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{ 
		RemoteConnect.ConnectState = 1;
		RemoteConnect.ConnectTimeCnt= 0 ;	
		uint8_t i;
		uint8_t CheckSum=0;
		cnt = 0;
		for(i=0;i<31;i++)
		{
			CheckSum +=  RC_rxData[i];
		}
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //如果接收到的遥控数据正确
		{
			Remote.roll		= ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];		//通道1
			Remote.pitch	= ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];		//通道2
			Remote.thr		= ((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];		//通道3
			Remote.yaw		= ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];		//通道4
			Remote.AUX1		= ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];		//通道5  左上角按键都属于通道5  
			Remote.AUX2		= ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];		//通道6  右上角按键都属于通道6 
			Remote.AUX3		= ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];		//通道7  左下边按键都属于通道7 
			Remote.AUX4		= ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];		//通道8  右下边按键都属于通道6  
			LIMIT(Remote.AUX2	,1000,2000);		
			LIMIT(Remote.roll	,1000,2000);
			LIMIT(Remote.pitch	,1000,2000);
			LIMIT(Remote.thr	,1000,2000);
			LIMIT(Remote.yaw	,1000,2000);
			LIMIT(Remote.AUX1	,1000,2000);
			LIMIT(Remote.AUX2	,1000,2000);
			LIMIT(Remote.AUX1	,1000,2000);

			const float roll_pitch_ratio = 0.04f;
//				const float yaw_ratio =  0.0015f;    
			pidPitch.desired	= -(Remote.pitch-1500)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值
			pidRoll.desired		= -(Remote.roll-1500)*roll_pitch_ratio;
			if(Remote.yaw>1820)
			{
				pidYaw.desired += 0.75f;	
			}
			else if(Remote.yaw <1180)
			{
				pidYaw.desired -= 0.75f;	
			}						
			remote_unlock();	
		}
  }
}
uint8_t status = WAITING_1;
/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
void remote_unlock(void)
{
//volatile static uint8_t status = WAITING_1;
	static uint16_t cnt = 0;
	
	//意外断开连接时，如果还在遥控状态，则退出遥控状态，转入待机状态
	if((status == PROCESS_31) && (RemoteConnect.ConnectState == 0)) status = WAITING_1;
	
	if(Remote.thr<1200 &&Remote.yaw<1200)                         //油门遥杆左下角锁定
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1:					//等待解锁
						{
							if(Remote.thr<1150) status = WAITING_2;         //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁		
						}
						break;
		case WAITING_2:					//检测油门杆上拉
						{
							if(Remote.thr>1800)          
							{		
								cnt++;		
								if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
								{	
									cnt=0;
									status = WAITING_3;
								}
							}	
						}	
						break;
		case WAITING_3:					//检测油门杆下拉，启动解锁
						{
							if(Remote.thr<1150) 
							{
								ALL_flag.unlock = 1;	//设置解锁标志
								LED.status = AlwaysOn;	//设置LED灯闪烁状态
								status = PROCESS_31;								
							}								
						}			
						break;		
		case PROCESS_31:				//进入解锁状态
						{
							if(Remote.thr<1020)
							{
								if(cnt++ > 20000)							// 油门遥杆处于最低60S自动上锁
								{								
									status = EXIT_255;								
								}
							}
							else if(!ALL_flag.unlock) status = EXIT_255;	//Other conditions lock 
							else cnt = 0;
						}
						break;
		case EXIT_255:					//进入锁定
						{
							cnt = 0;							
							LED.status = AllFlashLight;	                                 //exit
							LED.FlashTime = 100; //100*3ms		
							ALL_flag.unlock = 0;							
							status = WAITING_1;
						}
						break;
		default: status = EXIT_255;break;
	}
}
/***********************END OF FILE*************************************/







