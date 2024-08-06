/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                开源并不等于免费，先驱者的历史已经证明，在国内这个环境下，毫无收益的开源，单靠坊间个人爱好者，自发地参与项
								目完善的方式行不通，好的开源项目需要请专职人员做好售后服务、手把手教学、统计用户反馈需求、在实践中完成对产
								品的一次次完善与迭代升级。经过综合考虑，无名飞控开源代码中，程序仅保留公司正版激活功能代码，版本激活无实际
								功能，属于公司产品常规出厂操作，不影响客户学习，其余代码全部开放给客户学习，客户移植和二次开发代码请保留代
								码版权。
-----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：2号群465082224、1号群540707961（人员已满）
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               修改日期:2021/04/30                    
*               版本：筑梦者PLUS——GankerPilot_V3.0
*               版权所有，盗版必究。
*               Copyright(C) 2019-2025 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               作者，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "control_config.h"
#include "control_althold.h"

Vector3f UAV_Cushion_Stop_Point;




float ALT_VEL_FEEDFORWARD_ENABLE_Output=0;//竖直速度前馈控制器输出;
float ALT_VEL_FEEDFORWARD_ENABLE_Rate=1.0;//竖直速度前馈控制器，APM里面为1、0.45;
float ALT_VEL_FEEDFORWARD_ENABLE_Delta=0;//竖直期望速度变化率;
float Last_Yaw_Vel_Target=0;
float Yaw_Vel_Target=0;

uint16_t  Deadband=200;//油门中位死区
/*************以下为默认值，校准遥控器后会自动赋值**********************/
uint16_t  Deadzone_Min=400;
uint16_t  Deadzone_Max=600;
uint16_t  Thr_Top=1000;//油门最大上行程
uint16_t  Thr_Buttom=0;//油门最大下行程
uint16_t  Thr_Middle=500;//油门最大下行程
uint8_t Thr_Push_Over_State=0;
uint8_t Thr_Push_Over_Deadband(void)
{
  static uint8_t thr_now=0,thr_last=0;
  static uint8_t push_flag=0;
  thr_last=thr_now;
  if(TRPY[0]<=Deadzone_Min) thr_now=0;
  else if(TRPY[0]<=Deadzone_Max) thr_now=1;
  else thr_now=2;  
  if(thr_now==0&&thr_last==1)       return push_flag=0;//向下推过中位死区
  else if(thr_now==1&&thr_last==0)  return push_flag=1;//向上推到中位死区
  else if(thr_now==2&&thr_last==1)  return push_flag=2;//向上推过中位死区
  else if(thr_now==1&&thr_last==2)  return push_flag=3;//向下推到中位死区
  else push_flag=0;
  return push_flag;
}

void Thr_Scale_Set(Vector_RC *rc_date)
{
  Thr_Buttom=0;
  Thr_Top=(uint16_t)(rc_date[2].max-rc_date[2].min);
  Thr_Middle=(uint16_t)((rc_date[2].max-rc_date[2].min)/2);
  //Deadband=rc_date[2].deadband;//油门中位死区单独设置
  Deadzone_Min=Thr_Middle-Deadband/2;
  Deadzone_Max=Thr_Middle+Deadband/2;
}



void Flight_Alt_Hold_Control(uint8_t mode,float target_alt,float target_vel)
{	 
	 static uint8_t last_mode=0;	 
	 static systime althold_dt;
	 static uint8_t move_flag=0;
	 uint8_t alt_fixed_flag=0;
	 Get_Systime(&althold_dt);
	 High_Hold_Throttle=Thr_Hover_Default;
	 //如果本函数运行周期大于控制周期10倍
	 if(0.001f*althold_dt.period>=10*Imu_Sampling_Dt)
	 {
		//情况1:初次或者从其它模式切入本模式
		//情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
		alt_fixed_flag=1;
	 }
	 //高度控制从自动模式切到手动模式	 
	 if(last_mode!=mode)  alt_fixed_flag=1;
	 last_mode=mode;	
   switch(mode)
	 {
		 case ALTHOLD_MANUAL_CTRL:
		 {		 
			 //1、高度位置控制器
			 if(RC_Data.rc_rpyt[RC_THR]==0)//油门杆位于中位死区内，进行高度保持
			 {
					//高度位置环输出给定速度期望
					if(Total_Controller.High_Position_Control.Expect==0||alt_fixed_flag==1)//油门回中后，将回中时刻的高度设置为期望高度，仅设置1次
					{
						Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];	//油门回中后，更新高度期望
					}
					Total_Controller.High_Position_Control.FeedBack=NamelessQuad.Position[_YAW];	//高度位置反馈
					PID_Control(&Total_Controller.High_Position_Control);									//海拔高度位置控制器
					
					Total_Controller.High_Speed_Control.Expect=Total_Controller.High_Position_Control.Control_OutPut;
					move_flag=0;
								 
#ifdef ALT_VEL_FEEDFORWARD_ENABLE
					float delta=Total_Controller.High_Speed_Control.PID_Controller_Dt.Time_Delta/1000.0f;
					Yaw_Vel_Target=Total_Controller.High_Speed_Control.Expect;//本次速度期望
					ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Yaw_Vel_Target-Last_Yaw_Vel_Target)/(1*delta);//速度期望变化率
					ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
					Last_Yaw_Vel_Target=Yaw_Vel_Target;//上次速度期望
#endif
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]>0)//给定上升速度期望
			 {
			  	//油门杆上推、给定速度期望
			 	 Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大上升速度300cm/s
				 Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
				 move_flag=1;
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]<0)//给定下降速度期望
			 {
				//油门杆下推、给定速度期望
				 Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大下降速度200cm/s
				 Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
				 move_flag=2;
			 }
			 
				//2、高度速度控制器
				Total_Controller.High_Speed_Control.FeedBack=NamelessQuad.Speed[_YAW];//惯导速度估计给到速度反馈
				PID_Control_Div_LPF(&Total_Controller.High_Speed_Control);//海拔高度速度控制
				/*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
				if(move_flag==1)//在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
				{
					Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
																																						 -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
				}
				else if(move_flag==2)
				{
					Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
																																						 -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
				}
				//3、高度加速度控制器
				Total_Controller.High_Acce_Control.Expect=Total_Controller.High_Speed_Control.Control_OutPut;//加速度期望
#ifdef ALT_VEL_FEEDFORWARD_ENABLE	 
				Total_Controller.High_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//加上前馈控制器输出
#endif	 
				Total_Controller.High_Acce_Control.FeedBack=Acceleration_Feedback[_YAW];//加速度反馈
				Alt_Accel_Control(&Total_Controller.High_Acce_Control);//海拔高度加速度控制
				/**************************************
				加速度环前馈补偿，引用时请注明出处
				悬停油门 = 加速度环积分值 + 基准悬停油门
				此时输出力 F = mg
				当需要输出a的加速度时，输出力 F1=m(g+a)
				F1/F = 1 + a/g
				因此此时应输出：悬停油门*(1 + a/g)
				**************************************/
				Total_Controller.High_Acce_Control.Control_OutPut+=(Thr_Hover_Default+Total_Controller.High_Acce_Control.Integrate-Thr_Start)
																													*(Total_Controller.High_Acce_Control.Expect/980);
			}
			 break;
			
			case ALTHOLD_AUTO_POS_CTRL:
			{		 
			 //1、高度位置控制器
			 if(RC_Data.rc_rpyt[RC_THR]==0)//油门杆位于中位死区内，进行高度保持
			 {
					//高度位置环输出给定速度期望
					Total_Controller.High_Position_Control.Expect=target_alt;	//油门处于回中后，更新高度期望
					Total_Controller.High_Position_Control.FeedBack=NamelessQuad.Position[_YAW];	//高度位置反馈
					PID_Control(&Total_Controller.High_Position_Control);									//海拔高度位置控制器
					
					Total_Controller.High_Speed_Control.Expect=Total_Controller.High_Position_Control.Control_OutPut;
					move_flag=0;
#ifdef ALT_VEL_FEEDFORWARD_ENABLE
					float delta=Total_Controller.High_Speed_Control.PID_Controller_Dt.Time_Delta/1000.0f;
					Yaw_Vel_Target=Total_Controller.High_Speed_Control.Expect;//本次速度期望
					ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Yaw_Vel_Target-Last_Yaw_Vel_Target)/(1*delta);//速度期望变化率
					ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
					Last_Yaw_Vel_Target=Yaw_Vel_Target;//上次速度期望
#endif
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]>0)//给定上升速度期望
			 {
			  	//油门杆上推、给定速度期望
			 	 Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大上升速度300cm/s
				 Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
				 move_flag=1;
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]<0)//给定下降速度期望
			 {
				//油门杆下推、给定速度期望
				 Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大下降速度200cm/s
				 Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
				 move_flag=2;
			 }
			 
				//2、高度速度控制器
				Total_Controller.High_Speed_Control.FeedBack=NamelessQuad.Speed[_YAW];//惯导速度估计给到速度反馈
				PID_Control_Div_LPF(&Total_Controller.High_Speed_Control);//海拔高度速度控制
				/*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
				if(move_flag==1)//在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
				{
					Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
																																						 -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
				}
				else if(move_flag==2)
				{
					Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
																																						 -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
				}
				//3、高度加速度控制器
				Total_Controller.High_Acce_Control.Expect=Total_Controller.High_Speed_Control.Control_OutPut;//加速度期望
#ifdef ALT_VEL_FEEDFORWARD_ENABLE	 
				Total_Controller.High_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//加上前馈控制器输出
#endif	 
				Total_Controller.High_Acce_Control.FeedBack=Acceleration_Feedback[_YAW];//加速度反馈
				Alt_Accel_Control(&Total_Controller.High_Acce_Control);//海拔高度加速度控制
				/**************************************
				加速度环前馈补偿，引用时请注明出处
				悬停油门 = 加速度环积分值 + 基准悬停油门
				此时输出力 F = mg
				当需要输出a的加速度时，输出力 F1=m(g+a)
				F1/F = 1 + a/g
				因此此时应输出：悬停油门*(1 + a/g)
				**************************************/
				Total_Controller.High_Acce_Control.Control_OutPut+=(Thr_Hover_Default+Total_Controller.High_Acce_Control.Integrate-Thr_Start)
																													*(Total_Controller.High_Acce_Control.Expect/980);
			}
			 break;
			case ALTHOLD_AUTO_VEL_CTRL:
			{		 
			 //1、高度位置控制器
			 if(RC_Data.rc_rpyt[RC_THR]==0)//油门杆位于中位死区内，进行高度保持
			 {
					//高度位置环输出给定速度期望
					Total_Controller.High_Position_Control.Expect=0;	//油门处于回中后，更新高度期望					
					Total_Controller.High_Speed_Control.Expect=target_vel;
					move_flag=0;
#ifdef ALT_VEL_FEEDFORWARD_ENABLE
					float delta=Total_Controller.High_Speed_Control.PID_Controller_Dt.Time_Delta/1000.0f;
					Yaw_Vel_Target=Total_Controller.High_Speed_Control.Expect;//本次速度期望
					ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Yaw_Vel_Target-Last_Yaw_Vel_Target)/(1*delta);//速度期望变化率
					ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
					Last_Yaw_Vel_Target=Yaw_Vel_Target;//上次速度期望
#endif
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]>0)//给定上升速度期望
			 {
			  	//油门杆上推、给定速度期望
			 	 Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大上升速度300cm/s
				 Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
				 move_flag=1;
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]<0)//给定下降速度期望
			 {
				//油门杆下推、给定速度期望
				 Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大下降速度200cm/s
				 Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
				 move_flag=2;
			 }
			 
				//2、高度速度控制器
				Total_Controller.High_Speed_Control.FeedBack=NamelessQuad.Speed[_YAW];//惯导速度估计给到速度反馈
				PID_Control_Div_LPF(&Total_Controller.High_Speed_Control);//海拔高度速度控制
				/*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
				if(move_flag==1)//在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
				{
					Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
																																						 -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
				}
				else if(move_flag==2)
				{
					Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
																																						 -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
				}
				//3、高度加速度控制器
				Total_Controller.High_Acce_Control.Expect=Total_Controller.High_Speed_Control.Control_OutPut;//加速度期望
#ifdef ALT_VEL_FEEDFORWARD_ENABLE	 
				Total_Controller.High_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//加上前馈控制器输出
#endif	 
				Total_Controller.High_Acce_Control.FeedBack=Acceleration_Feedback[_YAW];//加速度反馈
				Alt_Accel_Control(&Total_Controller.High_Acce_Control);//海拔高度加速度控制
				/**************************************
				加速度环前馈补偿，引用时请注明出处
				悬停油门 = 加速度环积分值 + 基准悬停油门
				此时输出力 F = mg
				当需要输出a的加速度时，输出力 F1=m(g+a)
				F1/F = 1 + a/g
				因此此时应输出：悬停油门*(1 + a/g)
				**************************************/
				Total_Controller.High_Acce_Control.Control_OutPut+=(Thr_Hover_Default+Total_Controller.High_Acce_Control.Integrate-Thr_Start)
																													*(Total_Controller.High_Acce_Control.Expect/980);
			}
			break;
	}	
	 Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.High_Acce_Control.Control_OutPut);//油门来源于高度速度控制器输出
	 Alt_Hold_Output_LPF;
}





void ncq_control_althold()
{
  static uint8_t move_flag=0;
  static systime althold_dt;
  uint8_t alt_fixed_flag=0;
  Get_Systime(&althold_dt);
  //如果本函数运行周期大于控制周期10倍
  if(0.001f*althold_dt.period>=10*Imu_Sampling_Dt)
  {
 	 //情况1:初次或者从其它模式切入本模式
	 //情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
	 alt_fixed_flag=1;
  }
	 
  Thr_Scale_Set(&RC_Data.cal[2]);
  High_Hold_Throttle=Thr_Hover_Default;
  /*高度控制器第1步*/
  /********
      **
      **
      **
      **
      **
  ********/
  /*******************************高度控制器开始****************************************/
  /****************定高：高度位置环+速度环+加速度环，控制周期分别为8ms、4ms、4ms*******************/
  if(RC_Data.rc_rpyt[RC_THR]==0)
  {
    //高度位置环输出给定速度期望
    if(Total_Controller.High_Position_Control.Expect==0||alt_fixed_flag==1)//油门回中后，只设置一次
    {
      Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//油门回中后，更新高度期望
    }
      Total_Controller.High_Position_Control.FeedBack=NamelessQuad.Position[_YAW];//反馈
      PID_Control(&Total_Controller.High_Position_Control);//海拔高度位置控制器
      //内环速度期望
      Total_Controller.High_Speed_Control.Expect=Total_Controller.High_Position_Control.Control_OutPut;
			move_flag=0;
#ifdef ALT_VEL_FEEDFORWARD_ENABLE  //速度前馈控制器
      Yaw_Vel_Target=Total_Controller.High_Speed_Control.Expect;//本次速度期望
      ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Yaw_Vel_Target-Last_Yaw_Vel_Target)/(1*Delta);//速度期望变化率
      ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
      Last_Yaw_Vel_Target=Yaw_Vel_Target;//上次速度期望
#endif   
	}
  else if(RC_Data.rc_rpyt[RC_THR]>0)//给定上升速度期望
  {
    //油门杆上推、给定速度期望
    Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大上升速度300cm/s
    Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
    move_flag=1;
  }
  else if(RC_Data.rc_rpyt[RC_THR]<0)//给定下降速度期望
  {
    //油门杆下推、给定速度期望
    Total_Controller.High_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//最大下降速度200cm/s
    Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
    move_flag=2;
  }
 
  /*高度控制器第2步*/
  /********
					*
					*
   * * * *
  *
  *
  ********/
  
  /*******************************竖直速度控制器开始*******************************************************************/
    Total_Controller.High_Speed_Control.FeedBack=NamelessQuad.Speed[_YAW];//惯导速度估计给速度反馈
		Alt_Accel_Control(&Total_Controller.High_Speed_Control);//海拔高度速度控制
	  //ADRC_TD_Control(&ADRC_Alt_Vel_Controller,Total_Controller.High_Speed_Control.Expect,Total_Controller.High_Speed_Control.FeedBack);
	  //Total_Controller.High_Speed_Control.Control_OutPut=ADRC_Alt_Vel_Controller.u;
  /*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/ 
  if(move_flag==0)
	{
    Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
                                                                       -SDK_Mild_Acceleration_Max,SDK_Mild_Acceleration_Max);  
	}		
  else if(move_flag==1)//在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
  {
    Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
                                                                       -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
  }
  else if(move_flag==2)
  {
    Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
                                                                       -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
  }  
  /*******************************竖直速度控制器结束******************************************************************/
  
  /*高度控制器第3步*/
  /********
         **
         **
      **    
         **
         **
  ********/
  
  /*******************************竖直加速度控制器开始******************************************************************/
  Total_Controller.High_Acce_Control.Expect=Total_Controller.High_Speed_Control.Control_OutPut;//加速度期望
#ifdef ALT_VEL_FEEDFORWARD_ENABLE  //速度前馈控制器
  Total_Controller.High_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//加上前馈控制器输出
#endif
  Total_Controller.High_Acce_Control.FeedBack=Acceleration_Feedback[_YAW];//加速度反馈
	Alt_Accel_Control(&Total_Controller.High_Acce_Control);//海拔高度加速度控制
  /**************************************
  加速度环前馈补偿，引用时请注明出处
  悬停油门 = 加速度环积分值 + 基准悬停油门
  此时输出力 F = mg
  当需要输出a的加速度时，输出力 F1=m(g+a)
  F1/F = 1 + a/g
  因此此时应输出：悬停油门*(1 + a/g)
  **************************************/
  Total_Controller.High_Acce_Control.Control_OutPut+=(High_Hold_Throttle+Total_Controller.High_Acce_Control.Integrate-Thr_Start)*
    (Total_Controller.High_Acce_Control.Expect/980);
  /*******************************竖直加速度控制器结束******************************************************************/
  if(YAW_Pos_Control_Accel_Disable==0)
  {
    Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.High_Acce_Control.Control_OutPut);//油门来源于高度加速度控制器输出
  }
  else
  {
    Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.High_Speed_Control.Control_OutPut);//油门来源于高度速度控制器输出
  }
	
	Alt_Hold_Output_LPF;	
  /*****************************************高度控制器结束，给定油门控制量***********************************************************/
}


/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



