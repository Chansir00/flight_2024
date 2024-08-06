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
#include "WP_Ctrl.h"
#include "control_config.h"
/*****************遥控器行程设置**********************/
int16_t Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4;//4个电机输出PWM
int16_t Last_Motor_PWM_1,Last_Motor_PWM_2,Last_Motor_PWM_3,Last_Motor_PWM_4;//上次4个电机输出PWM
uint8_t Controler_High_Mode=1,Last_Controler_High_Mode=1;
uint8_t Controler_Horizontal_Mode=1,Last_Controler_Horizontal_Mode=1;
uint8_t Controler_SDK1_Mode=0,Last_Controler_SDK1_Mode=0;
uint8_t Controler_Land_Mode=1,Last_Controler_Land_Mode=1;
uint8_t Control_Mode_Change=0;
uint16_t High_Hold_Throttle=0;
uint16_t Throttle=0;
Controller_Output Controller;
/***************************************************
函数名: void Controler_Mode_Select(void)
说明:	控制器模式选择函数
入口:	无
出口:	无
备注:	中断任务调度持续运行
****************************************************/
void Controler_Mode_Select()
{
  Last_Controler_High_Mode=Controler_High_Mode;//上次高度控制模式
  Last_Controler_Horizontal_Mode=Controler_Horizontal_Mode;//上次位置控制模式
  Last_Controler_Land_Mode=Controler_Land_Mode;//上次返航模式
	
  if(PPM_Databuf[4]>=(RC_Data.cal[4].max-RC_Data.cal[4].deadband))       Controler_High_Mode=2;//气压计、超神波定高
  else if(PPM_Databuf[4]<=(RC_Data.cal[4].min+RC_Data.cal[4].deadband))  Controler_High_Mode=1;//纯姿态自稳
  
  if(PPM_Databuf[5]>=(RC_Data.cal[5].max-RC_Data.cal[5].deadband))       
	{
		Controler_SDK1_Mode=1;//SDK1模式
	}
  else
	{
		Controler_SDK1_Mode=0;//非SDK1模式
		NCQ_SDK_Reset();
	}

	
  if(PPM_Databuf[6]>=(RC_Data.cal[6].max-RC_Data.cal[6].deadband))            {Controler_Land_Mode=2;}//返航模式}
  else if(PPM_Databuf[6]<=(RC_Data.cal[6].middle+RC_Data.cal[6].deadband))    {Controler_Land_Mode=1;}//非返航模式

  if(PPM_Databuf[6]>=(RC_Data.cal[6].middle-RC_Data.cal[6].deadband)
   &&PPM_Databuf[6]<=(RC_Data.cal[6].middle+RC_Data.cal[6].deadband))//遥控器三段开关处于中位     
  {
    
  }
  else
  {

  }  
	
  if(PPM_Databuf[7]>=(RC_Data.cal[7].max-RC_Data.cal[7].deadband))         Controler_Horizontal_Mode=2;//水平位置控制
  else if(PPM_Databuf[7]<=(RC_Data.cal[7].min+RC_Data.cal[7].deadband))   
	{
		Controler_Horizontal_Mode=1;//姿态自稳控制
    OpticalFlow_SINS_Reset();
    OpticalFlow_Ctrl_Reset();		
	}
	 
  if(Unwanted_Lock_Flag==1)//定高模式解锁后，无任何操作
  {
    Thr_Push_Over_State=Thr_Push_Over_Deadband();
    if(Thr_Push_Over_State==2)//只要向上推过了中位死区，即把允许自动上锁操作
    {
      Unwanted_Lock_Flag=0;
    }
    else
    {
      Take_Off_Reset();//清积分
      //Throttle_Control_Reset();//清积分
    }
  }
  
  if(Controler_Land_Mode!=Last_Controler_Land_Mode)
  {
		Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//将开关拨动瞬间的惯导高度设置为期望高度
    if(Controler_Land_Mode==1)   land_reset();//返航模式切回正常模式 
  }
  
  if(Controler_High_Mode!=Last_Controler_High_Mode)
  {
    if(Controler_High_Mode==2)  {Control_Mode_Change=1;}//自稳切定高，设置基准油门值，悬停高度
    if(Controler_High_Mode==1)  {Control_Mode_Change=1;}//定高切自稳
  }
  
  if(Controler_Horizontal_Mode!=Last_Controler_Horizontal_Mode)//位置通道有切换
  {
    if(Controler_Horizontal_Mode==2)  Control_Mode_Change=2;
    if(Controler_Horizontal_Mode==1)  Control_Mode_Change=2;
  }
  
  
  if(Control_Mode_Change==1)//存在定高模式切换，高度只设置一次
  {
    if(Controler_High_Mode==High_Hold_Mode)//本次为定高模式，即自稳切定高
    {
      High_Hold_Throttle=Throttle_Control;//保存当前油门值，只存一次
      /*******************将当前惯导竖直位置估计作为目标高度***************************/
      Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//将开关拨动瞬间的惯导高度设置为期望高度
    }
    else//本次为自稳模式，即定高切自稳
    {
      //Throttle_Control_Reset();
    }
    Control_Mode_Change=0;//将模式切换位置0,有且仅处理一次
  }
  else if(Control_Mode_Change==2)//存在定点模式切换，悬停位置只设置一次
  {
    Control_Mode_Change=0;//已响应本次定点档位切换
  }
  
}



int16_t Value_Limit(int16_t Min,int16_t Max,int16_t Data)
{
  if(Data>=Max) Data=Max;
  else if(Data<=Min) Data=Min;
  return Data;
}




void Angle_Control_Target(uint8_t mode,float roll_target,float pitch_target,float yaw_target)//角度环节
{
	//姿态角度期望
	Total_Controller.Pitch_Angle_Control.Expect=pitch_target;
	Total_Controller.Roll_Angle_Control.Expect=roll_target;
  //姿态角度反馈
  Total_Controller.Roll_Angle_Control.FeedBack =(Roll-Roll_Offset);
	Total_Controller.Pitch_Angle_Control.FeedBack=(Pitch-Pitch_Offset);
	//运行PID控制器
  PID_Control(&Total_Controller.Roll_Angle_Control);  
	PID_Control(&Total_Controller.Pitch_Angle_Control);
	/***************内环角速度期望****************/
	Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
	Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
  switch(mode)
	{
	  case ROTATE:
		{		
			if(yaw_target==0)//偏航杆置于中位
			{
				if(Total_Controller.Yaw_Angle_Control.Expect==0)//初次切回中
				{
				  Total_Controller.Yaw_Angle_Control.Expect=Yaw;
				}
				Total_Controller.Yaw_Angle_Control.FeedBack=Yaw;//偏航角反馈
				PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control);//偏航角度控制
				Total_Controller.Yaw_Gyro_Control.Expect=Total_Controller.Yaw_Angle_Control.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
			}
			else//波动偏航方向杆后，只进行内环角速度控制
			{
				Total_Controller.Yaw_Angle_Control.Expect=0;//偏航角期望给0,不进行角度控制
				Total_Controller.Yaw_Gyro_Control.Expect=yaw_target;//偏航角速度环期望，直接来源于遥控器打杆量
			}
		}
		break;
	  case AZIMUTH:
	  {
			Total_Controller.Yaw_Angle_Control.Expect=yaw_target;
			Total_Controller.Yaw_Angle_Control.FeedBack=Yaw;//偏航角反馈
			PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control);//偏航角度控制
			Total_Controller.Yaw_Gyro_Control.Expect=Total_Controller.Yaw_Angle_Control.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
		}
		break;	
  }
}




void Yaw_Fault_Check(void)
{
	static uint16_t Yaw_Control_Fault_Cnt=0;
  /*******偏航控制异常情况判断，即偏航控制量很大时，偏航角速度很小，如此时为强外力干扰、已着地等******************************/
  if(ABS(Total_Controller.Yaw_Gyro_Control.Control_OutPut)>Total_Controller.Yaw_Gyro_Control.Control_OutPut_Limit/2//偏航控制输出相对较大
     &&ABS(Yaw_Gyro)<=30.0f)//偏航角速度相对很小
  {
    Yaw_Control_Fault_Cnt++;
    if(Yaw_Control_Fault_Cnt>=500) Yaw_Control_Fault_Cnt=500;
  }
  else Yaw_Control_Fault_Cnt/=2;//不满足，快速削减至0
  
  if(Yaw_Control_Fault_Cnt>=400)//持续5ms*400=2S,特殊处理
  {
    PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);//清空偏航角速度控制的积分
    PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);//清空偏航角控制的积分
    Total_Controller.Yaw_Angle_Control.Expect=Yaw;//将当前偏航角，作为期望偏航角
    Yaw_Control_Fault_Cnt=0;
  }
}

void Gyro_Control(void)//角速度环
{	
	//俯仰、横滚方向姿态内环角速度控制器采用PID控制器
	/***************内环角速度期望****************/
	Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
	Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
	/***************内环角速度反馈****************/
	Total_Controller.Pitch_Gyro_Control.FeedBack=Pitch_Gyro;
	Total_Controller.Roll_Gyro_Control.FeedBack=Roll_Gyro;
	/***************内环角速度控制：微分参数动态调整****************/
	PID_Control_Div_LPF_For_Gyro(&Total_Controller.Pitch_Gyro_Control);
	PID_Control_Div_LPF_For_Gyro(&Total_Controller.Roll_Gyro_Control);
  //偏航角控制
  Total_Controller.Yaw_Gyro_Control.FeedBack=Yaw_Gyro_Earth_Frame;//Yaw_Gyro;
  PID_Control_Div_LPF(&Total_Controller.Yaw_Gyro_Control);
	/*******偏航控制异常处理******************************/
  Yaw_Fault_Check();
}






void Main_Leading_Control(void)
{
/*********************根据遥控器切换档位，飞控进入不同模式****************************/	
	if(Controler_Land_Mode==2)//一键降落模式
	{
		land_run();
		Controller.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		return ;//最高优先级，因为直接return跳出了
	}
	
/************************第五通道控制高度模式************************/		
	if(Controler_High_Mode==1)//高度纯手动模式
	{
		Throttle=Throttle_Control;//油门直接来源于遥控器油门给定
		Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		Controller.yaw_outer_control_output=RC_Data.rc_rpyt[RC_YAW];		
	}
	else
	{		
		if(Controler_SDK1_Mode==1)//用户SDK开发者自动飞行模式：水平+高度控制
		{
			Auto_Flight_Ctrl(SDK1_Mode_Setup);//先判断SDK模式
		}
		else//普通定高、定点控制
		{
/************************第八通道控制水平位置模式************************/		
			if(Controler_Horizontal_Mode==1)//水平自稳
			{
				Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
				Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
				Controller.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];				
			}		
			else//水平光流定点模式
			{
				OpticalFlow_Control_Pure(0);//普通光流模式
				Controller.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
			}
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制		
		}
	}
}




/************姿态环控制器：角度+角速度****************/
void Attitude_Control(void)
{
	Angle_Control_Target(ROTATE,Controller.roll_outer_control_output,
										          Controller.pitch_outer_control_output,
										          Controller.yaw_outer_control_output);//角度控制	
  Gyro_Control();//角速度控制
}


float Active_Para1,Active_Para2;
/***************************************************
函数名: void Total_Control(void)
说明:	总控制器运行，大体分三步：
1、根据遥控器输入、当前状态，给定运行模式（自稳+油门手动、定高+自稳、定高+定点（控速）等）
2、主导上层控制器给定姿态期望，高度控制等
3、自稳（姿态）控制
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Total_Control(void)
{
  /*************主导控制器******************/
  Main_Leading_Control();		
  /*************姿态环控制器*****************/
  Attitude_Control();
}


void GankerPilot_Control(void)
{
  static uint8_t ctrl_cnt=0;
  ctrl_cnt++;  
  /*************控制器模式选择******************/
  if(ctrl_cnt>=8)//改此计数器值，可以调整控制周期
  {  
    Controler_Mode_Select();
    ctrl_cnt=0;
  }
  Total_Control();//总控制器：水平位置+水平速度+姿态（角度+角速度）控制器，高度位置+高度速度+高度加速度控制器	
  Control_Output();//控制量总输出
	Temperature_Ctrl();//IMU恒温控制
}



//#define Thr_Feedforward_Compensation//锂电池电压前馈补偿使能
#ifdef Thr_Feedforward_Compensation
	#define Battery_Cell_Voltage    4.2//1S锂电池充满4.2V
	#define Battery_Cell_Numeber    4//4S锂电池
	#define Battery_Full_Voltage    (Battery_Cell_Voltage*Battery_Cell_Numeber)
#endif

uint16_t Throttle_Output=0;
void Throttle_Angle_Compensate()//油门倾角补偿
{
  float CosPitch_CosRoll=ABS(Cos_Pitch*Cos_Roll);
  float Throttle_Makeup=0;
  float Temp=0;
  if(CosPitch_CosRoll>=0.999999f)  CosPitch_CosRoll=0.999999f;
  if(CosPitch_CosRoll<=0.000001f)  CosPitch_CosRoll=0.000001f;
  if(CosPitch_CosRoll<=0.50f)  CosPitch_CosRoll=0.50f;//Pitch,Roll约等于30度
  if(Throttle>=Thr_Start)//大于起转油门量
  {
    Temp=(uint16_t)(MAX(ABS(100*Pitch),ABS(100*Roll)));
    Temp=constrain_float(9000-Temp,0,3000)/(3000*CosPitch_CosRoll);
    Throttle_Makeup=(Throttle-Thr_Start)*Temp;//油门倾角补偿
    Throttle_Output=(uint16_t)(Thr_Start+Throttle_Makeup);
    Throttle_Output=(uint16_t)(constrain_float(Throttle_Output,Thr_Start,2000));
#ifdef Thr_Feedforward_Compensation 
		Throttle_Output+=(Throttle_Output-Thr_Start)*(Battery_Full_Voltage/Battery_Voltage-1.0f);
#endif
  }
  else Throttle_Output=Throttle;	
}


/**************************************************************
***************************************************************
X型安装方式，电机序号与姿态角关系
                   -
                 Pitch
					3#             1#
					   *          *
-   Roll          *         Roll   +
						 *          *
					2#             4#
				         Pitch
								   +
加速度传感器轴向与载体X、Y、Z同轴，沿轴向原点看，逆时针旋转角度为+
Y Aixs
*
*
*
*
*
*
* * * * * * * *   X Axis
(0)
*******************************************************************
******************************************************************/
uint16_t Idel_Cnt=0;
#define Idel_Transition_Gap 4//怠速递增间隔时间 2*5=10ms
#define Idel_Transition_Period (Thr_Idle-Thr_Min)//怠速启动最大计数器  10ms*100=1s
uint16_t Thr_Idle_Transition_Cnt=0;
void Control_Output()
{
  Throttle_Angle_Compensate();//油门倾角补偿
  landon_earth_check();//着陆条件自检
  if(Controler_State==Unlock_Controler)//解锁
  {
    if(Landon_Earth_Flag==1)//检测到着陆条件
    {
      if(Last_Motor_PWM_1<=Thr_Min
         &&Last_Motor_PWM_2<=Thr_Min
           &&Last_Motor_PWM_3<=Thr_Min
             &&Last_Motor_PWM_4<=Thr_Min
               )//只有上锁后再解锁时才会满足
      {
        //如果上次油门输出值为最低位，进入怠速时，安排过渡过程
        Thr_Idle_Transition_Cnt=Idel_Transition_Period;
      }
      else//其他时刻进入着陆条件
      {
        if(Last_Landon_Earth_Flag==0)//上次为起飞状态，本次为着陆状态，上锁电机
        {
          Controler_State=Lock_Controler;
          Bling_Set(&Light_1,3000,200,0.5,0,GPIO_PORT_P5,GPIO_PIN6,0);
        }
      }
      
      Idel_Cnt++;
      if(Idel_Cnt>=Idel_Transition_Gap)
      {
        if(Thr_Idle_Transition_Cnt>=1)
          Thr_Idle_Transition_Cnt--;
        Idel_Cnt=0;
      }
      Motor_PWM_1=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;//油门怠速
      Motor_PWM_2=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Motor_PWM_3=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Motor_PWM_4=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Take_Off_Reset();//清积分
      OpticalFlow_SINS_Reset();
      OpticalFlow_Ctrl_Reset();
    }
    else  //解锁后不满足着陆条件，默认起飞
    {
      if(Controler_High_Mode==1)//姿态档位
      {
        if(Throttle>=Thr_Fly_Start)//大于起飞油门
        {
            Motor_PWM_1=Int_Sort(
                                 Moter1_Thr_Scale*Throttle_Output
                                   +Moter1_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
            Motor_PWM_2=Int_Sort(
                                 Moter2_Thr_Scale*Throttle_Output
                                   +Moter2_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
            Motor_PWM_3=Int_Sort(
                                 Moter3_Thr_Scale*Throttle_Output
                                   +Moter3_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
            Motor_PWM_4=Int_Sort(
                                 Moter4_Thr_Scale*Throttle_Output
                                   +Moter4_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                     +Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                       +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
            
        }
        else//小于起飞油门
        {
          Motor_PWM_1=Int_Sort(Throttle_Output);
          Motor_PWM_2=Int_Sort(Throttle_Output);
          Motor_PWM_3=Int_Sort(Throttle_Output);
          Motor_PWM_4=Int_Sort(Throttle_Output);
          Take_Off_Reset();//清积分
        }
				
				
        Motor_PWM_1=Value_Limit(Thr_Idle,2500,Motor_PWM_1);//总输出限幅
        Motor_PWM_2=Value_Limit(Thr_Idle,2500,Motor_PWM_2);
        Motor_PWM_3=Value_Limit(Thr_Idle,2500,Motor_PWM_3);
        Motor_PWM_4=Value_Limit(Thr_Idle,2500,Motor_PWM_4);		
				Throttle_Control_Reset();
      }
      else if(Controler_High_Mode==2)//油门托管、定高档位
      {
          Motor_PWM_1=Int_Sort(
                               Moter1_Thr_Scale*Throttle_Output
                                 +Moter1_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter1_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
          Motor_PWM_2=Int_Sort(
                               Moter2_Thr_Scale*Throttle_Output
                                 +Moter2_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter2_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
          Motor_PWM_3=Int_Sort(
                               Moter3_Thr_Scale*Throttle_Output
                                 +Moter3_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter3_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
          Motor_PWM_4=Int_Sort(
                               Moter4_Thr_Scale*Throttle_Output
                                 +Moter4_Roll_Scale*Total_Controller.Roll_Gyro_Control.Control_OutPut
                                   +Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
                                     +Moter4_Yaw_Scale*Total_Controller.Yaw_Gyro_Control.Control_OutPut);
				
        Motor_PWM_1=Value_Limit(Thr_Idle,2500,Motor_PWM_1);//总输出限幅
        Motor_PWM_2=Value_Limit(Thr_Idle,2500,Motor_PWM_2);
        Motor_PWM_3=Value_Limit(Thr_Idle,2500,Motor_PWM_3);
        Motor_PWM_4=Value_Limit(Thr_Idle,2500,Motor_PWM_4);
      }
    }
  }
  else//未解锁，油门置于最低位，停转
  {
    Motor_PWM_1=Thr_Min;
    Motor_PWM_2=Thr_Min;
    Motor_PWM_3=Thr_Min;
    Motor_PWM_4=Thr_Min;
    Take_Off_Reset();//清积分
		OpticalFlow_SINS_Reset();
		OpticalFlow_Ctrl_Reset();
//    Throttle_Control_Reset();
	}
  Last_Motor_PWM_1=Motor_PWM_1;
  Last_Motor_PWM_2=Motor_PWM_2;
  Last_Motor_PWM_3=Motor_PWM_3;
  Last_Motor_PWM_4=Motor_PWM_4;  
  Motor_PWM_1=Value_Limit(0,2500,Motor_PWM_1);//总输出限幅
  Motor_PWM_2=Value_Limit(0,2500,Motor_PWM_2);
  Motor_PWM_3=Value_Limit(0,2500,Motor_PWM_3);
  Motor_PWM_4=Value_Limit(0,2500,Motor_PWM_4);
	PWM_Output(Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4);
}




















/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

