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
#include "control_land.h"

Vector2_Nav land_pos_target,land_pos_now,land_home_pos;
float target_yaw_rate=0,target_yaw_alt=0;
bool land_althold(float taret_climb_rate,float target_climb_alt)
{
  static uint8_t move_flag=0;
  bool handmove_flag=FALSE;
  float step_mapping=0,scale_mapping=0;
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
  if(target_climb_alt==0)
  {
    Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//一直更新高度期望
  }
  else
  {
    Total_Controller.High_Position_Control.Expect=target_climb_alt;//更新高度期望
  }
  if(TRPY[0]>=Deadzone_Min
     &&TRPY[0]<=Deadzone_Max)
  {
    //高度位置环输出给定速度期望
		Total_Controller.High_Position_Control.FeedBack=NamelessQuad.Position[_YAW];//反馈
		PID_Control(&Total_Controller.High_Position_Control);//海拔高度位置控制器
		//内环速度期望
		if(target_climb_alt==0) Yaw_Vel_Target=taret_climb_rate;//本次速度期望来源于外部直接给定
		else Yaw_Vel_Target=Total_Controller.High_Position_Control.Control_OutPut;//本次速度期望来源于位置控制器输出
#ifdef ALT_VEL_FEEDFORWARD_ENABLE  //速度前馈控制器
		ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Yaw_Vel_Target-Last_Yaw_Vel_Target)/(1*Delta);//速度期望变化率
		ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
		Last_Yaw_Vel_Target=Yaw_Vel_Target;//上次速度期望
#endif
		Total_Controller.High_Speed_Control.Expect=Yaw_Vel_Target;
    handmove_flag=FALSE;
    move_flag=0;
  }
  else if(TRPY[0]>Deadzone_Max)//给定上升速度期望
  {
    //油门杆上推、给定速度期望
    step_mapping=(float)(TRPY[0]-Deadzone_Max)/(Thr_Top-Deadzone_Max);//范围0~1
    scale_mapping=constrain_float(step_mapping*step_mapping,0,1);
    Total_Controller.High_Speed_Control.Expect=Max_Upvel*scale_mapping;//最大上升速度50cm/s
    handmove_flag=TRUE;
    move_flag=1;
  }
  else if(TRPY[0]<Deadzone_Min)//给定下降速度期望
  {
    //油门杆下推、给定速度期望
    step_mapping=(float)(TRPY[0]-Deadzone_Min)/(Deadzone_Min-Thr_Buttom);//范围0~-1
    scale_mapping=constrain_float(step_mapping*step_mapping,0,1);
    Total_Controller.High_Speed_Control.Expect=-Max_Downvel*scale_mapping;//最大下降速度40cm/s
    handmove_flag=TRUE;
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
		Alt_Hold_Output_LPF;
  }
  else
  {
    Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.High_Speed_Control.Control_OutPut);//油门来源于高度速度控制器输出
		Alt_Hold_Output_LPF;
  }
  /*****************************************高度控制器结束，给定油门控制量***********************************************************/
  return handmove_flag;
}






#define Minimal_Thrust_Threshold 1150//着陆检测油门最小值
uint16_t Landon_Earth_Cnt=0;
uint8_t Last_Landon_Earth_Flag=0,Landon_Earth_Flag=0;
uint8_t Landon_Earth_Flag_Set=0;

void landon_earth_check(void)//自检触地进入怠速模式
{
  Last_Landon_Earth_Flag=Landon_Earth_Flag;
  //油门控制处于较低行程：//1、姿态模式下，油门杆处于低位
  //2、定高模式下，期望速度向下，单加速度环反馈为角小值，
  //加速度控制输出由于长时间积分，到负的较大值，使得油门控制较低
  if(Throttle_Output<=Minimal_Thrust_Threshold
     &&Gyro_Length<=20.0f//触地后无旋转，合角速度小于20deg/s
       &&ABS(NamelessQuad.Speed[_YAW])<=30.0f//惯导竖直轴速度+-20cm/s
         &&Unwanted_Lock_Flag==0)
    Landon_Earth_Cnt++;
  else Landon_Earth_Cnt/=2;
  
  if(Landon_Earth_Cnt>=1500)  Landon_Earth_Cnt=1500;//防止溢出
  if(Landon_Earth_Cnt>=400*2.0)//持续3.0S
  {
    Landon_Earth_Flag=1;//着陆标志位
    //Landon_Earth_Cnt=0;
  }
  else
  {
    Landon_Earth_Flag=0;//着陆标志位
  }
  /*
  if(ABS(Throttle_Rate)>=1.0E-5f)//只要油门变化率不为0，即清空着陆标志位
  {
  Landon_Earth_Flag=0;
  Landon_Earth_Cnt=0;
	}
  */
  
  
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void land_nogps_run()
{
  if(Landon_Earth_Flag==0)//未触地，有姿态控制参与
  {
		Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
    
    if(NamelessQuad.Position[_YAW]<=Safe_Height)//相对初始高度10M以内,暂时忽略漂移
      target_yaw_rate=Nav_Decline_Rate;//以Nav_Decline_rate速度下降
    else target_yaw_rate=Nav_Rapid_Decline_Rate;//以Nav_Rapid_Decline_rate速度下降   
    target_yaw_alt=0;
   
//    land_althold(target_yaw_rate,target_yaw_alt);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,target_yaw_alt,target_yaw_rate);//高度控制
  }
  else
  {
		Controller.roll_outer_control_output =0;
		Controller.pitch_outer_control_output=0;
    Take_Off_Reset();//清空积分控制器
    target_yaw_rate=2*Nav_Rapid_Decline_Rate;//以2*Nav_Rapid_Decline_rate速度下降,使得油门量迅速满足怠速条件
    target_yaw_alt=0;
//   land_althold(target_yaw_rate,target_yaw_alt);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,target_yaw_alt,target_yaw_rate);//高度控制
  }
}



uint8_t last_handmove_change_mode=1,handmove_change_mode=1;
uint16 nav_transition_cnt=0;
uint16 nav_transition_flag=0;
uint16 nav_climb_rate_cnt=0;
int16 nav_decline_rate_cnt=0;
uint16 first_switch_flag=1;
uint8_t distance_land_mode=0,last_distance_land_mode=0;
uint8_t climb_safe_alt_flag=0,home_fixed_flag=0;
bool check_pos_handmove=FALSE;
bool check_alt_handmove=FALSE;

void land_reset()//着陆过程中存在各种标志位，在一次飞行过程中，可能会频繁切换，需要清空上次未进行完毕的标志位，在切换时会全部清空
{
  //复位过程中的所有标志位，对于应对较为复杂的逻辑过程的必备编程技巧，一定要掌握好
  last_handmove_change_mode=0;
  
  nav_transition_cnt=0;
  nav_transition_flag=0;
  nav_climb_rate_cnt=0;
  distance_land_mode=0;
  last_distance_land_mode=0;//将自动降落模式标志位置0，下一次循环会重新判断
  climb_safe_alt_flag=0;
  first_switch_flag=1;
  home_fixed_flag=0;
  handmove_change_mode=2;//水平位置手动模式
  first_switch_flag=1;//准备好下次回中时刻的切换
}




bool GPS_ok()// returns true if the GPS is ok and home position is set
{
    return FALSE;
}


// land_run - runs the land controller
// should be called at 100hz or more
void land_run()
{
  land_nogps_run();//GPS无效时，执行一键着陆操作
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



