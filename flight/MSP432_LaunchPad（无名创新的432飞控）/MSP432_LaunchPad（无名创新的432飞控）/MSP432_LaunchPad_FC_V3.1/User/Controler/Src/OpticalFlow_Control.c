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
#include "OpticalFlow_Control.h"


extern Vector2f OpticalFlow_Position;
extern Vector2f OpticalFlow_Speed;
extern Vector2f OpticalFlow_Speed_Err;
extern Vector2f OpticalFlow_Position_Err;

Vector2f OpticalFlow_Pos_Ctrl_Expect={0};
Vector2f OpticalFlow_Pos_Ctrl_Err={0};
Vector2f OpticalFlow_Pos_Ctrl_Integrate={0};
Vector2f OpticalFlow_Pos_Ctrl_Output={0};

Vector2f OpticalFlow_Ctrl_Expect={0};
Vector2f OpticalFlow_Ctrl_Err={0};
Vector2f OpticalFlow_Ctrl_Integrate={0};
Vector2f OpticalFlow_Ctrl_Output={0};

void OpticalFlow_SINS_Reset(void)
{ 
  OpticalFlow_SINS.Position[_PITCH]=0;
  OpticalFlow_SINS.Speed[_PITCH]=0;
  OpticalFlow_SINS.Position[_ROLL]=0;
  OpticalFlow_SINS.Speed[_ROLL]=0;
  OpticalFlow_Position.x=0;
  OpticalFlow_Position.y=0;
}


void OpticalFlow_Ctrl_Reset(void)
{ 
  OpticalFlow_Ctrl_Integrate.x=0.0f;
  OpticalFlow_Ctrl_Integrate.y=0.0f;
  OpticalFlow_Pos_Ctrl_Integrate.x=0.0f;
  OpticalFlow_Pos_Ctrl_Integrate.y=0.0f;
  OpticalFlow_Pos_Ctrl_Expect.x=0;
  OpticalFlow_Pos_Ctrl_Expect.y=0;
}

void SDK_Pos_Ctrl_Reset(void)
{ 
	Total_Controller.SDK_Pitch_Position_Control.Integrate=0;
	Total_Controller.SDK_Pitch_Position_Control.Last_Err=0;
	Total_Controller.SDK_Pitch_Position_Control.Err=0;
	Total_Controller.SDK_Pitch_Position_Control.Last_Err_LPF=0;		

	Total_Controller.SDK_Roll_Position_Control.Integrate=0;
	Total_Controller.SDK_Roll_Position_Control.Last_Err=0;
	Total_Controller.SDK_Roll_Position_Control.Err=0;
	Total_Controller.SDK_Roll_Position_Control.Last_Err_LPF=0;
}
float OpticalFlow_Expect_Speed_Mapping(float input,uint16_t input_max,float output_max)
{
  float output_speed=0;
  float temp_scale=(float)(input/input_max);
  temp_scale=constrain_float(temp_scale,-1.0f, 1.0f);
  if(temp_scale>=0) output_speed=(float)(output_max*temp_scale*temp_scale);
  else output_speed=(float)(-output_max*temp_scale*temp_scale); 
  return output_speed;
}

void OpticalFlow_Set_Target_Point(Vector2f target)
{
  OpticalFlow_Pos_Ctrl_Expect.x=target.x;
  OpticalFlow_Pos_Ctrl_Expect.y=target.y;
}

void OpticalFlow_Set_Target_Vel(Vector2f target)
{
  OpticalFlow_Ctrl_Expect.x=target.x;
  OpticalFlow_Ctrl_Expect.y=target.y;
}

//////////////////////////////////////////////////////////////////////////


bool OPT_Is_Fix(void)
{
 if(opt_data.valid==0||Sensor_Flag.Ground_Health==0)
 {
	 return false;
 }
 else 
 {
	 return true;
 }
}



void OpticalFlow_Pos_Control(void)
{
	if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
  static uint16_t OpticalFlow_Pos_Ctrl_Cnt=0;
  OpticalFlow_Pos_Ctrl_Cnt++;
  if(OpticalFlow_Pos_Ctrl_Cnt>=4)//10ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    //计算位置偏差  
    OpticalFlow_Pos_Ctrl_Err.x=OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_PITCH];
    OpticalFlow_Pos_Ctrl_Err.y=OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_ROLL];
		
    OpticalFlow_Pos_Ctrl_Err.x=constrain_float(OpticalFlow_Pos_Ctrl_Err.x,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);//100
    OpticalFlow_Pos_Ctrl_Err.y=constrain_float(OpticalFlow_Pos_Ctrl_Err.y,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);		
		
    //计算位置控制输出
    OpticalFlow_Pos_Ctrl_Output.x=Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.x;
    OpticalFlow_Pos_Ctrl_Output.y=Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.y;
    OpticalFlow_Pos_Ctrl_Cnt=0;
  }
}


void OpticalFlow_Vel_Control(Vector2f target)
{
	if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}	
	
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Ctrl_Expect.x= target.x;//速度期望
  OpticalFlow_Ctrl_Expect.y= target.y;
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    OpticalFlow_Ctrl_Err.x=constrain_float(OpticalFlow_Ctrl_Expect.x-OpticalFlow_SINS.Speed[_PITCH],-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);//30
    OpticalFlow_Ctrl_Err.y=constrain_float(OpticalFlow_Ctrl_Expect.y-OpticalFlow_SINS.Speed[_ROLL] ,-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);
    
    //if(ABS(OpticalFlow_Ctrl_Err.x)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Ctrl_Integrate.x+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Ctrl_Err.x;//0.1  15
    //if(ABS(OpticalFlow_Ctrl_Err.y)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Ctrl_Integrate.y+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Ctrl_Err.y;
    
    OpticalFlow_Ctrl_Integrate.x=constrain_float(OpticalFlow_Ctrl_Integrate.x,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    OpticalFlow_Ctrl_Integrate.y=constrain_float(OpticalFlow_Ctrl_Integrate.y,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    
    OpticalFlow_Ctrl_Output.x=OpticalFlow_Ctrl_Integrate.x+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Ctrl_Err.x;//4.5
    OpticalFlow_Ctrl_Output.y=OpticalFlow_Ctrl_Integrate.y+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Ctrl_Err.y;
    
    accel_target.y=-constrain_float(OpticalFlow_Ctrl_Output.y,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
    accel_target.x=-constrain_float(OpticalFlow_Ctrl_Output.x,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//期望运动加速度
    accel_to_lean_angles(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
		
		Controller.roll_outer_control_output =angle_target.x;
		Controller.pitch_outer_control_output=angle_target.y;
				
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}



void OpticalFlow_Control_Pure(uint8_t force_brake_flag)
{
  if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
	/////////////////////////////////SDK位置控制开始////////////////////////////////////////						
	if(ngs_sdk.update_flag==true)
	{
			if(ngs_sdk.move_flag.sdk_front_flag==true||ngs_sdk.move_flag.sdk_behind_flag==true)//前进/后退
			{
					OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH];										//左右保持
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL]+ngs_sdk.f_distance;	//前后调整		
					Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];	 				//上下保持																
			}
			
			if(ngs_sdk.move_flag.sdk_left_flag==true||ngs_sdk.move_flag.sdk_right_flag==true)//向左/向右
			{
				  OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH]+ngs_sdk.f_distance;//左右调整
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL];										 //前后保持			
					Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];			 	 //上下保持														
			}
			
			if(ngs_sdk.move_flag.sdk_up_flag==true||ngs_sdk.move_flag.sdk_down_flag==true)//上升/下降
			{
					OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH];															//前后保持
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL];																//左右保持
				  Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW]+ngs_sdk.f_distance; //上下调整										
			}
			ngs_sdk.move_flag.sdk_front_flag=false;
			ngs_sdk.move_flag.sdk_behind_flag=false;
			ngs_sdk.move_flag.sdk_left_flag=false;
			ngs_sdk.move_flag.sdk_right_flag=false;
			ngs_sdk.move_flag.sdk_up_flag=false;
			ngs_sdk.move_flag.sdk_down_flag=false;							
			ngs_sdk.update_flag=false;
	}

	
	if(Roll_Control==0
		 &&Pitch_Control==0)//无水平遥控量给定
	{ 
		/**************************光流位置控制器************************************/
		if((OpticalFlow_Pos_Ctrl_Expect.x==0&&OpticalFlow_Pos_Ctrl_Expect.y==0)
			 ||force_brake_flag==1)
		{
			OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH];
			OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL];
		}
		
		OpticalFlow_Pos_Control();
	 
		/***********当只需要速度控制时，开启以下注释，仅限调试时用*************/
		 //OpticalFlow_Pos_Ctrl_Output.x=0;
		 //OpticalFlow_Pos_Ctrl_Output.y=0;
		/**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
	}
	else
	{
		OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
		OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 
			
		
		
		if(OpticalFlow_Speed_Control_Mode==1)
		{
			OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
		}
		else
		{
				Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
				Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];    
		}
		OpticalFlow_Pos_Ctrl_Expect.x=0;
		OpticalFlow_Pos_Ctrl_Expect.y=0;
	}
}




void OpticalFlow_X_Vel_Control(float target_x)//机头左侧为X+
{ 
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Ctrl_Expect.x= target_x;//速度期望
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    OpticalFlow_Ctrl_Err.x=constrain_float(OpticalFlow_Ctrl_Expect.x-OpticalFlow_SINS.Speed[_PITCH],-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);//30

    //if(ABS(OpticalFlow_Ctrl_Err.x)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Ctrl_Integrate.x+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Ctrl_Err.x;//0.1  15

    OpticalFlow_Ctrl_Integrate.x=constrain_float(OpticalFlow_Ctrl_Integrate.x,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);

    OpticalFlow_Ctrl_Output.x=OpticalFlow_Ctrl_Integrate.x+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Ctrl_Err.x;//4.5

		accel_target.x=-constrain_float(OpticalFlow_Ctrl_Output.x,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//期望运动加速度
    accel_target.y=0;
		
		accel_to_lean_angles(accel_target,&angle_target);//期望运动加速度转期望姿态倾角
		Controller.roll_outer_control_output=angle_target.x;
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


void OpticalFlow_Y_Vel_Control(float target_y)//机头前侧为Y+
{ 
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Ctrl_Expect.y= target_y;
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms控制一次速度，避免输入频率过大，系统响应不过来
  {
    OpticalFlow_Ctrl_Err.y=constrain_float(OpticalFlow_Ctrl_Expect.y-OpticalFlow_SINS.Speed[_ROLL] ,-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);
    
    OpticalFlow_Ctrl_Integrate.y+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Ctrl_Err.y;
    
    OpticalFlow_Ctrl_Integrate.y=constrain_float(OpticalFlow_Ctrl_Integrate.y,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    
    OpticalFlow_Ctrl_Output.y=OpticalFlow_Ctrl_Integrate.y+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Ctrl_Err.y;
    
		accel_target.x=0;
    accel_target.y=-constrain_float(OpticalFlow_Ctrl_Output.y,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
    
		accel_to_lean_angles(accel_target,&angle_target);//期望运动加速度转期望姿态倾角

		Controller.pitch_outer_control_output=angle_target.y;
		
		
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


///////////////////////////////////////////////
void OpticalFlow_Control(uint8_t force_brake_flag)
{
	if(OPT_Is_Fix()==false)//OPT定位状态未锁定，姿态期望直接来源于遥控器给定
	{
		Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}

	static uint8_t miss_fixed_flag=0;
	static systime opt_poshold_dt;
	Get_Systime(&opt_poshold_dt);
	//如果本函数运行周期大于控制周期10倍
	if(0.001f*opt_poshold_dt.period>=20*0.0025f)
	{
		//情况1:初次从其它模式切入本模式
		//情况2:系统调度超时，在系统设计合理情况下，本情况不可能发生
		miss_fixed_flag=1;
	}
	
	if(miss_fixed_flag==1)//之前未满足定位条件,未锁定目标点
	{
		OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH];
		OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL];
		//控制器复位
		OpticalFlow_Ctrl_Reset();
		miss_fixed_flag=0;
	}
	
    if(Roll_Control==0
       &&Pitch_Control==0)//无水平遥控量给定
    {
      /**************************光流位置控制器************************************/
      if(OpticalFlow_Pos_Ctrl_Expect.x==0
         &&OpticalFlow_Pos_Ctrl_Expect.y==0)
      {
        if(force_brake_flag==1||(rMat[2][2]>=0.95f
                                 &&pythagorous2(OpticalFlow_SINS.Speed[_PITCH],OpticalFlow_SINS.Speed[_ROLL])<=40))//打杆回中后，根据当前速度、倾角判断是否进行悬停
        {
          OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH];
          OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL];
        }
        else  //打杆回中未满足悬停条件时，只进行速度控制 
        {
          OpticalFlow_Pos_Ctrl_Output.x=0;
          OpticalFlow_Pos_Ctrl_Output.y=0;
        }
      }
      else  OpticalFlow_Pos_Control();
      
      /***********当只需要速度控制时，开启以下注释，仅限调试时用*************/
      //OpticalFlow_Pos_Ctrl_Output.x=0;
      //OpticalFlow_Pos_Ctrl_Output.y=0;
      /**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
    }
    else
    {
      OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
      OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 
			
      if(OpticalFlow_Speed_Control_Mode==1)
      {
        OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望
      }
      else
      {
				Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
				Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];     
      }
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
    }
    PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
    PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
}





void Color_Block_Control_Pliot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=40)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;
			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag);
			
			Total_Controller.SDK_Pitch_Position_Control.Expect=0;
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag);
			
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Control_Pure(1);//20ms		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);//20ms		
		}
	}
  else//目标未丢失,10ms
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
	}			
}


void Top_APrilTag_Control_Pliot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=40)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;
			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag);
			
			Total_Controller.SDK_Pitch_Position_Control.Expect=0;
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag);
			
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Control_Pure(1);//20ms		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);//20ms		
		}
	}
  else//目标未丢失,10ms
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
	}			
}


#define Forward_Keep_Distance   100//cm
#define Deadband_Keep_Distance  10
void Front_AprilTag_Control_Pliot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=40)//100ms
	{	
		_cnt=0;		
	  if(Opv_Front_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Front_View_Target.target_ctrl_enable=0;

			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Front_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Front_View_Target.trust_flag);
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			
			
			
			Total_Controller.SDK_Pitch_Position_Control.Expect=Forward_Keep_Distance;//期望为前向保持距离，具体可根据实际自行定义
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Front_View_Target.apriltag_distance;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Front_View_Target.trust_flag);
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);		
		}
	}
  else//目标未丢失
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
	}			
}








void Self_Track_Control_Pliot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=40)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;

			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag);
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			
			static float target_speed;//5 3 2 1
			if(ABS(Opv_Top_View_Target.sdk_angle)<=10)      target_speed=15;//
			else if(ABS(Opv_Top_View_Target.sdk_angle)<=20) target_speed=10;//
			else if(ABS(Opv_Top_View_Target.sdk_angle)<=50) target_speed=5;//
			else target_speed=1;//5 3 2 1
			
//			Total_Controller.SDK_Pitch_Position_Control.Expect=Forward_Keep_Distance;//期望为前向保持距离，具体可根据实际自行定义
//			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.apriltag_distance;
//			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag);
			
			OpticalFlow_Pos_Ctrl_Output.y=target_speed;	
		}
		else//丢失目标
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//目标丢失
	{
		if(miss_cnt==1)//初始丢失跟踪目标后，锁定当前位置后，进行普通光流控制
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//丢失跟踪目标后，进行普通光流控制
		{
			OpticalFlow_Control_Pure(0);
			Controller.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
		}
	}
  else//目标未丢失
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度控制周期20ms
		Controller.yaw_outer_control_output  =-3.0f*Opv_Top_View_Target.sdk_angle;
	}			
}








//SDK接收模式
//  {
//    if(Roll_Control==0&&Pitch_Control==0)//无水平遥控量给定
//    {  
//      if(Opv_Top_View_Target.line_ctrl_enable==1)//线检测
//      {
//        /***********当只需要速度控制时，开启以下注释，仅限调试时用*************/
//        sdk_ctrl_cnt++;
//        if(sdk_ctrl_cnt>=4)//20ms
//        {
//          Total_Controller.SDK_Roll_Position_Control.Expect=0;
//          Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
//          PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,1);   
//          accel_target.x=constrain_float(Total_Controller.SDK_Roll_Position_Control.Control_OutPut,
//                                         -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//                                         Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);                             
//          Controller.roll_outer_control_output=constrain_float(fast_atan(accel_target.x/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//roll                                
//          sdk_ctrl_cnt=0;
//        }
//				
//        if(ABS(SDK_Target_Yaw_Gyro)<=10) OpticalFlow_Y_Vel_Control(5);
//        else if(ABS(SDK_Target_Yaw_Gyro)<=20) OpticalFlow_Y_Vel_Control(3);
//        else if(ABS(SDK_Target_Yaw_Gyro)<=50) OpticalFlow_Y_Vel_Control(2);
//        else OpticalFlow_Y_Vel_Control(1);
//        //OpticalFlow_Y_Vel_Control(0);
//        OpticalFlow_Pos_Ctrl_Expect.x=0;
//        OpticalFlow_Pos_Ctrl_Expect.y=0;
//        force_brake_flag=1;
//      }
//      else if(Opv_Top_View_Target.target_ctrl_enable==1)//目标点检测跟踪
//      {
//        sdk_ctrl_cnt++;
//        if(sdk_ctrl_cnt>=10)//50ms
//        {				
//					Opv_Top_View_Target.target_ctrl_enable=0;
//					Total_Controller.SDK_Roll_Position_Control.Expect=0;
//					Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
//					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag);
//					
//					Total_Controller.SDK_Pitch_Position_Control.Expect=0;
//					Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
//					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag);
//					
//					accel_target.x=constrain_float(Total_Controller.SDK_Roll_Position_Control.Control_OutPut,
//																				 -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//																				 Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);                             
//					Controller.roll_outer_control_output=constrain_float(fast_atan(accel_target.x/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//roll
//					
//					
//					accel_target.y=constrain_float(Total_Controller.SDK_Pitch_Position_Control.Control_OutPut,
//																				 -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//																				 Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
//					Controller.pitch_outer_control_output=constrain_float(fast_atan(accel_target.y*Cos_Roll/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//pitch 
//					sdk_ctrl_cnt=0;
//        }				
//        OpticalFlow_Pos_Ctrl_Expect.x=0;
//        OpticalFlow_Pos_Ctrl_Expect.y=0;
//        force_brake_flag=1;
//      }
//      else//SDK模式下，未检测对于特征
//      {
//        /**************************光流位置控制器************************************/
//        if(OpticalFlow_Pos_Ctrl_Expect.x==0
//           &&OpticalFlow_Pos_Ctrl_Expect.y==0)
//        {
//          if(force_brake_flag==1||(rMat[2][2]>=0.95f
//                                   &&pythagorous2(OpticalFlow_SINS.Speed[_PITCH],OpticalFlow_SINS.Speed[_ROLL])<=40))//打杆回中后，根据当前速度、倾角判断是否进行悬停
//          {
//            OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH];
//            OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL];
//          }
//          else  //打杆回中未满足悬停条件时，只进行速度控制 
//          {
//            OpticalFlow_Pos_Ctrl_Output.x=0;
//            OpticalFlow_Pos_Ctrl_Output.y=0;
//          }
//        }
//        else  OpticalFlow_Pos_Control();
//        /**************************基于模型的加速度-姿态角映射，相比直接给姿态，参数差异大概在20倍左右************************************/
//        OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//速度期望        
//        PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
//        PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
//				SDK_Pos_Ctrl_Reset();
//			}
//    }
//    else//SDK模式下存在手动打杆操作时，控制量来源于遥控器给定
//    {
//			Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
//			Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];		
//      OpticalFlow_Pos_Ctrl_Expect.x=0;
//      OpticalFlow_Pos_Ctrl_Expect.y=0;
//      force_brake_flag=1;
//			PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
//			PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
//			SDK_Pos_Ctrl_Reset();
//    }  
//    OpticalFlow_Pos_Ctrl_Expect.x=0;
//    OpticalFlow_Pos_Ctrl_Expect.y=0;
//  }
/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

