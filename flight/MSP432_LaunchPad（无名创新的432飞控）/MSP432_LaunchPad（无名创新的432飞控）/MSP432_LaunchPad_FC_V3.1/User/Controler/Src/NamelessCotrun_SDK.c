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
#include "NamelessCotrun_SDK.h"
bool auto_altland(float taret_climb_rate,float target_climb_alt)
{
  return land_althold(taret_climb_rate,target_climb_alt);
}
//SDK本模式需要在光流模式下使用
uint8_t move_with_speed_target(float x_target,float y_target,float delta,SDK_Status *Status,uint16_t number)
{
  static float end_time=0;
  Vector2f vel_target;
  Testime dt;
  vel_target.x=x_target;
  vel_target.y=y_target;
  Test_Period(&dt);
  ncq_control_althold();//高度控制依然进行
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {
      OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
      return 1;
  }
  else
  {
    if(Status->Status[number].Start_Flag==0) 
    {
      end_time=dt.Now_Time+delta;//单位ms 
      Status->Status[number].Start_Flag=1;
    } 
    if(dt.Now_Time>end_time)
    {
      Status->Status[number].Execute_Flag=1;
      Status->Status[number].End_flag=1;
      OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      end_time=0;
      Status->Transition_Time[number]=400;//400*5ms=2s
      return 1;//返回完成
    }
    else
    { 
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      Status->Status[number].Execute_Flag=1;
      OpticalFlow_Vel_Control(vel_target);//给定速度期望
      return 0;
    }
  }
}


uint8_t move_with_xy_target(float pos_x_target,float pos_y_target,SDK_Status *Status,uint16_t number)
{
  //static Vector2f pos_base;
  ncq_control_althold();//高度控制依然进行
  if(Status->Status[number].Start_Flag==0) 
  {
    //pos_base.x=OpticalFlow_SINS.Position[_PITCH];
    //pos_base.y=OpticalFlow_SINS.Position[_ROLL];
    OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH]+pos_x_target;
    OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL]+pos_y_target;
    Status->Status[number].Start_Flag=1;
  }
  
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {
     OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
     return 1;
  }
  else
  {    
    if(pythagorous2(OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_PITCH],
                    OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_ROLL])<=10.0f)
    {
      Status->Status[number].Execute_Flag=1;
      Status->Status[number].End_flag=1;
      //pos_base.x=0;
      //pos_base.y=0;
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      Status->Transition_Time[number]=200;
      OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      return 1;
    }
    else
    { 
      Status->Status[number].Execute_Flag=1; 
      OpticalFlow_Pos_Control();//光流位置控制
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//光流速度控制 
      return 0;
    }
  }
}

uint8_t move_with_target(float pos_x_target,float pos_y_target,Duty_Status *Status,uint8_t *Start_flag)
{
  //static Vector2f pos_base;
  ncq_control_althold();//高度控制依然进行
  if(*Start_flag==1)
  {
    //pos_base.x=OpticalFlow_SINS.Position[_PITCH];
    //pos_base.y=OpticalFlow_SINS.Position[_ROLL];
    *Start_flag=0;
  }
  
  if(Status->Start_Flag==0) 
  {
    OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_PITCH]+pos_x_target;
    OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_ROLL]+pos_y_target;
    Status->Start_Flag=1;
  }
  
  if(Status->Start_Flag==1
     &&Status->Execute_Flag==1
       &&Status->End_flag==1)
  {
     OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
     return 1;
  }
  else
  {    
    if(pythagorous2(OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_PITCH],
                OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_ROLL])<=8.0f)
    {
      Status->Execute_Flag=1;
      Status->End_flag=1;
      //pos_base.x=0;
      //pos_base.y=0;
      OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      return 1;
    }
    else
    { 
      Status->Execute_Flag=1; 
      OpticalFlow_Pos_Control();//光流位置控制
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//光流速度控制 
      return 0;
    }
  }
}


uint8_t move_with_base_target(float pos_x_target,float pos_y_target,Duty_Status *Status,uint8_t *Start_flag)
{
  //static Vector2f pos_base;
  ncq_control_althold();//高度控制依然进行
  if(*Start_flag==1)
  {
    //pos_base.x=OpticalFlow_SINS.Position[_PITCH];
    //pos_base.y=OpticalFlow_SINS.Position[_ROLL];
    *Start_flag=0;
  }
  
  if(Status->Start_Flag==0) 
  {
    OpticalFlow_Pos_Ctrl_Expect.x=pos_x_target;
    OpticalFlow_Pos_Ctrl_Expect.y=pos_y_target;
    Status->Start_Flag=1;
  }
  
  if(Status->Start_Flag==1
     &&Status->Execute_Flag==1
       &&Status->End_flag==1)
  {
     OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
     return 1;
  }
  else
  {    
    if(pythagorous2(OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_PITCH],
                OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_ROLL])<=8.0f)
    {
      Status->Execute_Flag=1;
      Status->End_flag=1;
      //pos_base.x=0;
      //pos_base.y=0;
      OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      return 1;
    }
    else
    { 
      Status->Execute_Flag=1; 
      OpticalFlow_Pos_Control();//光流位置控制
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//光流速度控制 
      return 0;
    }
  }
}



uint8_t move_with_z_target(float z_target,float z_vel,float delta,SDK_Status *Status,uint16_t number)
{
  static float target_rate=0;
  static float target_alt=0;
  static uint8_t end_flag=0;
  //static float z_base;
  static float end_time=0;
  Testime dt;
  Test_Period(&dt);
  OpticalFlow_Control_Pure(0);//水平位置控制依然进行
  
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {   
     ncq_control_althold();
     return 1;
  }
  else
  {
      if(Status->Status[number].Start_Flag==0) 
      {  
        if(z_target==0)//速度控制，一次任务中只设置一次
        {
          //z_base=NamelessQuad.Position[_YAW];
          target_rate=z_vel;
          target_alt=0;
          end_flag=1;
        }
        else//位置控制，一次任务中只设置一次
        {
          //z_base=NamelessQuad.Position[_YAW];
          target_rate=0;
          target_alt=NamelessQuad.Position[_YAW]+z_target;
          end_flag=2;
        }
        end_time=dt.Now_Time+delta;//单位ms 
        Status->Status[number].Start_Flag=1;
				
				Unwanted_Lock_Flag=0;//允许自动上锁
				OpticalFlow_Pos_Ctrl_Expect.x=0;
				OpticalFlow_Pos_Ctrl_Expect.y=0;
      }

    
    if(end_flag==1)
    {  
      auto_altland(target_rate,target_alt);
      if(dt.Now_Time>end_time)//结束条件
      { 
        end_flag=0;
        target_rate=0;
        target_alt=0;
        //z_base=0;
        end_time=0;
        Status->Status[number].Execute_Flag=1;
        Status->Status[number].End_flag=1;
        Status->Transition_Time[number]=200;
				
				OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
				OpticalFlow_Pos_Ctrl_Expect.x=0;
				OpticalFlow_Pos_Ctrl_Expect.y=0;
				
				Total_Controller.High_Position_Control.Expect=0;
        return 1;//返回完成
      }
      Status->Status[number].Execute_Flag=1;
    }
    else if(end_flag==2)
    {
      auto_altland(target_rate,target_alt);    
      if(ABS(target_alt-NamelessQuad.Position[_YAW])<=5.0f)//结束条件
      {
        end_flag=0;
        target_rate=0;
        target_alt=0;
        //z_base=0;
        end_time=0;
        Status->Status[number].Execute_Flag=1;
        Status->Status[number].End_flag=1;
        Status->Transition_Time[number]=200;
				
				OpticalFlow_Control_Pure(1);//完成之后，进行光流悬停
				OpticalFlow_Pos_Ctrl_Expect.x=0;
				OpticalFlow_Pos_Ctrl_Expect.y=0;
				
				Total_Controller.High_Position_Control.Expect=0;
        return 1;
      }
      Status->Status[number].Execute_Flag=1;
    } 
  }
  return 0;
}

//#define NCQ_SDK_DUTY1 move_with_speed_target(10,0,2000 ,&SDK_Duty_Status,1-1)//左
//#define NCQ_SDK_DUTY2 move_with_speed_target(0,10,2000 ,&SDK_Duty_Status,2-1)//前
//#define NCQ_SDK_DUTY3 move_with_speed_target(-10,0,2000,&SDK_Duty_Status,3-1)//右
//#define NCQ_SDK_DUTY4 move_with_speed_target(0,-10,2000,&SDK_Duty_Status,4-1)//后


#define NCQ_SDK_DUTY_MAX   3
#define NCQ_SDK_DUTY1 move_with_z_target(100,0,0,&SDK_Duty_Status,1-1)
#define NCQ_SDK_DUTY2 move_with_xy_target(0,50,&SDK_Duty_Status,2-1)
#define NCQ_SDK_DUTY3 move_with_z_target(-150,0,0,&SDK_Duty_Status,3-1)


SDK_Status SDK_Duty_Status;
uint16_t SDK_Duty_Cnt=0;
uint16_t SDK_Transition_Time=0;
void NCQ_SDK_Run(void)
{
  if(SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]>=1) 
    SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]--;//安排过渡时间
  
  if(SDK_Duty_Status.Status[SDK_Duty_Cnt].Start_Flag==1
     &&SDK_Duty_Status.Status[SDK_Duty_Cnt].Execute_Flag==1
       &&SDK_Duty_Status.Status[SDK_Duty_Cnt].End_flag==1
         &&SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]==0)
    SDK_Duty_Cnt++;
    
  if(SDK_Duty_Cnt>=NCQ_SDK_DUTY_MAX) SDK_Duty_Cnt=NCQ_SDK_DUTY_MAX;
  
  if(SDK_Duty_Cnt==0)        NCQ_SDK_DUTY1;
  else if(SDK_Duty_Cnt==1)   NCQ_SDK_DUTY2;
  else if(SDK_Duty_Cnt==2)   NCQ_SDK_DUTY3;
  //else if(SDK_Duty_Cnt==3)   NCQ_SDK_DUTY4;
  //else if(SDK_Duty_Cnt==4)   NCQ_SDK_DUTY5;
  //else if(SDK_Duty_Cnt==5)   NCQ_SDK_DUTY6;
  //else if(SDK_Duty_Cnt==6)   NCQ_SDK_DUTY7;
  else
  {
    ncq_control_althold();//高度控制
    OpticalFlow_Control_Pure(0);//位置控制
  }
}


void NCQ_SDK_Reset(void)
{
  uint16_t i=0;
  for(i=0;i<SDK_Duty_Max;i++)
  {
    SDK_Duty_Status.Status[i].Start_Flag=0;
    SDK_Duty_Status.Status[i].Execute_Flag=0;
    SDK_Duty_Status.Status[i].End_flag=0;
		SDK_Duty_Status.Transition_Time[i]=0;
  }
  SDK_Duty_Cnt=0;
}

#define Circle_Piont_Max  24
#define Circle_R  200
static Duty_Status circle_status[Circle_Piont_Max+1];
static uint8_t Circle_Start_Flag=1;
float circle_pos_x_delta[Circle_Piont_Max+1]={0},circle_pos_y_delta[Circle_Piont_Max+1]={0};
static Vector2f circle_pos_base; 
uint8_t NCQ_SDK_Circle(void)
{
  static uint16_t circle_cnt=0;
  static float circle_angle=0;
  uint16_t i=0;
  
  if(Circle_Start_Flag==1)
  {
    circle_pos_base.x=OpticalFlow_SINS.Position[_PITCH];
    circle_pos_base.y=OpticalFlow_SINS.Position[_ROLL];
    for(i=0;i<Circle_Piont_Max+1;i++)
    {
      circle_angle=15*i;
      circle_pos_x_delta[i]=circle_pos_base.x+(1-cos(circle_angle*DEG2RAD))*Circle_R;
      circle_pos_y_delta[i]=circle_pos_base.y+sin(circle_angle*DEG2RAD)*Circle_R;
    }
  }
  else if(Circle_Start_Flag==2)
  {
    ncq_control_althold();
    OpticalFlow_Control(0);//结束后的水平位置控制
    return 1;
  }
  
  if(move_with_base_target(circle_pos_x_delta[circle_cnt],circle_pos_y_delta[circle_cnt],&circle_status[circle_cnt],&Circle_Start_Flag))
  {
    circle_cnt++;
    if(circle_cnt==Circle_Piont_Max+1)
    {
      circle_cnt=0;
      for(uint16_t i=0;i<Circle_Piont_Max+1;i++)
      {
        circle_status[i].End_flag=0;
        circle_status[i].Execute_Flag=0;
        circle_status[i].Start_Flag=0;
        //circle_pos_x_delta[i]=0;
        //circle_pos_y_delta[i]=0;
        circle_angle=0;
      }
      Circle_Start_Flag=2;
      return 1;
    }
  }
  return 0;
}


unsigned char sdk_data_to_send[50];
void SDK_DT_Send_Data(unsigned char *dataToSend , unsigned char length)
{
  USART3_Send(sdk_data_to_send, length);
}

void SDK_DT_Send_Check(unsigned char mode,COM_SDK com)
{
  sdk_data_to_send[0]=0xFF;
  sdk_data_to_send[1]=0xFE;
  sdk_data_to_send[2]=0xA0;
  sdk_data_to_send[3]=2;
  sdk_data_to_send[4]=mode;
  sdk_data_to_send[5]=0
    ;
  u8 sum = 0;
  for(u8 i=0;i<6;i++) sum += sdk_data_to_send[i];
  sdk_data_to_send[6]=sum;
	if(com==UART3_SDK) USART3_Send(sdk_data_to_send, 7);
	if(com==UART1_SDK) USART1_Send(sdk_data_to_send, 7); 
}



#define SDK_TARGET_X_OFFSET  0
#define SDK_TARGET_Y_OFFSET  0//-12


Target_Check Opv_Top_View_Target,Opv_Front_View_Target;
float SDK_Target_Yaw_Gyro=0;
#define  Pixel_Size    0.0024f//6um=0.000006m=0.0006cm
                       //320---0.0012
                       //160---0.0024
                       //80 ---0.0048
#define  Focal_Length  0.42f

#define OV7725_Sensor_Width_MM    		3.984f//3984um
#define OV7725_Sensor_Height_MM   		2.952f//2952um
#define Pixel_Image_Width    		160//320
#define Pixel_Image_Height   		120//240
#define Pixel_Image_Focal_MM 		4.2f
#define Pixel_Image_View_Angle_X  (56.72/2)//deg(50.75/2)
#define Pixel_Image_View_Angle_Y  (44.07/2)//deg(38.72/2)

#define AprilTag_Side_Length  13.6f//cm13.6

float _Pixel_Image_View_Angle_X,_Pixel_Image_View_Angle_Y;
void Get_Camera_Wide_Angle(float view_angle)
{
	float fh=5.0f/FastTan(0.5f*view_angle*DEG2RAD);
  _Pixel_Image_View_Angle_X=2*RAD2DEG*fast_atan(4/fh);
  _Pixel_Image_View_Angle_Y=2*RAD2DEG*fast_atan(3/fh);
}
	
float _P1=0,_P2=0;
uint16_t _CX=0,_CY=0;
float _TX=0,_TY=0;
float _DX=0,_DY=0;
void Sensor_Parameter_Sort(uint16_t tx,uint16_t ty,float pitch,float roll,float alt)
{
//	Get_Camera_Wide_Angle(68);//根据摄像头广角，计算得到X、Y轴视角
	float theta_x_max=0,theta_y_max=0;
	theta_x_max=Pixel_Image_View_Angle_X;
	theta_y_max=Pixel_Image_View_Angle_Y;
  _P1=0.5f*Pixel_Image_Width/FastTan(theta_x_max*DEG2RAD);	
	_P2=0.5f*Pixel_Image_Height/FastTan(theta_y_max*DEG2RAD);
	
	_CX=Pixel_Image_Width/2;
	_CY=Pixel_Image_Height/2;
	
	float tmp_x=0,tmp_y=0;
	tmp_x=fast_atan((_CX-tx)/_P1);
	tmp_y=fast_atan((_CY-ty)/_P2);
	
	_TX= FastTan(tmp_x+roll*DEG2RAD) *_P1;
	_TY= FastTan(tmp_y+pitch*DEG2RAD)*_P2;
	
//	_DX=alt*_TX/_P1;
//	_DY=alt*_TY/_P2;
	_DX=0.5f*alt*_TX/_P1;
	_DY=0.5f*alt*_TY/_P2;

	Opv_Top_View_Target.sdk_target.x=_DX;
  Opv_Top_View_Target.sdk_target.y=_DY;
}


void SDK_DT_Reset()
{
  Opv_Top_View_Target.x=0;
  Opv_Top_View_Target.y=0; 
  Opv_Top_View_Target.pixel=0;
  Opv_Top_View_Target.flag=0;
	Opv_Top_View_Target.state=0;
	Opv_Top_View_Target.angle=0;
	Opv_Top_View_Target.distance=0;
	Opv_Top_View_Target.apriltag_id=0;
	Opv_Top_View_Target.width=0;
	Opv_Top_View_Target.height=0;
	Opv_Top_View_Target.fps=0;
	Opv_Top_View_Target.reserved1=0;
	Opv_Top_View_Target.reserved2=0;
	Opv_Top_View_Target.reserved3=0;
	Opv_Top_View_Target.reserved4=0;
	Opv_Top_View_Target.range_sensor1=0;
	Opv_Top_View_Target.range_sensor2=0;
	Opv_Top_View_Target.range_sensor3=0;
	Opv_Top_View_Target.range_sensor4=0;
	
	
	Opv_Front_View_Target=Opv_Top_View_Target;
}




uint16_t err_cnt[2]={0};
void Openmv_Data_Receive_Anl_1(uint8_t *data_buf,uint8_t num,Target_Check *target)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//不满足帧头条件
	
	target->x				  =*(data_buf+4)<<8|*(data_buf+5);
  target->y					=*(data_buf+6)<<8|*(data_buf+7);
	target->pixel			=*(data_buf+8)<<8|*(data_buf+9);  
	target->flag			=*(data_buf+10);
	target->state		  =*(data_buf+11);		
	target->angle		  =*(data_buf+12)<<8|*(data_buf+13);
	target->distance  =*(data_buf+14)<<8|*(data_buf+15);
	target->apriltag_id=*(data_buf+16)<<8|*(data_buf+17);
	target->width	    =*(data_buf+18)<<8|*(data_buf+19);
	target->height	  =*(data_buf+20)<<8|*(data_buf+21);
	target->fps  		  =*(data_buf+22);
	target->reserved1 =*(data_buf+23);
	target->reserved2 =*(data_buf+24);
	target->reserved3 =*(data_buf+25);
	target->reserved4 =*(data_buf+26);
	//扩展距离传感器
	target->range_sensor1 =*(data_buf+27)<<8|*(data_buf+28);
	target->range_sensor2 =*(data_buf+29)<<8|*(data_buf+30);
	target->range_sensor3 =*(data_buf+31)<<8|*(data_buf+32);
	target->range_sensor4 =*(data_buf+33)<<8|*(data_buf+34);

	target->x_pixel_size=Pixel_Size*(Pixel_Image_Width/target->width);
	target->y_pixel_size=Pixel_Size*(Pixel_Image_Height/target->height);

  target->apriltag_distance=AprilTag_Side_Length*Focal_Length/(target->x_pixel_size*FastSqrt(target->pixel));

  target->sdk_mode=*(data_buf+2);
	
  if(*(data_buf+2)==0XA1)//点检测
  {
		target->target_ctrl_enable=target->flag;		
		if(target->flag!=0)  
		{
			if(target->trust_cnt<20)	 
			{
				target->trust_cnt++;
				target->trust_flag=0;
			}
			else target->trust_flag=1;
		}
		else 
		{
			target->trust_cnt/=2;
		  target->trust_flag=0;
		}

    target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
    target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;
			
		Sensor_Parameter_Sort(target->x,target->y,Pitch,Roll,NamelessQuad.Position[_YAW]);
  }
	else if(*(data_buf+2)==0XA2)//AprilTag检测
  {
		target->target_ctrl_enable=target->flag;		
		if(target->flag!=0)  
		{
			if(target->trust_cnt<20)	 
			{
				target->trust_cnt++;
				target->trust_flag=0;
			}
			else target->trust_flag=1;
		}
		else 
		{
			target->trust_cnt/=2;
		  target->trust_flag=0;
		}

    target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
    target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;
			
		Sensor_Parameter_Sort(target->x,target->y,Pitch,Roll,NamelessQuad.Position[_YAW]);
  }
	else if(*(data_buf+2)==0XA3)//线检测
  {
		target->target_ctrl_enable=target->flag;		
		if(target->flag!=0)  
		{
			if(target->trust_cnt<20)	 
			{
				target->trust_cnt++;
				target->trust_flag=0;
			}
			else target->trust_flag=1;
		}
		else 
		{
			target->trust_cnt/=2;
		  target->trust_flag=0;
		}

    target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
    target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;
			
		Sensor_Parameter_Sort(target->x,target->y,Pitch,Roll,NamelessQuad.Position[_YAW]);
		
		if(target->angle>90) Opv_Top_View_Target.sdk_angle=target->angle-180;
		else Opv_Top_View_Target.sdk_angle=target->angle;	
  }
	else if(*(data_buf+2)==0XA4)//AprilTag检测
  {
		target->target_ctrl_enable=target->flag;		
		if(target->flag!=0)  
		{
			if(target->trust_cnt<20)	 
			{
				target->trust_cnt++;
				target->trust_flag=0;
			}
			else target->trust_flag=1;
		}
		else 
		{
			target->trust_cnt/=2;
		  target->trust_flag=0;
		}

    target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
    target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;
			
		Sensor_Parameter_Sort(target->x,target->y,Pitch,Roll,NamelessQuad.Position[_YAW]);
  }	
}


void Openmv_Data_Receive_Anl_2(uint8_t *data_buf,uint8_t num,Target_Check *target)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//不满足帧头条件
	
	target->x				  =*(data_buf+4)<<8|*(data_buf+5);
  target->y					=*(data_buf+6)<<8|*(data_buf+7);
	target->pixel			=*(data_buf+8)<<8|*(data_buf+9);  
	target->flag			=*(data_buf+10);
	target->state		  =*(data_buf+11);		
	target->angle		  =*(data_buf+12)<<8|*(data_buf+13);
	target->distance  =*(data_buf+14)<<8|*(data_buf+15);
	target->apriltag_id=*(data_buf+16)<<8|*(data_buf+17);
	target->width	    =*(data_buf+18)<<8|*(data_buf+19);
	target->height	  =*(data_buf+20)<<8|*(data_buf+21);
	target->fps  		  =*(data_buf+22);
	target->reserved1 =*(data_buf+23);
	target->reserved2 =*(data_buf+24);
	target->reserved3 =*(data_buf+25);
	target->reserved4 =*(data_buf+26);
	//扩展距离传感器
	target->range_sensor1 =*(data_buf+27)<<8|*(data_buf+28);
	target->range_sensor2 =*(data_buf+29)<<8|*(data_buf+30);
	target->range_sensor3 =*(data_buf+31)<<8|*(data_buf+32);
	target->range_sensor4 =*(data_buf+33)<<8|*(data_buf+34);

	target->x_pixel_size=Pixel_Size*(Pixel_Image_Width/target->width);
	target->y_pixel_size=Pixel_Size*(Pixel_Image_Height/target->height);
	
	
  target->apriltag_distance=AprilTag_Side_Length*Focal_Length/(target->x_pixel_size*FastSqrt(target->pixel));
	
	
	target->sdk_mode=*(data_buf+2);
	
	if(*(data_buf+2)==0XA4)//AprilTag检测
  {
		target->target_ctrl_enable=target->flag;		
		if(target->flag!=0)  
		{
			if(target->trust_cnt<20)	 
			{
				target->trust_cnt++;
				target->trust_flag=0;
			}
			else target->trust_flag=1;
		}
		else 
		{
			target->trust_cnt/=2;
		  target->trust_flag=0;
		}

    target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
    target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;
    		
    target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length;
    target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length;


		
  }
	else
	{
	
	}
		
}


static uint8_t state = 0;
uint8_t _buf[2][50];
void SDK_Data_Receive_Prepare(uint8_t data,uint8_t label)
{
  static uint8_t _data_len = 0,_data_cnt = 0;
  if(state==0&&data==0xFF)//帧头1
  {
    state=1;
    _buf[label][0]=data;
  }
  else if(state==1&&data==0xFC)//帧头2
  {
    state=2;
    _buf[label][1]=data;
  }
  else if(state==2&&data<0XFF)//功能字节
  {
    state=3;
    _buf[label][2]=data;
  }
  else if(state==3&&data<50)//数据长度
  {
    state = 4;
    _buf[label][3]=data;
    _data_len = data;
    _data_cnt = 0;
  }
  else if(state==4&&_data_len>0)//有多少数据长度，就存多少个
  {
    _data_len--;
    _buf[label][4+_data_cnt++]=data;
    if(_data_len==0) state = 5;
  }
  else if(state==5)//最后接收数据校验和
  {
    state = 0;
    _buf[label][4+_data_cnt]=data;
		if(label==0)  Openmv_Data_Receive_Anl_1(_buf[label],_data_cnt+5,&Opv_Top_View_Target);
		else Openmv_Data_Receive_Anl_2(_buf[label],_data_cnt+5,&Opv_Front_View_Target);
  }
  else state = 0;
}


uint16_t SDK_Data_Offset=0;
void SDK_Data_Prase_1(void)
{
  static uint16_t sdk_prase_cnt=0;
  uint16_t i=0;
  sdk_prase_cnt++;
  if(sdk_prase_cnt>=2)//5*2=10ms
  {
    if(COM3_Rx_Buf.Tail<SDK_Target_Length)//0-11数据位正在传输
    {
      SDK_Data_Offset=SDK_Target_Length;
    }
    else//12-23数据位正在传输
    {
      SDK_Data_Offset=0;
    }
    for(i=0;i<SDK_Target_Length;i++)
    {
      SDK_Data_Receive_Prepare(COM3_Rx_Buf.Ring_Buff[SDK_Data_Offset+i],0);
    }
    sdk_prase_cnt=0;
  }
}

void SDK_Data_Prase_2(void)
{
  static uint16_t sdk_prase_cnt=0;
  uint16_t i=0;
  sdk_prase_cnt++;
  if(sdk_prase_cnt>=2)//5*2=10ms
  {
    if(COM1_Rx_Buf.Tail<SDK_Target_Length)//0-11数据位正在传输
    {
      SDK_Data_Offset=SDK_Target_Length;
    }
    else//12-23数据位正在传输
    {
      SDK_Data_Offset=0;
    }
    for(i=0;i<SDK_Target_Length;i++)
    {
      SDK_Data_Receive_Prepare(COM1_Rx_Buf.Ring_Buff[SDK_Data_Offset+i],1);
    }
    sdk_prase_cnt=0;
  }
}





void SDK_Init(void)
{
  float sdk_mode_default=0;
  SDK_DT_Reset();//复位SDK线检测数据
  ReadFlashParameterOne(SDK1_MODE_DEFAULT,&sdk_mode_default);
  if(isnan(sdk_mode_default)==0)
  {
    SDK1_Mode_Setup=(uint8_t)(sdk_mode_default);
    SDK_DT_Send_Check(SDK1_Mode_Setup,UART3_SDK);//初始化opemmv工作模式，默认以上次工作状态配置
  }
	
  ReadFlashParameterOne(SDK2_MODE_DEFAULT,&sdk_mode_default);
  if(isnan(sdk_mode_default)==0)
  {
    SDK2_Mode_Setup=(uint8_t)(sdk_mode_default);
    SDK_DT_Send_Check(SDK2_Mode_Setup,UART1_SDK);//初始化opemmv工作模式，默认以上次工作状态配置
  } 	
}


















//void Sensor_Parameter_Sort(uint16_t tx,uint16_t ty,float pitch,float roll,float alt)
//{
//	float theta_x_max=0,theta_y_max=0;
//	theta_x_max=Pixel_Image_View_Angle_X/2;
//	theta_y_max=Pixel_Image_View_Angle_Y/2;
//  _P1=0.5f*Pixel_Image_Width/FastTan(theta_x_max*DEG2RAD);
//	_P2=0.5f*Pixel_Image_Width/FastTan(theta_y_max*DEG2RAD);
//	_CX=Pixel_Image_Width/2;
//	_CY=Pixel_Image_Height/2;
//	
//	float tmp_x=0,tmp_y=0;
//	tmp_x=fast_atan((_CX-tx)/_P1);
//	tmp_y=fast_atan((_CY-ty)/_P2);
//	
//	_TX= FastTan(tmp_x+roll*DEG2RAD) *_P1;
//	_TY= FastTan(tmp_y+pitch*DEG2RAD)*_P2;
//	
//	_DX=alt*_TX/_P1;
//	_DY=alt*_TY/_P2;
//		
//	Opv_Top_View_Target.sdk_target.x=_DX;
//  Opv_Top_View_Target.sdk_target.y=_DY;
//}
/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
//    target->sdk_target.x=(target->x_pixel_size*(0.5*target->width-target->x)*NamelessQuad.Position[_YAW])/Focal_Length
//													+NamelessQuad.Position[_YAW]*FastTan(Roll* DEG2RAD);
//    target->sdk_target.y=(target->y_pixel_size*(0.5*target->height-target->y)*NamelessQuad.Position[_YAW])/Focal_Length
//													+NamelessQuad.Position[_YAW]*FastTan(Pitch* DEG2RAD);  		
//    target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x +0.5*target->width *(FastTan(Roll*DEG2RAD) /FastTan(Pixel_Image_View_Angle_X*DEG2RAD)))*NamelessQuad.Position[_YAW])/Focal_Length;
//    target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y+0.5*target->height*(FastTan(Pitch*DEG2RAD)/FastTan(Pixel_Image_View_Angle_Y*DEG2RAD)))*NamelessQuad.Position[_YAW])/Focal_Length;	


