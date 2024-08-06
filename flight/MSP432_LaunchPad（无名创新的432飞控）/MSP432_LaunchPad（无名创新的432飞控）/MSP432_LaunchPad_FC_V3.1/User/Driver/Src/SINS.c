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
#include "Earth_Declination.h"
#include "SINS.h"

/***********************************************************
@函数名：Vector_From_BodyFrame2EarthFrame
@入口参数：Vector3f *bf,Vector3f *ef
@出口参数：无
功能描述：载体系向导航系转换
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef)
{
  ef->x=rMat[0][0]*bf->x+rMat[0][1]*bf->y+rMat[0][2]*bf->z;
  ef->y=rMat[1][0]*bf->x+rMat[1][1]*bf->y+rMat[1][2]*bf->z;
  ef->z=rMat[2][0]*bf->x+rMat[2][1]*bf->y+rMat[2][2]*bf->z;
}

/***********************************************************
@函数名：Vector_From_EarthFrame2BodyFrame
@入口参数：Vector3f *ef,Vector3f *bf
@出口参数：无
功能描述：导航系向载体系转换
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf)
{
  bf->x=rMat[0][0]*ef->x+rMat[1][0]*ef->y+rMat[2][0]*ef->z;
  bf->y=rMat[0][1]*ef->x+rMat[1][1]*ef->y+rMat[2][1]*ef->z;
  bf->z=rMat[0][2]*ef->x+rMat[1][2]*ef->y+rMat[2][2]*ef->z;
}

SINS NamelessQuad;
Butter_BufferData accel_feedback_filter_buf[3];
float Acceleration_Feedback[3];
Vector2f SINS_Accel_Body={0,0};
/***********************************************************
@函数名：SINS_Prepare
@入口参数：无
@出口参数：无
功能描述：惯导加速度准备
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void  SINS_Prepare(void)
{
  Vector2f SINS_Accel_Earth={0,0};
  Vector3f Body_Frame,Earth_Frame;
  /*Z-Y-X欧拉角转方向余弦矩阵
  //Pitch Roll  Yaw 分别对应Φ θ Ψ  
  X轴旋转矩阵
  R（Φ）
  {1      0        0    }
  {0      cosΦ    sinΦ}
  {0    -sinΦ    cosΦ }
  
  Y轴旋转矩阵
  R（θ）
  {cosθ     0        -sinθ}
  {0         1        0     }
  {sinθ     0        cosθ}
  
  Z轴旋转矩阵
  R（θ）
  {cosΨ      sinΨ       0}
  {-sinΨ     cosΨ       0}
  {0          0           1 }
  
  由Z-Y-X顺规有:
  载体坐标系到导航坐标系下旋转矩阵R(b2n)
  R(b2n) =R(Ψ)^T*R(θ)^T*R(Φ)^T
  
  R=
  {cosΨ*cosθ     -cosΦ*sinΨ+sinΦ*sinθ*cosΨ        sinΨ*sinΦ+cosΦ*sinθ*cosΨ}
  {cosθ*sinΨ     cosΦ*cosΨ +sinΦ*sinθ*sinΨ       -cosΨ*sinΦ+cosΦ*sinθ*sinΨ}
  {-sinθ          cosθsin Φ                          cosθcosΦ                   }
  */
  Body_Frame.x=ins_accel_filter.x;
  Body_Frame.y=ins_accel_filter.y;
  Body_Frame.z=ins_accel_filter.z;
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  NamelessQuad.Origion_Acceleration[_YAW]=Earth_Frame.z;
  NamelessQuad.Origion_Acceleration[_PITCH]=Earth_Frame.x;
  NamelessQuad.Origion_Acceleration[_ROLL]=Earth_Frame.y;
  
  
  NamelessQuad.Origion_Acceleration[_YAW]*=AcceGravity/AcceMax;
  NamelessQuad.Origion_Acceleration[_YAW]-=AcceGravity;//减去重力加速度
  NamelessQuad.Origion_Acceleration[_YAW]*=100;//加速度cm/s^2
  
  NamelessQuad.Origion_Acceleration[_PITCH]*=AcceGravity/AcceMax;
  NamelessQuad.Origion_Acceleration[_PITCH]*=100;//加速度cm/s^2
  
  NamelessQuad.Origion_Acceleration[_ROLL]*=AcceGravity/AcceMax;
  NamelessQuad.Origion_Acceleration[_ROLL]*=100;//加速度cm/s^2
  
  /******************************************************************************/
  //将无人机在导航坐标系下的沿着正东、正北方向的运动加速度旋转到当前航向的运动加速度:机头(俯仰)+横滚
  
  SINS_Accel_Earth.x=NamelessQuad.Origion_Acceleration[_PITCH];//沿地理坐标系，正东方向运动加速度,单位为CM
  SINS_Accel_Earth.y=NamelessQuad.Origion_Acceleration[_ROLL];//沿地理坐标系，正北方向运动加速度,单位为CM
  
  SINS_Accel_Body.x= SINS_Accel_Earth.x*Cos_Yaw+SINS_Accel_Earth.y*Sin_Yaw;  //横滚正向运动加速度  X轴正向
  SINS_Accel_Body.y=-SINS_Accel_Earth.x*Sin_Yaw+SINS_Accel_Earth.y*Cos_Yaw; //机头正向运动加速度  Y轴正向
  
  
  Body_Frame.x=LPButterworth(accel.x,&accel_feedback_filter_buf[0],&Butter_5HZ_Parameter_Acce);
  Body_Frame.y=LPButterworth(accel.y,&accel_feedback_filter_buf[1],&Butter_5HZ_Parameter_Acce);
  Body_Frame.z=LPButterworth(accel.z,&accel_feedback_filter_buf[2],&Butter_5HZ_Parameter_Acce);
//	Body_Frame.x=accel.x;
//	Body_Frame.y=accel.y;
//	Body_Frame.z=accel.z;		
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  Acceleration_Feedback[_YAW]=Earth_Frame.z;
  Acceleration_Feedback[_PITCH]=Earth_Frame.x;
  Acceleration_Feedback[_ROLL]=Earth_Frame.y;
  Acceleration_Feedback[_YAW]*=AcceGravity/AcceMax;
  Acceleration_Feedback[_YAW]-=AcceGravity;//减去重力加速度
  Acceleration_Feedback[_YAW]*=100;//加速度cm/s^2
  Acceleration_Feedback[_PITCH]*=AcceGravity/AcceMax;
  Acceleration_Feedback[_PITCH]*=100;//加速度cm/s^2
  Acceleration_Feedback[_ROLL]*=AcceGravity/AcceMax;
  Acceleration_Feedback[_ROLL]*=100;//加速度cm/s^2
}





#define Ra  1.0f//1.0 0.5
#define KALMAN_DT 0.1f
float Q[2]={0.5f*Ra*KALMAN_DT*KALMAN_DT,Ra*KALMAN_DT};//{1.0e-6f,5.0e-4f};{5.0e-6f,6.0e-4f}

float R_Baro=15;//80
float R_Ground=1.0;//1.5
float R=1.0;// 15  5  30  10  500

#define  Observation_Err_Max   500//cm
float Acce_Bias_Gain_Ki[3]={
  0.025,//0.075    0.005
  0.0005,//0.001
  0.0005,//0.001
};
float Acce_Bias_Gain_Kp={
  0.03,//0.1~0.75
};
float Pre_conv[4]=
{
  0.5,0.25,
  0.25,0.5
};//上一次协方差
/***********************************************************
@函数名：KalmanFilter
@入口参数：float Observation,//位置观测量
					 uint16 Pos_Delay_Cnt,//观测传感器延时量
					 SINS *Ins_Kf,//惯导结构体
					 float System_drive,//系统原始驱动量，惯导加速度
					 float *R,
					 float Q,
					 float dt,
					 uint16 N,
					 uint8_t update_flag
@出口参数：无
功能描述：高度方向惯导卡尔曼滤波
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
float observation_div=0;
float last_observation=0;
float observation_acc;
uint8_t observation_reset_flag=0;
void  KalmanFilter(float Observation,//位置观测量
                   uint16 Pos_Delay_Cnt,//观测传感器延时量
                   SINS *Ins_Kf,//惯导结构体
                   float System_drive,//系统原始驱动量，惯导加速度
                   float *R,
                   float Q,
                   float dt,
                   uint16 N,
                   uint8_t *update_flag)
{
  uint16 Cnt=0;
  static uint16 Speed_Sync_Cnt=0;
	float Q_Temp=0;
  float Temp_conv[4]={0};//先验协方差
  float Conv_Z=0,Z_Cor=0;
  float k[2]={0};//增益矩阵
  float Ctemp=0;

	Q_Temp=constrain_float(Q,0,1500);
  //先验状态
  Ins_Kf->Acceleration[N]=System_drive;
  Ins_Kf->Acceleration[N]=Ins_Kf->Acce_Bias_All[N]+Ins_Kf->Acceleration[N];
  Ins_Kf->Position[N] +=Ins_Kf->Speed[N]*dt+(Ins_Kf->Acceleration[N]*dt*dt)/2.0f;
  Ins_Kf->Speed[N]+=Ins_Kf->Acceleration[N]*dt;
  if(*update_flag==1)
  {
    //先验协方差
    Ctemp=Pre_conv[1]+Pre_conv[3]*KALMAN_DT;
    Temp_conv[0]=Pre_conv[0]+Pre_conv[2]*KALMAN_DT+Ctemp*KALMAN_DT+R[0];
    Temp_conv[1]=Ctemp;
    Temp_conv[2]=Pre_conv[2]+Pre_conv[3]*KALMAN_DT;;
    Temp_conv[3]=Pre_conv[3]+R[1];
    //计算卡尔曼增益
    Conv_Z=Temp_conv[0]+Q_Temp;
    k[0]=Temp_conv[0]/Conv_Z;
    k[1]=Temp_conv[2]/Conv_Z;
    //融合数据输出
//		Z_Cor=constrain_float(Observation-Ins_Kf->Pos_History[N][Pos_Delay_Cnt],-Observation_Err_Max,Observation_Err_Max);	
		Z_Cor=Observation-Ins_Kf->Pos_History[N][Pos_Delay_Cnt];	
    Ins_Kf->Position[N] +=k[0]*Z_Cor;
    Ins_Kf->Speed[N] +=k[1]*Z_Cor;
    Ins_Kf->Acce_Bias[N]+=Acce_Bias_Gain_Ki[_YAW]*Z_Cor;
		Ins_Kf->Acce_Bias[N]=constrain_float(Ins_Kf->Acce_Bias[N],-100,100);
		Ins_Kf->Acce_Bias_All[N]=Acce_Bias_Gain_Kp*Z_Cor+Ins_Kf->Acce_Bias[N];
		
		
    //更新状态协方差矩阵
    Pre_conv[0]=(1-k[0])*Temp_conv[0];
    Pre_conv[1]=(1-k[0])*Temp_conv[1];
    Pre_conv[2]=Temp_conv[2]-k[1]*Temp_conv[0];
    Pre_conv[3]=Temp_conv[3]-k[1]*Temp_conv[1]; 
		*update_flag=0;
  }
  
  for(Cnt=Num-1;Cnt>0;Cnt--)//5ms滑动一次
  {
    Ins_Kf->Pos_History[N][Cnt]=Ins_Kf->Pos_History[N][Cnt-1];
  }
  Ins_Kf->Pos_History[N][0]=Ins_Kf->Position[N];
  
  Speed_Sync_Cnt++;
  if(Speed_Sync_Cnt>=2)//10ms滑动一次
  {
    for(Cnt=Num-1;Cnt>0;Cnt--)
    {
      Ins_Kf->Vel_History[N][Cnt]=Ins_Kf->Vel_History[N][Cnt-1];
    }
    Speed_Sync_Cnt=0;
  }
  Ins_Kf->Vel_History[N][0]=Ins_Kf->Speed[N];
}



uint16 SPL06_Sync_Cnt=5;//5  30
uint16 HCSR04_Sync_Cnt=5;//0
uint16 High_Delay_Cnt=0;
float Observation_Altitude=0;
uint8_t altitude_updtate_flag=0;
/***********************************************************
@函数名：Observation_Tradeoff
@入口参数：uint8_t HC_SR04_Enable
@出口参数：无
功能描述：高度方向观测传感器自由切换
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Observation_Tradeoff(uint8_t HC_SR04_Enable)
{
  static uint8_t observation_flag=1,last_observation_flag=1;
  uint16 Cnt=0;
  last_observation_flag=observation_flag;
  if(HC_SR04_Enable==0)//无超声波/对地传感器参与
  {
    Observation_Altitude=WP_Sensor.baro_altitude;//高度观测量
    High_Delay_Cnt=SPL06_Sync_Cnt;
		observation_div=WP_Sensor.baro_altitude_div;
		observation_acc=WP_Sensor.baro_altitude_acc;
		if(WP_Sensor.baro_updtate_flag==1)
		{
			altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
			WP_Sensor.baro_updtate_flag=0;
		}
		R=R_Baro;
  }
  else//有超声波/对地传感器参与
  {
#if Ground_Distance_Sensor==US100    
    if(Sensor_Flag.Ground_Health==1)
    {
			float _US_Distance=Ground_Distance*Cos_Pitch*Cos_Roll;
      Observation_Altitude=_US_Distance;
      High_Delay_Cnt=HCSR04_Sync_Cnt;
      observation_flag=2;
			observation_div=Ground_Distance_Div;
			observation_acc=Ground_Distance_Acc;
			if(WP_Sensor.us100_updtate_flag==1)
			{
				altitude_updtate_flag=WP_Sensor.us100_updtate_flag;
				WP_Sensor.us100_updtate_flag=0;
			}
			R=R_Ground;
    }
#else		
    if(tfdata.health==1)
    {
      Observation_Altitude=tfdata.distance;
      High_Delay_Cnt=5;
      observation_flag=2;
			observation_div=tfdata.div;
			observation_acc=tfdata.acc;
			if(WP_Sensor.tfmini_updtate_flag==1)
			{
				altitude_updtate_flag=WP_Sensor.tfmini_updtate_flag;
				WP_Sensor.tfmini_updtate_flag=0;
			}
			R=R_Ground;
    }
#endif		
		
    else
    {
      Observation_Altitude=WP_Sensor.baro_altitude;//高度观测量
      High_Delay_Cnt=SPL06_Sync_Cnt;
			observation_div=WP_Sensor.baro_altitude_div;
			observation_acc=WP_Sensor.baro_altitude_acc;
			if(WP_Sensor.baro_updtate_flag==1)
			{
				altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
				WP_Sensor.baro_updtate_flag=0;
			}
			R=R_Baro;
    } 
    
    if(observation_flag==2&&last_observation_flag==1)//气压计切超声波
    {
#if Ground_Distance_Sensor==US100
			float _US_Distance=Ground_Distance*Cos_Pitch*Cos_Roll;			
      NamelessQuad.Position[_YAW]=_US_Distance;
      for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_History[_YAW][Cnt]=_US_Distance;}
      NamelessQuad.Pos_History[_YAW][0]=_US_Distance;
      Total_Controller.High_Position_Control.Expect=_US_Distance;//将惯导高度设置为期望高度，有且仅设置一次
      
      NamelessQuad.Position[_YAW]=_US_Distance;
      NamelessQuad.Speed[_YAW]=0;
#else	
      NamelessQuad.Position[_YAW]=tfdata.distance;
      for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_History[_YAW][Cnt]=tfdata.distance;}
      NamelessQuad.Pos_History[_YAW][0]=tfdata.distance;
      Total_Controller.High_Position_Control.Expect=tfdata.distance;//将惯导高度设置为期望高度，有且仅设置一次
      
      Origion_NamelessQuad.Position[_YAW]=tfdata.distance;
      Origion_NamelessQuad.Speed[_YAW]=0;
#endif
			R=R_Ground;
    }
    else if(observation_flag==1&&last_observation_flag==2)//超声波切气压计
    {
      NamelessQuad.Position[_YAW]=Observation_Altitude;
      for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_History[_YAW][Cnt]=Observation_Altitude;}
      NamelessQuad.Pos_History[_YAW][0]=Observation_Altitude;
      Total_Controller.High_Position_Control.Expect=Observation_Altitude;//将惯导高度设置为期望高度，有且仅设置一次
      NamelessQuad.Position[_YAW]=Observation_Altitude;
      NamelessQuad.Speed[_YAW]=0;
			R=R_Baro;
    }
  }
}

Testime SINS_High_Delta;
/***********************************************************
@函数名：Strapdown_INS_High_Kalman
@入口参数：无
@出口参数：无
功能描述：高度方向卡尔曼滤波
@作者：无名小哥
@日期：2019年01月27日
*************************************************************/
void Strapdown_INS_High_Kalman(void)
{
	float Delta_T;
  Test_Period(&SINS_High_Delta);
  Delta_T=SINS_High_Delta.Time_Delta/1000.0f;
  if(Delta_T>2*WP_Duty_Dt||Delta_T<WP_Duty_Dt||isnan(Delta_T)!=0)   Delta_T=WP_Duty_Dt;
  Observation_Tradeoff(1);
	if(Gyro_Safety_Calibration_Flag==1)
	{
		KalmanFilter(Observation_Altitude,//位置观测量
								 High_Delay_Cnt,//观测传感器延时量
								 &NamelessQuad,//惯导结构体
								 NamelessQuad.Origion_Acceleration[_YAW],//系统原始驱动量，惯导加速度
								 Q,
								 R,
								 Delta_T,
								 _YAW,
								 &altitude_updtate_flag);
	}
}







void Strapdown_INS_Reset(SINS *Ins,uint8_t Axis,float Pos_Target,float Vel_Target)
{
  uint16_t Cnt=0;
  Ins->Position[Axis]=Pos_Target;//位置重置
  Ins->Speed[Axis]=Vel_Target;//速度重置
  Ins->Acceleration[Axis]=0.0f;//加速度清零
  Ins->Acce_Bias[Axis]=0.0f;
  for(Cnt=Num-1;Cnt>0;Cnt--)//历史位置值，全部装载为当前观测值
  {
    Ins->Pos_History[Axis][Cnt]=Pos_Target;
  }
  Ins->Pos_History[Axis][0]=Pos_Target;
  for(Cnt=Num-1;Cnt>0;Cnt--)//历史速度值，全部装载为当前观测值
  {
    Ins->Vel_History[Axis][Cnt]=Vel_Target;
  }
  Ins->Vel_History[Axis][0]=Vel_Target;
}


