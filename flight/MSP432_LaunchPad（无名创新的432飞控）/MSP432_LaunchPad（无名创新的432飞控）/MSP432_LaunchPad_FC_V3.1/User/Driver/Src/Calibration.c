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
#include "Calibration.h"
#include "CalibrationRoutines.h"

int16_t Mag_Offset[3]={0,0,0};
float Mag_Data[3]={0};
Vector2f MagN={0,0};
Vector3f Accel_Hor_Read;
/***************加速度计6面矫正，参考APM代码，配合遥控器进行现场矫正**************************/
/***********************************************************
@函数名：Calibrate_Reset_Matrices
@入口参数：float dS[6], float JS[6][6]
@出口参数：无
@功能描述：矩阵数据复位
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
  int16_t j,k;
  for( j=0; j<6; j++ )
  {
    dS[j] = 0.0f;
    for( k=0; k<6; k++ )
    {
      JS[j][k] = 0.0f;
    }
  }
}

/***********************************************************
@函数名：Calibrate_Find_Delta
@入口参数：float dS[6], float JS[6][6], float delta[6]
@出口参数：无
@功能描述：求解矩阵方程JS*x = dS，第一步把矩阵化上三角阵，
将JS所在的列下方的全部消为0，然后回代得到线性方程的解
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
  //Solve 6-d matrix equation JS*x = dS
  //first put in upper triangular form
  int16_t i,j,k;
  float mu;
  //make upper triangular
  for( i=0; i<6; i++ ) {
	//eliminate all nonzero entries below JS[i][i]
	for( j=i+1; j<6; j++ ) {
		mu = JS[i][j]/JS[i][i];
		if( mu != 0.0f ) {
			dS[j] -= mu*dS[i];
			for( k=j; k<6; k++ ) {
				JS[k][j] -= mu*JS[k][i];
			}
		}
	}
  }
  //back-substitute
  for( i=5; i>=0; i-- ) {
    dS[i] /= JS[i][i];
    JS[i][i] = 1.0f;
    
    for( j=0; j<i; j++ ) {
      mu = JS[i][j];
      dS[j] -= mu*dS[i];
      JS[i][j] = 0.0f;
    }
  }
  for( i=0; i<6; i++ ) {
    delta[i] = dS[i];
  }
}

void Calibrate_Update_Matrices(float dS[6],
                               float JS[6][6],
                               float beta[6],
                               float data[3])
{
  int16_t j, k;
  float dx, b;
  float residual = 1.0;
  float jacobian[6];
  for(j=0;j<3;j++)
  {
    b = beta[3+j];
    dx = (float)data[j] - beta[j];
    residual -= b*b*dx*dx;
    jacobian[j] = 2.0f*b*b*dx;
    jacobian[3+j] = -2.0f*b*dx*dx;
  }
  
  for(j=0;j<6;j++)
  {
    dS[j]+=jacobian[j]*residual;
    for(k=0;k<6;k++)
    {
      JS[j][k]+=jacobian[j]*jacobian[k];
    }
  }
}

uint8 Calibrate_accel(Acce_Unit accel_sample[6],
                      Acce_Unit *accel_offsets,
                      Acce_Unit *accel_scale)
{
  int16_t i;
  int16_t num_iterations = 0;
  float eps = 0.000000001;
  float change = 100.0;
  float data[3]={0};
  float beta[6]={0};
  float delta[6]={0};
  float ds[6]={0};
  float JS[6][6]={0};
  bool success = TRUE;
  // reset
  beta[0] = beta[1] = beta[2] = 0;
  beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
  while( num_iterations < 20 && change > eps ) {
    num_iterations++;
    Calibrate_Reset_Matrices(ds, JS);
    for( i=0; i<6; i++ ) {
      data[0] = accel_sample[i].x;
      data[1] = accel_sample[i].y;
      data[2] = accel_sample[i].z;
      Calibrate_Update_Matrices(ds, JS, beta, data);
    }
    Calibrate_Find_Delta(ds, JS, delta);
    change =    delta[0]*delta[0] +
      delta[0]*delta[0] +
        delta[1]*delta[1] +
          delta[2]*delta[2] +
            delta[3]*delta[3] / (beta[3]*beta[3]) +
              delta[4]*delta[4] / (beta[4]*beta[4]) +
                delta[5]*delta[5] / (beta[5]*beta[5]);
    for( i=0; i<6; i++ ) {
      beta[i] -= delta[i];
    }
  }
  // copy results out
  accel_scale->x = beta[3] * GRAVITY_MSS;
  accel_scale->y = beta[4] * GRAVITY_MSS;
  accel_scale->z = beta[5] * GRAVITY_MSS;
  accel_offsets->x = beta[0] * accel_scale->x;
  accel_offsets->y = beta[1] * accel_scale->y;
  accel_offsets->z = beta[2] * accel_scale->z;
  
  // sanity check scale
  if(fabsf(accel_scale->x-1.0f) > 0.5f
     || fabsf(accel_scale->y-1.0f) > 0.5f
       || fabsf(accel_scale->z-1.0f) > 0.5f )
  {
    success = FALSE;
  }
  // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(fabsf(accel_offsets->x) > 5.0f
     || fabsf(accel_offsets->y) > 5.0f
       || fabsf(accel_offsets->z) > 5.0f )
  {
    success = FALSE;
  }
  // return success or failure
  return success;
}

float Aoco[6]={1,1,1};
float Aoc[6][6]={1,1,1};
Acce_Unit new_offset={
  0,0,0,
};
Acce_Unit new_scales={
  1.0,1.0,1.0,
};

Acce_Unit Accel_Offset_Read={
  0,0,0,
};
Acce_Unit Accel_Scale_Read={
  0,0,0,
};
uint8_t Cal_Flag=0;
void Calibrationer(void)
{
  uint16 i=0;
  Acce_Unit Test_Calibration[6]=
  {
    20,     21,    4152,
    4062,  -24,      78,
    -4082, 1,        -8,
    -45,   -4071,   30,
    20,    4035,    -8,
    30,     -60,   -3980
  };
  for(i=0;i<6;i++)
  {
    Test_Calibration[i].x *=ACCEL_TO_1G;
    Test_Calibration[i].y *=ACCEL_TO_1G;
    Test_Calibration[i].z *=ACCEL_TO_1G;
  }
  
  Cal_Flag=Calibrate_accel(Test_Calibration,
                           &new_offset,
                           &new_scales);
}

uint8_t flight_direction=6;
uint8_t Accel_Calibration_Flag=0;//加速度计校准模式
uint8_t Accel_Calibration_Finished[6]={0,0,0,0,0,0};//对应面校准完成标志位
uint8_t Accel_Calibration_All_Finished=0;//6面校准全部校准完成标志位
uint16_t Accel_Calibration_Makesure_Cnt=0;
uint16_t Accel_flight_direction_cnt=0;
/***********************************************************
@函数名：Accel_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：加速度计标定函数遥控器动作位检测
数据
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Accel_Calibration_Check(void)
{
  uint16_t  i=0;
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
  {
    Accel_Calibration_Makesure_Cnt++;
  }
  if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max
				 &&Accel_Calibration_Makesure_Cnt>=200*3)||cal_flag==0x02)//持续三秒
	       &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
  {
    Bling_Mode=1;
    Accel_Calibration_Flag=1;//加速度校准模式
    Cal_Flag=0;	
		Bling_Set(&Light_2,1000,100,0.5,0,GPIO_PORT_PJ,GPIO_PIN4,1);
    flight_direction=6;
    Accel_Calibration_All_Finished=0;//全部校准完成标志位清零
    Accel_Calibration_Makesure_Cnt=0;
    for(i=0;i<6;i++)
    {
      Accel_Calibration_Finished[i]=0;//对应面标志位清零
      acce_sample[i].x=0; //清空对应面的加速度计量
      acce_sample[i].y=0; //清空对应面的加速度计量
      acce_sample[i].z=0; //清空对应面的加速度计量
    }
    Page_Number=10;//OLED加速度计矫正页面
    Reset_Mag_Calibartion(1);
    Reset_RC_Calibartion(1);
    Forced_Lock_Makesure_Cnt=0;
		cal_flag=0x00;
  }
  
  if(Accel_Calibration_Flag==1)
  {
    if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
			||cal_step==0x01)
		{
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x01)
      {
        flight_direction=0;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
      cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
				 ||cal_step==0x02)
		{
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x02)
      {
        flight_direction=1;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
				 ||cal_step==0x03)
		{
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x03)
      {
        flight_direction=2;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
    ||cal_step==0x04)
		{
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x04)
      {
        flight_direction=3;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
    ||cal_step==0x05)
		{
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x05)
      {
        flight_direction=4;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control>Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
			||cal_step==0x06)
		{
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x06)
      {
        flight_direction=5;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else
    {
      Accel_flight_direction_cnt/=2;
    }
    
    if(Accel_flight_direction_cnt>=200)  Accel_flight_direction_cnt=0;   
  }
}

Acce_Unit acce_sample[6]={0};//三行6列，保存6面待矫正数据
uint8_t Flash_Buf[12]={0};
/***********************************************************
@函数名：Accel_Calibartion
@入口参数：无
@出口参数：无
@功能描述：加速度标定、利用遥控器直接进入
@作者：WUST
@日期：2019年01月27日
*************************************************************/
uint8_t Accel_Calibartion(void)
{
  uint16 i,j=0;
  float acce_sample_sum[3]={0,0,0};//加速度和数据
  /*第一面飞控平放，Z轴正向朝着正上方，Z axis is about 1g,X、Y is about 0g*/
  /*第二面飞控平放，X轴正向朝着正上方，X axis is about 1g,Y、Z is about 0g*/
  /*第三面飞控平放，X轴正向朝着正下方，X axis is about -1g,Y、Z is about 0g*/
  /*第四面飞控平放，Y轴正向朝着正下方，Y axis is about -1g,X、Z is about 0g*/
  /*第五面飞控平放，Y轴正向朝着正上方，Y axis is about 1g,X、Z is about 0g*/
  /*第六面飞控平放，Z轴正向朝着正下方，Z axis is about -1g,X、Y is about 0g*/
  if(flight_direction<=5)//检测到对应面数据
  {
    uint16_t num_samples=0;
    while(num_samples<1000)//采样200次
    {
      if(Gyro_Length<=20.0f
         &&WP_Sensor.imu_updtate_flag==1)//通过陀螺仪模长来确保机体静止
      {
        for(j=0;j<3;j++){
          acce_sample_sum[j]+=WP_Sensor.acce_filter[j]*ACCEL_TO_1G;//加速度计转化为1g量程下
        }
        //delay_ms(4);//间隔10ms，1s内数据取平均
        num_samples++;
        WP_Sensor.imu_updtate_flag=0;
      }
      Accel_Calibration_Finished[flight_direction]=1;//对应面校准完成标志位置1
    }
    acce_sample[flight_direction].x=acce_sample_sum[0]/num_samples; //保存对应面的加速度计量
    acce_sample[flight_direction].y=acce_sample_sum[1]/num_samples; //保存对应面的加速度计量
    acce_sample[flight_direction].z=acce_sample_sum[2]/num_samples; //保存对应面的加速度计量
    flight_direction=6;//单面矫正完毕
  }
  
  if((Accel_Calibration_Finished[0]
      &Accel_Calibration_Finished[1]
        &Accel_Calibration_Finished[2]
          &Accel_Calibration_Finished[3]
            &Accel_Calibration_Finished[4]
              &Accel_Calibration_Finished[5])
     &&Accel_Calibration_All_Finished==0)//6面全部校准完毕
  {
    Accel_Calibration_All_Finished=1;//加速度计6面校准完成标志
    Accel_Calibration_Flag=0;//加速度计校准结束，释放遥感操作
    Cal_Flag=Calibrate_accel(acce_sample,
                             &new_offset,
                             &new_scales);//将所得6面数据
    for(i=0;i<6;i++)
    {
      Accel_Calibration_Finished[i]=0;//对应面标志位清零
    }
    if(Cal_Flag==TRUE)//加速度计校准成功
    {
      WriteFlashParameter_Three(ACCEL_X_OFFSET,
                                new_offset.x,
                                new_offset.y,
                                new_offset.z,
                                &Table_Parameter);
      WriteFlashParameter_Three(ACCEL_X_SCALE,
                                new_scales.x,
                                new_scales.y,
                                new_scales.z,
                                &Table_Parameter);
      
      Parameter_Init();//读取写入参数
      Bling_Mode=0;//恢复正常指示模式
			Bling_Set(&Light_2,3000,1000,0.5,0,GPIO_PORT_PJ,GPIO_PIN4,0);
			NCLink_Send_Check_Flag[9]=0x01;//校准完毕后发送校准数据
    }
    else//加速度计校准失败
    {
      Bling_Mode=0;//恢复正常指示模式
			Bling_Set(&Light_2,5000,200,0.5,0,GPIO_PORT_PJ,GPIO_PIN4,0);
      Page_Number=0;//OLED恢复首页
    }
    return TRUE;
  }
  return FALSE;
}

/***********************************************************
@函数名：Reset_Accel_Calibartion
@入口参数：uint8_t Type
@出口参数：无
@功能描述：加速度标定清空数据与强制复位
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Reset_Accel_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<6;i++)
  {
    Accel_Calibration_Finished[i]=0;//对应面标志位清零
    acce_sample[i].x=0; //清空对应面的加速度计量
    acce_sample[i].y=0; //清空对应面的加速度计量
    acce_sample[i].z=0; //清空对应面的加速度计量
  }
  Accel_Calibration_All_Finished=0;//全部校准完成标志位清零
  if(Type==1)  Accel_Calibration_Flag=0;
}

typedef struct
{
  uint8_t accel_off;
  uint8_t accel_scale;
  uint8_t mag;
}Parameter_Flag;

Parameter_Flag Parameter_Read_Flag;
float Accel_Simple_Mode=1;//默认加速度计简单校准模式
/***********************************************************
@函数名：Parameter_Init
@入口参数：无
@出口参数：无
@功能描述：传感器校准参数初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
bool Parameter_Init(void)//加速度计、磁力计校准参数获取
{
  bool success=TRUE;
  /************加速度计零偏与标度值*******/
  ReadFlashParameterThree(ACCEL_X_OFFSET,
                          &Accel_Offset_Read.x,
                          &Accel_Offset_Read.y,
                          &Accel_Offset_Read.z);
  
  ReadFlashParameterThree(ACCEL_X_SCALE,
                          &Accel_Scale_Read.x,
                          &Accel_Scale_Read.y,
                          &Accel_Scale_Read.z);	
  // sanity check scale
  if(ABS(Accel_Scale_Read.x-1.0f)>0.5f
     || ABS(Accel_Scale_Read.y-1.0f)>0.5f
       || ABS(Accel_Scale_Read.z-1.0f)>0.5f)
  {
    success = FALSE;
  }
  // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(ABS(Accel_Offset_Read.x) > 5.0f
     || ABS(Accel_Offset_Read.y) > 5.0f
       || ABS(Accel_Offset_Read.z) > 5.0f)
  {
    success = FALSE;
  }

  
  if(ReadFlashParameterThree(ACCEL_X_OFFSET,
                          &Accel_Offset_Read.x,
                          &Accel_Offset_Read.y,
                          &Accel_Offset_Read.z)==1
	&&ReadFlashParameterThree(ACCEL_X_SCALE,
                          &Accel_Scale_Read.x,
                          &Accel_Scale_Read.y,
                          &Accel_Scale_Read.z)==1)//Flash内数据正常，更新加速度校正值
  {    
    B[0]=Accel_Offset_Read.x;
    B[1]=Accel_Offset_Read.y;
    B[2]=Accel_Offset_Read.z;
    K[0]=Accel_Scale_Read.x;
    K[1]=Accel_Scale_Read.y;
    K[2]=Accel_Scale_Read.z;	
  }
  else if(success==TRUE)
  {
    B[0]=0;
    B[1]=0;
    B[2]=0;
    K[0]=1;
    K[1]=1;
    K[2]=1;
  }
	

	float _accel_simple_mode=0;	
  if(ReadFlashParameterOne(ACCEL_SIMPLE_MODE,&_accel_simple_mode))   Accel_Simple_Mode=_accel_simple_mode;
  vector3f _accel_hor;
	if(ReadFlashParameterThree(HOR_CAL_ACCEL_X,&_accel_hor.x,&_accel_hor.y,&_accel_hor.z))
	{
	  Accel_Hor_Read.x=_accel_hor.x;
		Accel_Hor_Read.y=_accel_hor.y;
		Accel_Hor_Read.z=_accel_hor.z;
	}
	
	
	
	if(Accel_Simple_Mode==1)//加速度计校准——简单模式
	{
		 if(isnan(Accel_Hor_Read.x)==0
			&&isnan(Accel_Hor_Read.y)==0
			&&isnan(Accel_Hor_Read.z)==0)//Flash内数据正常
		{
			B[0]=Accel_Hor_Read.x;
			B[1]=Accel_Hor_Read.y;
			B[2]=Accel_Hor_Read.z;
			K[0]=1;
			K[1]=1;
			K[2]=1;
			Pitch_Offset=0;
			Roll_Offset=0;			
		}
		else
		{
			B[0]=0;
			B[1]=0;
			B[2]=0;
			K[0]=1;
			K[1]=1;
			K[2]=1;
		}
	}
	
	
	
  /**********磁力计中心偏执获取************/
  if(ReadFlashParameterThree(MAG_X_OFFSET,
                          &Mag_Offset_Read.x,
                          &Mag_Offset_Read.y,
                          &Mag_Offset_Read.z)==1)
  {
    mag_offset.x=(int16_t)(Mag_Offset_Read.x);
    mag_offset.y=(int16_t)(Mag_Offset_Read.y);
    mag_offset.z=(int16_t)(Mag_Offset_Read.z);
  }
  else
  {
    mag_offset.x=0;
    mag_offset.y=0;
    mag_offset.z=0;    
  }
  return success;
}
/************加速度计6面矫正结束***********************/
/***********磁力计中心矫正，取单轴最大、最小值平均******/
uint8_t Mag_Calibration_Flag=0,Mag_Calibration_All_Finished;
uint8_t Mag_Calibration_Finished[3]={0};
uint16_t Mag_Calibration_Makesure_Cnt=0;
uint8_t  Mag_Calibration_Mode=3;
uint16_t Mag_Calibration_Cnt=0;
float Yaw_Correct=0;
/*********************************************/
const int16_t Mag_360_define[36]={
  0,10,20,30,40,50,60,70,80,90,
  100,110,120,130,140,150,160,170,180,190,
  200,210,220,230,240,250,260,270,280,290,
  300,310,320,330,340,350
};//磁力计矫正遍历角度，确保数据采集充分
uint8_t Last_Mag_360_Flag[3][36]={0};
uint8_t Mag_360_Flag[3][36]={0};
uint16_t Mag_Is_Okay_Flag[3];
Calibration Mag;
Mag_Unit DataMag;
Mag_Unit Mag_Offset_Read={
  0,0,0,
};
/***********************************************************
@函数名：Mag_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：磁力计标定函数遥控器动作位检测数据
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Mag_Calibration_Check(void)
{
  uint16_t  i=0,j=0;
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
    Mag_Calibration_Makesure_Cnt++;
  
  if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max
           &&Mag_Calibration_Makesure_Cnt>200*5)||cal_flag==0x03)//持续5S
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
    //进入磁力计校准模式
  {
    Bling_Mode=2;
    Mag_Calibration_Flag=1;//磁力计校准模式
    Mag_Calibration_Mode=3;
		Bling_Set(&Light_2,1000,500,0.5,0,GPIO_PORT_PJ,GPIO_PIN4,1);
    
    Mag_Calibration_Makesure_Cnt=0;
    Mag_Calibration_All_Finished=0;//全部校准完成标志位清零
    for(i=0;i<3;i++)
    {
      Mag_Calibration_Finished[i]=0;//对应面标志位清零
      for(j=0;j<36;j++) {Mag_360_Flag[i][j]=0;}
    }
    Page_Number=11;
    Reset_Accel_Calibartion(1);
    Reset_RC_Calibartion(1);
    Forced_Lock_Makesure_Cnt=0;
		cal_flag=0x00;
  }
  
  if(Mag_Calibration_Flag==1)
  {
    if((Throttle_Control<=(1000+Buttom_Safe_Deadband)
       &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
         &&Roll_Control==0
           &&Pitch_Control==0) //第一面矫正
					 ||cal_step==0x01) 
    {
      Mag_Calibration_Cnt++;
      if(Mag_Calibration_Cnt>=20||cal_step==0x01)
      {
        Mag_Calibration_Mode=0;
        Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
        for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
        LS_Init(&Mag_LS);//清空拟合中间变量
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
      cal_step=0x00;
    }
    else if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
            &&Yaw_Control>Yaw_Max*Scale_Pecent_Max
              &&Roll_Control==0
                &&Pitch_Control==0) //第二面矫正
    {
      Mag_Calibration_Cnt++;
      if(Mag_Calibration_Cnt>=20)
      {
        Mag_Calibration_Mode=1;
        Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
        Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
        for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
        for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
        LS_Init(&Mag_LS);//清空拟合中间变量
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
    }
    else
    {
      Mag_Calibration_Cnt/=2;
    }
    if(Mag_Calibration_Cnt>=200)  Mag_Calibration_Cnt=200;    
  } 
}

/***********************************************************
@函数名：Reset_Mag_Calibartion
@入口参数：uint8_t Type
@出口参数：无
@功能描述：磁力计标定清空数据与强制复位
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Reset_Mag_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Mag_360_Flag[0][i]=0;//清空采集角点
    Mag_360_Flag[1][i]=0;//清空采集角点
    Mag_360_Flag[2][i]=0;//清空采集角点
  }
  Mag_Is_Okay_Flag[0]=0;
  Mag_Is_Okay_Flag[1]=0;
  Mag_Is_Okay_Flag[2]=0;
  Mag_Calibration_Mode=3;
  if(Type==1)  Mag_Calibration_Flag=0;
}


uint8_t Check_Plane_Sampling_Okay(uint8_t plane_number)
{
  uint8_t finished_flag=0;
  if(Mag_360_Flag[plane_number][0]&Mag_360_Flag[plane_number][1]&Mag_360_Flag[plane_number][2]
     &Mag_360_Flag[plane_number][3]&Mag_360_Flag[plane_number][4]&Mag_360_Flag[plane_number][5]
       &Mag_360_Flag[plane_number][6]&Mag_360_Flag[plane_number][7]&Mag_360_Flag[plane_number][8]
         &Mag_360_Flag[plane_number][9]&Mag_360_Flag[plane_number][10]&Mag_360_Flag[plane_number][11]
           &Mag_360_Flag[plane_number][12]&Mag_360_Flag[plane_number][13]&Mag_360_Flag[plane_number][14]
             &Mag_360_Flag[plane_number][15]&Mag_360_Flag[plane_number][16]&Mag_360_Flag[plane_number][17]
               &Mag_360_Flag[plane_number][18]&Mag_360_Flag[plane_number][19]&Mag_360_Flag[plane_number][20]
                 &Mag_360_Flag[plane_number][21]&Mag_360_Flag[plane_number][22]&Mag_360_Flag[plane_number][23]
                   &Mag_360_Flag[plane_number][24]&Mag_360_Flag[plane_number][25]&Mag_360_Flag[plane_number][26]
                     &Mag_360_Flag[plane_number][27]&Mag_360_Flag[plane_number][28]&Mag_360_Flag[plane_number][29]
                       &Mag_360_Flag[plane_number][30]&Mag_360_Flag[plane_number][31]&Mag_360_Flag[plane_number][32]
                         &Mag_360_Flag[plane_number][33]&Mag_360_Flag[plane_number][34]&Mag_360_Flag[plane_number][35])
    finished_flag=1;
  return finished_flag;
}

/***********************************************************
@函数名：Mag_Calibartion
@入口参数：陀螺仪积分角度值、三轴磁力计原始值
@出口参数：无
@功能描述：磁力计中心标定、利用遥控器直接进入
@作者：WUST
@日期：2019年01月27日
*************************************************************/
uint8_t Mag_Calibartion(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }
  /********第一面Z轴正向朝着正上方，
  开始绕竖直轴旋转，Z axis is about 1g,X、Y is about 0g*/
  /********第二面Y轴正向朝着正上方，
  开始绕竖直轴旋转，Y axis is about 1g,X、Z is about 0g*/
  if(Mag_Calibration_Mode<3)//检测到对应面数据
  {
    for(i=0;i<36;i++)
    {
      if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=10.0f
         &&Mag_Calibration_Mode==0
           &&WP_Sensor.acce_filter[2]>=AcceMax_1G/2)//Z轴基本竖直
      {
        Mag_360_Flag[0][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=10.0f
         &&Mag_Calibration_Mode==1
           &&WP_Sensor.acce_filter[1]>=AcceMax_1G/2)//Y轴基本竖直
      {
        Mag_360_Flag[1][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=10.0f
         &&Mag_Calibration_Mode==2
           &&WP_Sensor.acce_filter[0]>=AcceMax_1G/2)//X轴基本竖直
      {
        Mag_360_Flag[2][i]=1;
      }
    }
    if(magdata->x >= Mag.x_max)   Mag.x_max = (int16_t)(magdata->x);
    if(magdata->x <  Mag.x_min)   Mag.x_min = (int16_t)(magdata->x);
    if(magdata->y >= Mag.y_max)   Mag.y_max = (int16_t)(magdata->y);
    if(magdata->y <  Mag.y_min)   Mag.y_min = (int16_t)(magdata->y);
    if(magdata->z >= Mag.z_max)   Mag.z_max = (int16_t)(magdata->z);
    if(magdata->z <  Mag.z_min)   Mag.z_min = (int16_t)(magdata->z);
  }
  if(Check_Plane_Sampling_Okay(0))
  {
    Mag_Is_Okay_Flag[0]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[1]==0)//另外一面数据未完成
      Mag_Calibration_Mode=1;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;//
  }
  
  if(Check_Plane_Sampling_Okay(1))
  {
    Mag_Is_Okay_Flag[1]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[2]==0)//另外一面数据未完成
      Mag_Calibration_Mode=2;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
	
  if(Check_Plane_Sampling_Okay(2))
  {
    Mag_Is_Okay_Flag[2]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[0]==0)//另外一面数据未完成
      Mag_Calibration_Mode=0;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
	
  if(Mag_Calibration_Mode==0)  Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;
  
  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//三面数据全部采集完毕，计算磁力计零点
  {
    Mag.x_offset=(Mag.x_min+Mag.x_max)/2.0;
    Mag.y_offset=(Mag.y_min+Mag.y_max)/2.0;
    Mag.z_offset=(Mag.z_min+Mag.z_max)/2.0;
    Mag_Offset_Read.x=Mag.x_offset;
    Mag_Offset_Read.y=Mag.y_offset;
    Mag_Offset_Read.z=Mag.z_offset;
    Mag_Is_Okay_Flag[0]=0;
    Mag_Is_Okay_Flag[1]=0;
    Mag_Is_Okay_Flag[2]=0;
    Mag_Calibration_Flag=0;//磁力计校准结束，释放遥感操作
    Bling_Mode=0;//恢复正常指示模式
    
    Mag_Offset[0]=(int16_t)(Mag_Offset_Read.x);
    Mag_Offset[1]=(int16_t)(Mag_Offset_Read.y);
    Mag_Offset[2]=(int16_t)(Mag_Offset_Read.z);
    
		Bling_Set(&Light_2,5000,1000,0.5,0,GPIO_PORT_PJ,GPIO_PIN4,0);   
    //写入加速度零点偏执与磁力计中心偏执 
    WriteFlashParameter_Three(MAG_X_OFFSET,
                              Mag.x_offset,
                              Mag.y_offset,
                              Mag.z_offset,
                              &Table_Parameter);
		NCLink_Send_Check_Flag[9]=0x01;//校准完毕后发送校准数据   
    return TRUE;
  }
  return FALSE;
}

void Mag_LS_Init()
{
  LS_Init(&Mag_LS);
}
float mag_a,mag_b,mag_c,mag_r;
/***********************************************************
@函数名：Mag_Calibartion_LS
@入口参数：陀螺仪积分角度值、三轴磁力计原始值
@出口参数：无
@功能描述：磁力计最小二乘法拟合球面、利用遥控器直接进入
@作者：WUST
@日期：2019年01月27日
*************************************************************/
uint8_t Mag_Calibartion_LS(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }
  
  /********第一面Z轴正向朝着正上方，
  开始绕竖直轴旋转，Z axis is about 1g,X、Y is about 0g*/
  /********第二面Y轴正向朝着正上方，
  开始绕竖直轴旋转，Y axis is about 1g,X、Z is about 0g*/
  if(Mag_Calibration_Mode<3)//检测到对应面数据
  {
    for(i=0;i<36;i++)
    {
      if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=10.f
         &&Mag_Calibration_Mode==0
           &&WP_Sensor.accel_raw.z>=AcceMax_1G/2)//Z轴基本竖直
      {
        Mag_360_Flag[0][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=10.0f
         &&Mag_Calibration_Mode==1
           &&WP_Sensor.accel_raw.y>=AcceMax_1G/2)//Y轴基本竖直
      {
        Mag_360_Flag[1][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=10.0f
         &&Mag_Calibration_Mode==2
           &&WP_Sensor.accel_raw.x>=AcceMax_1G/2)//X轴基本竖直
      {
        Mag_360_Flag[2][i]=1;
      }
    }
    
    for(i=0;i<36;i++)
    {
      if((Last_Mag_360_Flag[0][i]==0&&Mag_360_Flag[0][i]==1)
         ||(Last_Mag_360_Flag[1][i]==0&&Mag_360_Flag[1][i]==1)
           ||(Last_Mag_360_Flag[2][i]==0&&Mag_360_Flag[2][i]==1))
      {
        LS_Accumulate(&Mag_LS, magdata->x,magdata->y,magdata->z);
        //LS_Calculate(&Mag_LS,36*3,0.0f,&mag_a, &mag_b, &mag_c,&mag_r);
      }
    }
  }
  
  if(Check_Plane_Sampling_Okay(0))
  {
    Mag_Is_Okay_Flag[0]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[1]==0)//另外一面数据未完成
      Mag_Calibration_Mode=1;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;//
  }
  
  if(Check_Plane_Sampling_Okay(1))
  {
    Mag_Is_Okay_Flag[1]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[2]==0)//另外一面数据未完成
      Mag_Calibration_Mode=2;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
  
  if(Check_Plane_Sampling_Okay(2))
  {
    Mag_Is_Okay_Flag[2]=1;//单面数据采集完成标志位置1
    for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角点
    if(Mag_Is_Okay_Flag[0]==0)//另外一面数据未完成
      Mag_Calibration_Mode=0;//自动进入下一面数据采集模式
    else Mag_Calibration_Mode=3;
  }
  
  if(Mag_Calibration_Mode==0)  			Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;
  
  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//三面数据全部采集完毕，计算磁力计零点
  {
    LS_Calculate(&Mag_LS,36*3,0.0f,&mag_a, &mag_b, &mag_c,&mag_r);
    Mag_Offset_Read.x=mag_a;
    Mag_Offset_Read.y=mag_b;
    Mag_Offset_Read.z=mag_c;
    Mag_Is_Okay_Flag[0]=0;
    Mag_Is_Okay_Flag[1]=0;
    Mag_Is_Okay_Flag[2]=0;
    Mag_Calibration_Flag=0;//磁力计校准结束，释放遥感操作
    Bling_Mode=0;//恢复正常指示模式
    
    Mag_Offset[0]=(int16_t)(Mag_Offset_Read.x);
    Mag_Offset[1]=(int16_t)(Mag_Offset_Read.y);
    Mag_Offset[2]=(int16_t)(Mag_Offset_Read.z);
    
		Bling_Set(&Light_2,5000,1000,0.5,0,GPIO_PORT_PJ,GPIO_PIN4,0);
    WriteFlashParameter_Three(MAG_X_OFFSET,
                              Mag_Offset_Read.x,
                              Mag_Offset_Read.y,
                              Mag_Offset_Read.z,
                              &Table_Parameter);
		NCLink_Send_Check_Flag[9]=0x01;//校准完毕后发送校准数据
    return TRUE;
  }
  return FALSE;
}



#define  RC_TOP_DEFAULT       2000
#define  RC_BUTTOM_DEFAULT    1000
#define  RC_MIDDLE_DEFAULT    1500
#define  RC_DEADBAND_DEFAULT  100
#define  RC_DEADBAND_PERCENT   		0.1f		//中位死区占实际行程的百分比
#define  RC_THR_DEADBAND_PERCENT   0.2f		//油门中位死区占实际行程的百分比
#define  RC_RESET_DEFAULT  1500


uint8_t RC_Read_Flag[8];
/***********************************************************
@函数名：RC_Calibration_Init
@入口参数：无
@出口参数：无
@功能描述：遥控器行程标定初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void RC_Calibration_Init()
{
	float _rc_deadband_percent=0;
  uint16_t i=0;
  float max_read[8]={0},min_read[8]={0};
  for(i=0;i<8;i++)
  {	
    if(ReadFlashParameterTwo(RC_CH1_MAX+2*i,&max_read[i],&min_read[i])==1)  
      RC_Read_Flag[i]=0x01;
  }
  
  if(RC_Read_Flag[0]!=0x00
     &&RC_Read_Flag[1]!=0x00
       &&RC_Read_Flag[2]!=0x00
         &&RC_Read_Flag[3]!=0x00
           &&RC_Read_Flag[4]!=0x00
             &&RC_Read_Flag[5]!=0x00
               &&RC_Read_Flag[6]!=0x00
                 &&RC_Read_Flag[7]!=0x00)//flash中存在数据
  {
    for(i=0;i<8;i++)
    {
			if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
			else _rc_deadband_percent=RC_DEADBAND_PERCENT;
			
      RC_Data.cal[i].max=max_read[i];
      RC_Data.cal[i].min=min_read[i];
      RC_Data.cal[i].middle=(float)((max_read[i]+min_read[i])/2.0f);
      RC_Data.cal[i].deadband=(float)((max_read[i]-min_read[i])*_rc_deadband_percent/1.0f);
			RC_Data.cal[i].reverse_flag=false;
			RC_Data.cal[i].scale=(RC_Data.cal[i].max-RC_Data.cal[i].min-RC_Data.cal[i].deadband)*0.5f;
    }
  }
  else//flash中不存在数据
  {
    for(i=0;i<8;i++)
    {
			if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
			else _rc_deadband_percent=RC_DEADBAND_PERCENT;
			
      RC_Data.cal[i].max=RC_TOP_DEFAULT;
      RC_Data.cal[i].min=RC_BUTTOM_DEFAULT;
      RC_Data.cal[i].middle=RC_MIDDLE_DEFAULT;
      RC_Data.cal[i].deadband=(float)((RC_Data.cal[i].max-RC_Data.cal[i].min)*_rc_deadband_percent/1.0f);;
			RC_Data.cal[i].reverse_flag=false;
			RC_Data.cal[i].scale=(RC_Data.cal[i].max-RC_Data.cal[i].min-RC_Data.cal[i].deadband)*0.5f;
    }
  }
}
/***********************************************************
@函数名：RC_Calibration_RESET
@入口参数：无
@出口参数：无
@功能描述：遥控器行程标定复位
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void RC_Calibration_RESET()
{
  uint16_t i=0;
  for(i=0;i<8;i++)
  {
    RC_Data.cal[i].max=RC_RESET_DEFAULT;
    RC_Data.cal[i].min=RC_RESET_DEFAULT;
  }
}

uint8_t RC_Calibration_Trigger_Flag=0;
/***********************************************************
@函数名：RC_Calibration_Trigger
@入口参数：无
@出口参数：无
@功能描述：遥控器行程标定触发
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void RC_Calibration_Trigger(void)
{
  if(QuadKey2==0)
  {
    delay_ms(500);
    if(QuadKey2==0)
    {
      while(QuadKey2==0);
      RC_Calibration_RESET();//复位遥控器行程值，等待校准完毕
      RC_Calibration_Trigger_Flag=1;
      Page_Number=13;
      Key_Right_Release=1;
    }
  }
  else
  {
    RC_Calibration_Init();//直接从flash里面（或者DEFAULT值）获取遥控器行程输出
    RC_Calibration_Trigger_Flag=0;
  }
}

/***********************************************************
@函数名：RC_Calibration_Check
@入口参数：uint16 *rc_date
@出口参数：无
@功能描述：遥控器行程校准检测，涉及内部Flash读、写操作
@作者：WUST
@日期：2019年01月27日
*************************************************************/
bool RC_Calibration_Check(uint16 *rc_date)
{
  uint16_t i=0;
  bool success_flag=FALSE;
  if(RC_Calibration_Trigger_Flag==0) return success_flag;
  for(i=0;i<8;i++)
  {
    if(rc_date[i] >= RC_Data.cal[i].max)   RC_Data.cal[i].max = rc_date[i];//最大行程值
    if(rc_date[i] <  RC_Data.cal[i].min)   RC_Data.cal[i].min = rc_date[i];//最小行程值
    RC_Data.cal[i].middle=(float)((RC_Data.cal[i].max+RC_Data.cal[i].min)/2.0f);//行程中位
    RC_Data.cal[i].deadband=(float)((RC_Data.cal[i].max-RC_Data.cal[i].min)*RC_DEADBAND_PERCENT/1.0f);//设置满量程的百分之RC_DEADBAND_PERCENT为中位死区
  }
  
	
	if(_button.state[UP].press==LONG_PRESS&&Page_Number==13)
	{
			_button.state[UP].press=NO_PRESS;
      RC_Calibration_Trigger_Flag=0;
      Key_Right_Release=0;
      success_flag=TRUE;
      for(i=0;i<8;i=i+1)
      {
        WriteFlashParameter_Two(RC_CH1_MAX+2*i,
                                RC_Data.cal[i].max,
                                RC_Data.cal[i].min,
                                &Table_Parameter);
      }
	}
	
//  if(QuadKey2==0)//遥控器标定完成后，通过按键来结束标定过程
//  {
//    delay_ms(2000);
//    if(QuadKey2==0)
//    {
//      while(QuadKey2==0);
//      RC_Calibration_Trigger_Flag=0;
//      Key_Right_Release=0;
//      success_flag=TRUE;
//      for(i=0;i<8;i=i+1)
//      {
//        WriteFlashParameter_Two(RC_CH1_MAX+2*i,
//                                RC_Data.cal[i].max,
//                                RC_Data.cal[i].min,
//                                &Table_Parameter);
//      }
//    }
//  }
  return success_flag;
}

/***********************************************************
@函数名：Reset_RC_Calibartion
@入口参数：uint8_t Type
@出口参数：无
@功能描述：遥控器行程校准强制复位
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Reset_RC_Calibartion(uint8_t Type)
{
  if(Type==1)  
  {
    RC_Calibration_Trigger_Flag=0;
    Key_Right_Release=0;
  }
}
uint16_t ESC_Calibration_Makesure_Cnt=0;
float ESC_Calibration_Flag=0;
void ESC_Calibration_Check(void)
{
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
    ESC_Calibration_Makesure_Cnt++;
  //else ESC_Calibration_Makesure_Cnt/=2;
  
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max
           &&ESC_Calibration_Makesure_Cnt>200*5//持续5S
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
    //进入ESC校准模式
  {
    ESC_Calibration_Flag=1;
    ESC_Calibration_Makesure_Cnt=0;
    Forced_Lock_Makesure_Cnt=0;
    WriteFlashParameter(ESC_CALIBRATION_FLAG,
                        ESC_Calibration_Flag,
                        &Table_Parameter);
    Bling_Set(&Light_2,5000,500,0.2,0,GPIO_PORT_PJ,GPIO_PIN4,1);
    Page_Number=14;
  }
}

#define Thr_Chl_Num  2
/***********************************************************
@函数名：ESC_Calibration
@入口参数：无
@出口参数：无
@功能描述：电调行程校准
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void ESC_Calibration(void)
{
  PWM_Output(PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num],PPM_Databuf[Thr_Chl_Num]);
}

/***********************************************************
@函数名：Check_Calibration_Flag
@入口参数：无
@出口参数：无
@功能描述：当前校准模式检测
@作者：WUST
@日期：2019年01月27日
*************************************************************/
uint8_t Check_Calibration_Flag(void)
{
  uint8_t cal_flag=0x00; 
  if(Key_Right_Release==1)      cal_flag|=0x01;//遥控器校准
  if(Accel_Calibration_Flag==1) cal_flag|=0x02;//加速度计校准
  if(Mag_Calibration_Flag==1)   cal_flag|=0x04;//磁力计校准
  return cal_flag;
}

float Pitch_Offset=0,Roll_Offset=0;
float Hor_Accel_Offset[3]={0};
uint16_t Horizontal_Calibration_Makesure_Cnt=0;
/***********************************************************
@函数名：Horizontal_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：机架水平校准
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Horizontal_Calibration_Check(void)
{
  float acce_sample_sum[3]={0,0,0};//加速度和数据
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max&&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
  {
    Horizontal_Calibration_Makesure_Cnt++;
  }
  if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max
           &&Horizontal_Calibration_Makesure_Cnt>=200*5)||cal_flag==0x04)//持续三秒
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
  {
    Pitch_Offset=Pitch;
    Roll_Offset=Roll;
    uint16_t num_samples=0;
    while(num_samples<200)//采样200次
    {
      if(Gyro_Length<=20.0f)//通过陀螺仪模长来确保机体静止
      {
        for(int16_t j=0;j<3;j++){
          acce_sample_sum[j]+=WP_Sensor.acce_filter[j];//加速度计转化为1g量程下
        }
        delay_ms(5);//间隔10ms，1s内数据取平均
        num_samples++;
      }
    }
    Hor_Accel_Offset[0]=(acce_sample_sum[0]/num_samples)*ACCEL_TO_1G; //保存对应面的加速度计量
    Hor_Accel_Offset[1]=(acce_sample_sum[1]/num_samples)*ACCEL_TO_1G; //保存对应面的加速度计量
    //Hor_Accel_Offset[2]=acce_sample_sum[2]/num_samples; //保存对应面的加速度计量
    Hor_Accel_Offset[2]=(acce_sample_sum[2]/num_samples)*ACCEL_TO_1G-safe_sqrt(sq2(GRAVITY_MSS)-sq2(Hor_Accel_Offset[0])-sq2(Hor_Accel_Offset[1]));
		
		WriteFlashParameter(HOR_CAL_ACCEL_X,Hor_Accel_Offset[0],&Table_Parameter);
		WriteFlashParameter(HOR_CAL_ACCEL_Y,Hor_Accel_Offset[1],&Table_Parameter);
		WriteFlashParameter(HOR_CAL_ACCEL_Z,Hor_Accel_Offset[2],&Table_Parameter);
		
    Horizontal_Calibration_Makesure_Cnt=0;
    WriteFlashParameter(PITCH_OFFSET,Pitch_Offset,&Table_Parameter);
    WriteFlashParameter(ROLL_OFFSET,Roll_Offset,&Table_Parameter);
		
		
		Parameter_Init();
    Bling_Set(&Light_2,500,100,0.2,0,GPIO_PORT_PJ,GPIO_PIN4,0);
		NCLink_Send_Check_Flag[9]=0x01;
		cal_flag=0x00;
  }
}

/***********************************************************
@函数名：Horizontal_Calibration_Init
@入口参数：无
@出口参数：无
@功能描述：机架水平校准初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Horizontal_Calibration_Init(void)
{
  float pitch_offset_temp=0,roll_offset_temp=0;	
  if(ReadFlashParameterOne(PITCH_OFFSET,&pitch_offset_temp))   Pitch_Offset=pitch_offset_temp;
  if(ReadFlashParameterOne(ROLL_OFFSET,&roll_offset_temp))    Roll_Offset=roll_offset_temp;
}


uint16_t Headless_Mode_Makesure_Cnt=0;
float Headless_Mode_Yaw=0.0f;
/***********************************************************
@函数名：Headless_Mode_Calibration_Check
@入口参数：无
@出口参数：无
@功能描述：无头模式初始偏航角获取
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Headless_Mode_Calibration_Check(void)
{
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control==0)
  {
    Headless_Mode_Makesure_Cnt++;
  }
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control==0
           &&Headless_Mode_Makesure_Cnt>=200*5//持续三秒
             &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
  {    
    Headless_Mode_Yaw=Yaw;
    Horizontal_Calibration_Makesure_Cnt=0;
    WriteFlashParameter(HEADLESS_MODE_YAW,Headless_Mode_Yaw,&Table_Parameter);
    Bling_Set(&Light_2,500,100,0.2,0,GPIO_PORT_PJ,GPIO_PIN4,0);
  }
}

uint8_t Gyro_Safety_Calibration_Flag=0;
uint8_t Gyro_Calibration_Check(vector3f *gyro)
{
	static vector3f offset;
	static uint16_t offset_cnt=0;
  static vector3f last_gyro;
 if((gyro->x-last_gyro.x)*GYRO_CALIBRATION_COFF<=5.0f
	 &&(gyro->y-last_gyro.y)*GYRO_CALIBRATION_COFF<=5.0f
			&&(gyro->z-last_gyro.z)*GYRO_CALIBRATION_COFF<=5.0f
			 &&Temperature_State_Check()==1)
	{
	  offset.x+=gyro->x;
		offset.y+=gyro->y;
		offset.z+=gyro->z;
		offset_cnt++;
	}
  else
	{
		offset.x=0;
		offset.y=0;
		offset.z=0;
		offset_cnt=0;
	}
  last_gyro.x=gyro->x;
	last_gyro.y=gyro->y;
	last_gyro.z=gyro->z;
	
	if(offset_cnt>=200)//持续满足1s
	{
		gyro_offset.x =(offset.x/offset_cnt);//得到标定偏移
		gyro_offset.y =(offset.y/offset_cnt);
		gyro_offset.z =(offset.z/offset_cnt);
		Euler_Angle_Init_Again();
    Bling_Set(&Light_2,2000,100,0.2,0,GPIO_PORT_PJ,GPIO_PIN4,0);
		NCLink_Send_Check_Flag[9]=0x01;
		
		WriteFlashParameter_Three(GYRO_X_OFFSET,
															gyro_offset.x,
															gyro_offset.y,
															gyro_offset.z,
															&Table_Parameter);
		
	  return 1;
	}
	return 0;
}


uint16_t Target_Height=0,Safe_Height=0,Safe_Vbat=0,Max_Height=0,Max_Radius=0,Max_Upvel=0,Max_Downvel=0,Max_Horvel=0;
void Other_Parameter_Init(void)
{
	float targeheight,safeheight,safevbat,maxheight,maxradius,maxupvel,maxdownvel,maxhorvel;
	ReadFlashParameterOne(TARGET_HEIGHT,&targeheight);
	ReadFlashParameterOne(SAFE_HEIGHT,&safeheight);
	ReadFlashParameterOne(SAFE_VBAT,&safevbat);
	ReadFlashParameterOne(MAX_HEIGHT,&maxheight);
	ReadFlashParameterOne(MAX_RADIUS,&maxradius);
	ReadFlashParameterOne(MAX_UPVEL,&maxupvel);
	ReadFlashParameterOne(MAX_DOWNVEL,&maxdownvel);
	ReadFlashParameterOne(MAX_HORVEL,&maxhorvel);
  
	if(isnan(targeheight)==0)   Target_Height=targeheight;
	else Target_Height=Auto_Launch_Target;
	
	if(isnan(safeheight)==0)    Safe_Height=safeheight;
	else Safe_Height=Nav_Safety_Height;
	
	if(isnan(safevbat)==0)      Safe_Vbat=safevbat;
	else Safe_Vbat=Flight_Safe_Vbat;//
	
	if(isnan(maxheight)==0)     Max_Height=maxheight;
	else Max_Height=Flight_Max_Height;//
	
	if(isnan(maxradius)==0)     Max_Radius=maxradius;
	else Max_Radius=Flight_Max_Radius;//
	
	if(isnan(maxupvel)==0)      Max_Upvel=maxupvel;
	else Max_Upvel=Climb_Up_Speed_Max;
	
	if(isnan(maxdownvel)==0)    Max_Downvel=maxdownvel;
	else Max_Downvel=Climb_Down_Speed_Max;
	
	if(isnan(maxhorvel)==0)     Max_Horvel=maxhorvel;
	else Max_Horvel=Nav_Speed_Max;
}

void Other_Parameter_Default(void)
{
	Target_Height=Auto_Launch_Target;
	Safe_Height=Nav_Safety_Height;
	Safe_Vbat=Flight_Safe_Vbat;
	Max_Height=Flight_Max_Height;
	Max_Radius=Flight_Max_Radius;
	Max_Upvel=Climb_Up_Speed_Max;
	Max_Downvel=Climb_Down_Speed_Max;
	Max_Horvel=Nav_Speed_Max;
	WriteFlashParameter(TARGET_HEIGHT,Target_Height,&Table_Parameter);
	WriteFlashParameter(SAFE_HEIGHT,Safe_Height,&Table_Parameter);
	WriteFlashParameter(SAFE_VBAT,Safe_Vbat,&Table_Parameter);
	WriteFlashParameter(MAX_HEIGHT,Max_Height,&Table_Parameter);
	WriteFlashParameter(MAX_RADIUS,Max_Radius,&Table_Parameter);
	WriteFlashParameter(MAX_UPVEL,Max_Upvel,&Table_Parameter);
	WriteFlashParameter(MAX_DOWNVEL,Max_Downvel,&Table_Parameter);
	WriteFlashParameter(MAX_HORVEL,Max_Horvel,&Table_Parameter);
}



void Reset_All_Calibartion(void)
{
	Reset_Mag_Calibartion(1);
	Reset_Accel_Calibartion(1);
	Reset_RC_Calibartion(1);
}

void Check_All_Calibartion(void)
{
	Accel_Calibration_Check();				//加速度校准检测
    Horizontal_Calibration_Check();			//机架水平校准检测
	Mag_Calibration_Check();				//磁力计校准检测
    ESC_Calibration_Check();				//电调行程校准检测
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

