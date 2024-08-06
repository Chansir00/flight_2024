#ifndef __HEADFILE_H__
#define __HEADFILE_H__
/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供本校学生学习使用，版权著作权属于武汉科技大学，武汉科技大学将飞控程序源码提供给本校学生，
*               本校学生要为武汉科技大学提供保护，未经学校许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，武汉科技大学将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrunWUST
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               修改日期:2019/4/12
*               版本：躺赢者——CarryPilot_V1.0
*               版权所有，盗版必究。
*               Copyright(C) 2017-2025 武汉无名创新科技有限公司 
*               All rights reserved
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               学校，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/



#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))


#define Imu_Sampling_Freq  400
#define FC_Control_Freq  400
#define Imu_Sampling_Dt  (1.0f/Imu_Sampling_Freq)


#define WP_Duty_Freq  200
#define WP_Duty_Dt  (1.0f/WP_Duty_Freq)


typedef   signed           char int8;
typedef unsigned           char u8;
typedef unsigned           char uint8;
typedef unsigned           char byte;
typedef   signed short     int int16;
typedef unsigned short     int uint16;
typedef unsigned short     int u16;
typedef unsigned long     int u32; 

#define US100  0
#define TFMINI 1
#define Ground_Distance_Sensor  US100




#define _YAW    0
#define _PITCH  1
#define _ROLL   2

#define Delta 0.0025f
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
/////////////////////////
#include "WP_DataType.h"
#include "Usart.h"
//#include "uartstdio.h"
#include "Time.h"
#include "Time_Cnt.h"
//#include "Schedule.h"
#include "License.h"
#include "ssd1306.h"
#include "oled.h"
#include "mpu6050.h"
#include "Soft_I2C.h"
#include "ist8310.h"
#include "spl06.h"
#include "PPM.h"
#include "wp_flash.h"
#include "softiic.h"
#include "stflash.h"

#include "PID.h"
#include "ADRC.h"
#include "Ringbuf.h"
#include "WP_PWM.h"
#include "Sensor.h"
#include "Key.h"
#include "Filter.h"
#include "SINS.h"
#include "Calibration.h"
#include "CalibrationRoutines.h"
#include "US_100.h"
#include "GY53L1x.h"
#include "tfmini.h"
#include "NamelessCorun_OpticalFlow.h"
#include "OpticalFlow_LC306.h"
#include "OpticalFlow_Control.h"
#include "NamelessCotrun_SDK.h"
#include "WP_ADC.h"
#include "Bling.h"
#include "GPS.h"
#include "RC.h"
#include "SYSTEM.h"
#include "WP_Ctrl.h"
#include "Developer_Mode.h"
#include "control_config.h"
#include "Temperature_Ctrl.h"
#include "Compass.h"
#include "NCLink.h"
extern SINS NamelessQuad;
extern Sensor WP_Sensor;

#endif

//void Angle_Control()//角度环节
//{
//  //角度反馈
//  Total_Controller.Pitch_Angle_Control.FeedBack=(Pitch-Pitch_Offset);
//  PID_Control(&Total_Controller.Pitch_Angle_Control);
//  Total_Controller.Roll_Angle_Control.FeedBack =(Roll-Roll_Offset);
//  PID_Control(&Total_Controller.Roll_Angle_Control);
//  
//  if(Yaw_Control==0)//偏航杆置于中位
//  {
//    if(Total_Controller.Yaw_Angle_Control.Expect==0//回中时赋角度期望值
//       ||Landon_Earth_Flag==1)//或者满足触地条件、复位偏航角期望
//    {
//      Total_Controller.Yaw_Angle_Control.Expect=Yaw;
//    }
//    Total_Controller.Yaw_Angle_Control.FeedBack=Yaw;//偏航角反馈
//    PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control);//偏航角度控制
//    Total_Controller.Yaw_Gyro_Control.Expect=Total_Controller.Yaw_Angle_Control.Control_OutPut;//偏航角速度环期望，来源于偏航角度控制器输出
//    
//    if(Opv_Top_View_Target.line_ctrl_enable==1&&Controler_SDK2_Mode==1)//SDK模式下的循迹模式，偏航角速度期望来源于SDK数据
//    {
//      Total_Controller.Yaw_Gyro_Control.Expect=3*SDK_Target_Yaw_Gyro;
//      Total_Controller.Yaw_Angle_Control.Expect=0;//偏航角期望给0,不进行角度控制
//      //SDK_Line.flag=0;
//    }
//  }
//  else//波动偏航方向杆后，只进行内环角速度控制
//  {
//    Total_Controller.Yaw_Angle_Control.Expect=0;//偏航角期望给0,不进行角度控制
//    Total_Controller.Yaw_Gyro_Control.Expect=Yaw_Control;//偏航角速度环期望，直接来源于遥控器打杆量
//  }
//}


