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
#include "GY53L1x.h"




void GY53L1X_In1_Init(void)
{
  MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN7);
	GPIO_interruptEdgeSelect(GPIO_PORT_P5,GPIO_PIN7,GPIO_LOW_TO_HIGH_TRANSITION);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN7);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN7);
	MAP_Interrupt_enableInterrupt(INT_PORT5);	
}


void GY53L1X_In1_UP()//上升沿触发
{
  GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN7);
	GPIO_interruptEdgeSelect(GPIO_PORT_P5,GPIO_PIN7,GPIO_LOW_TO_HIGH_TRANSITION); 
  GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN7);
}

void GY53L1X_In1_DN()//下降沿触发
{
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN7);
	GPIO_interruptEdgeSelect(GPIO_PORT_P5,GPIO_PIN7,GPIO_HIGH_TO_LOW_TRANSITION);
  GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN7);	
}



void GY53L1X_In2_Init(void)
{
  MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN0);
	GPIO_interruptEdgeSelect(GPIO_PORT_P3,GPIO_PIN0,GPIO_LOW_TO_HIGH_TRANSITION);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN0);
	MAP_Interrupt_enableInterrupt(INT_PORT3);	
}


void GY53L1X_In2_UP()//上升沿触发
{
  GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN0);
	GPIO_interruptEdgeSelect(GPIO_PORT_P3,GPIO_PIN0,GPIO_LOW_TO_HIGH_TRANSITION); 
  GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
}

void GY53L1X_In2_DN()//下降沿触发
{
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN0);
	GPIO_interruptEdgeSelect(GPIO_PORT_P3,GPIO_PIN0,GPIO_HIGH_TO_LOW_TRANSITION);
  GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);	
}


void GY53L1X_Init(void)
{
	GY53L1X_In1_Init();
	GY53L1X_In2_Init();
}









vl53l1x_data vl53l1x_in1,vl53l1x_in2;
systime GY53L1_t;
void GY53L1X_In1_IRQHandler(void)
{
	static uint8_t _flag=0; 
	if(_flag==0)//先上升沿
	{
	  vl53l1x_in1.gy53l1x_start_time=10000*(TIME_ISR_CNT+1)-(uint32_t)(Timer32_getValue(TIMER32_BASE+0x00020)/48);//单位us
		GY53L1X_In1_DN();
	  _flag=1;
	}
  else if(_flag==1)//后下降沿
	{		
	  vl53l1x_in1.gy53l1x_end_time=10000*(TIME_ISR_CNT+1)-(uint32_t)(Timer32_getValue(TIMER32_BASE+0x00020)/48);//单位us
		vl53l1x_in1.gy53l1x_delta=vl53l1x_in1.gy53l1x_end_time-vl53l1x_in1.gy53l1x_start_time;
		vl53l1x_in1.gy53l1x_update_flag=1;		
		GY53L1X_In1_UP();
		_flag=0;
	}		  
}



void GY53L1X_In2_IRQHandler(void)
{
	static uint8_t _flag=0; 
	if(_flag==0)//先上升沿
	{
		Get_Systime(&GY53L1_t);
	  vl53l1x_in2.gy53l1x_start_time=10000*(TIME_ISR_CNT+1)-(uint32_t)(Timer32_getValue(TIMER32_BASE+0x00020)/48);//单位us
		GY53L1X_In2_DN();
	  _flag=1;
	}
  else if(_flag==1)//后下降沿
	{		
	  vl53l1x_in2.gy53l1x_end_time=10000*(TIME_ISR_CNT+1)-(uint32_t)(Timer32_getValue(TIMER32_BASE+0x00020)/48);//单位us
		vl53l1x_in2.gy53l1x_delta=vl53l1x_in2.gy53l1x_end_time-vl53l1x_in2.gy53l1x_start_time;
		vl53l1x_in2.gy53l1x_update_flag=1;		
		GY53L1X_In2_UP();
		_flag=0;
	}		  
}



void PORT3_IRQHandler(void)
{
	uint32_t status= MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
	if(status & GPIO_PIN0)
  {
    GY53L1X_In2_IRQHandler();
  }
}

void PORT5_IRQHandler(void)
{
	uint32_t status= MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
	if(status & GPIO_PIN7)
  {
    GY53L1X_In1_IRQHandler();
  }
}



void GY53L1X_Data_Update()
{	
	static uint16_t _cnt=0;
	static uint8_t fault_flag=0;
	if(vl53l1x_in1.gy53l1x_update_flag==1)
	{
		vl53l1x_in1.pre_last_distance=vl53l1x_in1.last_distance;//上上次高度
		vl53l1x_in1.last_distance=vl53l1x_in1.distance;//上次高度
		vl53l1x_in1.distance=vl53l1x_in1.gy53l1x_delta/100.0f;//本次高度		
    vl53l1x_in1.last_vel=vl53l1x_in1.vel;
		vl53l1x_in1.vel=(vl53l1x_in1.distance-vl53l1x_in1.last_distance)/0.05f;
		vl53l1x_in1.acc=(vl53l1x_in1.vel-vl53l1x_in1.last_vel)/0.05f;
		
		if(ABS(vl53l1x_in1.vel)<1000)
		{
			if(fault_flag==1&&vl53l1x_in1.distance==vl53l1x_in1.pre_last_distance)//本次=上上次
			{
				fault_flag=1;
			}			
			else if(fault_flag==1&&vl53l1x_in1.distance==vl53l1x_in1.last_distance)//本次=上次
			{
				fault_flag=1;
			}
			else if(fault_flag==1&&vl53l1x_in1.last_distance==vl53l1x_in1.pre_last_distance)//上次=上上次
			{
			  fault_flag=1;		
			}
			//以上为传感器异常处理
			else
			{
				Ground_Distance=vl53l1x_in1.distance;
				Ground_Distance_Div=vl53l1x_in1.vel;
				Ground_Distance_Acc=vl53l1x_in1.acc;
							
			}
		}
		else
		{
			fault_flag=1;		
		}
		_cnt++;		
		vl53l1x_in1.gy53l1x_update_flag=0; 		
	}
	
	if(_cnt>=2)
	{
		WP_Sensor.us100_updtate_flag=1;
		_cnt=0;
	}
	
	
	if(vl53l1x_in2.gy53l1x_update_flag==1)//数据已解析，留作配用
	{
		vl53l1x_in2.distance=vl53l1x_in2.gy53l1x_delta/100.0f;		
		vl53l1x_in2.gy53l1x_update_flag=0;	
	}	
	
  if(Ground_Distance<=380&&Ground_Distance>0)  Sensor_Flag.Ground_Health=1;
  else  Sensor_Flag.Ground_Health=0; 
}





/////////////////////////////////////////////////////








