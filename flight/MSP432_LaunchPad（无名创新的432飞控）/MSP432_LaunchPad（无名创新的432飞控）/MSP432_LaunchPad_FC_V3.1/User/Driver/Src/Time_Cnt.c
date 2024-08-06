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
#include "Time_Cnt.h"



const Timer_A_UpModeConfig upConfig2 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
	TIMER_A_CLOCKSOURCE_DIVIDER_12,         // SMCLK/1 = 3MHz
	2500,                           				// 2500 tick period
	TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
	TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
	TIMER_A_DO_CLEAR                        // Clear value
};
void TIMER_A2_Configuration(void)
{
   /* Configuring Timer_A1 for Up Mode */
	MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig2);

	/* Enabling interrupts and starting the timer */
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
	MAP_Interrupt_enableInterrupt(INT_TA2_0);
	MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
}

/***************************************************
函数名: void NVIC_Configuration(void)
说明:	中断优先级定义
入口:	无
出口:	无
备注:	系统中断时间在整体设计时，已全部测试过最大开销，
      确保程序运动的稳健性        
注释者：WUST
****************************************************/
void NVIC_Configuration(void)
{
  sysTickUptime=0;
	Interrupt_setPriorityGrouping(4);
  MAP_Interrupt_setPriority(FAULT_SYSTICK,0x00);	
	MAP_Interrupt_setPriority(INT_T32_INT2,0x00);
	MAP_Interrupt_setPriority(INT_EUSCIA2 ,0x20);//串口2  地面站 波特率921600
	MAP_Interrupt_setPriority(INT_PORT6   ,0x20);//PPM  US100
	MAP_Interrupt_setPriority(INT_PORT3   ,0x40);//GY53L1X_IN2
	MAP_Interrupt_setPriority(INT_PORT5   ,0x40);//GY53L1X_IN2
	
	MAP_Interrupt_setPriority(INT_EUSCIA3 ,0x60);//串口3  OPENMV 波特率921600
	MAP_Interrupt_setPriority(INT_EUSCIA0 ,0x80);//串口0  LC306  波特率19200
	MAP_Interrupt_setPriority(INT_EUSCIB1 ,0xa0);//串口1  OPENMV 波特率921600	
	
	MAP_Interrupt_setPriority(INT_ADC14   ,0xc0);
	MAP_Interrupt_setPriority(INT_TA2_0   ,0xe0);//任务调度器
		
}
/***********************************************************
@函数名：Time_init
@入口参数：无
@出口参数：无
@功能描述：系统调度定时器初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Time_init(void)//系统调度定时器初始化
{
  Timer32_init();							 //程序计时定时器
  TIMER_A2_Configuration();    //TIMER_A2任务调度定时器
  NVIC_Configuration();        //中断优先级设置
}








void Task_200hz_Thread(void)
{
	Get_Status_Feedback();					//获取姿态数据、水平与竖直方向惯导数据
	GankerPilot_Control();					//总控制器
}


void Task_400hz_Thread(void)
{
  Remote_Control();								//遥控器数据解析
#if (Ground_Distance_Sensor==US100)	
		US_100_Statemachine();			  //超声波传感器状态机更新
#else
		TFmini_Statemachine();        //激光TFMINI_PLUS传感器状态机更新
#endif
	GY53L1X_Data_Update();					//激光测距（用不到）
	Optflow_Statemachine();					//光流状态机，初始化时存在光流外设
	SDK_Data_Prase_1();							//SDK数据解析
	SDK_Data_Prase_2();			
	Check_All_Calibartion();			//所有传感器校准按键检测
  Bling_Working(Bling_Mode);			//状态指示灯运行
  NCLink_SEND_StateMachine();			//无名创新开源地面站，下载链接http://nameless.tech/download.html	
	Get_Battery_Voltage();		      //测量电池电压	
	Read_Button_State_All();
}

Sensor WP_Sensor;
Testime Time0_Delta;
Testime Time0_Delta1;
float test_time;
/***********************************************************
@函数名：TA2_0_IRQHandler
@入口参数：无
@出口参数：无
@功能描述：系统调度定时器中断服务函数：主要进行遥控器解析、
传感数据采集、数字滤波、姿态解算、惯性导航、控制等对周期有
严格要求的函数
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void TA2_0_IRQHandler(void)//5ms刷新一次
{
	Test_Period(&Time0_Delta);
	static uint16_t _cnt_200hz=0,_cnt_400hz=0;
	_cnt_200hz++;
	_cnt_400hz++;
	if(_cnt_200hz>=1)
	{
		_cnt_200hz=0;
	  Task_200hz_Thread();
	}
	
	if(_cnt_400hz>=2)
	{
		_cnt_400hz=0;
	  Task_400hz_Thread();
	}
	Test_Period(&Time0_Delta1);
	if(Time0_Delta1.Now_Time-Time0_Delta.Now_Time>test_time)    test_time=Time0_Delta1.Now_Time-Time0_Delta.Now_Time; 	
	MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);	
//  Vcan_Send();//山外地面站（多功能调试助手）
}


/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/


