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


#include "WP_PWM.h"

Timer_A_PWMConfig pwmConfig1 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,//12/12M=1M=1us
	2500-1,//2.5ms
	TIMER_A_CAPTURECOMPARE_REGISTER_1,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};
Timer_A_PWMConfig pwmConfig2 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	2500-1,
	TIMER_A_CAPTURECOMPARE_REGISTER_2,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};
Timer_A_PWMConfig pwmConfig3 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	2500-1,
	TIMER_A_CAPTURECOMPARE_REGISTER_3,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};
Timer_A_PWMConfig pwmConfig4 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	2500-1,
	TIMER_A_CAPTURECOMPARE_REGISTER_4,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};



void PWM_GPIO_Init(void )
{
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P10,
            GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8,
					GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P9,
            GPIO_PIN2 | GPIO_PIN3 , GPIO_PRIMARY_MODULE_FUNCTION);
	
}
/***********************************************************
@函数名：Init_PWM
@入口参数：无
@出口参数：无
@功能描述：PWM初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Init_PWM(void)
{
  PWM_GPIO_Init();
	/* Configuring Timer_A1 for UpDown Mode and starting */
	MAP_Timer_A_generatePWM(TIMER_A3_BASE, &pwmConfig1);
	MAP_Timer_A_generatePWM(TIMER_A3_BASE, &pwmConfig2);
	MAP_Timer_A_generatePWM(TIMER_A3_BASE, &pwmConfig3);
	MAP_Timer_A_generatePWM(TIMER_A3_BASE, &pwmConfig4);
	PWM_Output(1000,1000,1000,1000);
	Init_EPWM();
}


/***********************************************************
@函数名：PWM_Output
@入口参数：uint16_t width1,uint16_t width2,uint16_t width3,
uint16_t width4,uint16_t width5,uint16_t width6,
uint16_t width7,uint16_t width8
@出口参数：无
@功能描述：PWM输出
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
{
  MAP_Timer_A_setCompareValue(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1,width1);
	MAP_Timer_A_setCompareValue(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2,width2);
	MAP_Timer_A_setCompareValue(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_3,width3);
	MAP_Timer_A_setCompareValue(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_4,width4);

//	EPWM_Output(width1,width2,width3,width4);
	EPWM_Output(width4,width3,width2,width1);
}






//////////////////////////////////////////////////////////////////
Timer_A_PWMConfig pwmConfig5 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	2500-1,
	TIMER_A_CAPTURECOMPARE_REGISTER_1,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};
Timer_A_PWMConfig pwmConfig6 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	2500-1,
	TIMER_A_CAPTURECOMPARE_REGISTER_2,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};
Timer_A_PWMConfig pwmConfig7 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	2500-1,
	TIMER_A_CAPTURECOMPARE_REGISTER_3,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};
Timer_A_PWMConfig pwmConfig8 =
{
	TIMER_A_CLOCKSOURCE_SMCLK,
	TIMER_A_CLOCKSOURCE_DIVIDER_12,
	2500-1,
	TIMER_A_CAPTURECOMPARE_REGISTER_4,
	TIMER_A_OUTPUTMODE_RESET_SET,
	0
};



void EPWM_GPIO_Init(void)
{
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
					GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 , GPIO_PRIMARY_MODULE_FUNCTION);
}


void EPWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
{
  MAP_Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1,width1);
	MAP_Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2,width2);
	MAP_Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_3,width3);
	MAP_Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_4,width4);
}


void Init_EPWM(void)
{
  EPWM_GPIO_Init();
	/* Configuring Timer_A1 for UpDown Mode and starting */
	MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig5);
	MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig6);
	MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig7);
	MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig8);
	EPWM_Output(1000,1000,1000,1000);
}
/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



