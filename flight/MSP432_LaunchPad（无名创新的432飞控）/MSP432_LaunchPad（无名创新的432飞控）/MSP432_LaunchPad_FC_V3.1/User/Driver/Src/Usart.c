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
#include "Usart.h"
#include "Ringbuf.h"
//串口循环队列缓冲数据定义
RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;

uint8_t controlTable[512];
//void Serial_Data_Send_Dma(u32 SendBuff,u16 len)//DMA---USART2传输
void Serial_Data_Send_Dma(u32 SendBuff,u16 len)//DMA---USART2传输
{
	 //![Simple DMA Example]
	/* Configuring DMA module */
	MAP_DMA_enableModule();
	MAP_DMA_setControlBase(controlTable);
	 /* Assigning Channel 2 to EUSCIB1TX0, and Channel 5 to EUSCIB2RX0  and
     * enabling channels 2 and 5*/
	MAP_DMA_assignChannel(DMA_CH4_EUSCIA2TX);
	 /* Disabling channel attributes */
	MAP_DMA_disableChannelAttribute(DMA_CH4_EUSCIA2TX,
																	 UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
																	 UDMA_ATTR_HIGH_PRIORITY |
																	 UDMA_ATTR_REQMASK);
	/* Setting Control Indexes */
	MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH4_EUSCIA2TX,
					UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
	
	MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH4_EUSCIA2TX,
					UDMA_MODE_BASIC, (void*) SendBuff,
					(void*) UART_getTransmitBufferAddressForDMA(EUSCI_A2_BASE), len);
	/* Now that the DMA is primed and setup, enabling the channels. The EUSCI
     * hardware should take over and transfer/receive all bytes */	
	MAP_DMA_enableChannel(4);
}


/*
//UARTA2 Ring Buffer Global Variables
#define UARTA2_BUFFERSIZE 2048
volatile uint8_t UARTA2Data[UARTA2_BUFFERSIZE];
volatile uint32_t UARTA2ReadIndex;
volatile uint32_t UARTA2WriteIndex;
#define UARTA2_ADVANCE_READ_INDEX          MAP_Interrupt_disableMaster(); UARTA2ReadIndex = (UARTA2ReadIndex + 1) % UARTA2_BUFFERSIZE; MAP_Interrupt_enableMaster();
#define UARTA2_ADVANCE_WRITE_INDEX         UARTA2WriteIndex = (UARTA2WriteIndex + 1) % UARTA2_BUFFERSIZE
#define UARTA2_BUFFER_EMPTY                UARTA2ReadIndex == UARTA2WriteIndex ? true : false
#define UARTA2_BUFFER_FULL                 (UARTA2WriteIndex + 1) % UARTA2_BUFFERSIZE == UARTA2ReadIndex ? true : false
*/

/***********************************************************
@函数名：EUSCIA2_IRQHandler
@入口参数：无
@出口参数：无
功能描述：串口2数据接收
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void EUSCIA2_IRQHandler(void)
{
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
	if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
	{
		uint8_t data=MAP_UART_receiveData(EUSCI_A2_BASE);
		NCLink_Data_Prase_Prepare(data);//(UCA2RXBUF);
	}	
	MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);
}

/*	
	  if(UARTA2_BUFFER_FULL)
		{
				//TODO: Buffer Overflow, add handler here
		}
		else
		{
				UARTA2Data[UARTA2WriteIndex] = data;
				UARTA2_ADVANCE_WRITE_INDEX;
		}
*/
/********************************************/
//波特率计算公式：
// N=时钟主频/波特率
// 如果 N<16  UCBRx=INT(N),UCBRF=0;
// 如果 N>16  UCBRx=INT(N/16),UCBRF=INT{[(N/16)-INT(N/16)]*16}
// UCBRS 取值见MSP432 USER'S GUIDE P731
// 或者 使用计算器 
// software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
/********************************************/
const eUSCI_UART_Config uartConfig =
{
	EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
	13,                                       // BRDIV = 13   6
	0,                                       // UCxBRF = 0   8
	0,                                       // UCxBRS = 0   32
	EUSCI_A_UART_NO_PARITY,                  // No Parity
	EUSCI_A_UART_LSB_FIRST,                  // MSB First
	EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
	EUSCI_A_UART_MODE,                       // UART mode
	//EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
	EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
};
/***********************************************************
@函数名：USART2_Init
@入口参数：无
@出口参数：无
功能描述：串口2配置（板上标注为UART3）波特率115200
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void USART2_Init()
{
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A2_BASE);

	//MAP_UART_clearInterruptFlag (EUSCI_A2_BASE);
	/* Enabling interrupts */
	MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
}

/***********************************************************
@函数名：USART2_Send
@入口参数：uint8_t *pui8Buffer, uint32_t ui32Count
@出口参数：无
功能描述：串口2发送N个字节数据
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
  while(ui32Count--)
	{
		MAP_UART_transmitData(EUSCI_A2_BASE, *pui8Buffer);
		pui8Buffer++;
	}
}
/***********************************************************
@函数名：wust_sendware
@入口参数：unsigned char *wareaddr, int16_t waresize
@出口参数：无
功能描述：山外虚拟示波器发送
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void wust_sendware(unsigned char *wareaddr, int16_t waresize)//山外发送波形
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//帧头
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//帧尾
  USART2_Send(cmdf, sizeof(cmdf));
  USART2_Send(wareaddr, waresize);
  USART2_Send(cmdr, sizeof(cmdr));
}


float VCAN_DataBuf[8];
DMA_Vcan_Buff  Vcan_Buff;
/***********************************************************
@函数名：Vcan_Send
@入口参数：无
@出口参数：无
功能描述：山外数据发送函数，默认发送8个通道，数据类型为float
，每个通道数据可以自己定义
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Vcan_Send(void)//山外地面站发送
{
 static float DataBuf[8];	

  //static float DataBuf[8];	
	Vcan_Buff.Head=0xfc030000;     
  DataBuf[0]=Vcan_Buff.DataBuf[0]=NamelessQuad.Position[_YAW];//惯导高度
  DataBuf[1]=Vcan_Buff.DataBuf[1]=NamelessQuad.Speed[_YAW];//惯导速度
  DataBuf[2]=Vcan_Buff.DataBuf[2]=observation_div;//NamelessQuad.Acceleration[_YAW];;//惯导加速度
  DataBuf[3]=Vcan_Buff.DataBuf[3]=Observation_Altitude;//气压计高度
	DataBuf[4]=Vcan_Buff.DataBuf[4]=observation_acc;
  DataBuf[5]=Vcan_Buff.DataBuf[5]=tfdata.distance;
  DataBuf[6]=Vcan_Buff.DataBuf[6]=tfdata.div;
	DataBuf[7]=Vcan_Buff.DataBuf[7]=Ground_Distance;     
	Vcan_Buff.End=0x000003fc;
//  Serial_Data_Send_Dma((u32)(&Vcan_Buff),sizeof(Vcan_Buff));
	wust_sendware((unsigned char *)DataBuf,sizeof(DataBuf));
}

/* Copyright (c)  2018-2025 WUST All rights reserved.*/





void EUSCIA0_IRQHandler(void)
{
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

	if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
	{
		RingBuf_Write(MAP_UART_receiveData(EUSCI_A0_BASE),&COM0_Rx_Buf,FLOW_BUF_CNT);//往环形队列里面写数据
	}
	MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);
}


void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
  while(ui32Count--)
	{
		MAP_UART_transmitData(EUSCI_A0_BASE, *pui8Buffer);
		pui8Buffer++;
	}
}

void USART0_Send_Byte(uint8_t ch)
{
	MAP_UART_transmitData(EUSCI_A0_BASE, ch);
}

/* Current setting: 19200 bps */
const eUSCI_UART_Config uart0Config = {
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		39,                                      // BRDIV
		1,                                       // UCxBRF
		0,                                       // UCxBRS
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // LSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
		};
/***********************************************************
@函数名：USART2_Init
@入口参数：无
@出口参数：无
功能描述：串口2配置（板上标注为UART3）波特率115200
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void USART0_Init()
{
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A0_BASE, &uart0Config);

	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A0_BASE);
	//MAP_UART_clearInterruptFlag (EUSCI_A2_BASE);
	/* Enabling interrupts */
	MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);  
	
	OpticalFlow_Init();//光流滤波参数初始化 
	OpticalFlow_Is_Work=Config_Init_Uart();	
	MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
}



void EUSCIA1_IRQHandler(void)
{
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

	if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
	{
		RingBuf_Write(MAP_UART_receiveData(EUSCI_A1_BASE),&COM1_Rx_Buf,SDK_Target_Length*2);//往环形队列里面写数据
		if(COM1_Rx_Buf.Ring_Buff[0]!=0xFF)
		{
			COM1_Rx_Buf.Head=1;
			COM1_Rx_Buf.Tail=0; 
		}	
	}
	MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, status);	
}


void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
  while(ui32Count--)
	{
		MAP_UART_transmitData(EUSCI_A1_BASE, *pui8Buffer);
		pui8Buffer++;
	}
}

void USART1_Send_Byte(uint8_t ch)
{
	MAP_UART_transmitData(EUSCI_A1_BASE, ch);
}

/* Current setting: 921600 bps */
const eUSCI_UART_Config uart1Config = {
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		13,                                      // BRDIV = 13   6
		0,                                       // UCxBRF = 0   8
		0,                                       // UCxBRS = 0   32
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
	
//#if  (Ground_Distance_Sensor==US100)		
//			EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
//			78,                                      // BRDIV
//			2,                                       // UCxBRF
//			0,                                       // UCxBRS
//			EUSCI_A_UART_NO_PARITY,                  // No Parity
//			EUSCI_A_UART_LSB_FIRST,                  // LSB First
//			EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
//			EUSCI_A_UART_MODE,                       // UART mode
//			EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
//#else	
//			EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
//			6,                                       // BRDIV = 13
//			8,                                       // UCxBRF = 0
//			32,                                    // UCxBRS = 37 0X20
//			EUSCI_A_UART_NO_PARITY,                  // No Parity
//			EUSCI_A_UART_LSB_FIRST,                  // MSB First
//			EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
//			EUSCI_A_UART_MODE,                       // UART mode
//			EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
//#endif
};
/***********************************************************
@函数名：USART1_Init
@入口参数：无
@出口参数：无
功能描述：串口1配置（板上标注为UART1）波特率921600
@作者：WUST
@日期：2019年01月27日
*************************************************************/
		
void USART1_Init()
{
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A1_BASE, &uart1Config);

	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A1_BASE);
	/* Enabling interrupts */
	MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);  	
  MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
}



///////////////////////////////////////////
void EUSCIA3_IRQHandler(void)
{
	uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A3_BASE);

	if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
	{
		RingBuf_Write(MAP_UART_receiveData(EUSCI_A3_BASE),&COM3_Rx_Buf,SDK_Target_Length*2);//往环形队列里面写数据
		if(COM3_Rx_Buf.Ring_Buff[0]!=0xFF)
		{
			COM3_Rx_Buf.Head=1;
			COM3_Rx_Buf.Tail=0; 
		}		
	}
	
	
	MAP_UART_clearInterruptFlag(EUSCI_A3_BASE, status);
}


void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
  while(ui32Count--)
	{
		MAP_UART_transmitData(EUSCI_A3_BASE, *pui8Buffer);
		pui8Buffer++;
	}
}

void USART3_Send_Byte(uint8_t ch)
{
	MAP_UART_transmitData(EUSCI_A3_BASE, ch);
}

/* Current setting: 9600 bps */
const eUSCI_UART_Config uart3Config = {
//		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
//		6,                                       // BRDIV = 13
//		8,                                       // UCxBRF = 0
//		32,                                    // UCxBRS = 37 0X20
//		EUSCI_A_UART_NO_PARITY,                  // No Parity
//		EUSCI_A_UART_LSB_FIRST,                  // MSB First
//		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
//		EUSCI_A_UART_MODE,                       // UART mode
//		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		13,                                       // BRDIV = 13   6
		0,                                       // UCxBRF = 0   8
		0,                                       // UCxBRS = 0   32
		EUSCI_A_UART_NO_PARITY,                  // No Parity
		EUSCI_A_UART_LSB_FIRST,                  // MSB First
		EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
		EUSCI_A_UART_MODE,                       // UART mode
		//EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
		EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
};
/***********************************************************
@函数名：USART2_Init
@入口参数：无
@出口参数：无
功能描述：串口2配置（板上标注为UART3）波特率115200
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void USART3_Init()
{
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9,
             GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A3_BASE, &uart3Config);

	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A3_BASE);
	/* Enabling interrupts */
	MAP_UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);  	
  MAP_Interrupt_enableInterrupt(INT_EUSCIA3);
}
/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

