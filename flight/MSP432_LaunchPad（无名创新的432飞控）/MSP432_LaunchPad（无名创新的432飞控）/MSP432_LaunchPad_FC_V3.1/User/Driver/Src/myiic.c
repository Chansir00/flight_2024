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
#include "myiic.h"
uint8 i2c_r=2;
const eUSCI_I2C_MasterConfig i2cConfig =
{
	EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
	12000000,                                // SMCLK = 3MHz (default)
	EUSCI_B_I2C_SET_DATA_RATE_1MBPS,      // Desired I2C Clock of 100khz
	0,                                      // No byte counter threshold
	EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};
/***********************************************************
@函数名：Init_I2C
@入口参数：无
@出口参数：无
功能描述：MSP432软件I2C初始化
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void Init_I2C(void) 
{
	GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);
	GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN5);
	delay_ms(10);
	GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN4);
	GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN5);
}
///////////////////////////////////////
void SDA_IOOUT(void)
{
	GPIO_setAsOutputPin(GPIO_PORT_P6,GPIO_PIN4);//SDA
}

void SDA_IOIN(void)
{
	GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PIN4);//SDA
}

//static void I2C_delay(void)
//{
//  //volatile int i = 1;	//7
//  //while (i)
//  //    i--;
//}


//**************************************
void I2C_Start(void)
{
  SDA_IOOUT();
  SCL_H;
  SDA_H;
//  I2C_delay();
  SDA_L;
//  I2C_delay();
}
//**************************************
void I2C_Stop(void)
{
  SDA_IOOUT();
  SCL_L;
//  I2C_delay();
  SDA_L;
//  I2C_delay();
  SCL_H;
//  I2C_delay();
  SDA_H;
//  I2C_delay();
}

//**************************************
unsigned char I2C_SlaveAck(void)
{
  SDA_IOOUT();
  SCL_L;
//  I2C_delay();
  SDA_H;
  
  SDA_IOIN();
//  I2C_delay();
  SCL_H;
  
//  I2C_delay();
  
  if(SDA_read)
  {
    SCL_L;
    return 0;
  }
  SCL_L;
//  I2C_delay();
  return 1;
}

//**************************************
void I2C_SendByte(unsigned char byte)
{
  unsigned char i = 8;
  SDA_IOOUT();
  while (i--)
  {
    SCL_L;
//    I2C_delay();
    if (byte & 0x80)
      SDA_H;
    else
      SDA_L;
    byte <<= 1;
//    I2C_delay();
    SCL_H;
//    I2C_delay();
  }
  SCL_L;
  if(I2C_SlaveAck()==0)
  {
    return ;
  }
}

//**************************************
unsigned char I2C_RecvByte(void)
{
  unsigned char i;
  unsigned char dat = 0;
  SDA_IOIN();
  for (i=0; i<8; i++)
  {
    dat <<= 1;
    SCL_H;
//    I2C_delay();
    dat |= SDA_read;
    SCL_L;
//    I2C_delay();
  }
  return dat;
}

//**************************************

bool OK=0;
unsigned char reg_data;
//void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
//{
//	I2C_Start();
//  I2C_SendByte(SlaveAddress);
//  I2C_SendByte(REG_Address);
//  I2C_SendByte(REG_data);
//  I2C_Stop();
//}
//unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
//{
//  unsigned char REG_data;
//  I2C_Start();
//  I2C_SendByte(SlaveAddress);
//  I2C_SendByte(REG_Address);
//  I2C_Start();
//  I2C_SendByte(SlaveAddress+1);
//  REG_data=I2C_RecvByte();
//  I2C_SlaveAck();
//  I2C_Stop();
//  return REG_data;
//}

/////////////////////////////////
//int16_t Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
//{
//  uint8_t msb , lsb ;
//  msb = Single_ReadI2C(SlaveAddress,REG_Address);
//  lsb = Single_ReadI2C(SlaveAddress,REG_Address+1);
//  return ((msb<< 8 )+ lsb) ;
//}

//unsigned long int Three_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
//{
//  unsigned char msb , lsb,xlsb;
//  msb = Single_ReadI2C(SlaveAddress,REG_Address);
//  lsb = Single_ReadI2C(SlaveAddress,REG_Address+1);
//  xlsb= Single_ReadI2C(SlaveAddress,REG_Address+2);
//  return (((unsigned long int)msb<< 16)|((unsigned long int)lsb<<8)|xlsb) ;
//}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define ack 1      //主应答
#define no_ack 0   //从应答

#define SCL1()         GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN5)
#define SCL0()         GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN5)
#define SDA1()         GPIO_setOutputHighOnPin(GPIO_PORT_P6,GPIO_PIN4)
#define SDA0()         GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN4)
#define SDA      			 GPIO_getInputPinValue(GPIO_PORT_P6,GPIO_PIN4)

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC延时
//  @return     void						
//  @since      v1.0
//  Sample usage:				如果IIC通讯失败可以尝试增加j的值
//-------------------------------------------------------------------------------------------------------------------
static void nc_iic_delay(void)
{
	//j=10通讯速率大约为100K （内核频率40M）
  //j=0 通讯速率大约为140K （内核频率40M）
  //uint16_t j=10;   
	//while(j--);
}

//内部使用，用户无需调用
void IIC_start(void)
{
	SDA1();
	SCL1();
	nc_iic_delay();
	SDA0();
	nc_iic_delay();
	SCL0();
}

//内部使用，用户无需调用
void IIC_stop(void)
{
	SDA0();
	SCL0();
	nc_iic_delay();
	SCL1();
	nc_iic_delay();
	SDA1();
	nc_iic_delay();
}

//主应答(包含ack:SDA=0和no_ack:SDA=0)
//内部使用，用户无需调用
static void I2C_SendACK(unsigned char ack_dat)
{
    SCL0();
    nc_iic_delay();
		SDA_IOOUT();
    if(ack_dat) SDA0();
    else   SDA1();

    SCL1();
    nc_iic_delay();
    SCL0();
    nc_iic_delay();
}

static int WaitAck(void)
{
    SCL0();
    SDA_IOIN();
    nc_iic_delay();
    SCL1();
    nc_iic_delay();
    if(SDA)           //应答为高电平，异常，通信失败
    {
			SDA_IOOUT();
			SCL0();
			return 0;
    }
    SDA_IOOUT();
    SCL0();
    nc_iic_delay();
    return 1;
}

//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void send_ch(uint8_t c)
{
    uint8_t i = 8;
    while(i--)
    {
        if(c & 0x80)	      SDA1();//SDA 输出数据
        else		      SDA0();
        c <<= 1;
        nc_iic_delay();
        SCL1();                //SCL 拉高，采集信号
        nc_iic_delay();
        SCL0();                //SCL 时钟线拉低
    }
	WaitAck();
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|IIC_ack_main()使用
//内部使用，用户无需调用
uint8_t read_ch(uint8_t ack_x)
{
    uint8_t i;
    uint8_t c;
    c=0;
    SCL0();
    nc_iic_delay();
    SDA1();             //置数据线为输入方式
    SDA_IOIN();
    for(i=0;i<8;i++)
    {
        nc_iic_delay();
        SCL0();         //置时钟线为低，准备接收数据位
        nc_iic_delay();
        SCL1();         //置时钟线为高，使数据线上数据有效
        nc_iic_delay();
        c<<=1;
        if(SDA) c+=1;   //读数据位，将接收的数据存c
    }
    SDA_IOOUT();
    SCL0();
    nc_iic_delay();
    I2C_SendACK(ack_x);
	
    return c;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC写数据到设备寄存器函数
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat				写入的数据
//  @return     void						
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void nc_iic_write_reg(uint8_t dev_add, uint8_t reg, uint8_t dat)
{
    IIC_start();
    send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
    send_ch( reg );   				 //发送从机寄存器地址
    send_ch( dat );   				 //发送需要写入的数据
    IIC_stop();
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC从设备寄存器读取数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8_t			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8_t nc_iic_read_reg(uint8_t dev_add, uint8_t reg)
{
    uint8_t dat;
    IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
    send_ch( reg );   				//发送从机寄存器地址
	
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	dat = read_ch(no_ack);   				//发送需要写入的数据
	IIC_stop();
	
	return dat;
}
//  @brief      模拟IIC读取多字节数据
//  @param      dev_add			设备地址(低七位地址)
//  @param      reg				寄存器地址
//  @param      dat_add			数据保存的地址指针
//  @param      num				读取字节数量
//  @param      type			选择通信方式是IIC  还是 SCCB
//  @return     uint8			返回寄存器的数据			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void nc_iic_read_regs(uint8_t dev_add, uint8_t reg, uint8_t *dat_add, uint8_t num)
{
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	
	
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
    while(--num)
    {
        *dat_add = read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = read_ch(no_ack); //读取数据
	IIC_stop();
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      模拟IIC端口初始化
//  @param      NULL
//  @return     void	
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------

uint8_t nc_iic_read_len(uint8_t dev_add, uint8_t reg, uint8_t len, uint8_t *dat)
{
	uint8_t i;

	IIC_start();
    send_ch( (dev_add<<1) | 0x00);  //发送器件地址加写位
	send_ch( reg );   				//发送从机寄存器地址
	IIC_start();
	send_ch( (dev_add<<1) | 0x01);  //发送器件地址加读位
	for (i = 0; i < len-1; i++)
	{
		dat[i] = read_ch(1);	//发送需要写入的数据
	}
	dat[i] = read_ch(0);	//发送需要写入的数据
	IIC_stop();
	
	return 0;
}



uint8_t nc_iic_write_len(uint8_t dev_add, uint8_t reg, uint8_t len,uint8_t  *dat)
{
	uint8_t i;
	
	IIC_start();
    send_ch( (dev_add<<1) | 0x00);   //发送器件地址加写位
	send_ch( reg );   				 //发送从机寄存器地址
	for (i = 0; i < len; i++)
	{
		send_ch(dat[i]);   				 //发送需要写入的数据
	}
	IIC_stop();
	
	return 0;
}


void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
	nc_iic_write_reg(SlaveAddress,REG_Address,REG_data);
}

unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
	return nc_iic_read_reg(SlaveAddress,REG_Address);
}

///////////////////////////////
int16_t Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
	uint8_t buf[2];
	nc_iic_read_len(SlaveAddress,REG_Address,2,buf);
  return ((buf[0]<< 8 )+ buf[1]) ;
}

unsigned long int Three_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
	uint8_t buf[3];
	nc_iic_read_len(SlaveAddress,REG_Address,3,buf);
  return (((unsigned long int)buf[2]<< 16)|((unsigned long int)buf[1]<<8)|buf[0]) ;
}

void Nbyte_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address,uint16_t N,uint8_t *buf)
{
	nc_iic_read_len(SlaveAddress,REG_Address,N,buf);
}




















static void I2C_Delay_Adjust(void)
{
  volatile int i = 1;	//7
  while (i)
    i--;
}


//**************************************
void I2C_Start_Adjust(void)
{
  SDA_IOOUT();
  SCL_H;
  SDA_H;
  I2C_Delay_Adjust();
  SDA_L;
  I2C_Delay_Adjust();
}
//**************************************
void I2C_Stop_Adjust(void)
{
  SDA_IOOUT();
  SCL_L;
  I2C_Delay_Adjust();
  SDA_L;
  I2C_Delay_Adjust();
  SCL_H;
  I2C_Delay_Adjust();
  SDA_H;
  I2C_Delay_Adjust();
}

//**************************************
unsigned char I2C_SlaveAck_Adjust(void)
{
  SDA_IOOUT();
  SCL_L;
  I2C_Delay_Adjust();
  SDA_H;
  
  SDA_IOIN();
  I2C_Delay_Adjust();
  SCL_H;
  
  I2C_Delay_Adjust();
  
  if(SDA_read)
  {
    SCL_L;
    return 0;
  }
  SCL_L;
  I2C_Delay_Adjust();
  return 1;
}

//**************************************
void I2C_SendByte_Adjust(unsigned char byte)
{
  unsigned char i = 8;
  SDA_IOOUT();
  while (i--)
  {
    SCL_L;
    I2C_Delay_Adjust();
    if (byte & 0x80)
      SDA_H;
    else
      SDA_L;
    byte <<= 1;
    I2C_Delay_Adjust();
    SCL_H;
    I2C_Delay_Adjust();
  }
  SCL_L;
  if(I2C_SlaveAck_Adjust()==0)
  {
    return ;
  }
}

//**************************************
unsigned char I2C_RecvByte_Adjust(void)
{
  unsigned char i;
  unsigned char dat = 0;
  SDA_IOIN();
  for (i=0; i<8; i++)
  {
    dat <<= 1;
    SCL_H;
    I2C_Delay_Adjust();
    dat |= SDA_read;
    SCL_L;
    I2C_Delay_Adjust();
  }
  return dat;
}

//**************************************
void Single_WriteI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
	MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SlaveAddress>>1);
	while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE));
	/* Send out EEPROM Mock Read Cmd (2 databytes) */
	MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, REG_Address);  // Start + 1Byte
	MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, REG_data); // Poll for TXINT,Send 1Byte
}

//**************************************
unsigned char Single_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char REG_data;
	OK=0;
	MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SlaveAddress>>1);
	while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE));
	/* Send out EEPROM Mock Read Cmd (2 databytes) */
	MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, REG_Address);  // Start + 1Byte
	/* Now we need to initiate the read */
	/* Wait until 2nd Byte has been output to shift register */
	while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
	// Send the restart condition, read one byte, send the stop condition right away
	EUSCI_B1->CTLW0 &= ~(EUSCI_B_CTLW0_TR);
	EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
	while(MAP_I2C_masterIsStartSent(EUSCI_B1_BASE));
	EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
	while(!OK);
	REG_data=reg_data;
  return REG_data;
}

unsigned short int Double_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char msb , lsb ;
  msb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address);
  lsb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address+1);
  return ( ((unsigned short int)msb) << 8 | lsb) ;
}

unsigned long int Three_ReadI2C_Adjust(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char msb , lsb,xlsb;
  msb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address);
  lsb = Single_ReadI2C_Adjust(SlaveAddress,REG_Address+1);
  xlsb= Single_ReadI2C_Adjust(SlaveAddress,REG_Address+2);
  return (((unsigned long int)msb<< 16)|((unsigned long int)lsb<<8)|xlsb) ;
}
void EUSCIB1_IRQHandler(void)
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B1_BASE);
    MAP_I2C_clearInterruptFlag(EUSCI_B1_BASE, status);

    /* Receives bytes into the receive buffer. If we have received all bytes,
     * send a STOP condition */
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        // One-byte Read
			reg_data = MAP_I2C_masterReceiveSingle(EUSCI_B1_BASE);
			OK=1;
			
    }
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

