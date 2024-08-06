#ifndef __OLED_H
#define __OLED_H


//#define  OLED_WORK_MODE_I2C//OLED使用I2C模式驱动，当采用SPI模式时，注释本行即可

#define Bit_SET   1
#define Bit_RESET 0
#define LCD_DCout(data)		GPIO_WriteBit(GPIO_PORT_P4,GPIO_PIN2,data)//DC
#define LCD_RSTout(data)	GPIO_WriteBit(GPIO_PORT_P4,GPIO_PIN1,data)//RES
#define LCD_SDAout(data)	GPIO_WriteBit(GPIO_PORT_P4,GPIO_PIN0,data)//D1
#define LCD_SCLout(data)	GPIO_WriteBit(GPIO_PORT_P6,GPIO_PIN1,data)//D0


void LCD_WrDat(unsigned char dat);
void LCD_WrCmd(unsigned char cmd);
void OLED_WrCmd(unsigned char IIC_Data);
void OLED_WrDat(unsigned char IIC_Data);
void LCD_Set_Pos(unsigned char x, unsigned char y);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void LCD_P6x8Char(unsigned char x,unsigned char  y,unsigned char ch);
void LCD_P6x8Str(unsigned char x,unsigned char  y,unsigned char ch[]);
void write_6_8_number(unsigned char x,unsigned char  y,float number);
void LCD_P8x16Char(unsigned char x,unsigned char  y,unsigned char ch);
void LCD_P8x16Str(unsigned char x,unsigned char  y,unsigned char ch[]);
void write_8_16_number(unsigned char x,unsigned char  y,float number);
void LCD_clear_L(unsigned char x,unsigned char y);
void OLED_Init(void);
void OLED_Init_Fast(void);
void Draw_Logo(void);
void Draw_Logo1(void);
void LCD_CLS(void);
#endif



