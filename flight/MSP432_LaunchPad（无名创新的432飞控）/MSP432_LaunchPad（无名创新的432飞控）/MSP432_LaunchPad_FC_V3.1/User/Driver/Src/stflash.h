#ifndef __24CXX_H
#define __24CXX_H
#include "softiic.h"




//AT24C02的地址是256个字节，地址从0-255即 0x000 到 0xFFH

#define AT24C01 127
#define AT24C02 255
#define AT24C04 511
#define AT24C08 1023
#define AT24C16 2047
#define AT24C32 4095
#define AT24C64 8191
#define AT24C128 16383
#define AT24C256 32767


#define EE_TYPE AT24C16

u8 AT24CXX_ReadOneByte(u16 ReadAddr);							   //指定地址读取一个字节
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite);		   //指定地址写入一个字节
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len); //指定地址开始写入指定长度的数据
u32 AT24CXX_ReadLenByte(u16 ReadAddr, u8 Len);					   //指定地址开始读取指定长度数据
void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead);	   //从指定地址开始读出指定长度的数据

void AT24CXX_Init(void); //初始化IIC


int EEPROM_Read_N_Data(uint32_t ReadAddress,float *ReadBuf,int32_t ReadNum);
void EEPROM_Write_N_Data(uint32_t WriteAddress,float *Buf,int32_t length);	

#endif
