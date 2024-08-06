#ifndef __USART_H__
#define __USART_H__
#include "Ringbuf.h"

typedef struct
{
  uint32_t Head;
  float DataBuf[8];
  uint32_t End;
}DMA_Vcan_Buff;


void wust_sendware(unsigned char *wareaddr, int16_t waresize);
void Vcan_Send(void);


void USART0_Init(void);
void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count);	
void USART0_Send_Byte(uint8_t ch);

void USART2_Init(void);
void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count);	
void Serial_Data_Send_Dma(u32 SendBuff,u16 len);
	

void USART1_Init(void);
void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count);	
void USART1_Send_Byte(uint8_t ch);

void USART3_Init(void);
void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count);	
void USART3_Send_Byte(uint8_t ch);


extern RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;
extern float height,height_T;
#endif


