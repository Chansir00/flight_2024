#ifndef _MY_COMMON_H_
#define _MY_COMMON_H_

#include "stm32f10x.h"
#include <string.h>
#include <math.h>

#include "I2C.h"
#include "SPI.h"
#include "TIM.h"
#include "USART.h"

#include "mpu6050.h"
#include "nrf24l01.h"
#include "LED.h"

#include "delay.h"
#include "myMath.h"
#include <math.h>

#include "kalman.h"
#include "filter.h"

#include "imu.h"
#include "Remote.h"
#include "control.h"
#include "ANO_DT.h"

#undef SUCCESS
	#define SUCCESS 0
#undef FAILED
	#define FAILED  1

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       long long int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       long long uint64_t;


#define NULL 0

typedef struct{
	int16_t magX;
	int16_t magY;
	int16_t magZ;
}_st_Mag;


typedef struct{
	float rate;
	float height;
}High;

typedef struct
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t thr;
	uint16_t yaw;
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
}_st_Remote;




typedef volatile struct
{
	uint8_t unlock;
	

}_st_ALL_flag;


extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_Mag AK8975; //保留，需外接磁力计
extern _st_AngE Angle;


extern _st_ALL_flag ALL_flag;

extern	PidObject pidRateX;
extern	PidObject pidRateY;
extern	PidObject pidRateZ;

extern	PidObject pidPitch;
extern	PidObject pidRoll;
extern	PidObject pidYaw;

extern	PidObject pidHeightRate;
extern	PidObject pidHeightHigh;

extern uint32_t SysTick_count; //系统时间计数

#endif

