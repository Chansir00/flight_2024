#ifndef __RC_H__
#define __RC_H__

#define  Scale_Pecent_Max  0.75   //最大解锁幅值量程因子
#define  Pit_Rol_Max 30           //最大俯仰、横滚期望
#define  Yaw_Max     200          //最大偏航期望  
#define  Buttom_Safe_Deadband  100 //油门底部安全死区


#define Lock_Controler  0
#define Unlock_Controler  1


typedef enum 
{
	RC_ROLL_CHANNEL=0,
	RC_PITCH_CHANNEL,
	RC_THR_CHANNEL,
	RC_YAW_CHANNEL,
	RC_AUX1_CHANNEL,
	RC_AUX2_CHANNEL,
	RC_AUX3_CHANNEL,
	RC_AUX4_CHANNEL,
	RC_CHL_MAX
}RC_CHL_MAP;

typedef enum 
{
  RC_ROLL=0,
	RC_PITCH,
	RC_YAW,
	RC_THR
}RC_RPYT;

typedef struct
{
  int16_t rc_rpyt[4];
	uint16_t  rcdata[8];
	Vector_RC cal[8];
}rc;

extern rc RC_Data;


extern uint16 Controler_State;
extern int16_t Throttle_Rate;
extern int16_t Auto_ReLock_Cnt;//自动上锁计数器
extern uint16_t Unlock_Makesure_Cnt,Lock_Makesure_Cnt,Forced_Lock_Makesure_Cnt;
extern uint16_t Unwanted_Lock_Flag;
extern uint16 Throttle_Control;
extern int16 Pitch_Control,Roll_Control,Yaw_Control;
extern uint16_t PPM_LPF_Databuf[4];
extern int16 TRPY[4];
extern int16 Target_Angle[2];
void Remote_Control(void);
#endif

