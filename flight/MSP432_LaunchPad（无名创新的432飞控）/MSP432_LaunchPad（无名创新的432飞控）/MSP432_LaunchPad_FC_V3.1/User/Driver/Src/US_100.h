#ifndef _US_100_H_
#define _US_100_H_

typedef struct
{
	uint32_t us100_start_time;
	uint32_t us100_end_time;
	uint32_t us100_delta;
	uint8_t us100_update_flag;
	float distance;
	float last_distance;
	float pre_last_distance;
	float vel;
	float acc;
	float last_vel;
	uint32_t us100_rssi_cnt;
}us100_data;


//US100
#define User_Height_Max  240//用户使用的最大高度值，单位为cm，需要预留一定裕度给气压计/超声波切换过渡



#define US_100_Distance_CMD    0x55
#define US_100_Temperature_CMD 0x50

void US_100_Statemachine(void);
void US100_Init(void);
void US100_IRQHandler(void);

extern float Ground_Distance,Last_Ground_Distance,Ground_Distance_Div,Ground_Distance_Acc,Last_Ground_Distance_Div;


extern us100_data us100;
#endif


