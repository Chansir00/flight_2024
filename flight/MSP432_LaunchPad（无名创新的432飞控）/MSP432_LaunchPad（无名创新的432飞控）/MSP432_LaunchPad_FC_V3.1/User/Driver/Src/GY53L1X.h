#ifndef __GY53L1X_H
#define __GY53L1X_H



void GY53L1X_Init(void);
void GY53L1X_Data_Update(void);


typedef struct
{
	uint32_t gy53l1x_start_time;
	uint32_t gy53l1x_end_time;
	uint32_t gy53l1x_delta;
	uint8_t gy53l1x_update_flag;
	float distance;
	float last_distance;
	float pre_last_distance;
	float vel;
	float acc;
	float last_vel;
}vl53l1x_data;

extern vl53l1x_data vl53l1x_in1,vl53l1x_in2;


#endif



