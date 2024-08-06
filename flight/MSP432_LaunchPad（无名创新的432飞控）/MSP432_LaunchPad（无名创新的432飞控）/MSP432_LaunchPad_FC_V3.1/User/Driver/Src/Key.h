#ifndef __KEY_H__
#define __KEY_H__


#define QuadKey1  MAP_GPIO_getInputPinValue(GPIO_PORT_P1 , GPIO_PIN4)
#define QuadKey2  MAP_GPIO_getInputPinValue(GPIO_PORT_P1 , GPIO_PIN1)



#define LONG_PRESS_LIMIT  1000//2000ms
#define IN_PRESS_LIMIT  	250//2000ms


#define RC_MAX_LIMIT 2100 
#define RC_MIN_LIMIT 900
#define RC_MID_LIMIT 1500
#define RC_OFFSET_LIMIT 100


typedef enum {
	UP=0,
	DOWN,
	BUTTON_NUM
}BUTTON;

typedef enum
{
	NO_PRESS=0,
	SHORT_PRESS,
  LONG_PRESS,
  IN_PRESS,	
}press_state;

typedef struct
{
	press_state press;
  uint8_t value;
	uint8_t last_value;	
	uint_fast8_t  port;
	uint_fast16_t pin;
	uint32_t press_time;
  uint32_t release_time; 
  uint32_t in_time; 
	uint32_t in_press_cnt;
	uint32_t state_lock_time;
}button_state;

typedef struct
{
  button_state state[BUTTON_NUM];
}rc_buttton;


void Key_Init(void);
bool Key_Scan(uint8_t release);
void QuadShow(uint8_t show_enable);
void Read_Button_State_All(void);

extern uint8_t Key_Right_Release,Oled_Show_Enable;
extern int16_t Page_Number;

extern rc_buttton _button;
#endif


