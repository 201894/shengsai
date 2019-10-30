


#ifndef __CHASSIS_MS_DECODE_H__
#define __CHASSIS_MS_DECODE_H__

#include "stm32f4xx_hal.h"
typedef struct{
  
  uint8_t  path_flag;
  uint8_t  state_flag;	
  uint8_t  robot_id;		
} send_chassis_data;


void bt_ms_receive(void);
void bt_ms_send(uint8_t path_flag,uint8_t state_flag,uint8_t robot_id,int position_flag);
extern send_chassis_data send_chassis;
#endif