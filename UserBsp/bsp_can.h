
/** @file bsp_can.h
 *  @version 1.0
 *  @date  Sep 2019
 *
 *  @brief receive external can device message
 *
 */

#ifndef  __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stm32f4xx_hal.h"

typedef enum
{
  CAN_YAW_ID	                         = 0x201,
  CAN_FRIC_ID                            = 0x202,	
} can_msg_id;

typedef union
{
	uint8_t c[2];
	int16_t d;
	uint16_t ud;
}wl2data;

typedef union
{	
	uint8_t c[4];
	float f;
	uint32_t d;
}wl4data;

typedef struct
{
	float pit_speed;
	float angle_z;
	float yaw_speed;

}gyro_param;


/* can receive motor parameter structure */
#define FILTER_BUF 5
typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  uint16_t temp;	
  int16_t  speed_rpm;
  int16_t  current;
  int16_t  torque;	
  int32_t  round_cnt;
  int32_t  total_ecd;
  float      total_angle;
  float      stir_angle;
  float      angle_offset;
  uint16_t offset_ecd;
  uint32_t msg_cnt;
 
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
	
} moto_param;

extern	moto_param    moto_yaw;
extern	moto_param    moto_fric;
extern  wl4data           data4bytes;

void can_device_init(void);
void can_receive_start(void);
void send_chassis_ms(uint32_t id,uint8_t data[8]);
void send_ygyro(uint32_t id,int16_t gy,int16_t gz,float angle);
void encoder_data_handle(moto_param* ptr,uint8_t RxData[8]);
void gyro_data_handle(wl2data* ptr,wl4data* ptrr,gyro_param* gyro,uint8_t RxData[8]);
void send_gimbal_cur(uint32_t id,int16_t iq1, int16_t iq2);  //CAN 1

#endif
