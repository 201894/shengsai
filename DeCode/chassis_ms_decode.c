
/** @file chassis_ms_decode.c
 *  @version 1.0
 *  @date  Sep 19
 *
 *  @brief receive message from chassis and send flag to chassis part
 *
 */

#include "chassis_ms_decode.h"
#include "string.h"
#include "math.h"
#include "CRC_Check.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "main_task.h"
wl4data chassis_data;
send_chassis_data send_chassis;
void bt_ms_receive(void)
{
	if(uart7_buff[0]==0xA5&&uart7_buff[1]==0x5A&&Verify_CRC8_Check_Sum(uart7_buff,20))
	{
		location_ms.last_color_flag = location_ms.color_flag ;		
	   	chassis_data.c[0] = uart7_buff[2];
	   	chassis_data.c[1] = uart7_buff[3];
	   	chassis_data.c[2] = uart7_buff[4];
	   	chassis_data.c[3] = uart7_buff[5];	
		location_ms.pos_x = chassis_data.f;
	   	chassis_data.c[0] = uart7_buff[6];
	   	chassis_data.c[1] = uart7_buff[7];
	   	chassis_data.c[2] = uart7_buff[8];
	   	chassis_data.c[3] = uart7_buff[9]; 
		location_ms.pos_y = chassis_data.f;	
	   	chassis_data.c[0] = uart7_buff[10];  
	   	chassis_data.c[1] = uart7_buff[11]; 
	   	chassis_data.c[2] = uart7_buff[12]; 
	   	chassis_data.c[3] = uart7_buff[13]; 
		location_ms.zangle = chassis_data.f; 
		location_ms.start_flag = uart7_buff[14]; 
		location_ms.color_flag  = uart7_buff[15]; 
		location_ms.state_flag  = uart7_buff[16]; 		
		
		if(location_ms.state_flag)
		{			
            if(location_ms.last_color_flag==location_ms.color_flag)
			{
			   location_ms.state_cnt ++;
			} 
			else 
			{
			   location_ms.state_cnt = 0;
			}  
			if (location_ms.state_cnt >=60)
			{
			    logic_data.color_flag = location_ms.color_flag ;
				location_ms.state_cnt = 0;
			}	
		}		
		else
		{
              logic_data.color_flag = 0; 
		      location_ms.state_cnt = 0;			
		}  
		if (!location_ms.start_flag)
		 {logic_data.start_flag1 = 1;}
		if(logic_data.start_flag1&&location_ms.start_flag)
		 {
		    logic_data.start_flag1 = 0;
			 logic_data.start_cnt ++;
		 }
			 		
	}
	 
	__HAL_UART_CLEAR_PEFLAG(&huart7);	  
}
int data_send_cnt;
void bt_ms_send(uint8_t path_flag,uint8_t state_flag,uint8_t robot_id,int position_flag)
{
	
	static uint8_t uart7_data[8];
	data_send_cnt++;
	if (data_send_cnt>=10)
	{
     uart7_data[0] = 0xA5;
	 uart7_data[1] = 0x5A;	
	 uart7_data[2] = path_flag; 
	 uart7_data[3] = state_flag; 
	 uart7_data[4] = robot_id;  
	 uart7_data[5] = position_flag;	
	 Append_CRC8_Check_Sum(uart7_data,8);	
	 HAL_UART_Transmit(&huart7,uart7_data,8,20);	
	 data_send_cnt = 0;	
	}
}


