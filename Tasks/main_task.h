/** @file main_task.c
 *  @version 1.0
 *  @date  Sep 2019
 *
 *  @brief 
 */
 
#ifndef  __MAIN_TASK_H__
#define __MAIN_TASK_H__

#include "stm32f4xx_hal.h"
#include "DR16_decode.h"

#define RC_YAW_RATIO                      0.0002 // 0.0001    
#define  debug_mode  1

typedef enum
{
  SAFETY_MODE	=0,  
  PATROL_MODE,	   // Ѳ��ģʽ  
  NORMAL_MODE,	   // 	
  SICK_MODE,	           // ����ģʽ
} hero_mode_e;

typedef enum
{
  BLANK	=0,  
  WHITE,	   
  PINK,	       
} ball_color_e;

typedef struct
{
  float           offset;
  float           left_limit;
  float           right_limit;
  float           middle_limit;	
  float           Target_angle[5];
  float            total_angle;
  float            speed_rpm;
  int16_t         speed;
  float            angle;	
  int8_t          mode;    
   float           target;	
   float           buffer;
   float           distance;
   int16_t       current;
}gimbal_t;

typedef struct
{

   uint8_t    flag; 
   uint8_t    speed_flag; 
   uint8_t    pink_flag; 	
   uint8_t    last_speed_flag; 	
   int16_t    number;
   int16_t    pink_cnt;	
	
   int16_t    speed;
   int16_t    cnt1;	
   int16_t    cnt2;		
}fric_data_t;

typedef struct
{

   uint8_t    task_cnt; 
   uint8_t    sick_flag; 
   uint8_t    circular_flag; 
	   uint8_t    inverse_flag;
	   uint8_t    calibration_flag;	
   uint8_t    color_flag;	
   uint8_t    fric_flag;		
   uint8_t    last_task_cnt;	
   uint8_t    patrol_flag; 	
   uint8_t    launch_flag; 
   uint8_t    start_flag;	
   uint8_t    start_flag1;	
   uint8_t    start_flag2;			
   int16_t    cnt;
   int16_t   start_cnt;	
   int16_t   stall_cnt;
   int16_t   sick_cnt; 		
   float 	deal_angle;
	   float  pos_x;
   float  pos_y;	
}logic_data_t;

typedef struct
{

   uint8_t   flag; 

   uint8_t   target_cnt;    
   uint8_t   distance_flag; 	
   uint8_t   current_flag; 
   uint8_t   last_flag;	
   uint8_t   end_flag;
   int cw_flag; 
   int ccw_flag;
	
   int32_t cnt;	
   int cnt1;		
   float   left_dis;
   float   right_dis;
   float   dis_error;
   float   mid_dis;
}lazer_data_t;

typedef struct
{
   uint8_t start_flag;
   int16_t start_cnt;
   int16_t timer_cnt;
   uint8_t color_flag;	
   uint8_t last_color_flag;	
	
	uint8_t target_flag;
	uint8_t state_flag;	
	uint8_t angle_flag;	
	uint8_t angle_flag1;	
	uint8_t angle_flag2;		
	int16_t state_cnt;
   float pos_x;
   float pos_y;
   float zangle;
   float angle_A;	
   float angle_B;
   float angle_C;
   float angle_D;    
   float angle_E; 
   float distance_A;	
   float distance_B;
   float distance_C;
   float distance_D;    
   float distance_E; 	
}location_t;

extern gimbal_t  gimbal_yaw;

void task_arrange_handle(void);     //��̨�΄սy�I�{��
void FlAG_CLEAR_HANDLE(void); //������И�־λ
void control_info_handle(void);       // ��̨�Ƕ���Ϣ����  
void angle_algorithm(float pos_x,float pos_y);  // �����������Ͱ�ĽǶȣ�������Ϣ���� �����붨λģ����Ϣ
void angle_algorithm_pro(float pos_x,float pos_y); // �����������Ͱ�ĽǶȣ�������Ϣ���� ���������ۼ�����Ϣ������У׼
void launch_dectect(void);  // �����̲����Ƿ���������
void angle_area_handle(void); // ���㵱ǰ��̨�����Ƕȷ�Χ����ǰָ��Ϊ��һ��Ŀ��Ͱ
void target_ton_handle(void); // ���ݳ�ǩ��Ŀ��Ͱ��ѡ��˳��ε�Ŀ��Ƕȣ�����ɨ��ģʽ
void target_ton_handle_pro(void); // ���ݳ�ǩ��Ŀ��Ͱ��ѡ��˳��ε�Ŀ��Ƕȣ�������һ��
void launch_position_detect(float lazer_left,float lazer_right); // ����ɨ���㷨
void location_inverse_handle(uint8_t flag,float distance,float angleM,float angleZ); //λ����ϢУ׼
void dis_sp_trans(int flag,float lazer_dis); // ���ٶ�Ӧ����
void collect_path_handle(void); // �������˶��켣ѡ��

extern location_t location_ms;
extern lazer_data_t lazer_data;
extern logic_data_t logic_data;
extern fric_data_t fric_data;

#endif
