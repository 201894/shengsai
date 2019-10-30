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
  PATROL_MODE,	   // 巡逻模式  
  NORMAL_MODE,	   // 	
  SICK_MODE,	           // 激光模式
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

void task_arrange_handle(void);     //云台任務統籌調度
void FlAG_CLEAR_HANDLE(void); //清空所有標志位
void control_info_handle(void);       // 云台角度信息处理  
void angle_algorithm(float pos_x,float pos_y);  // 机器人至五个桶的角度，距离信息计算 ，输入定位模块信息
void angle_algorithm_pro(float pos_x,float pos_y); // 机器人至五个桶的角度，距离信息计算 ，输入理论计算信息，用于校准
void launch_dectect(void);  // 检测底盘拨盘是否做出动作
void angle_area_handle(void); // 计算当前云台所处角度范围，当前指向为哪一个目标桶
void target_ton_handle(void); // 根据抽签得目标桶，选择此场次的目标角度，用于扫描模式
void target_ton_handle_pro(void); // 根据抽签得目标桶，选择此场次的目标角度，用作第一轮
void launch_position_detect(float lazer_left,float lazer_right); // 激光扫描算法
void location_inverse_handle(uint8_t flag,float distance,float angleM,float angleZ); //位置信息校准
void dis_sp_trans(int flag,float lazer_dis); // 射速对应函数
void collect_path_handle(void); // 机器人运动轨迹选择

extern location_t location_ms;
extern lazer_data_t lazer_data;
extern logic_data_t logic_data;
extern fric_data_t fric_data;

#endif
