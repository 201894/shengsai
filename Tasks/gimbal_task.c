/** @file gimbal_task.c
 *  @version 1.0
 *  @date Sep 2019
 *
 *  @brief gimbal control task
 *
 *
 */
#include "sick_task.h"
#include "bsp_can.h"	
#include "main_task.h"
#include "pid.h" 
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "STMGood.h"
#include "math.h"
#include "string.h"
float Laser_Right,Laser_Left,fric_speed;
extern float limit_distance;
void gimbal_task(void const *argu)
{
   for(;;)
   {
	   Laser_Left = LaserBack_Back*1.85f+114.46f;
	   lazer_data.left_dis =(float)LaserBack_Back;
	   Laser_Right = LaserBack_Front*1.705f+66.635f;
	   lazer_data.right_dis  =(float)LaserBack_Front;

	   #if pid_fric_adjust
	    pid_adjust(&pid_fric,_kp,_ki,_kd);
	   #endif
	   #if pid_yaw_adjust
	    pid_adjust(&pid_yaw_out,_kp,_ki,_kd);
	    pid_adjust(&pid_yaw_in,_kkp,_kki,_kkd);
	    pid_yaw_out.MaxOut = maxout1;
	    pid_yaw_in.MaxOut = maxout2;
	   #endif 

	     if ((Laser_Left-Laser_Right)>1000)
		 {dis_sp_trans(location_ms.target_flag,Laser_Right);}
		 else
		 {dis_sp_trans(location_ms.target_flag,Laser_Left);}	
		 if((Laser_Left)>5000&&Laser_Right>5000)
		 {dis_sp_trans(location_ms.target_flag,limit_distance-80);}			
         
	    if(lazer_data.cw_flag||lazer_data.ccw_flag)
		  fric_data.speed = 0;
		pid_ast(&pid_fric,fric_data.speed,moto_fric.speed_rpm,0);
	    yaw_mode_handle(gimbal_yaw.mode);
        
	   #if debug_mode 
		gimbal_yaw.buffer = pid_yaw_out.ctrOut; 
	   #else 
		gimbal_yaw.buffer = -pid_yaw_out.ctrOut; 
	   #endif 
		pid_ast(&pid_yaw_in,gimbal_yaw.buffer,gimbal_yaw.speed_rpm,0);
	    gimbal_yaw.current = (int)pid_yaw_in.ctrOut;
	    if(lazer_data.cw_flag||lazer_data.ccw_flag)
   	       send_gimbal_cur(0x200,0,(int)pid_fric.ctrOut);  
		else
   	      send_gimbal_cur(0x200,gimbal_yaw.current,(int)pid_fric.ctrOut);
	     
        osDelay(3);	   
   }
}

int position_flag=0;
extern float left_limit,right_limit;
void yaw_mode_handle(int mode_flag)
{
    if (mode_flag==SICK_MODE)
	{
     if (!lazer_data.end_flag)
	 {
	      gimbal_yaw.target -=0.07;
		  pid_ast(&pid_yaw_out,gimbal_yaw.target,gimbal_yaw.total_angle,1);	     
	 }
	 else
	 {
		 gimbal_yaw.target = (1.5+gimbal_yaw.left_limit +gimbal_yaw.right_limit)/2;
         pid_ast(&pid_yaw_out,gimbal_yaw.target,gimbal_yaw.total_angle,1); 
	 }
	}
	else	
	  {
	      pid_ast(&pid_yaw_out,gimbal_yaw.target,gimbal_yaw.total_angle,1); 
	  }    		
}

void gimbal_param_init(void)
{
    fric_data.speed = -2000;
	PID_struct_init(&pid_fric,fric_errILim,fric_maxOut,fric_kp,fric_ki,fric_kd);	
	PID_struct_init(&pid_yaw_out,200,500, \
	  32,0,0);   //       3508             YAW
	PID_struct_init(&pid_yaw_in,6000,8500, \
	  45,0.2,0);  //        3508             YAW 	
}

