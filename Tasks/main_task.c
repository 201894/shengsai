
/** @file main_task.c
 *  @version 1.0
 *  @date  Sep 2019
 *
 *  @brief 
 */
 
#include "main_task.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "pid.h"
#include <math.h>
#include "cmsis_os.h"
#include "imu_handle.h"
#include "ano_dt.h"
#include "STMGood.h"
#include "chassis_ms_decode.h"

#define TARGET_BLANK      0
//// RED     ==  WHITE 
//// BLUE    ==  BLANK  
int target_base = 34;    
location_t location_ms; 
gimbal_t  gimbal_yaw;
fric_data_t fric_data;
lazer_data_t lazer_data;
logic_data_t logic_data;
float limit_distance;
int lazer_cnt;
int state_cnt;
extern float Laser_Right,Laser_Left;

void info_handle_task(void const *argu)
{
   for(;;)
   {
	   imu_cal_update();	   
       control_info_handle();
       launch_dectect();
    //  ANO_DT_Data_Exchange();
	   if(!logic_data.calibration_flag)
	   {	    
			angle_algorithm(location_ms.pos_x+124*sin(location_ms.zangle*3.1415926f/180),\
			location_ms.pos_y-124*cos(location_ms.zangle*3.1415926f/180));	   
	   }
	   else
	   {
  		angle_algorithm_pro(logic_data.pos_x,logic_data.pos_y);
	   }
	   angle_area_handle();
	   collect_path_handle();
	  if(logic_data.circular_flag ==1)
	   {target_ton_handle();}
	  else
	   {target_ton_handle_pro();}		  
	   bt_ms_send(send_chassis.path_flag, send_chassis.state_flag, \
	                      send_chassis.robot_id, target_base);
       osDelay(2);
   }
}
int start_cnt;
void main_task(void const *argu)
{
   for(;;)
   {
      #if	TARGET_BLANK   
	    send_chassis.robot_id = 1;   
      #else  
	    send_chassis.robot_id = 2;	   
	  #endif  	
	   if(location_ms.start_flag)
	   {
           task_arrange_handle();	
	   }
	   else
	   {
         #if	!TARGET_BLANK   
	        logic_data.fric_flag = 1;	  
         #else  
	    	logic_data.fric_flag = 2;	  
	     #endif  	
          logic_data.calibration_flag = 0;	   
		}        
	    if((fric_data.pink_cnt >=4) || (logic_data.start_cnt > 1))
		{ logic_data.circular_flag = 1;}
		if(lazer_data.end_flag&&(logic_data.start_flag2==1)&&send_chassis.state_flag&&(logic_data.start_cnt >2))
		{ 
			 location_inverse_handle(logic_data.inverse_flag,Laser_Left,\
			 gimbal_yaw.angle,location_ms.zangle);
			logic_data.calibration_flag = 1;
        }		
	   if(state_cnt>=400)
	   {
   	      FlAG_CLEAR_HANDLE(); 	
	   }
       if(logic_data.sick_flag&&!lazer_data.end_flag)	
	   {
		   gimbal_yaw.mode = SICK_MODE;		
	       launch_position_detect(lazer_data.left_dis,lazer_data.right_dis); 	   
	   }		   
        osDelay(2);
   }
}

void task_arrange_handle(void)
{
	
  logic_data.last_task_cnt = logic_data.task_cnt ;
  switch (logic_data.task_cnt)
  {
	  gimbal_yaw.mode = NORMAL_MODE;  
	  case 0:
	  {
		  switch (logic_data.color_flag)
		  {
           /*
            white   2
            pink   3
            blank  1
            background   4
           */				  
           #if  !TARGET_BLANK 		 
		   case 1 :  // blank
           {
			      logic_data.task_cnt  = 2;  
			      logic_data.task_cnt ++; 	 
			      logic_data.fric_flag = 1;	  
//			      gimbal_yaw.target = 180;   
		          break; 
	       }			 
		   case 2 :  // white
           {
		         gimbal_yaw.target = gimbal_yaw.Target_angle[2]; 
			     logic_data.task_cnt  = 2;
			     logic_data.fric_flag = 2;	 
				 logic_data.task_cnt ++; 		
		      break; 
	       }			
             #else
		   case 1 :  // blank
           {
		          gimbal_yaw.target = gimbal_yaw.Target_angle[2]; 
			      logic_data.task_cnt  = 2; logic_data.fric_flag = 1;	 
				   logic_data.task_cnt ++; 			 
		      break; 
	         }			 
			case 2 :  // white
            {
			     logic_data.task_cnt  = 2;
				 logic_data.fric_flag = 2;	 
//				 gimbal_yaw.target = 180;
			     logic_data.task_cnt ++; 				 
		      break; 
	         }			  			  
             #endif			  
			case 3 :  //pink 
            {	
                logic_data.fric_flag = 3;								
			   if (fric_data.number==0) 
			   {
				   fric_data.number = !fric_data.number; 
		           gimbal_yaw.target = gimbal_yaw.Target_angle[1];  // ²¹³¥ 
				   logic_data.start_flag2 = 1;
			   }
               else 
			   {
				   fric_data.number = !fric_data.number;	
		             gimbal_yaw.target = gimbal_yaw.Target_angle[0]; 
				   logic_data.start_flag2 = 2;				   
			   }	
			   if(logic_data.circular_flag ==1)
			     { logic_data.task_cnt ++; }
			   else
			     { logic_data.task_cnt = 2; }			   
		      break; 
	         }
	     }
 
	   }break;
	  case 1:
	  {
		  
		/*Gimbal get to the pre place, start patrol,then find the right place,start the fric motor*/		  
        if (fabs(pid_yaw_out.errNow)<=0.8f)
		{
          logic_data.patrol_flag  = 1;
	      logic_data.cnt  ++;	
		} 
		if (logic_data.patrol_flag&& logic_data.cnt >=50)
		{
          logic_data.sick_flag =1;
		}
			if (lazer_data.end_flag)
			{
		      logic_data.task_cnt ++;
			}
	  }break;
	  case 2:
	  {
		   gimbal_yaw.mode = NORMAL_MODE;	
		  //waiting for the gimbal to the right place
        if (fabs(pid_yaw_out.errNow)<=0.6)
		  logic_data.task_cnt ++;
	  }break;
	  case 3:
	  {
		   gimbal_yaw.mode = NORMAL_MODE;	
		   logic_data.stall_cnt  ++;
	     /* open the Friction motor .send the state_flag to chassis part*/		   
		 if ((logic_data.stall_cnt>=30)&&fabs(pid_fric.errNow)<=20&&fabs(pid_yaw_out.errNow)<=0.6f) 
		 {
		    send_chassis.state_flag = 1; 
		    logic_data.task_cnt ++; 
		 }
	  }break;   
	  case 4:  
	  {
		   gimbal_yaw.mode = NORMAL_MODE; 
		 		  
          if (logic_data.launch_flag) 
		  {
		     logic_data.task_cnt ++; 
		  }
	  }break;  	  	  
	  case 5:
	  {
		   gimbal_yaw.mode = NORMAL_MODE;  
           logic_data.cnt ++;
		   if (location_ms.state_flag&&(logic_data.cnt >=50))
		   {
                logic_data.cnt = 0;
				logic_data.task_cnt ++; 			   
		   } 
	  }break;	 
	  case 6:
	  {
		   gimbal_yaw.mode = NORMAL_MODE;  
		  // let all flag to 0
   	       FlAG_CLEAR_HANDLE(); 
	  }break;		  
	  default:
	   break;			  
  }     
}

void  FlAG_CLEAR_HANDLE(void)
{
     logic_data.start_flag = 0;
     logic_data.patrol_flag  = 0; 
     logic_data.launch_flag = 0; 
	 send_chassis.state_flag = 0;	
     logic_data.task_cnt = 0;
	 logic_data.sick_flag =0;
     lazer_data.last_flag = 0;
	 logic_data.sick_cnt = 0;
	 lazer_data.distance_flag = 0;
	 lazer_data.current_flag = 0;
     lazer_data.end_flag = 0;
     lazer_data.flag =0;
 	 lazer_cnt = 0;
	 state_cnt = 0;
	 logic_data.cnt  = 0;
	 logic_data.stall_cnt  = 0;
}

void control_info_handle(void)
{
	#if debug_mode
	 gimbal_yaw.total_angle = (moto_yaw.total_angle - moto_yaw.angle_offset)/5.49;   //   GYRO
	 gimbal_yaw.angle = gimbal_yaw.total_angle - 360.0f*(int)(gimbal_yaw.total_angle/360.0f);	
    #else
	 gimbal_yaw.total_angle = imunow + cnt*360.0 - imu_first;   //   GYRO
    #endif

//     gimbal_yaw.speed_rpm = 0 ;   //  GYRO
     gimbal_yaw.speed_rpm = moto_yaw.speed_rpm;   //  GYRO	   
}

void launch_dectect(void)
{
	if (location_ms.start_flag&&location_ms.state_flag)	
	  logic_data.start_flag =1;
	fric_data.last_speed_flag = fric_data.speed_flag ;
    if (location_ms.state_flag) 
	{
        fric_data.speed_flag = 1; 
		if(logic_data.task_cnt ==4)
	    {	state_cnt ++;}
	}
	else
	{ fric_data.speed_flag = 0;}
	if (!fric_data.last_speed_flag &&fric_data.speed_flag&&send_chassis.state_flag)
	{logic_data.launch_flag = 1;	}
	if(logic_data.color_flag !=3)
	{
	  fric_data.pink_flag = 1;
	}
	if(fric_data.pink_flag&&(logic_data.color_flag==3))
	{
	  fric_data.pink_flag = 0;	  
	  fric_data.pink_cnt ++;		  
	}
}

void angle_algorithm(float pos_x,float pos_y)
{
   location_ms.angle_A = atan2((2200.0f-pos_x),(4354.0f-pos_y));
	location_ms.angle_A *= 180.0f/3.1415926f;   
	 location_ms.distance_A =  sqrt((22-pos_x/100)*(22-pos_x/100) + (43.54-pos_y/100)*(43.54-pos_y/100));
	  location_ms.distance_A = location_ms.distance_A *100;
   location_ms.angle_B = atan2((2200.0f-pos_x),(pos_y+46.0f));
	location_ms.angle_B *= 180.0f/3.1415926f;
	 location_ms.angle_B = 180.0f - location_ms.angle_B; 
	 location_ms.distance_B =  sqrt((22-pos_x/100)*(22-pos_x/100) + (pos_y+0.46)/100*(pos_y+0.46)/100);
	  location_ms.distance_B = location_ms.distance_B *100;	
   location_ms.angle_C = atan2((2200.0f+pos_x),(pos_y+46));
	location_ms.angle_C *= 180.0f/3.1415926f;	
	 location_ms.angle_C = 180.0f + location_ms.angle_C;
	 location_ms.distance_C =  sqrt((22+pos_x/100)*(22+pos_x/100) + (pos_y+0.46)/100*(pos_y+0.46)/100);
	  location_ms.distance_C = location_ms.distance_C *100;	
   location_ms.angle_D = atan2((2200.0f+pos_x),(4354.0f-pos_y)); 
	location_ms.angle_D *= 180.0f/3.1415926f;		
	 location_ms.angle_D = 360.0f - location_ms.angle_D;	
	 location_ms.distance_D =  sqrt((22+pos_x/100)*(22+pos_x/100) + (43.54-pos_y/100)*(43.54-pos_y/100));
	  location_ms.distance_D = location_ms.distance_D *100;	
  if (pos_x>=0)
  {
	 if(pos_y<=2154) 
	 {
      location_ms.angle_E = atan2(pos_x,(2154.0f-pos_y));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = - location_ms.angle_E;		
	 }
	 else
	 {
      location_ms.angle_E = atan2(pos_x,(pos_y-2154.0f));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = -180+location_ms.angle_E;			 
	 }
  }
  else
  {
	 if(pos_y<=2154) 
	 {
      location_ms.angle_E = atan2(-pos_x,(2154.0f-pos_y));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = -360+ location_ms.angle_E;		
	 }
	 else
	 {
      location_ms.angle_E = atan2(-pos_x,(pos_y-2154.0f));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = - location_ms.angle_E-180;			 
	 }
  }		 
	 location_ms.distance_E =  sqrt(pos_x/100*pos_x/100+ (21.54-pos_y/100)*(21.54-pos_y/100));
	  location_ms.distance_E = location_ms.distance_E *100;	
  
}

void angle_algorithm_pro(float pos_x,float pos_y)
{
   location_ms.angle_A = atan2((2200.0f-pos_x),(4600.0f-pos_y));
	location_ms.angle_A *= 180.0f/3.1415926f;   
	 location_ms.distance_A =  sqrt((22-pos_x/100)*(22-pos_x/100) + (46-pos_y/100)*(46-pos_y/100));
	  location_ms.distance_A = location_ms.distance_A *100;
   location_ms.angle_B = atan2((2200.0f-pos_x),(pos_y-200));
	location_ms.angle_B *= 180.0f/3.1415926f;
	 location_ms.angle_B = 180.0f - location_ms.angle_B; 
	 location_ms.distance_B =  sqrt((22-pos_x/100)*(22-pos_x/100) + (pos_y-2)/100*(pos_y-2)/100);
	  location_ms.distance_B = location_ms.distance_B *100;	
   location_ms.angle_C = atan2((2200.0f+pos_x),(pos_y-200));
	location_ms.angle_C *= 180.0f/3.1415926f;	
	 location_ms.angle_C = 180.0f + location_ms.angle_C;
	 location_ms.distance_C =  sqrt((22+pos_x/100)*(22+pos_x/100) + (pos_y-2)/100*(pos_y-2)/100);
	  location_ms.distance_C = location_ms.distance_C *100;	
   location_ms.angle_D = atan2((2200.0f+pos_x),(4600.0f-pos_y)); 
	location_ms.angle_D *= 180.0f/3.1415926f;		
	 location_ms.angle_D = 360.0f - location_ms.angle_D;	
	 location_ms.distance_D =  sqrt((22+pos_x/100)*(22+pos_x/100) + (46-pos_y/100)*(46-pos_y/100));
	  location_ms.distance_D = location_ms.distance_D *100;	
  if (pos_x>=0)
  {
	 if(pos_y<=2400) 
	 {
      location_ms.angle_E = atan2(pos_x,(2400.0f-pos_y));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = - location_ms.angle_E;		
	 }
	 else
	 {
      location_ms.angle_E = atan2(pos_x,(pos_y-2400.0f));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = -180+location_ms.angle_E;			 
	 }
  }
  else
  {
	 if(pos_y<=2400) 
	 {
      location_ms.angle_E = atan2(-pos_x,(2400.0f-pos_y));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = -360+ location_ms.angle_E;		
	 }
	 else
	 {
      location_ms.angle_E = atan2(-pos_x,(pos_y-2400.0f));
	  location_ms.angle_E *= 180.0f/3.1415926f;
	  location_ms.angle_E = - location_ms.angle_E-180;			 
	 }
  }		 
	 location_ms.distance_E =  sqrt(pos_x/100*pos_x/100+ (24-pos_y/100)*(24-pos_y/100));
	  location_ms.distance_E = location_ms.distance_E *100;	   
}

void launch_position_detect(float lazer_left,float lazer_right)
{
   if (!lazer_data.end_flag) 
   {
	 if (Laser_Left<=limit_distance)
       lazer_data.flag =1;
         
	 if ((Laser_Left>=limit_distance)&&!lazer_data.distance_flag)
		 lazer_data.distance_flag  = 0;
	 else 
	 {
		  if (!lazer_data.distance_flag) 
		  {
		    gimbal_yaw.left_limit  = gimbal_yaw.angle;		 
	        lazer_data.distance_flag = 1; 
		  }
	 }
	 if (Laser_Left<=limit_distance)
	 {
		 if (gimbal_yaw.total_angle<gimbal_yaw.left_limit)
           lazer_data.current_flag = 1;
	 }
     else
	 {
		 if (lazer_data.current_flag&&lazer_data.flag)	
		 {
		   gimbal_yaw.right_limit  = gimbal_yaw.total_angle;
		   lazer_data.end_flag = 1;
		 }
	 }
   }
}



int16_t round_cnt;
float raw_angle;
void angle_area_handle(void)
{
    round_cnt = (int)((gimbal_yaw.total_angle-location_ms.zangle)/360);
    raw_angle = gimbal_yaw.total_angle -location_ms.zangle  - 360.0f*round_cnt;
  #if 	!TARGET_BLANK	
    if (((raw_angle>=0.0f)&&(raw_angle<=90.0f))||((raw_angle>-360.0f)&&(raw_angle<-270.0f)))
	{
		limit_distance = location_ms.distance_A;
        location_ms.target_flag = 2;
	}
    if (((raw_angle>90.0f)&&(raw_angle<=180.0f))||((raw_angle>-270.0f)&&(raw_angle<-180.0f)))
	{
		limit_distance = location_ms.distance_B;
		location_ms.target_flag = 3;
	}
	if (((raw_angle>180.0f)&&(raw_angle<=270.0f))||((raw_angle>-180.0f)&&(raw_angle<-90.0f)))
	{
		limit_distance = location_ms.distance_C;
		location_ms.target_flag = 4;
	}
	if (((raw_angle>270.0f)&&(raw_angle<360.0f))||((raw_angle>-90.0f)&&(raw_angle<0.0f)))
	{
		limit_distance = location_ms.distance_D;
        location_ms.target_flag = 1;		
	}
  #else 
    if (((raw_angle>=0.0f)&&(raw_angle<=90.0f))||((raw_angle>-360.0f)&&(raw_angle<-270.0f)))
	{
		limit_distance = location_ms.distance_A;
        location_ms.target_flag = 4;		
	}
    if (((raw_angle>90.0f)&&(raw_angle<=180.0f))||((raw_angle>-270.0f)&&(raw_angle<-180.0f)))
	{
		limit_distance = location_ms.distance_B;
		location_ms.target_flag = 1;	
	}
	if (((raw_angle>180.0f)&&(raw_angle<=270.0f))||((raw_angle>-180.0f)&&(raw_angle<-90.0f)))
	{
		limit_distance = location_ms.distance_C;
		location_ms.target_flag = 2;			
	}
	if (((raw_angle>270.0f)&&(raw_angle<360.0f))||((raw_angle>-90.0f)&&(raw_angle<0.0f)))
	{
		limit_distance = location_ms.distance_D;
        location_ms.target_flag = 3;				
	}
   #endif
	  limit_distance = limit_distance+500;
}

void target_ton_handle(void)
{
	
   #if  !TARGET_BLANK 	
	//  WIHTE
     switch(target_base)
	 {
		 // CD
		 case 14:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_C - 0.00474*location_ms.distance_C+22.38;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D - 0.00474*location_ms.distance_D+22.38;
            logic_data.inverse_flag = 3;			 
		 }break; 
		 // BD
		 case 13:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B - 0.00474*location_ms.distance_B+22.38;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D - 0.00474*location_ms.distance_D+22.38;		
            logic_data.inverse_flag = 3;			 
		 }break; 
          // AD		 
		 case 12:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A - 0.00474*location_ms.distance_A+22.38;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D - 0.00474*location_ms.distance_D+22.38;		
            logic_data.inverse_flag = 3;			 
		 }break;	
		 //AC
		 case 24:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A - 0.00474*location_ms.distance_A+22.38;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C - 0.00474*location_ms.distance_C+22.38;		
              logic_data.inverse_flag = 2;			 
		 }break;	
		 //AB
		 case 23:
		 {
			gimbal_yaw.Target_angle[1] = location_ms.angle_A - 0.00474*location_ms.distance_A+22.38; 
			gimbal_yaw.Target_angle[0] = location_ms.angle_B- 0.00474*location_ms.distance_B+22.38;
            logic_data.inverse_flag = 1;			 
		 }break;	
		 //BC
		 case 34:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B -0.00474*location_ms.distance_B+22.38;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C - 0.00474*location_ms.distance_C+22.38;	
            logic_data.inverse_flag = 2;			 
		 }break;			 
	 }
	 
	 #else
	//  BLANK  
     switch(target_base)
	 {
		 // AB
		 case 14:
		 {
			gimbal_yaw.Target_angle[1] = location_ms.angle_A - 0.00474*location_ms.distance_A+23.38;
			gimbal_yaw.Target_angle[0] = location_ms.angle_B - 0.00474*location_ms.distance_B+23.38;	
            logic_data.inverse_flag = 1;			 
		 }break; 
		 // DB
		 case 13:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B - 0.00474*location_ms.distance_B+30;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D - 0.00474*location_ms.distance_D+26.38;
            logic_data.inverse_flag = 3;			 
		 }break; 
          // BC		 
		 case 12:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B - 0.00474*location_ms.distance_B+24.38;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C - 0.00474*location_ms.distance_C+24.38;	
            logic_data.inverse_flag = 2;			 
		 }break;	
		 //AC
		 case 24:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A - 0.00474*location_ms.distance_A+24.38;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C - 0.00474*location_ms.distance_C+24.38;	
             logic_data.inverse_flag = 2;			 
		 }break;	
		 //DC
		 case 23:
		 {
			gimbal_yaw.Target_angle[1] = location_ms.angle_D - 0.00474*location_ms.distance_D+24.38;
			gimbal_yaw.Target_angle[0] = location_ms.angle_C - 0.00474*location_ms.distance_C+24.38;			
              logic_data.inverse_flag = 3;			 
		 }break;	
		 //AD
		 case 34:
		 {
			gimbal_yaw.Target_angle[1] = location_ms.angle_D - 0.00474*location_ms.distance_D+24.38;
			gimbal_yaw.Target_angle[0] = location_ms.angle_A - 0.00474*location_ms.distance_A+24.38;			
            logic_data.inverse_flag = 3;			 
		 }break;
	 }
	 #endif
	  if (abs(gimbal_yaw.total_angle-gimbal_yaw.Target_angle[1])<=15000/limit_distance)
	  {location_ms.angle_flag1 = 1;}
	  else
	  {location_ms.angle_flag1 = 0;}
	  if (abs(gimbal_yaw.total_angle-gimbal_yaw.Target_angle[0])<=15000/limit_distance)
	  {location_ms.angle_flag2 = 1;}
	  else
	  {location_ms.angle_flag2 = 0;}
	  if (location_ms.angle_flag1||location_ms.angle_flag2)
	  {location_ms.angle_flag = 1;}
	  else
	  {location_ms.angle_flag = 0;}

      gimbal_yaw.Target_angle[2] = location_ms.angle_E +location_ms.zangle - 360.0f*(int)((location_ms.angle_E+ location_ms.zangle)/360) ;
	  gimbal_yaw.Target_angle[1] = gimbal_yaw.Target_angle[1] +location_ms.zangle - 360.0f*(int)((gimbal_yaw.Target_angle[1]+ location_ms.zangle)/360) ;
	  gimbal_yaw.Target_angle[0] = gimbal_yaw.Target_angle[0] +location_ms.zangle - 360.0f*(int)((gimbal_yaw.Target_angle[0]+ location_ms.zangle)/360) ;	 
}

void target_ton_handle_pro(void)
{
	
   #if  !TARGET_BLANK 	
	//  WIHTE
     switch(target_base)
	 {
		 // CD
		 case 14:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_C -1;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D +1;			 
		 }break; 
		 // BD
		 case 13:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B+1;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D+1;						 
		 }break; 
          // AD		 
		 case 12:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A - 1;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D - 2;				 
		 }break;	
		 //AC
		 case 24:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A +1;
			 gimbal_yaw.Target_angle[1] = location_ms.angle_C +1;		
 		 }break;	
		 //AB
		 case 23:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A - 1; 
			gimbal_yaw.Target_angle[1] = location_ms.angle_B;		   			 
		 }break;	
		 //BC
		 case 34:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B -1;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C +1.5;				 
		 }break;			 
	 }
	 
	 #else
	//  BLANK  
     switch(target_base)
	 {
		 // AB
		 case 14:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A ;
			gimbal_yaw.Target_angle[1] = location_ms.angle_B+1;			 
		 }break;   
		 // DB
		 case 13:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B +1;
			gimbal_yaw.Target_angle[1] = location_ms.angle_D +1;						 
		 }break; 
          // BC		 
		 case 12:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_B - 1;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C ;				 
		 }break;	
		 //AC
		 case 24:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_A +1;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C +1;				 
		 }break;	
		 //DC
		 case 23:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_D -0.5 ;
			gimbal_yaw.Target_angle[1] = location_ms.angle_C - 2;				 
		 }break;	
		 //AD
		 case 34:
		 {
			gimbal_yaw.Target_angle[0] = location_ms.angle_D;
			gimbal_yaw.Target_angle[1] = location_ms.angle_A -0.5;				 
		 }break;
	 }
	 #endif
	  if (abs(gimbal_yaw.total_angle-gimbal_yaw.Target_angle[1])<=15000/limit_distance)
	  {location_ms.angle_flag1 = 1;}
	  else
	  {location_ms.angle_flag1 = 0;}
	  if (abs(gimbal_yaw.total_angle-gimbal_yaw.Target_angle[0])<=15000/limit_distance)
	  {location_ms.angle_flag2 = 1;}
	  else
	  {location_ms.angle_flag2 = 0;}
	  if (location_ms.angle_flag1||location_ms.angle_flag2)
	  {location_ms.angle_flag = 1;}
	  else
	  {location_ms.angle_flag = 0;}

      gimbal_yaw.Target_angle[2] = location_ms.angle_E +location_ms.zangle - 360.0f*(int)((location_ms.angle_E+ location_ms.zangle)/360) ;
	  gimbal_yaw.Target_angle[1] = gimbal_yaw.Target_angle[1] +location_ms.zangle - 360.0f*(int)((gimbal_yaw.Target_angle[1]+ location_ms.zangle)/360) ;
	  gimbal_yaw.Target_angle[0] = gimbal_yaw.Target_angle[0] +location_ms.zangle - 360.0f*(int)((gimbal_yaw.Target_angle[0]+ location_ms.zangle)/360) ;	 
}

extern float fric_speed;
void dis_sp_trans(int flag,float lazer_dis)
{
   if(logic_data.fric_flag==3)
   {
	if(!logic_data.circular_flag)
	{
     switch (flag)
	 {
		// A ton
		 case  2:
		 {
		    fric_data.speed = -7389*sin(0.0001612f*lazer_dis+0.277f)-160;     				
		 }break;  
		// B ton
		 case  3:
		 {
			 if(limit_distance>=3500)
			 {fric_data.speed = -8344*sin(0.0001348f*lazer_dis+0.2511f)-100;}	
			 else
			 {fric_data.speed = -8344*sin(0.0001348f*lazer_dis+0.2511f)-100;}					 
		 }break;
		// C ton
		 case  4:
		 {			 
            fric_data.speed = -9910*sin(0.0001055f*lazer_dis+0.2438f)-180;   			
		 }break; 
		// D ton		 
		 case  1:
		 {
            fric_data.speed = -7014*sin(0.0001902f*lazer_dis+0.25f)-90;		//100	
		 }break; 	
	  }
     }
	else
	{
     switch (flag)
	 {
		 case  2:
		 {
		    fric_data.speed = -7389*sin(0.0001612f*lazer_dis+0.277f)-160;     				
		 }break;  
		
		 case  3:
		 {
			 if(limit_distance>=3500)			 
			 {fric_data.speed = -8344*sin(0.0001348f*lazer_dis+0.2511f)-110;}
			 else
			 {fric_data.speed = -8344*sin(0.0001348f*lazer_dis+0.2511f)-110;}				 
		 }break;  

		 case  4:
		 {
            fric_data.speed = -9910*sin(0.0001055f*lazer_dis+0.2438f)-190;   			
		 }break; 
				 
		 case  1:
		 {
            fric_data.speed = -7014*sin(0.0001902f*lazer_dis+0.25f)-90;			
		 }break; 	
	  }	
	}
 }
   
   #if  !TARGET_BLANK    
   else if(logic_data.fric_flag==2)	
   { fric_data.speed = - 4779*sin(0.00046*location_ms.distance_E+0.173);}
   else
    { fric_data.speed = -5000;} 
	#else   
   else if(logic_data.fric_flag==1)	
   { fric_data.speed  =-4651*sin(0.0004435*location_ms.distance_E+0.1977)+10;}
   else
   {fric_data.speed = -5000;}	   
	#endif  	  
    if (fric_data.speed<=-6600) 
	{fric_data.speed = -6600;} 
}

int lazer_cnt,lazer_cnt1;
void collect_path_handle(void)
{
    if (lazer_data.left_dis <=20.0f)
	{
		lazer_data.cw_flag = 1;
	    lazer_cnt ++;
	}
	else
		lazer_data.cw_flag = 0;
    if (lazer_data.right_dis <=20.0f)	
	{
		lazer_data.ccw_flag = 1;
	   lazer_cnt1 ++; 
	}
	else
	{lazer_data.ccw_flag = 0;}

	if (lazer_cnt >=100)
	{
	   lazer_cnt = 0;
	   send_chassis.path_flag = 2;
	}

	if (lazer_cnt1 >=100)
	{
	   lazer_cnt1= 0; 
	   send_chassis.path_flag = 1; 
	}
}

void location_inverse_handle(uint8_t flag,float distance,float angleM,float angleZ)
{
	static  float Target_angle;
	static  float Target_pox_x;
	static  float Target_pox_y;	
	Target_angle =  (angleM - angleZ)*3.1415926f/180;	
	Target_pox_x = fabs(distance*sin(Target_angle));
	Target_pox_y = fabs(distance*cos(Target_angle));		
    logic_data.deal_angle  = Target_angle;
   switch (flag)
   {
	   case 1 :
	   {
	   	   logic_data.pos_x = -Target_pox_x + 2400;
	   	   logic_data.pos_y = -Target_pox_y + 4800;		   	   	   
	   }break;
	   case 2 :
	   {
	   	   logic_data.pos_x = Target_pox_x - 2400;
	   	   logic_data.pos_y = Target_pox_y ;		   	   	   
	   }break;   
	   case 3 :
	   {
	   	   logic_data.pos_x = Target_pox_x - 2400;
	   	   logic_data.pos_y = -Target_pox_y + 4800;		   
	   }break;      
   }
}

/* 
SICK_MODE :
idea1:
IN
    use memory data under patrol mode, get the probably position,when the 
error is enough small, get the flag into sick_mode; 
OUT
   use bullet_detect function to known whether there is a ball lanuch;
    
PATROL_MODE :


*/