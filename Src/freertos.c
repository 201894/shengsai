/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "bsp_adc.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "STMGood.h"

#include "sick_task.h"
#include "gimbal_task.h"
#include "imu_handle.h"
#include "main_task.h"
#include "chassis_ms_decode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId DX35TaskHandle;
osThreadId GimbalTaskHandle;
osThreadId MainTaskHandle;
osThreadId InfoHandleTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void sick_task(void const * argument);
void gimbal_task(void const * argument);
void main_task(void const * argument);
void info_handle_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DX35Task */
  osThreadDef(DX35Task, sick_task, osPriorityIdle, 0, 256);
  DX35TaskHandle = osThreadCreate(osThread(DX35Task), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, gimbal_task, osPriorityIdle, 0, 256);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, main_task, osPriorityIdle, 0, 512);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of InfoHandleTask */
  osThreadDef(InfoHandleTask, info_handle_task, osPriorityIdle, 0, 1024);
  InfoHandleTaskHandle = osThreadCreate(osThread(InfoHandleTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
extern  rc_info_t   rc;
extern int bullet_number,lazer_cnt,start_cnt;
extern location_t location_ms;
extern float limit_distance,fric_speed;
extern float Laser_Right,Laser_Left;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {	  
//  printf("location_ms.distance_E = %.3f\r\n",location_ms.distance_E); 	  
//	printf("gimbal_yaw.Target_angle[2] = %.2f\r\n",gimbal_yaw.Target_angle[2]); 
//	printf("pid_fric.errNow = %.3f\r\n",pid_fric.errNow);	  
	#if  0
	printf("path_flag = %d\r\n",send_chassis.path_flag);
	printf("robot_id = %d\r\n",send_chassis.robot_id);	 
	printf("state_flag = %d\r\n",send_chassis.state_flag);		  
	printf("start_flag = %d\r\n",location_ms.start_flag);
	printf("color_flag = %d\r\n",location_ms.color_flag);
	printf("target_flag = %d\r\n",location_ms.target_flag);	  
	printf("lazer_cnt = %d\r\n",lazer_cnt);	 
	printf("fric_data.speed = %d\r\n",fric_data.speed);	
	printf("fric_data.number = %d\r\n",fric_data.number);		  
	printf("gimbal_yaw.Target_angle[2] = %.2f\r\n",gimbal_yaw.Target_angle[2]); 
	printf("gimbal_yaw.Target_angle[1] = %.2f\r\n",gimbal_yaw.Target_angle[1]); 
	printf("gimbal_yaw.Target_angle[0] = %.2f\r\n",gimbal_yaw.Target_angle[0]);
	printf("limit_distance = %.2f\r\n",limit_distance);	  
	#endif	  
	#if 0
	printf("lazer_data.left_dis = %.2f\r\n",Laser_Left);
	printf("moto_fric.speed_rpm = %d\r\n",fric_data.speed);
	printf("pid_fric.errNow = %.3f\r\n",pid_fric.errNow);	
////	
////	printf("CURRENT = %.2f\r\n",pid_yaw_in.ctrOut);	
   #endif
	#if 0
//	printf("LaserBack_Error = %.2f\r\n",lazer_data.dis_error);
	printf("location_ms.distance_A = %.2f\r\n",location_ms.distance_A );
	printf("location_ms.distance_B = %.2f\r\n",location_ms.distance_B );
	printf("location_ms.distance_C = %.2f\r\n",location_ms.distance_C );
	printf("location_ms.distance_D = %.2f\r\n",location_ms.distance_D );
	printf("location_ms.distance_E = %.2f\r\n",location_ms.distance_E );
   #endif  	
	#if 1
//   printf("launch_flag = %d\r\n",logic_data.launch_flag);
//	printf("yaw_out.errNow = %.2f\r\n",pid_yaw_out.errNow);
	printf("ldeal_angle= %.2f\r\n",logic_data.deal_angle);	
	printf("pos_x = %.2f\r\n",(location_ms.pos_x+124*sin(location_ms.zangle*3.1415926f/180)));
	printf("logic_data.pos_x = %.2f\r\n",logic_data.pos_x);	
	printf("pos_y = %.2f\r\n",(location_ms.pos_y-124*cos(location_ms.zangle*3.1415926f/180)));
	printf("logic_data.pos_y = %.2f\r\n",logic_data.pos_y);		  
    printf("calibration_flag = %d\r\n",logic_data.calibration_flag);
	
    printf("circular_flag = %d\r\n",logic_data.circular_flag );	
	printf("start_cnt = %d\r\n",logic_data.start_cnt);	
//	printf("state_flag = %d\r\n",send_chassis.state_flag);
	printf("location_ms.state_flag = %d\r\n",location_ms.state_flag);	
	printf("fric_data.speed = %d\r\n",fric_data.speed);		
	printf("lazer_data.left_dis = %.2f\r\n",Laser_Left);
	printf("lazer_data.right_dis = %.2f\r\n",Laser_Right);    
	printf("limit_distance = %.2f\r\n",limit_distance);
	printf("gimbal_yaw.total_angle = %.2f\r\n",gimbal_yaw.total_angle);		
	printf("location_ms.distance_E = %.2f\r\n",location_ms.distance_E );
	printf("angle_middle= %.2f\r\n",gimbal_yaw.Target_angle[2]); 

   #endif       
	#if 0

	printf("color_flag = %d\r\n",logic_data.color_flag);
	printf("state_flag = %d\r\n",location_ms.state_flag);
	printf("gimbal_yaw.target = %.2f\r\n",gimbal_yaw.target);	
    printf("send_chassis.state_flag = %d\r\n",send_chassis.state_flag);
	printf("launch_flag = %d\r\n",logic_data.launch_flag);
	printf("location_ms.start_flag = %d\r\n",location_ms.start_flag);	
		printf("start_flag = %d\r\n",logic_data.start_flag);
		printf("limit_distance = %.2f\r\n",limit_distance);
			printf("lazer_data.left_dis = %.2f\r\n",Laser_Left);
	printf("lazer_data.right_dis = %.2f\r\n",Laser_Right); 
   #endif     
	#if 0
	printf("moto_yaw.ecd = %d\r\n",moto_yaw.ecd);
	printf("gimbal_yaw.target = %.2f\r\n",gimbal_yaw.target);	  
	printf("gimbal_yaw.total_angle = %.2f\r\n",gimbal_yaw.total_angle);
	printf("yaw_out.errNow = %.2f\r\n",pid_yaw_out.errNow);
	printf("OUTCRTOUT = %.2f\r\n",pid_yaw_out.ctrOut);
	printf("gimbal_yaw.speed_rpm = %.3f\r\n",gimbal_yaw.speed_rpm);
	printf("yaw_in.errNow = %.2f\r\n",pid_yaw_in.errNow);
	printf("CURRENT = %.2f\r\n",pid_yaw_in.ctrOut);
	#endif  	  
	#if 0
	printf("max_error = %.2f\r\n",max_error);	  
	printf("yaw_out.errNow = %.2f\r\n",pid_sick_out.errNow);
	printf("OUTCRTOUT = %.2f\r\n",pid_sick_out.ctrOut);
	printf("gimbal_yaw.speed_rpm = %.3f\r\n",gimbal_yaw.speed_rpm);
	printf("yaw_in.errNow = %.2f\r\n",pid_yaw_in.errNow);
	printf("CURRENT = %.2f\r\n",pid_yaw_in.ctrOut);
	#endif  	
	#if 0
	printf("moto_fric.speed_rpm = %d\r\n",moto_fric.speed_rpm);
	printf("pid_fric.errNow = %.3f\r\n",pid_fric.errNow);
	printf("pid_fric.ctrOut = %.3f\r\n",pid_fric.ctrOut);	  	  
   #endif  	  
	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);	  
	HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);	
    osDelay(200);	  
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_sick_task */
/**
* @brief Function implementing the DX35Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sick_task */
__weak void sick_task(void const * argument)
{
  /* USER CODE BEGIN sick_task */
  /* Infinite loop */
  for(;;)
  {
	  
    osDelay(1);
  }
  /* USER CODE END sick_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_main_task */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_main_task */
__weak void main_task(void const * argument)
{
  /* USER CODE BEGIN main_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END main_task */
}

/* USER CODE BEGIN Header_info_handle_task */
/**
* @brief Function implementing the InfoHandleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_info_handle_task */
__weak void info_handle_task(void const * argument)
{
  /* USER CODE BEGIN info_handle_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END info_handle_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
