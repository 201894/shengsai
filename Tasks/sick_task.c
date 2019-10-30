/** @file sick_task.c
 *  @version 1.0
 *  @date  June 2019
 *
 *  @brief 
 */
 
#include "sick_task.h"
#include "bsp_adc.h"
#include "cmsis_os.h"
#include "imu_handle.h"
uint32_t LaserBack_Back;
uint32_t LaserBack_Front;

void sick_task(void const *argu)
{
   for(;;)
   {
	 LaserBack_Back = Get_Adc1_Average(20);
	 LaserBack_Front = Get_Adc2_Average(20);	    	   
   }
}
 