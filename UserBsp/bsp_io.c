
/** @file bsp_io.c
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief basic IO port operation
 *
 *
 */

#include "tim.h"
#include "bsp_io.h"
#include "RMsystem.h"
#include "cmsis_os.h"


void vcc_out_init(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
}

uint16_t beep_delay = 150;
void busser_task(void)
{
   	HAL_Delay(3000);
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	HAL_Delay(beep_delay);	
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	HAL_Delay(beep_delay);
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	HAL_Delay(beep_delay);	
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	HAL_Delay(beep_delay);	
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	HAL_Delay(2*beep_delay);		
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
}


