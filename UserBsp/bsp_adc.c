
/** @file bsp_adc.c
 *  @version 4.0
 *  @date  June 2019
 *

 */
#include "bsp_adc.h"
#include "STMGood.h"
#include "stdio.h"
#include "cmsis_os.h"

uint32_t Get_Adc1(void )
{
    HAL_ADC_Start(&hadc1); 
    HAL_ADC_PollForConversion(&hadc1,3);
	while(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	return (HAL_ADC_GetValue(&hadc1)); 	//+ 40.0
}

uint32_t Get_Adc2(void)
{
//		static uint32_t back = 0.0f;
		HAL_ADC_Start(&hadc2); 
		HAL_ADC_PollForConversion(&hadc2,3);
		while(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
		//{back = HAL_ADC_GetValue(&hadc1)*3.3f/4096.0f * 4715.0f/3.3f;}
		return (HAL_ADC_GetValue(&hadc2)); //
}

uint32_t Get_Adc1_Average(uint8_t times)
{
	uint32_t sum =0.0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		sum+=Get_Adc1();
		osDelay(5);
	}
	return sum/times;
}

uint32_t Get_Adc2_Average(uint8_t times)
{
	uint32_t sum =0.0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		sum+=Get_Adc2();
		osDelay(5);
	}
	return sum/times;
}


