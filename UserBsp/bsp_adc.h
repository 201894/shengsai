/** @file bsp_adc.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief uart receive data from DBus/judge_system/manifold etc.
 *
 *
 */

#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include "adc.h"
#include <stdio.h>
 
uint32_t Get_Adc1(void );
uint32_t Get_Adc1_Average(uint8_t times);
uint32_t Get_Adc2(void );
uint32_t Get_Adc2_Average(uint8_t times); 
 
 #endif