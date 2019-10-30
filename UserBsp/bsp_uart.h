
/** @file bsp_uart.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief uart receive data from DBus/judge_system/manifold etc.
 *
 *
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
 #include <stdio.h>
/* usart relevant */

#define BT_usart                 huart8 //for debug 
#define PC_usart                 huart7 //for  

#define BT_USART              UART8 //for debug
#define PC_USART              UART7 //for 

void uart7_device_init(void);
void uart8_device_init(void);
void uart_receive_handler(UART_HandleTypeDef *huart);
void 	UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart);
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
void UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart);
void data_ex(void);
extern  uint8_t uart8_buff[50],uart7_buff[50];
#endif
