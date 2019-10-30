/** @file bsp_uart.c
 *  @version 4.0
 *  @date  June 2019
 *
 *  @brief receive uart message and send it to usart2 ,deal with the message in STMGood
 */
#include "bsp_uart.h"
#include "STMGood.h"
#include "bsp_can.h"
#include "usart.h"
#include "stdio.h"
#include "DR16_decode.h"
/* dma double buffer */

uint8_t uart7_buff[50],uart8_buff[50];
/**
  * @brief   initialize uart device 
  */

void uart8_device_init(void)
{
	HAL_DMA_Start_IT(&hdma_uart8_rx,(uint32_t)BT_usart.Instance->DR,(uint32_t)uart8_buff,1);
	BT_usart.Instance->CR3 |= USART_CR3_DMAR;
	__HAL_UART_ENABLE_IT(&BT_usart,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&BT_usart,uart8_buff,1);	
}

void uart7_device_init(void)
{
	HAL_DMA_Start_IT(&hdma_uart7_rx,(uint32_t)PC_usart.Instance->DR,(uint32_t)uart7_buff,20);
	PC_usart.Instance->CR3 |= USART_CR3_DMAR;
	__HAL_UART_ENABLE_IT(&PC_usart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&PC_usart,uart7_buff,20);  
}
/**
 * @brief Error Callback function
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->ErrorCode == HAL_UART_ERROR_ORE)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //清除错误标志位，清空SR、DR寄存器
	}
}

int fputc(int ch, FILE *f)
{
	while((BT_USART->SR&0X40)==0); 
	BT_USART->DR = (uint8_t) ch;      
	return ch;
}
/**
 * @brief uart Interrupt function
 * @param None
 * @return None
 * @attention Replace huart1 interrupt in stm32f4xx_it.c
 */
void UART_RX_IDLE_IRQ(UART_HandleTypeDef *huart){
	

	if(huart->Instance == BT_USART)
	{
			if(__HAL_UART_GET_FLAG(&BT_usart,UART_FLAG_IDLE) != RESET){
			__HAL_UART_CLEAR_IDLEFLAG(&BT_usart);		
			HAL_UART_DMAStop(&BT_usart);
			HAL_UART_Receive_DMA(&BT_usart,uart8_buff,1);
		}
	}
	
	if(huart->Instance == PC_USART)
	{
			if(__HAL_UART_GET_FLAG(&PC_usart,UART_FLAG_IDLE) != RESET){
			__HAL_UART_CLEAR_IDLEFLAG(&PC_usart);		
			HAL_UART_DMAStop(&PC_usart);
			HAL_UART_Receive_DMA(&PC_usart,uart7_buff,20);
		}
	}	
}

