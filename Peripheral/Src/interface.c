/*
 * interface.c
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */
#include "gpio.h"
#include "interface.h"
#include "typedef.h"


void Indicate_LED(uint8_t led_num)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (led_num >> 5)&0x01);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (led_num >> 3)&0x01);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (led_num >> 2)&0x01);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (led_num >> 1)&0x01);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (led_num >> 0)&0x01);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, (led_num >> 4)&0x01);
}

uint8_t Return_LED_Status()
{
	uint8_t led_state = 0;
	led_state |= (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)&0x01) << 5;
	led_state |= (HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)&0x01) << 3;
	led_state |= (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin)&0x01) << 2;
	led_state |= (HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin)&0x01) << 1;
	led_state |= (HAL_GPIO_ReadPin(LED5_GPIO_Port, LED5_Pin)&0x01) << 0;
	led_state |= (HAL_GPIO_ReadPin(LED6_GPIO_Port, LED6_Pin)&0x01) << 4;
	return led_state;
}

t_bool Button_Status()
{
	return HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
}
