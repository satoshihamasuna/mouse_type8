/*
 * interface.c
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */
#include "Core/Inc/gpio.h"
#include "Peripheral/Inc/interface.h"
#include "Component/Inc/typedef.h"


// Base requests and temporary detection pulses are composed independently.
static uint8_t led_requested_state;
static uint8_t led_pulse_mask;
static uint16_t led_pulse_remaining_ms[6];

static void Write_LED(uint8_t led_num)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (led_num >> 5)&0x01);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (led_num >> 3)&0x01);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (led_num >> 2)&0x01);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (led_num >> 1)&0x01);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (led_num >> 0)&0x01);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, (led_num >> 4)&0x01);
}

void Indicate_LED(uint8_t led_num)
{
	led_requested_state = led_num & 0x3fU;
	Write_LED(led_requested_state | led_pulse_mask);
}

uint8_t Return_LED_Status()
{
	// Read/modify/write callers must not latch a temporary pulse into the base.
	return led_requested_state;
}

void Indicate_LED_Pulse(uint8_t mask, uint16_t duration_ms)
{
	if (duration_ms == 0U) return;
	mask &= 0x3fU;
	for (unsigned i = 0; i < 6; ++i) {
		if (mask & (1U << i)) led_pulse_remaining_ms[i] = duration_ms;
	}
	led_pulse_mask |= mask;
	Write_LED(led_requested_state | led_pulse_mask);
}

void Update_LED_Pulses(uint16_t elapsed_ms)
{
	uint8_t mask = 0;
	for (unsigned i = 0; i < 6; ++i) {
		uint16_t *remaining = &led_pulse_remaining_ms[i];
		*remaining = *remaining > elapsed_ms ? *remaining - elapsed_ms : 0U;
		if (*remaining) mask |= 1U << i;
	}
	if (mask != led_pulse_mask) {
		led_pulse_mask = mask;
		Write_LED(led_requested_state | mask);
	}
}

t_bool Button_Status()
{
	return HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
}
