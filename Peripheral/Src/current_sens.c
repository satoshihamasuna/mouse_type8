/*
 * current_sens.c
 *
 *  Created on: Jul 28, 2025
 *      Author: sato1
 */


#include "main.h"
#include "adc.h"
#include "gpdma.h"
#include "icache.h"
#include "lptim.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "current_sens.h"

#define NUM_ADC4				(4)
#define GET_ADC4_DATA(x)		adc4_value[x-1]

static uint16_t		adc4_value[NUM_ADC4];		// AD変換値

void Fan_Motor_Current_Start(){
	HAL_LPTIM_IC_Start(&hlptim3, LPTIM_CHANNEL_2);
	HAL_ADC_Start_DMA(&hadc4, (uint32_t*) adc4_value, NUM_ADC4);
}

void Fan_Motor_Current_Stop(){
	HAL_LPTIM_IC_Stop(&hlptim3, LPTIM_CHANNEL_2);
	HAL_ADC_Stop_DMA(&hadc4);
}

int16_t ADC4_get_value(int num)
{
	return adc4_value[num];
}

int16_t Fan_Motor_Current_avg()
{
	int16_t adc4_avg = 0;
	for(int i = 0;i < NUM_ADC4;i++)
	{
		adc4_avg += adc4_value[i];
	}
	adc4_avg /= NUM_ADC4;
	return adc4_avg;
}

