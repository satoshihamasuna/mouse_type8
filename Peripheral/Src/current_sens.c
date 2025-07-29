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

#define NUM_ADC				(4)
#define GET_ADC_DATA(x)		adc_value[x-1]

static uint16_t		adc_value[NUM_ADC];		// AD変換値

void Fan_Motor_Current_Start(){
	HAL_LPTIM_IC_Start(&hlptim3, LPTIM_CHANNEL_2);
	HAL_ADC_Start_DMA(&hadc4, (uint32_t*) adc_value, NUM_ADC);
}

void Fan_Motor_Current_Stop(){
	HAL_LPTIM_IC_Stop(&hlptim3, LPTIM_CHANNEL_2);
	HAL_ADC_Stop_DMA(&hadc4);
}


