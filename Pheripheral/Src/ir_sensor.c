/*
 * ir_sensor.c
 *
 *  Created on: Jul 4, 2025
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
#include <ir_sensor.h>


#define NUM_ADC				(10)
#define GET_ADC_DATA(x)		adc_value[x-1]

#define	SENSOR_ALL_PATTERN		((IR1_Pin|IR2_Pin|IR3_Pin|IR4_Pin))

static uint32_t		led_on_pattern[NUM_ADC]  	= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};				// LED点灯コマンド

static uint32_t		led_off_pattern[NUM_ADC] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
										 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};

//static uint16_t		adc_value[NUM_ADC];		// AD変換値

void Sensor_TurnOffLED()
{
	for( int8_t i = 0; i < NUM_ADC; i++ ) {
		led_on_pattern[i] = 0x00000000;
		led_off_pattern[i] = (uint32_t)SENSOR_ALL_PATTERN << 16;
	}
}

void Sensor_TurnOnLED()
{
	Sensor_TurnOffLED();
	led_on_pattern[LED_SL_ON] = IR1_Pin;
	led_on_pattern[LED_SR_ON] = IR2_Pin;
	led_on_pattern[LED_FL_ON] = IR3_Pin;
	led_on_pattern[LED_FR_ON] = IR4_Pin;
}

void Sensor_Initialize()
{

	  Sensor_TurnOnLED();
	  DMA_NodeConfTypeDef nodeConf1,nodeConf2;
	  HAL_DMAEx_List_GetNodeConfig(&nodeConf1, &Node_GPDMA1_Channel0);
	  HAL_DMAEx_List_GetNodeConfig(&nodeConf2, &Node_GPDMA1_Channel1);
	  // 書き換えたい部分だけ変更
	  nodeConf1.SrcAddress = (uint32_t)led_on_pattern;
	  nodeConf1.DstAddress = (uint32_t)(&(GPIOA->BSRR));
	  nodeConf1.DataSize   = 4*NUM_ADC;

	  nodeConf2.SrcAddress = (uint32_t)led_off_pattern;
	  nodeConf2.DstAddress = (uint32_t)(&(GPIOA->BSRR));
	  nodeConf2.DataSize   = 4*NUM_ADC;

	  // ノードを再構築（再設定）
	  if (HAL_DMAEx_List_BuildNode(&nodeConf1, &Node_GPDMA1_Channel0) != HAL_OK)
	      Error_Handler();
	  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel0, NULL, &Node_GPDMA1_Channel0) != HAL_OK)
	      Error_Handler();
	  //Circularモードに
	  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel0) != HAL_OK)
	      Error_Handler();


	  if (HAL_DMAEx_List_BuildNode(&nodeConf2, &Node_GPDMA1_Channel1) != HAL_OK)
	      Error_Handler();
	  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel1, NULL, &Node_GPDMA1_Channel1) != HAL_OK)
	        Error_Handler();
	  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel1) != HAL_OK)
	        Error_Handler();


	  if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel0) != HAL_OK)
	  {
		Error_Handler();
	  }

	  if (HAL_DMAEx_List_Start_IT(&handle_GPDMA1_Channel1) != HAL_OK)
	  {
	  	Error_Handler();
	  }

	  if (HAL_LPTIM_IC_Start(&hlptim1, LPTIM_CHANNEL_1) != HAL_OK)
	  {
	      Error_Handler();
	  }

	  if (HAL_LPTIM_IC_Start(&hlptim1, LPTIM_CHANNEL_2) != HAL_OK)
	  {
	      Error_Handler();
	  }

}


void Sensor_StopADC()
{
	HAL_LPTIM_IC_Stop(&hlptim1, LPTIM_CHANNEL_1);
	HAL_LPTIM_IC_Stop(&hlptim1, LPTIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOA, SENSOR_ALL_PATTERN, GPIO_PIN_RESET);
}
/*
int16_t ADC_get_value(int num)
{
	return adc_value[num];
}

int16_t Sensor_GetValue(t_sensor_dir dir)
{
	switch(dir)
	{
		case sensor_fl:
			return ((int16_t)adc_value[LED_FL_ON] - (int16_t)adc_value[LED_FL_OFF]);
			break;
		case sensor_fr:
			return ((int16_t)adc_value[LED_FR_ON] - (int16_t)adc_value[LED_FR_OFF]);
			break;
		case sensor_sr:
			return ((int16_t)adc_value[LED_SR_ON] - (int16_t)adc_value[LED_SR_OFF]);
			break;
		case sensor_sl:
			return ((int16_t)adc_value[LED_SL_ON] - (int16_t)adc_value[LED_SL_OFF]);
			break;
	}
	return 0;
}

int16_t Sensor_GetBatteryValue(){
	return (adc_value[8] + adc_value[9])/2 ;
}
*/
