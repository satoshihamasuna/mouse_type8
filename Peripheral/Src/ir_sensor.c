/*
 * ir_sensor.c
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */


#include "Core/Inc/main.h"
#include "Core/Inc/adc.h"
#include "Core/Inc/gpdma.h"
#include "Core/Inc/icache.h"
#include "Core/Inc/lptim.h"
#include "Core/Inc/spi.h"
#include "Core/Inc/tim.h"
#include "Core/Inc/gpio.h"
#include "Peripheral/Inc/ir_sensor.h"
#include "Peripheral/Inc/ir_sensor_activity.h"


#define NUM_ADC				(10)
#define GET_ADC_DATA(x)		adc_value[x-1]

#define	SENSOR_ALL_PATTERN		((IR1_Pin|IR2_Pin|IR3_Pin|IR4_Pin))

static volatile uint32_t		led_on_pattern[NUM_ADC]  	= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};				// LED点灯コマンド

static volatile uint32_t		led_off_pattern[NUM_ADC] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
										 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};

static uint16_t		adc_value[NUM_ADC];		// AD変換値
static volatile uint8_t motion_active;
static volatile uint8_t hand_wait_active;
static volatile uint8_t leds_enabled;
static volatile uint8_t fresh_adc_cycles;

static void Sensor_TurnOffLED()
{
	for( int8_t i = 0; i < NUM_ADC; i++ ) {
		led_on_pattern[i] = 0x00000000;
		led_off_pattern[i] = (uint32_t)SENSOR_ALL_PATTERN << 16;
	}
	__DMB();
	HAL_GPIO_WritePin(GPIOA, SENSOR_ALL_PATTERN, GPIO_PIN_RESET);
}

static void Sensor_TurnOnLED()
{
	Sensor_TurnOffLED();
	led_on_pattern[LED_FR_ON] = IR1_Pin;
	led_on_pattern[LED_SR_ON] = IR2_Pin;
	led_on_pattern[LED_SL_ON] = IR3_Pin;
	led_on_pattern[LED_FL_ON] = IR4_Pin;
}

static void Sensor_UpdateLEDActivity(void)
{
	const uint8_t enabled = (motion_active || hand_wait_active) ? 1U : 0U;
	if (enabled == leds_enabled) return;
	if (enabled) {
		// The first completion can contain a partly unlit DMA scan.
		fresh_adc_cycles = 2U;
		Sensor_TurnOnLED();
	} else {
		Sensor_TurnOffLED();
		fresh_adc_cycles = 0U;
	}
	__DMB();
	leds_enabled = enabled;
}

void Sensor_SetMotionActive(uint8_t active)
{
	const uint32_t irq_mask = __get_PRIMASK();
	__disable_irq();
	const uint8_t was_active = motion_active;
	motion_active = active ? 1U : 0U;
	// 実走行への移行・停止時は待機を消費する。error IRQが停止状態を
	// 繰り返し通知しても、その後に選び直した手かざし待機は解除しない。
	if (motion_active || was_active) hand_wait_active = 0U;
	Sensor_UpdateLEDActivity();
	__set_PRIMASK(irq_mask);
}

void Sensor_SetHandWaitActive(uint8_t active)
{
	const uint32_t irq_mask = __get_PRIMASK();
	__disable_irq();
	hand_wait_active = active ? 1U : 0U;
	Sensor_UpdateLEDActivity();
	__set_PRIMASK(irq_mask);
}

void Sensor_AdcComplete(void)
{
	if (leds_enabled && fresh_adc_cycles > 0U) --fresh_adc_cycles;
}

uint8_t Sensor_IrSamplesReady(void)
{
	return leds_enabled && fresh_adc_cycles == 0U;
}

void Sensor_Initialize()
{

	  motion_active = hand_wait_active = leds_enabled = fresh_adc_cycles = 0U;
	  Sensor_TurnOffLED();
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

	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, NUM_ADC);

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
	Sensor_SetHandWaitActive(0U);
	Sensor_SetMotionActive(0U);
	HAL_LPTIM_IC_Stop(&hlptim1, LPTIM_CHANNEL_1);
	HAL_LPTIM_IC_Stop(&hlptim1, LPTIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOA, SENSOR_ALL_PATTERN, GPIO_PIN_RESET);
	HAL_ADC_Stop_DMA(&hadc1);
}


int16_t ADC_get_value(int num)
{
	return adc_value[num];
}

int16_t Sensor_GetValue(t_sensor_dir dir)
{
	if (!Sensor_IrSamplesReady()) return 0;
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

