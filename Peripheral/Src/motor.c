/*
 * motor.c
 *
 *  Created on: Jul 18, 2025
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
#include "math_utils.h"
#include "motor.h"

#define PCLK1			(50000000)//(HAL_RCC_GetPCLK1Freq())//25,000,000
#define PCLK2			(50000000)//(HAL_RCC_GetPCLK2Freq())//50,000,000
#define PWMFREQ			(100000)
#define FANPWMFREQ		(100000)
#define MOT_DUTY_MIN	(30)
#define MOT_DUTY_MAX	(950)

#define MOT_SET_COMPARE_R_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, x)
#define MOT_SET_COMPARE_R_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, x)
#define MOT_SET_COMPARE_L_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, x)
#define MOT_SET_COMPARE_L_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, x)

void Motor_Initialize()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	MOT_SET_COMPARE_L_FORWARD( 0 );
	MOT_SET_COMPARE_L_REVERSE( 0 );
	MOT_SET_COMPARE_R_FORWARD( 0 );
	MOT_SET_COMPARE_R_REVERSE( 0 );
	HAL_Delay(200);
}

void Motor_Stop(){
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	//FAN_Motor_Stop();
}


void Motor_SetDuty_Left( int16_t duty_l )
{
	uint32_t	pulse_l;

	if( ABS(duty_l) > MOT_DUTY_MAX ) {
		pulse_l = (uint32_t)((PCLK1) / PWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_l) < MOT_DUTY_MIN ) {
		pulse_l = (uint32_t)((PCLK1) / PWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_l = (uint32_t)((PCLK1) / PWMFREQ * ABS(duty_l) / 1000) - 1;
	}

	if( duty_l > 0 ) {
		MOT_SET_COMPARE_L_FORWARD( pulse_l );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	} else if( duty_l < 0 ) {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( pulse_l );
	} else {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	}
}

void Motor_SetDuty_Right( int16_t duty_r )
{
	uint32_t	pulse_r;

	if( ABS(duty_r) > MOT_DUTY_MAX ) {
		pulse_r = (uint32_t)((PCLK1) / PWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_r) < MOT_DUTY_MIN ) {
		pulse_r = (uint32_t)((PCLK1) / PWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_r = (uint32_t)((PCLK1) / PWMFREQ * ABS(duty_r) / 1000) - 1;
	}

	if( duty_r > 0 ) {
		MOT_SET_COMPARE_R_FORWARD( pulse_r );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	} else if( duty_r < 0 ) {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( pulse_r );
	} else {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	}
}


void FAN_Motor_Initialize()
{
	HAL_GPIO_WritePin(fn1_GPIO_Port, fn1_Pin, 1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_Delay(200);
}

void FAN_Motor_Stop(){
	HAL_GPIO_WritePin(fn1_GPIO_Port, fn1_Pin, 0);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
}

void FAN_Motor_SetDuty(int16_t duty_f)
{
	uint32_t	pulse_f;


	if( ABS(duty_f) > MOT_DUTY_MAX ) {
		pulse_f = (uint32_t)((PCLK1) / PWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	}else if(duty_f == 0){
		pulse_f = 0;
	}else if( ABS(duty_f) < MOT_DUTY_MIN ) {
		pulse_f = (uint32_t)((PCLK1) / PWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_f = (uint32_t)((PCLK1) / PWMFREQ * ABS(duty_f) / 1000) - 1;
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_f);
}

