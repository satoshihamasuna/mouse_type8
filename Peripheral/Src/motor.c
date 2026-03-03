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
#include "mouse_config.h"

#define PCLK1			(50000000)//(HAL_RCC_GetPCLK1Freq())//25,000,000
#define PCLK2			(50000000)//(HAL_RCC_GetPCLK2Freq())//50,000,000
#define PWMFREQ			(100000)//(100000)
#define FANPWMFREQ		(100000)
#define MOT_DUTY_MIN	(3) // (DUTY_MIN)
#define MOT_DUTY_MAX	(1000)	   // (980)

#define MOT_R_FORWARD_CH	TIM_CHANNEL_4
#define MOT_R_REVERSE_CH	TIM_CHANNEL_3
#define MOT_L_FORWARD_CH	TIM_CHANNEL_2
#define MOT_L_REVERSE_CH	TIM_CHANNEL_1


#define MOT_SET_COMPARE_R_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, x)
#define MOT_SET_COMPARE_R_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, x)
#define MOT_SET_COMPARE_L_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, x)
#define MOT_SET_COMPARE_L_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, x)

#define MOT_SET_L_FORWARD_START			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2)
#define MOT_SET_L_FORWARD_STOP			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2)
#define MOT_SET_L_REVERSE_START			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1)
#define MOT_SET_L_REVERSE_STOP			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1)


void PWM_ForceLow(TIM_HandleTypeDef *htim, uint32_t channel)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode       = TIM_OCMODE_FORCED_INACTIVE; // 常時LOW
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
}

void PWM_BackToPWM2(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse  = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
}


void Motor_Initialize()
{

	//Motor_Enable();

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	MOT_SET_COMPARE_L_FORWARD(  0 );
	MOT_SET_COMPARE_L_REVERSE(  0 );
	MOT_SET_COMPARE_R_FORWARD(  0 );
	MOT_SET_COMPARE_R_REVERSE(  0 );

	HAL_Delay(200);
}


void Motor_Stop(){

	/*
	MOT_SET_COMPARE_L_FORWARD( TIM4->ARR );
	MOT_SET_COMPARE_L_REVERSE( TIM4->ARR );
	MOT_SET_COMPARE_R_FORWARD( TIM4->ARR );
	MOT_SET_COMPARE_R_REVERSE( TIM4->ARR );
	*/

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	HAL_Delay(200);

}

#if defined(MOUSE_A)
void Motor_SetDuty_Left( int16_t duty_l )
{
	uint32_t	pulse_l;

	// duty_l: -1000 ～ +1000
	float Din = (float)ABS(duty_l) / 1000.0f;  // 0.0 ～ 1.0 に正規化

	// MOT_DUTY_MIN, MOT_DUTY_MAX: 0～1000（例: 100=10%, 900=90%）
	float Dmin = MOT_DUTY_MIN / 1000.0f;       // 0.0～1.0
	float Dmax = MOT_DUTY_MAX / 1000.0f;       // 0.0～1.0

	// 線形マッピング（上限付き）
	float Dout = Dmin + (Dmax - Dmin) * Din;

	// PWMパルス幅計算
	pulse_l = (uint32_t)(uint32_t)(((PCLK1 / PWMFREQ) * Dout) - 1);


	if( duty_l < 0 ) {
		MOT_SET_COMPARE_L_FORWARD( TIM4->ARR - pulse_l );
		MOT_SET_COMPARE_L_REVERSE( TIM4->ARR );

	} else if( duty_l > 0 ) {
		MOT_SET_COMPARE_L_FORWARD( TIM4->ARR  );
		MOT_SET_COMPARE_L_REVERSE( TIM4->ARR - pulse_l );

	} else {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	}
}

void Motor_SetDuty_Right( int16_t duty_r )
{
	uint32_t	pulse_r;

	float Din = (float)ABS(duty_r) / 1000.0f;  // 0.0 ～ 1.0 に正規化

	// MOT_DUTY_MIN, MOT_DUTY_MAX: 0～1000（例: 100=10%, 900=90%）
	float Dmin = MOT_DUTY_MIN / 1000.0f;       // 0.0～1.0
	float Dmax = MOT_DUTY_MAX / 1000.0f;       // 0.0～1.0

	// 線形マッピング（上限付き）
	float Dout = Dmin + (Dmax - Dmin) * Din;

	// PWMパルス幅計算
	pulse_r = (uint32_t)(((PCLK1 / PWMFREQ) * Dout) - 1);


	if( duty_r < 0 ) {
		MOT_SET_COMPARE_R_FORWARD( TIM4->ARR - pulse_r );
		MOT_SET_COMPARE_R_REVERSE( TIM4->ARR );
	} else if( duty_r > 0 ) {
		MOT_SET_COMPARE_R_FORWARD( TIM4->ARR );
		MOT_SET_COMPARE_R_REVERSE( TIM4->ARR - pulse_r );

	} else {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	}
}

#elif defined(MOUSE_B)
void Motor_SetDuty_Left( int16_t duty_l )
{
	uint32_t	pulse_l;

	// duty_l: -1000 ～ +1000
	float Din = (float)ABS(duty_l) / 1000.0f;  // 0.0 ～ 1.0 に正規化

	// MOT_DUTY_MIN, MOT_DUTY_MAX: 0～1000（例: 100=10%, 900=90%）
	float Dmin = MOT_DUTY_MIN / 1000.0f;       // 0.0～1.0
	float Dmax = MOT_DUTY_MAX / 1000.0f;       // 0.0～1.0

	// 線形マッピング（上限付き）
	float Dout = Dmin + (Dmax - Dmin) * Din;

	// PWMパルス幅計算
	pulse_l = (uint32_t)(uint32_t)(((PCLK1 / PWMFREQ) * Dout) - 1);


	if( duty_l > 0 ) {
		MOT_SET_COMPARE_L_FORWARD( pulse_l );
		MOT_SET_COMPARE_L_REVERSE( 0);
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

	float Din = (float)ABS(duty_r) / 1000.0f;  // 0.0 ～ 1.0 に正規化

	// MOT_DUTY_MIN, MOT_DUTY_MAX: 0～1000（例: 100=10%, 900=90%）
	float Dmin = MOT_DUTY_MIN / 1000.0f;       // 0.0～1.0
	float Dmax = MOT_DUTY_MAX / 1000.0f;       // 0.0～1.0

	// 線形マッピング（上限付き）
	float Dout = Dmin + (Dmax - Dmin) * Din;

	// PWMパルス幅計算
	pulse_r = (uint32_t)(((PCLK1 / PWMFREQ) * Dout) - 1);


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
#endif


void FAN_Motor_Initialize()
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	//HAL_GPIO_WritePin(fn1_GPIO_Port, fn1_Pin, 1);
	//HAL_GPIO_WritePin(fn2_GPIO_Port, fn2_Pin, 1);
	HAL_Delay(200);
	//HAL_GPIO_WritePin(fn1_GPIO_Port, fn1_Pin, 0);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 495);
}

void FAN_Motor_Stop(){
	//HAL_GPIO_WritePin(fn1_GPIO_Port, fn1_Pin, 0);
	//HAL_GPIO_WritePin(fn2_GPIO_Port, fn2_Pin, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
}

void FAN_Motor_SetDuty(int16_t duty_f)
{
	uint32_t	pulse_f;


	if( ABS(duty_f) > MOT_DUTY_MAX ) {
		pulse_f = (uint32_t)((PCLK1) / FANPWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	}else if(duty_f == 0){
		pulse_f = (uint32_t)((PCLK1) / FANPWMFREQ * 10/ 1000) - 1;
	}else if( ABS(duty_f) < MOT_DUTY_MIN ) {
		pulse_f = (uint32_t)((PCLK1) / FANPWMFREQ * 10/ 1000) - 1;
	} else {
		pulse_f = (uint32_t)((PCLK1) / FANPWMFREQ * ABS(duty_f) / 1000) - 1;
	}

	//HAL_GPIO_WritePin(fn1_GPIO_Port, fn1_Pin, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_f);

}

