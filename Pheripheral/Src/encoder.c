/*
 * encoder.c
 *
 *  Created on: Jul 17, 2025
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
#include "ir_sensor.h"
#include "encoder.h"


//#define ENC_CNT_L 		(TIM2 -> CNT)
//#define ENC_CNT_R 		(TIM3 -> CNT)

#define ENC_ZERO		(20000)
#define ENC_CNT_HALF (500)
#define ENC_CNT_MAX  (1000)
float   m_dt = 1.0;

t_encoder enc_R,enc_L;

void set_ms_dt(float _m_dt)
{
	m_dt = _m_dt;
}

void Encoder_Initialize(){
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	Encoder_ResetPosition_Left();
	Encoder_ResetPosition_Right();
}

void Encoder_ResetPosition_Left(){
	ENC_CNT_L = ENC_ZERO;
}

void Encoder_ResetPosition_Right(){
	ENC_CNT_R = ENC_ZERO;
}

uint32_t Encoder_Counts_Left(){
	return (uint32_t)ENC_CNT_L;
}

uint32_t Encoder_Counts_Right(){
	return (uint32_t)ENC_CNT_R;
}

int32_t Encoder_GetPosition_Right(){
	return (-1)*((int32_t)ENC_CNT_R - (int32_t)ENC_ZERO);
}

int32_t Encoder_GetPosition_Left(){
	return (-1)*((int32_t)ENC_CNT_L - (int32_t)ENC_ZERO);
}

void Encoder_SetSpeed_Right(){
	enc_R.prev_sp_pulse = enc_R.sp_pulse;

	enc_R.sp_pulse = Encoder_GetPosition_Right();

	Encoder_ResetPosition_Right();
	//raw dataの確認．
	enc_R.cnt = (enc_R.cnt == ACC_BUFF_SIZE - 1) ? 0 : enc_R.cnt + 1;
	enc_R.sum = enc_R.sum - enc_R.buff[enc_R.cnt];
	enc_R.buff[enc_R.cnt] = enc_R.sp_pulse;
	enc_R.sum += enc_R.sp_pulse;


	enc_R.prev_wheel_speed = enc_R.wheel_speed;

	//enc_R.wheel_speed =   ((float)(enc_R.sum))/((float)(ACC_BUFF_SIZE))*MMPP * m_dt; //計測はmm mm/ms-> m/s

}

void Encoder_SetSpeed_Left(){
	enc_L.prev_sp_pulse = enc_L.sp_pulse;

	enc_L.sp_pulse = Encoder_GetPosition_Left();

	Encoder_ResetPosition_Left();

	enc_L.cnt = (enc_L.cnt == ACC_BUFF_SIZE - 1) ? 0 : enc_L.cnt + 1;
	enc_L.sum = enc_L.sum - enc_L.buff[enc_L.cnt];
	enc_L.buff[enc_L.cnt] = enc_L.sp_pulse;
	enc_L.sum += enc_L.sp_pulse;


	enc_L.prev_wheel_speed = enc_L.wheel_speed;

	//enc_L.wheel_speed =    ((float)(enc_L.sum))/((float)(ACC_BUFF_SIZE))* MMPP * m_dt;
}

t_encoder Encoder_GetProperty_Right(){
	return enc_R;
}

t_encoder Encoder_GetProperty_Left(){
	return enc_L;
}
