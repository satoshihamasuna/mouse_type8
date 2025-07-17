/*
 * encoder.h
 *
 *  Created on: Jul 17, 2025
 *      Author: sato1
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif


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


#define ENC_CNT_L 		(TIM2 -> CNT)
#define ENC_CNT_R 		(TIM3 -> CNT)

#define ACC_BUFF_SIZE 30

typedef struct{
	int32_t sp_pulse;
	int32_t prev_sp_pulse;
	float 	wheel_speed;
	float	prev_wheel_speed;
	int32_t buff[ACC_BUFF_SIZE];
	int32_t sum;
	int cnt;
}t_encoder;

//encoder
void Encoder_Initialize();
void Encoder_ResetPosition_Left();
void Encoder_ResetPosition_Right();
uint32_t Encoder_Counts_Left();
uint32_t Encoder_Counts_Right();
int32_t Encoder_GetPosition_Right();
int32_t Encoder_GetPosition_Left();
void Encoder_SetSpeed_Right();
void Encoder_SetSpeed_Left();
t_encoder Encoder_GetProperty_Right();
t_encoder Encoder_GetProperty_Left();


#ifdef __cplusplus
}
#endif


#endif /* INC_ENCODER_H_ */
