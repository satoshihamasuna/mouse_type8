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


#include "Core/Inc/main.h"
#include "Core/Inc/adc.h"
#include "Core/Inc/gpdma.h"
#include "Core/Inc/icache.h"
#include "Core/Inc/lptim.h"
//#include "memorymap.h"
#include "Core/Inc/spi.h"
#include "Core/Inc/tim.h"
#include "Core/Inc/gpio.h"
#include "Peripheral/Inc/ir_sensor.h"
#include "Peripheral/Inc/encoder.h"
#include "Component/Inc/math_utils.h"

#define ENC_CNT_L 		(TIM2 -> CNT)
#define ENC_CNT_R 		(TIM3 -> CNT)


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
