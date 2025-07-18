/*
 * motor.h
 *
 *  Created on: Jul 18, 2025
 *      Author: sato1
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

//motor
void Motor_Initialize();
void Motor_Stop();
void Motor_SetDuty_Left( int16_t duty_l );
void Motor_SetDuty_Right( int16_t duty_r );
void Motor_SetDuty_Left( int16_t duty_l );
void FAN_Motor_Initialize();
void FAN_Motor_SetDuty(int16_t duty_f);
void FAN_Motor_Stop();

#ifdef __cplusplus
}
#endif



#endif /* INC_MOTOR_H_ */
