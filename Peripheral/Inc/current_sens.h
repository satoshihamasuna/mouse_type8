/*
 * current_sens.h
 *
 *  Created on: Jul 28, 2025
 *      Author: sato1
 */

#ifndef INC_CURRENT_SENS_H_
#define INC_CURRENT_SENS_H_

#ifdef __cplusplus
extern "C" {
#endif

void Fan_Motor_Current_Start();
void Fan_Motor_Current_Stop();

int16_t ADC4_get_value(int num);
int16_t Fan_Motor_Current_avg();

#ifdef __cplusplus
}
#endif

#endif /* INC_CURRENT_SENS_H_ */
