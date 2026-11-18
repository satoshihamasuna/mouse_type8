/*
 * interface.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

//uint8_t led_state = 0;
#include <stdio.h>
#include "Core/Inc/main.h"
#include "Component/Inc/typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

void Indicate_LED(uint8_t led_num);
// Logical base state, excluding temporary detection pulses.
uint8_t Return_LED_Status();
void Indicate_LED_Pulse(uint8_t mask, uint16_t duration_ms);
void Update_LED_Pulses(uint16_t elapsed_ms);
t_bool Button_Status();

#ifdef __cplusplus
}
#endif


#endif /* INC_INTERFACE_H_ */
