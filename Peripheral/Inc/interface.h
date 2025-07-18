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
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Indicate_LED(uint8_t led_num);
uint8_t Return_LED_Status();
GPIO_PinState Button_Status();

#ifdef __cplusplus
}
#endif


#endif /* INC_INTERFACE_H_ */
