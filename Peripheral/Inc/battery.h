/*
 * Battery.h
 *
 *  Created on: Jul 20, 2025
 *      Author: sato1
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "main.h"

float Battery_GetVoltage();

void Battery_LimiterVoltage();

#ifdef __cplusplus
}
#endif


#endif /* INC_BATTERY_H_ */
