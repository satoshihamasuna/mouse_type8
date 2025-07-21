/*
 * battery.c
 *
 *  Created on: Jul 20, 2025
 *      Author: sato1
 */

#define BATTRY_REFERENCE	(3.25f)
#define BATTERY_LIMIT		(3.0f)

#include "ir_sensor.h"
#include "battery.h"
#include "interface.h"

float Battery_GetVoltage(){
	return (BATTRY_REFERENCE * (140.0f+10.0f)/(10.0f) * (float)Sensor_GetBatteryValue())/4096.f;
}

void Battery_LimiterVoltage()
{
	volatile int	i;
	volatile float	battery_voltage_average;

	for( i = 0; i < 10; i++) {
		HAL_Delay(5);
		battery_voltage_average += Battery_GetVoltage();
	}
	battery_voltage_average /= 10;

	if( battery_voltage_average < BATTERY_LIMIT ) {
		while( 1 ) {
			Indicate_LED(0x01);
			HAL_Delay(200);
			Indicate_LED(0x00);
			HAL_Delay(200);
		}
	} else;
}


