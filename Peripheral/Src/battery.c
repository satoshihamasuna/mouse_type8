/*
 * battery.c
 *
 *  Created on: Jul 20, 2025
 *      Author: sato1
 */

#define BATTRY_REFERENCE	(3.25f)
#define BATTERY_LIMIT		(3.5f)
#define BATTERY_CELL_COUNT	(2)
#define BATTERY_SAMPLE_COUNT	(10)

#include "Peripheral/Inc/ir_sensor.h"
#include "Peripheral/Inc/battery.h"
#include "Peripheral/Inc/interface.h"

float Battery_GetVoltage(void){
	return (BATTRY_REFERENCE * (143.0f+10.0f)/(10.0f) * (float)Sensor_GetBatteryValue())/(4096.f);
}

float Battery_GetAverageVoltage(void)
{
	float battery_voltage_average = 0.0f;

	for(int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
		HAL_Delay(5);
		battery_voltage_average += Battery_GetVoltage();
	}
	return battery_voltage_average / BATTERY_SAMPLE_COUNT;
}

float Battery_GetLimitVoltage(void)
{
	return BATTERY_LIMIT * BATTERY_CELL_COUNT;
}

int Battery_IsVoltageError(float voltage)
{
	return voltage < Battery_GetLimitVoltage();
}

void Battery_LimiterVoltage(void)
{
	if(Battery_IsVoltageError(Battery_GetAverageVoltage())) {
		while( 1 ) {
			Indicate_LED(0x01);
			HAL_Delay(200);
			Indicate_LED(0x00);
			HAL_Delay(200);
		}
	}
}


