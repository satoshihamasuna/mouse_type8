/*
 * ir_sensor.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_IR_SENSOR_H_
#define INC_IR_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef  enum {
	LED_FL_ON 	= 2,
	LED_FL_OFF 	= 3,
	LED_SL_ON 	= 6,
	LED_SL_OFF 	= 7,
	LED_SR_ON 	= 4,
	LED_SR_OFF 	= 5,
	LED_FR_ON 	= 0,
	LED_FR_OFF 	= 1,
}t_sensor_mode;

typedef enum{
	sensor_fl = 3,
	sensor_fr = 4,
	sensor_sl = 1,
	sensor_sr = 2,
}t_sensor_dir;

void Sensor_Initialize();
#ifdef __cplusplus
}
#endif



#endif /* INC_IR_SENSOR_H_ */
