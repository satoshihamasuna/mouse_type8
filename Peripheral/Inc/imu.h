/*
 * imu.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdio.h>
#include "Core/Inc/main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
	x_axis = 0,
	y_axis = 1,
	z_axis = 2,
}t_axis;

//imu
uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t data);
void IMU_initialize();
void IMU_initialize_lsm6dsrx();
void IMU_initialize_lsm6dsv16x();
uint16_t IMU_Check();
void IMU_read_DMA_Start();
float read_gyro_x_axis();
float read_gyro_y_axis();
float read_gyro_z_axis();
float read_accel_x_axis();
float read_accel_y_axis();
float read_accel_z_axis();


#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */
