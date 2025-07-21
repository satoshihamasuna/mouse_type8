/*
 * imu.c
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#include "imu.h"
#include "spi.h"
#include "lsm6dsr_reg.h"

uint8_t imu_address = OUTX_L_G|0x80; //ACCEL_X_HIGH_BYTE
uint8_t imu_value[13];

int16_t accel_data[3];
int16_t gyro_data[3];

uint8_t read_byte(uint8_t reg){
	reg = reg | 0x80; //mask

	uint8_t value_imu[2] 	= {0x00,0x00};
	uint8_t register_imu[2] = {reg,0x00};

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, register_imu, value_imu, 2, 100);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_SET);

	return value_imu[1];
}

void write_byte(uint8_t reg, uint8_t data){
	reg = reg & 0x7F;
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg, 1 , 100);
	HAL_SPI_Transmit(&hspi2, &data, 1 , 100);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_SET);
}

void IMU_initialize()
{
	  HAL_Delay(50);
	  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_SET);
	  HAL_Delay(50);
	  read_byte(WHO_AM_I);
	  HAL_Delay(50);
	  write_byte(CTRL1_XL, ACCEL_ODR_SET|ACCEL_8G);
	  HAL_Delay(50);
	  write_byte(0x17, 0x29);
	  HAL_Delay(50);
	  write_byte(CTRL2_G, GYRO_ODR_SET|GYRO_4000_DPS);
	  HAL_Delay(50);
	  write_byte(0x15,0x00);
	  HAL_Delay(50);

}


void IMU_read_DMA_Start(){
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(&hspi2, &imu_address, imu_value, sizeof(imu_value)/sizeof(uint8_t));
}

void IMU_read_DMA_Stop(){
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_DMAStop(&hspi2);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
	    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_SET);

		gyro_data[x_axis] = (((int16_t)imu_value[2]<<8 ) | ( (int16_t)imu_value[1]&0x00ff ) );
		gyro_data[y_axis] = (((int16_t)imu_value[4]<<8 ) | ( (int16_t)imu_value[3]&0x00ff ) );
		gyro_data[z_axis] = (((int16_t)imu_value[6]<<8 ) | ( (int16_t)imu_value[5]&0x00ff ) );
		accel_data[x_axis] = (((int16_t)imu_value[8]<<8 ) | ( (int16_t)imu_value[7]&0x00ff ) );
		accel_data[y_axis] = (((int16_t)imu_value[10]<<8 ) | ( (int16_t)imu_value[9]&0x00ff ) );
		accel_data[z_axis] = (((int16_t)imu_value[12]<<8 ) | ( (int16_t)imu_value[11]&0x00ff ) );

		//IMU_read_DMA_Start();
}

float read_gyro_x_axis(){
	return  (float)gyro_data[x_axis]*(1.0f) *140.0f/1000.0f;
}

float read_gyro_y_axis(){
	return  (float)gyro_data[y_axis]*(1.0f) *140.0f/1000.0f;
}

float read_gyro_z_axis(){
	return  (float)gyro_data[z_axis]*(1.0f) *140.0f/1000.0f;
}

float read_accel_x_axis(){
	return  (float)accel_data[x_axis]*0.244/1000.0f*9.8;
}

float read_accel_y_axis(){
	return  (float)accel_data[y_axis]*0.244/1000.0f*9.8;
}

float read_accel_z_axis(){
	return  (float)accel_data[z_axis]*0.244/1000.0f*9.8;
}

