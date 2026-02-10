/*
 * imu.c
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#include "interface.h"
#include "imu.h"
#include "spi.h"
#include "mouse_config.h"
#include "lsm6dsr_reg.h"
#include "lsm6dsrx_reg.h"
#include "lsm6dsv16x_reg.h"

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
#if defined(MOUSE_A)
	IMU_initialize_lsm6dsv16x();
#elif defined(MOUSE_B)
	IMU_initialize_lsm6dsrx();
#endif
}

/*
void IMU_initialize()
{
	HAL_Delay(50);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	read_byte(LSM6DSRX_WHO_AM_I);
	HAL_Delay(50);
	//write_byte(LSM6DSRX_CTRL1_XL, 0x80|0x0E);

	write_byte(LSM6DSRX_CTRL1_XL,
			   LSM6DSRX_XL_ODR_1KHZ |
			   LSM6DSRX_XL_FS_8G |
			   LSM6DSRX_LPF2_XL_EN);

	HAL_Delay(50);
	//write_byte(LSM6DSRX_CTRL8_XL, 0x29);

	write_byte(LSM6DSRX_CTRL8_XL,
			   LSM6DSRX_XL_HPCF_ODR_10 |
			   LSM6DSRX_XL_FAST_SETTLE |
			   LSM6DSRX_XL_LPF_ON_6D);

	HAL_Delay(50);
	//write_byte(LSM6DSRX_CTRL2_G, 0x80|0x01);

	write_byte(LSM6DSRX_CTRL2_G,
			   LSM6DSRX_GY_ODR_1KHZ |
			   LSM6DSRX_GY_FS_4000DPS);

	HAL_Delay(50);
	//write_byte(LSM6DSRX_CTRL6_C,0x00);

	write_byte(LSM6DSRX_CTRL6_C,
			   LSM6DSRX_GY_LPF1_FTYPE_0);
	HAL_Delay(50);

}
*/
void IMU_initialize_lsm6dsrx()
{
	HAL_Delay(5);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_SET);
	HAL_Delay(5);
	read_byte(LSM6DSRX_WHO_AM_I);
	HAL_Delay(5);
	write_byte(LSM6DSRX_CTRL1_XL,
			   LSM6DSRX_XL_ODR_1KHZ |
			   LSM6DSRX_XL_FS_8G |
			   LSM6DSRX_LPF2_XL_EN);

	HAL_Delay(5);
	write_byte(LSM6DSRX_CTRL8_XL,
			   LSM6DSRX_XL_HPCF_ODR_10 |
			   LSM6DSRX_XL_FAST_SETTLE |
			   LSM6DSRX_XL_LPF_ON_6D);

	HAL_Delay(5);
	write_byte(LSM6DSRX_CTRL2_G,
			   LSM6DSRX_GY_ODR_1KHZ |
			   LSM6DSRX_GY_FS_4000DPS);

	HAL_Delay(5);
	write_byte(LSM6DSRX_CTRL6_C,
			   LSM6DSRX_GY_LPF1_FTYPE_0);
	HAL_Delay(5);
}
void IMU_initialize_lsm6dsv16x()
{
	HAL_Delay(5);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,GPIO_PIN_SET);
	HAL_Delay(5);
	read_byte(LSM6DSV16X_WHO_AM_I );							HAL_Delay(5);
	write_byte(LSM6DSV16X_CTRL3_C , LSM6DSV16X_SW_RESET );		HAL_Delay(5);

	write_byte(LSM6DSV16X_CTRL1_XL ,
				LSM6DSV16X_XL_ODR_960HZ |
				LSM6DSV16X_XL_HP_MODE);
	HAL_Delay(5);

	write_byte(LSM6DSV16X_CTRL8_XL,
				LSM6DSV16X_XL_CF_ODR_400  |
				LSM6DSV16X_XL_DualIC_DIS |
				LSM6DSV16X_XL_FS_8G );

	HAL_Delay(5);

	write_byte(LSM6DSV16X_CTRL9_C,
				LSM6DSV16X_XL_HP_REF_MODE_XL_DIS |
				LSM6DSV16X_XL_HP_SLOPE_XL_EN  |
				LSM6DSV16X_XL_FASTSETTL_MODE_DIS |
				LSM6DSV16X_XL_LPF2_XL_DIS );

	HAL_Delay(5);


	write_byte(LSM6DSV16X_CTRL2_G,
				LSM6DSV16X_G_HP_MODE |
				LSM6DSV16X_G_ODR_2KHZ );
	HAL_Delay(5);

	write_byte(LSM6DSV16X_CTRL6_G,
				LSM6DSV16X_GY_LPF1_FTYPE_0 |
				LSM6DSV16X_GY_FS_4000DPS);
	HAL_Delay(5);

	write_byte(LSM6DSV16X_CTRL7_C,
				LSM6DSV16X_GY_LPF1_ENABLE );
	HAL_Delay(5);


}


uint16_t IMU_Check()
{
    float first_value = read_gyro_z_axis();
    uint16_t all_same = 0xff; // すべて同じかどうか（1=同じ, 0=違う）
    HAL_Delay(2);
    // ==== 100回分データ取得 ====
    for (int i = 0; i < 100; i++) {
    	float value = read_gyro_z_axis();

        if (i == 0) first_value = value; // 1回目の値を記録
    	if (value != first_value) all_same = 0; // 違う値が出たらフラグを落とす
    	HAL_Delay(2);
	}

	if( all_same == 0xff ) {
		while( 1 ) {
			Indicate_LED(0x03);
			HAL_Delay(200);
			Indicate_LED(0x00);
			HAL_Delay(200);
		}
	}

    return all_same;
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

