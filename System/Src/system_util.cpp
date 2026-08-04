/*
 * main.cpp
 *
 *  Created on: Jul 19, 2025
 *      Author: sato1
 */

#include "System/Inc/mode.h"
#include "System/Inc/system_util.h"
#include "Module/Inc/interrupt.h"
#include "Peripheral/Inc/peripheral.h"

void Module_Initialize()
{


	  IMU_initialize();
	  Sensor_Initialize();
	  Communicate_Initialize();
	  Encoder_Initialize();
	  Motor_Initialize();
	  FAN_Motor_Initialize();
	  Interrupt_Initialize();
	  IMU_read_DMA_Start();
	  Motor_Stop();
	  //Fan_Motor_Current_Start();
	  IMU_Check();
}

void CPP_Main()
{
	  Module_Initialize();
	  uint8_t setup = 0x01;
	  for (int i = 0;i < 8; i++)
	  {
		  Indicate_LED(setup << i);
		  HAL_Delay(50);
	  }
	  Mode::Select_Mode();
}


