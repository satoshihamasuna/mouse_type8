/*
 * debug.cpp
 *
 *  Created on: 2023/12/03
 *      Author: sato1
 */



#include<stdio.h>

#include "peripheral.h"
#include "communicate.h"

#include "../../Subsys/Inc/search_class.h"
#include "../../Subsys/Inc/make_map_class.h"
#include "../../Subsys/Inc/wall_class.h"
#include "../../Subsys/Inc/make_path.h"

#include "../../Component/Inc/controller.h"
#include "../../Component/Inc/queue.h"
#include "../../Component/Inc/priority_queue.h"
#include "typedef.h"
#include "math_utils.h"

#include "../../Task/Inc/sensing_task.h"
#include "../../Task/Inc/ctrl_task.h"

#include "../../Module/Inc/interrupt.h"
#include "../../Module/Inc/log_data.h"
#include "../../Module/Inc/flash.h"

#include "../../Params/run_param.h"

#include "../Inc/mode.h"

#define ENABLE (0x01 << 4)

namespace Mode
{
	void Debug3()
	{
	t_bool debug_end = False;
	uint8_t mode = Return_LED_Status() & 0x30;
	uint8_t param = 0x00;
	uint8_t enable = 0x00;

	IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

	uint32_t time = Interrupt::getInstance().return_time_count();

	while(debug_end == False)
	{
		enable = Mode::Select(enable,0x01,Encoder_GetProperty_Left());
		param = (enable == 0x00) ? Mode::Select(param,0x0f,Encoder_GetProperty_Right()) : param;

		Battery_LimiterVoltage();
		if(enable == 0x01)
		{
			if((Interrupt::getInstance().return_time_count() - time) > 400)
			{
				time = Interrupt::getInstance().return_time_count();
				Indicate_LED((Return_LED_Status() != (mode|param)) ?  mode|param : 0x00);
			}
		}
		else
		{
			Indicate_LED(mode|param);
		}
		switch((enable<<4)|param)
		{
			case ENABLE|0x00:
		  	   if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
					  enable = 0x00;
					  HAL_Delay(500);
		  	    }
		  	    break;
			case ENABLE|0x01:
		  	   if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
		  	    }
				break;
			case ENABLE|0x02:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
			  		  enable = 0x00;
			  		  HAL_Delay(500);
				}
				break;
			case ENABLE|0x03:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
					  enable = 0x00;
					  HAL_Delay(500);
				}
				break;
			case ENABLE|0x04:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
			case ENABLE|0x05:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
			case ENABLE|0x06:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
			case ENABLE|0x07:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
			case ENABLE|0x08:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
			case ENABLE|0x09:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
			case ENABLE|0x0A:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
				case ENABLE|0x0B:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
				}
				break;
			case ENABLE|0x0C:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }
					  HAL_Delay(500);
				}
				break;
			case ENABLE|0x0D:
				if(irsens->IrSensor_Avg() > 2000){
					  for(int i = 0;i < 11;i++)
					  {
						  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						  HAL_Delay(50);
					  }

				}
				break;
			case ENABLE|0x0E:
				if(irsens->IrSensor_Avg() > 2000){
					for(int i = 0;i < 11;i++)
				    {
				    	(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
				    	HAL_Delay(50);
					}
					LogData::getInstance().indicate_data();
					enable = 0x00;
				}
				break;
			case ENABLE|0x0F:
				if(irsens->IrSensor_Avg() > 2000){
					for(int i = 0;i < 11;i++)
					{
					  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
					  HAL_Delay(50);
					}
					debug_end = True;
				}
				break;
			default:
				break;
			}
		}

	}
}

