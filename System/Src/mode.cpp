/*
 * mode.cpp
 *
 *  Created on: 2023/07/22
 *      Author: sato1
 */

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
#include "myshell.h"

#define ENABLE (0x01 << 4)

namespace
{
	bool Shell_Command_Received()
	{
		static const char command[] = "shell";
		static uint8_t command_length = 0;
		static bool invalid_line = false;
		uint8_t ch;
		bool received = false;

		while(Communicate_RxTryPopData(&ch) != 0U)
		{
			if(ch == '\r' || ch == '\n')
			{
				if(!invalid_line && command_length == sizeof(command) - 1U)
				{
					received = true;
				}
				command_length = 0;
				invalid_line = false;
				continue;
			}

			if(ch == '\b' || ch == 0x7fU)
			{
				if(command_length > 0U)
				{
					command_length--;
				}
				invalid_line = false;
				continue;
			}

			if(invalid_line || command_length >= sizeof(command) - 1U ||
				ch != static_cast<uint8_t>(command[command_length]))
			{
				invalid_line = true;
				continue;
			}

			command_length++;
		}

		return received;
	}
}

namespace Mode
{

	void indicate_error()
	{
		while(Button_Status() != True)
		{
			Indicate_LED(0xff);
			HAL_Delay(80);
			Indicate_LED(0x00);
			HAL_Delay(80);
		}
	}

	uint8_t Select(uint8_t _param,uint8_t max,t_encoder enc)
	{
		uint8_t param =  _param;
		if(enc.wheel_speed > 0.2){
			param = (param == max) ? 0 : param + 1 ;
			HAL_Delay(100);
		}
		else if(enc.wheel_speed  < -0.2){
			param = (param == 0) ? max : param - 1 ;
			HAL_Delay(100);
		}
		return param;
	}


	void Select_Mode()
	{
		uint8_t mode = 0;
		uint8_t enable = 0;
		Communicate_RxStartContinuous();

		while(1)
		{
			enable 	= Mode::Select(enable,0x01,Encoder_GetProperty_Left() );
			mode = (enable == 0x00) ? Mode::Select(mode  ,0x03,Encoder_GetProperty_Right()):mode;
			if(Shell_Command_Received())
			{
				mode = 0x03;
				enable = 0x01;
			}
			Battery_LimiterVoltage();
			Indicate_LED(mode << 4);
			switch((enable << 4)|mode)
			{
				case ENABLE|0x00:
					Communicate_RxStopContinuous();
					Mode::Interface_Check();
					Communicate_RxStartContinuous();
					enable = 0;
					break;
				case ENABLE|0x01:
					Communicate_RxStopContinuous();
					Mode::Demo();
					Mode::Demo2();
					Communicate_RxStartContinuous();
					enable = 0;
					break;
				case ENABLE|0x02:
					Communicate_RxStopContinuous();
					Mode::Debug(&st_param_500,mode_500,0);
					Mode::Debug(&st_param_700,mode_700,200);
					//Mode::Debug(&st_param_1200,mode_1200,400);
				    //Mode::Debug(&st_param_1400_acc2G,mode_1400,500);
					//Mode::Debug(&st_param_1500_acc2G,mode_1500,500);
					Mode::Debug(&st_param_1600_acc2G,mode_1600,650);
					Mode::Debug(&st_param_1800_acc3G,mode_1800,650);
					Mode::Debug(&st_param_2000_acc3G,mode_2000,700);
					Communicate_RxStartContinuous();
					enable = 0;
					break;
				case ENABLE|0x03:
					Myshell_Initialize();
					Myshell_Execute();
					enable = 0;
					break;
				default:
					break;
			}
		}
	}
}

