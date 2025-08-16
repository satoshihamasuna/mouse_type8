/*
 * interface_check.cpp
 *
 *  Created on: 2023/12/03
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
#define ENABLE (0x01 << 4)

namespace Mode
{
	void Interface_Check()
	{
		t_bool debug_end = False;
		uint8_t mode = Return_LED_Status() & 0x30;
		uint8_t param = 0x00;
		uint8_t enable = 0x00;

		Motion *motion = &(CtrlTask_type7::getInstance());
		IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

		uint32_t time = Interrupt::getInstance().return_time_count();
		float fr,fl,sr,sl;
		int16_t int_fr,int_fl,int_sr,int_sl;
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
					 fr = irsens->sen_fr.distance;	fl = irsens->sen_fl.distance;
					 sr = irsens->sen_r.avg_distance;	sl = irsens->sen_l.avg_distance;
					 int_fr = irsens->sen_r.value_sum;	int_fl = irsens->sen_l.value_sum;
					 int_sr = irsens->sen_r.value;	int_sl = irsens->sen_l.value;
					 printf("fr:%f,fl:%f,sr:%f,sl:%f\n",fr,fl,sr,sl);
					 printf("fr:%4d,fl:%4d,sr:%4d,sl:%4d\n",int_fr,int_fl,int_sr,int_sl);
					 break;
				case ENABLE|0x01:
					 fr = irsens->sen_fr.distance;	fl = irsens->sen_fl.distance;
					 sr = irsens->sen_r.distance;	sl = irsens->sen_l.distance;
					 int_fr = irsens->sen_fr.value;	int_fl = irsens->sen_fl.value;
					 int_sr = irsens->sen_r.value;	int_sl = irsens->sen_l.value;
					 printf("\033[2A"); // カーソルを2行上に戻す（上書きする準備）
					 printf("fr:%f,fl:%f,sr:%f,sl:%f\n", fr, fl, sr, sl);
					 printf("fr:%4d,fl:%4d,sr:%4d,sl:%4d\n", int_fr, int_fl, int_sr, int_sl);
					 break;
				case ENABLE|0x02:
					printf("gyro:%lf\n",(-1.0)*read_gyro_z_axis()*PI/180);
					HAL_Delay(10);
					break;
				case ENABLE|0x03:
						while(1)
						{
							printf("length:%lf,",CtrlTask_type7::getInstance().return_vehicleObj()->ego.length.get());
							printf("angle:%lf,",CtrlTask_type7::getInstance().return_vehicleObj()->ego.radian.get());
							printf("battery:%lf\n",Battery_GetVoltage());
							HAL_Delay(10);
							if(Button_Status() == 1) break;
						}
						  enable = 0x00;
						  HAL_Delay(500);
						break;
				case ENABLE|0x04:
					if(irsens->IrSensor_Avg() > 1500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  motion->Motion_start();
						  motion->Init_Motion_free_rotation_set();
						  while(motion->motion_exeStatus_get() == execute){
								printf("encoder:%ld,%ld\n",Encoder_GetProperty_Right().sp_pulse,Encoder_GetProperty_Left().sp_pulse);
								HAL_Delay(10);
						  }
						  motion->Motion_end();
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x05:
					if(irsens->IrSensor_Avg() > 2500){

								  for(int i = 0;i < 11;i++)
								  {
									  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
									  HAL_Delay(50);
								  }

								  motion->Motion_start();
								  LogData::getInstance().data_count = 0;
								  LogData::getInstance().log_enable = True;
								  motion->exe_Motion_pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
								  LogData::getInstance().log_enable = False;
								  motion->Motion_end();
								  enable = 0x00;
								  HAL_Delay(500);

					}
					break;
				case ENABLE|0x06:
					if(irsens->IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }

							  motion->Motion_start();
							  LogData::getInstance().data_count = 0;
							  LogData::getInstance().log_enable = True;
							  motion->exe_Motion_fix_wall(3000);
							  LogData::getInstance().log_enable = False;
							  motion->Motion_end();
							  enable = 0x00;
							  HAL_Delay(500);
					}
					break;
				case ENABLE|0x07:
					if(irsens->IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  motion->Motion_start();
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->Init_Motion_straight(45.0, 4.0, 0.32f, 0.32f);
						  motion->execute_Motion();
						  motion->Init_Motion_search_turn(&param_R90_search_320 );
						  motion->execute_Motion();
						  motion->Init_Motion_straight(45.0, 4.0, 0.32f, 0.0f);
						  motion->execute_Motion();
						  LogData::getInstance().log_enable = False;
						  motion->Motion_end();
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x08:
					if(irsens->IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  motion->Motion_start();
						  LogData::getInstance().data_count = 0;
						  LogData::getInstance().log_enable = True;
						  motion->Init_Motion_straight(45.0*4, 4.0, 0.32f, 0.0f);
						  motion->execute_Motion();
						  LogData::getInstance().log_enable = False;
						  motion->Motion_end();
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x09:
					if(irsens->IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  float gyro_sum = 0.0f;
						  for(int i = 0; i < 1000;i++)
						  {
							printf("gyro:%lf\n",(-1.0)*read_gyro_z_axis()*PI/180);
							HAL_Delay(2);
							gyro_sum += read_gyro_z_axis();
						  }
						  printf("offset - >%f,%f",gyro_sum/1000.0f,DEG2RAD((-1.0)*gyro_sum/1000.0f));
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
					case ENABLE|0x0A:
					if(irsens->IrSensor_Avg() > 1500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
					      const static t_pid_gain sp_gain = {0.0,0.0,0.0};
					      const static t_pid_gain om_gain = {0.0,0.0,0.0};
						  motion->Motion_start();
						  motion->Init_Motion_suction_start(4.0, 3000, &sp_gain, &om_gain);
						  while(motion->motion_exeStatus_get() == execute){
								printf("current->,");
								for(int i = 0; i < 4 ; i++)
								{
									printf("%ld,",ADC4_get_value(i));
								}
								printf("%ld\n",Fan_Motor_Current_avg());
								HAL_Delay(1);
						  }
						  motion->Motion_end();
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0B:
					if(irsens->IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0C:
					if(irsens->IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0D:
					if(irsens->IrSensor_Avg() > 2500){
						  for(int i = 0;i < 11;i++)
						  {
							  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
							  HAL_Delay(50);
						  }
						  enable = 0x00;
						  HAL_Delay(500);
					}
					break;
				case ENABLE|0x0E:
					if(irsens->IrSensor_Avg() > 2500){
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
					if(irsens->IrSensor_Avg() > 2500){
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






