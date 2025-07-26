/*
 * demo.cpp
 *
 *  Created on: 2024/07/16
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

void search_error_process(int init_histry_cnt, wall_class *wall_data)
{
	if(wall_data->wall_histry.get_histry_cnt() > init_histry_cnt + 10)
	{
		wall_data->wall_histry.histry_delete(0x0A);
		write_histry_flash(&(wall_data->wall_histry));
	}
	else if(wall_data->wall_histry.get_histry_cnt() - init_histry_cnt > 0)
	{
		int delete_num = wall_data->wall_histry.get_histry_cnt() - init_histry_cnt;
		wall_data->wall_histry.histry_delete(delete_num);
		write_histry_flash(&(wall_data->wall_histry));
	}
	Mode::indicate_error();
}

namespace Mode
{
void Demo()
{
	t_bool demo_end = False;
	uint8_t mode = Return_LED_Status() & 0x30;
	uint8_t param = 0x00;
	uint8_t enable = 0x00;
	uint32_t time = Interrupt::getInstance().return_time_count();


	Motion *motion = &(CtrlTask_type7::getInstance());
	IrSensTask *irsens = (CtrlTask_type7::getInstance().return_irObj());

	Search solve_maze;
	solve_maze.set_search_limit_time((6*60*1000));

	wall_class wall_data(irsens);
	wall_data.init_maze();
	wall_data.wall_histry.histry_init();
	write_wall_WorkRam(&wall_data);
	write_histry_WorkRam(&(wall_data.wall_histry));
	int init_histry_cnt = 0;


	ring_queue<1024,t_MapNode> maze_q;
	make_map map_data(&wall_data,&maze_q);
	Dijkstra run_path(&wall_data);


	t_position start,goal;
	start.x = start.y = 0;start.dir = North;
	goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
	uint8_t goal_size = MAZE_GOAL_SIZE;



	while(demo_end == False)
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

					Indicate_LED(mode|param);

					//outward
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = solve_maze.search_adachi_1_acc(start, goal, goal_size, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					motion->Motion_end();
					write_save_data(&wall_data);

					//return
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.search_adachi_2_acc(return_pos, start, 1, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);
					motion->Motion_end();
					enable = 0x00;
				}
				break;
			case ENABLE|0x01:
				if(irsens->IrSensor_Avg() > 2000){
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}
					Indicate_LED(mode|param);

					//outward
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = solve_maze.search_adachi_1(start, goal, goal_size, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);

					//return
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.search_adachi_1(return_pos, start, 1, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);
					enable = 0x00;
				}
				break;
			case ENABLE|0x02:
				if(irsens->IrSensor_Avg() > 2000){
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}

					Indicate_LED(mode|param);

					//outward
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = solve_maze.search_adachi_3_acc(start, goal, goal_size, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);

					//return
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.reset_search_time();
					solve_maze.search_adachi_2_acc(return_pos, start, 1, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);
					enable = 0x00;
				}
				break;
			case ENABLE|0x03:
				if(irsens->IrSensor_Avg() > 2000){
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}

					Indicate_LED(mode|param);

					//outward
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = solve_maze.search_adachi_3_acc(start, goal, goal_size, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);

					//return
					init_histry_cnt =  wall_data.wall_histry.get_histry_cnt();
					solve_maze.search_adachi_3_acc(return_pos, start, 1, &wall_data, &map_data,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_histry_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);
					enable = 0x00;
				}
				break;
			case ENABLE|0x04:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}
					Indicate_LED(mode|param);

					run_path.turn_time_set(mode_1000);
					t_bool flag = False;
					flag = run_path.check_DijkstraPath(start, Dir_None, goal, MAZE_GOAL_SIZE);
					if(flag == True)
					{
						for(int j = 0;j < 2;j++)
						{
						  uint8_t setup = 0x01;
						  for (int i = 0;i < 8; i++)
						  {
							  Indicate_LED(setup << i);
							  HAL_Delay(50);
						  }
						}
					}
					else
					{
						Mode::indicate_error();
					}
					Indicate_LED(mode|param);
					enable = 0x00;
			   }
				break;
			case ENABLE|0x05:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}

			  		run_path.turn_time_set(mode_1000);
					run_path.run_Dijkstra(		start, Dir_None, goal,MAZE_GOAL_SIZE,
														st_mode_500_v0, (int)(sizeof(st_mode_500_v0)/sizeof(t_straight_param *const)),
														di_mode_500_v0, (int)(sizeof(di_mode_500_v0)/sizeof(t_straight_param *const)), mode_500,motion);


					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}
				break;
			case ENABLE|0x06:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}
					int32_t time = Mode::Seach_Time_Select();
					solve_maze.set_search_limit_time(time);
					enable = 0x00;
				}
				break;
			case ENABLE|0x07:
				if(irsens->IrSensor_Avg() > 2000)
				{
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}
					Mode::Wall_Histry_Check(&wall_data);
					enable = 0x00;
				}
				break;
			case ENABLE|0x08:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}

			  		run_path.turn_time_set(mode_1000);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal,MAZE_GOAL_SIZE,600,
														st_mode_1000_v0, (int)(sizeof(st_mode_1000_v0)/sizeof(t_straight_param *const)),
														di_mode_1000_v0, (int)(sizeof(di_mode_1000_v0)/sizeof(t_straight_param *const)), mode_1000,motion);

					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}
				break;
			case ENABLE|0x09:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}

			  		run_path.turn_time_set(mode_1600);
					run_path.run_Dijkstra_suction_acc(	start, Dir_None, goal, MAZE_GOAL_SIZE,800,
														st_mode_1600_v3, (int)(sizeof(st_mode_1600_v3)/sizeof(t_straight_param *const))	,
														di_mode_1600_v2, (int)(sizeof(di_mode_1600_v2)/sizeof(t_straight_param *const))	,
														acc_mode_1600_v1,(int)(sizeof(acc_mode_1600_v1)/sizeof(t_param *const*const))	,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}
				break;
			case ENABLE|0x0A:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}

			  		run_path.turn_time_set(mode_1200);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,650,
														st_mode_1200_v0, (int)(sizeof(st_mode_1200_v0)/sizeof(t_straight_param *const)),
														di_mode_1200_v0, (int)(sizeof(di_mode_1200_v0)/sizeof(t_straight_param *const)), mode_1200,motion);

					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}
				break;
			case ENABLE|0x0B:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}


			  		run_path.turn_time_set(mode_1600);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,800,
														st_mode_1600_v1, (int)(sizeof(st_mode_1600_v1)/sizeof(t_straight_param *const)),
														di_mode_1600_v1, (int)(sizeof(di_mode_1600_v1)/sizeof(t_straight_param *const)), mode_1600,motion);

					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}
				break;
			case ENABLE|0x0C:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}


			  		run_path.turn_time_set(mode_1600);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,800,
														st_mode_1600_v2, (int)(sizeof(st_mode_1600_v2)/sizeof(t_straight_param *const)),
														di_mode_1600_v1, (int)(sizeof(di_mode_1600_v1)/sizeof(t_straight_param *const)), mode_1600,motion);

					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}
				break;
			case ENABLE|0x0D:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}


			  		run_path.turn_time_set(mode_1600);
					run_path.run_Dijkstra_suction_acc(	start, Dir_None, goal, MAZE_GOAL_SIZE,800,
														st_mode_1600_v2, (int)(sizeof(st_mode_1600_v2)/sizeof(t_straight_param *const))	,
														di_mode_1600_v1, (int)(sizeof(di_mode_1600_v1)/sizeof(t_straight_param *const))	,
														acc_mode_1600_v2,(int)(sizeof(acc_mode_1600_v2)/sizeof(t_param *const*const))	,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}
				break;
			case ENABLE|0x0E:
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}


			  		run_path.turn_time_set(mode_1600);
					run_path.run_Dijkstra_suction_acc(	start, Dir_None, goal, MAZE_GOAL_SIZE,800,
														st_mode_1600_v2, (int)(sizeof(st_mode_1600_v2)/sizeof(t_straight_param *const))	,
														di_mode_1600_v1, (int)(sizeof(di_mode_1600_v1)/sizeof(t_straight_param *const))	,
														acc_mode_1600_v3,(int)(sizeof(acc_mode_1600_v3)/sizeof(t_param *const*const))	,motion);
					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
					enable = 0x00;
				}

			   /*
			   if(irsens->IrSensor_Avg() > 2000)
			   {
					for(int i = 0;i < 11;i++)
					{
						(i%2 == 0) ? Indicate_LED(Mode_State()):Indicate_LED(0x00|0x00);
						HAL_Delay(50);
					}
					t_position start,goal;
			  		start.x = start.y = 0;start.dir = North;
			  		goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
			  		run_path.di_param_set(di_mode_1000_v0, 1);
			  		run_path.di_param_set(di_mode_1000_v0, 1);
			  		run_path.turn_time_set(mode_1000);
					run_path.check_run_Dijkstra(start, Dir_None, goal, 2);

					Mode_Disable();
				}
				*/
				break;
			case ENABLE|0x0F:
				if(irsens->IrSensor_Avg() > 2000){
					for(int i = 0;i < 11;i++)
					{
					  (i%2 == 0) ? Indicate_LED(mode|param):Indicate_LED(0x00|0x00);
					  HAL_Delay(50);
					}
					demo_end = True;
				}
				break;
			default:
				break;
		}
	}
}
}

