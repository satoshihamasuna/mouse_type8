/*
 * demo.cpp
 *
 *  Created on: 2024/07/16
 *      Author: sato1
 */


#include "Peripheral/Inc/peripheral.h"
#include "Peripheral/Inc/communicate.h"

#include "Subsys/Inc/search_class.h"
#include "Subsys/Inc/make_map_class.h"
#include "Subsys/Inc/wall_class.h"
#include "Subsys/Inc/make_path.h"

#include "Component/Inc/controller.h"
#include "Component/Inc/queue.h"
#include "Component/Inc/priority_queue.h"
#include "Component/Inc/typedef.h"
#include "Component/Inc/math_utils.h"

#include "Task/Inc/sensing_task.h"
#include "Task/Inc/ctrl_task.h"

#include "Module/Inc/interrupt.h"
#include "Module/Inc/log_data.h"
#include "Module/Inc/flash.h"

#include "Params/run_param.h"

#include "System/Inc/mode.h"
#include "System/Inc/demo_util.h"

#define ENABLE (0x01 << 4)

void search_error_process(int init_history_cnt, wall_class *wall_data)
{
	if(wall_data->wall_history.get_history_cnt() > init_history_cnt + 10)
	{
		wall_data->wall_history.history_delete(0x0A);
		write_history_flash(&(wall_data->wall_history));
	}
	else if(wall_data->wall_history.get_history_cnt() - init_history_cnt > 0)
	{
		int delete_num = wall_data->wall_history.get_history_cnt() - init_history_cnt;
		wall_data->wall_history.history_delete(delete_num);
		write_history_flash(&(wall_data->wall_history));
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


	Motion *motion = &(CtrlTask_type8::getInstance());
	IrSensTask *irsens = (CtrlTask_type8::getInstance().return_irObj());

	Search solve_maze;
	solve_maze.set_search_limit_time((6*60*1000));

	wall_class wall_data(irsens);
	wall_data.init_maze();
	wall_data.wall_history.history_init();
	write_wall_WorkRam(&wall_data);
	write_history_WorkRam(&(wall_data.wall_history));
	int init_history_cnt = 0;


	t_position start,goal;
	start.x = start.y = 0;start.dir = North;
	goal.x = MAZE_GOAL_X, goal.y = MAZE_GOAL_Y;
	uint8_t goal_size = MAZE_GOAL_SIZE;
	solve_maze.set_virtual_wall_protected_area(start, goal, goal_size);



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
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = DemoUtil::run_search(&solve_maze, DemoUtil::ACCELERATED_GOAL_SEARCH_FIRST_PRIORITY, start, goal, goal_size, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					motion->Motion_end();
					write_save_data(&wall_data);

					//return
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					DemoUtil::run_search(&solve_maze, DemoUtil::ACCELERATED_PRUNED_FULL_EXPLORATION, return_pos, start, 1, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
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
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = DemoUtil::run_search(&solve_maze, DemoUtil::GOAL_SEARCH_FIRST_PRIORITY, start, goal, goal_size, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);

					//return
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					DemoUtil::run_search(&solve_maze, DemoUtil::GOAL_SEARCH_FIRST_PRIORITY, return_pos, start, 1, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
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
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = DemoUtil::run_search(&solve_maze, DemoUtil::ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY, start, goal, goal_size, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);

					//return
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					solve_maze.reset_search_time();
					DemoUtil::run_search(&solve_maze, DemoUtil::ACCELERATED_FULL_EXPLORATION, return_pos, start, 1, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
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
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					solve_maze.search_param_init();
					solve_maze.reset_search_time();
					t_position return_pos = DemoUtil::run_search(&solve_maze, DemoUtil::ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY, start, goal, goal_size, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
						enable = 0x00;
						break;
					}
					write_save_data(&wall_data);

					//return
					init_history_cnt =  wall_data.wall_history.get_history_cnt();
					DemoUtil::run_search(&solve_maze, DemoUtil::ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY, return_pos, start, 1, &wall_data, motion);
					if(motion->motion_exeStatus_get() == error)
					{
						search_error_process(init_history_cnt, &wall_data);
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

					Dijkstra run_path(&wall_data);
					run_path.run_config_set(find_run_config("uniform1000"));
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

					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra(		start, Dir_None, goal,MAZE_GOAL_SIZE,
												find_run_config("plain700"),motion);


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
					
					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra_suction_acc(	start, Dir_None, goal, MAZE_GOAL_SIZE,
												find_run_config("acc1600_v1"),motion);


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

					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal,MAZE_GOAL_SIZE,
												find_run_config("suction1400_v1_600"),motion);

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

					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,
												find_run_config("suction1400_v2"),motion);

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


					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,
												find_run_config("suction1600_v1"),motion);

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


					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra_suction(		start, Dir_None, goal, MAZE_GOAL_SIZE,
												find_run_config("suction1600_v2"),motion);

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


					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra_suction_acc(	start, Dir_None, goal, MAZE_GOAL_SIZE,
												find_run_config("acc1600_v2"),motion);
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


					Dijkstra run_path(&wall_data);
					run_path.run_Dijkstra_suction_acc(	start, Dir_None, goal, MAZE_GOAL_SIZE,
												find_run_config("acc1600_v3"),motion);
					if(motion->motion_exeStatus_get() == error)
					{
						Mode::indicate_error();
						enable = 0x00;
						break;
					}
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
					demo_end = True;
				}
				break;
			default:
				break;
		}
	}
}
}
