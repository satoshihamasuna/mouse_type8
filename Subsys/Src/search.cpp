/*
 * search.cpp
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */


#include "queue.h"
#include "../Inc/make_map_class.h"
#include "typedef.h"
#include "maze_typedef.h"
#include "peripheral.h"
#include "../Inc/search_class.h"
#include "../Inc/adachi_class.h"
#include "../../Params/run_param.h"
#include "../../Task/Inc/ctrl_task.h"
//#include "glob_var_machine.h"

#define ALLOW_SIDE_DIFF 15.0


t_bool Search::i_am_goal(int x,int y,int gx,int gy,int goal_size){
	t_bool flag = False;
	if(gx <= x && x < (gx + goal_size))
	{
		if(gy <= y && y < (gy + goal_size))
		{
			flag = True;
		}
	}
	return flag;
}

t_bool Search::i_am_goal(t_position pos,t_position g_pos,int goal_size)
{
	int x = pos.x,		y = pos.y;
	int gx = g_pos.x,	gy = g_pos.y;

	t_bool flag = False;
	if(gx <= x && x < (gx + goal_size))
	{
		if(gy <= y && y < (gy + goal_size))
		{
			flag = True;
		}
	}
	return flag;
}

void Search::update_map(int x, int y,t_position expand_end,int size,int mask,make_map *_map)
{
	_map->make_map_queue_closeWall();
	if(full_search == True)
		_map->make_map_queue_zenmen(x, y, expand_end, size, mask);
	else
		_map->make_map_queue(x, y, expand_end, size, mask);

}

t_exeStatus Search::updateMap_half_straight(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion)
{
	t_exeStatus result;
	if(expand_end.x == 0 && expand_end.y == 0)
		motion->Init_Motion_search_straight(45.0+15.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);

	else
		motion->Init_Motion_search_straight(45.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);

	update_map(x, y, expand_end, size, mask,_map);
	result = motion->execute_Motion();
	return result;
}

t_exeStatus Search::updateMap_half_straight_and_stop(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion)
{
	t_exeStatus result;
	motion->Init_Motion_straight(45.0, search_st_param.param->acc, search_st_param.param->max_velo, 0.0f);
	update_map(x, y, expand_end, size, mask,_map);
	result = motion->execute_Motion();
	return result;
}

t_exeStatus Search::updateMap_straight(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion)
{
	t_exeStatus result;
	motion->Init_Motion_search_straight(90.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);
	update_map(x, y, expand_end, size, mask,_map);
	result = motion->execute_Motion();
	return result;
}
t_exeStatus Search::updateMap_left_turn(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion)
{
	t_exeStatus result;
	//motion->Init_Motion_search_turn(&param_L90_search_320);
	motion->Init_Motion_search_turn(&param_L90_search);
	update_map(x, y, expand_end, size, mask,_map);
	result = motion->execute_Motion();
	return result;
}
t_exeStatus Search::updateMap_right_turn(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion)
{
	t_exeStatus result;
	//motion->Init_Motion_search_turn(&param_R90_search_320);
	motion->Init_Motion_search_turn(&param_R90_search);
	update_map(x, y, expand_end, size, mask,_map);
	result = motion->execute_Motion();
	return result;
}

t_exeStatus Search::turn_right_process(t_position my_position,t_position tmp_my_pos,t_position goal_pos,int goal_size,int mask,
										wall_class *_wall,make_map *_map,Motion *motion)
{
	IrSensTask *ir_sens = (_wall->return_irObj());
	t_exeStatus result;
	if(ir_sens->sen_l.is_wall == True && ABS(ir_sens->sen_l.distance - 45.0) >= ALLOW_SIDE_DIFF)
	{
		result = updateMap_half_straight_and_stop(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, 0x01,_map,motion);

		if(_wall->get_WallState(my_position) == WALL)
		{
			result = motion->exe_Motion_fix_wall(500);
		}

		result = motion->exe_Motion_pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);

		result = motion->exe_Motion_fix_wall(500);

		result = motion->exe_Motion_pivot_turn(DEG2RAD(-180.0f), -40.0*PI, -4.0*PI);

		result = motion->exe_Motion_straight(45.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);
	}
	else if(_wall->get_WallState(my_position) == WALL && ABS(ir_sens->sen_fr.distance - ir_sens->sen_fl.distance) >= ALLOW_SIDE_DIFF)
	{
		result = updateMap_half_straight_and_stop(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);

		if(_wall->get_WallState(my_position) == WALL)
		{
			result = motion->exe_Motion_fix_wall(500);
		}

		result = motion->exe_Motion_pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);

		result = motion->exe_Motion_straight(45.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);
	}
	else
	{
		result = updateMap_right_turn(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
	}
	return result;
}

t_exeStatus Search::turn_left_process (	t_position my_position,t_position tmp_my_pos,t_position goal_pos,int goal_size,int mask,
										wall_class *_wall,make_map *_map,Motion *motion)
{
	IrSensTask *ir_sens = (_wall->return_irObj());
	t_exeStatus result;
	if(ir_sens->sen_r.is_wall == True && ABS(ir_sens->sen_r.distance - 45.0) >= ALLOW_SIDE_DIFF)
	{
		result = updateMap_half_straight_and_stop(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);

		if(_wall->get_WallState(my_position) == WALL)
		{
			result = motion->exe_Motion_fix_wall(500);
		}

		result = motion->exe_Motion_pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);

		result = motion->exe_Motion_fix_wall(500);

		result = motion->exe_Motion_pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);

		result = motion->exe_Motion_straight(45.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);
	}
	else if(_wall->get_WallState(my_position) == WALL && ABS(ir_sens->sen_fr.distance - ir_sens->sen_fl.distance) >= ALLOW_SIDE_DIFF)
	{
		result = updateMap_half_straight_and_stop(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);

		if(_wall->get_WallState(my_position) == WALL)
		{
			result = motion->exe_Motion_fix_wall(500);
		}
		result = motion->exe_Motion_pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);

		result = motion->exe_Motion_straight(45.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);
	}
	else
	{
		result = updateMap_left_turn(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
	}
	return result;
}

t_exeStatus Search::turn_rear_process (	t_position my_position,t_position tmp_my_pos,t_position goal_pos,int goal_size,int mask,
										wall_class *_wall,make_map *_map,Motion *motion)
{
	IrSensTask *ir_sens = (_wall->return_irObj());
	float length = 45.0;
	if(_wall->get_WallState(my_position) == WALL)
	{
		length = length + ((ir_sens->sen_fr.distance + ir_sens->sen_fl.distance)/2.0 - 90.0f);
	}

	t_exeStatus result;
	motion->Init_Motion_straight(length , search_st_param.param->acc, search_st_param.param->max_velo, 0.0f);
	update_map(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map);
	result = motion->execute_Motion();



	if(_wall->get_WallState(my_position) == WALL)
	{
		result = motion->exe_Motion_fix_wall( 300);

	}

	t_position r_pos = my_position;	r_pos.dir = (t_direction)(((int)(r_pos.dir) + 1) % 4);
	t_position l_pos = my_position; l_pos.dir = (t_direction)(((int)(l_pos.dir) + 3) % 4);


	if(_wall->get_WallState(r_pos) == WALL)
	{
		result = motion->exe_Motion_pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);

		result = motion->exe_Motion_fix_wall( 300);

		result = motion->exe_Motion_pivot_turn( DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
	}
	else if(_wall->get_WallState(l_pos) == WALL)
	{
		result = motion->exe_Motion_pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);

		result = motion->exe_Motion_fix_wall( 300);

		result = motion->exe_Motion_pivot_turn( DEG2RAD(90.0f), 40.0*PI, 4.0*PI);

	}
	else{

		result = motion->exe_Motion_pivot_turn( DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
	}

	if(_wall->get_WallState(r_pos) == WALL && _wall->get_WallState(l_pos) == WALL)
	{
		result = motion->exe_Motion_backward();
		result = motion->exe_Motion_straight(45.0 + 15.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);
	}
	else
	{
		result = motion->exe_Motion_straight(45.0 + 0.0, search_st_param.param->acc, search_st_param.param->max_velo, search_st_param.param->max_velo);
	}
	return result;
}

t_position Search::search_adachi(	t_position start_pos,t_position goal_pos,int goal_size,
									wall_class *_wall,make_map *_map,Motion *motion)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(_wall,_map);
	_wall->goal_set_vwall(goal_pos.x, goal_pos.y, goal_size);
	//IrSensTask *ir_sens = (_wall->return_irObj());
	uint8_t mask = 0x01;
	_map->init_map(goal_pos.x, goal_pos.y, goal_size);
	update_map(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map);


	int direction;

	motion->Motion_start();

	switch(search_priority)
	{
		case priority_first:
			direction = search_algolithm.get_next_dir(my_position, mask, &tmp_my_pos);
			break;
		case priority_second:
			direction = search_algolithm.get_next_dir2(my_position,goal_pos, mask, &tmp_my_pos);
			break;
	}

	switch(direction)
	{
		case Front:
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
			break;
		case Right:
			motion->exe_Motion_pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
			break;
		case Left:
			motion->exe_Motion_pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);

			break;
		case Rear:
			motion->exe_Motion_pivot_turn(DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
			break;

	}
	my_position = tmp_my_pos;

	while(i_am_goal(my_position, goal_pos, goal_size) != True)
	{

		if(goal_pos.x == 0 && goal_pos.y == 0)
		{
			if(return_search_time() >= search_limit_time)
			{
				full_search = False;
				mask = 0x03;
			}
		}

		if(motion->motion_exeStatus_get() == error)
		{
			break;
		}

		//if(_wall->is_unknown(my_position.x, my_position.y) == True)
		//{
				_wall->set_wall(my_position);
		//}
		switch(search_priority)
		{
			case priority_first:
				direction = search_algolithm.get_next_dir(my_position, mask, &tmp_my_pos);
				break;
			case priority_second:
				direction = search_algolithm.get_next_dir2(my_position,goal_pos, mask, &tmp_my_pos);
				break;
		}

		switch(direction)
		{
			case Front:
				updateMap_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
				break;
			case Right:
				turn_right_process(my_position,tmp_my_pos,goal_pos,goal_size,mask,_wall,_map,motion);
		  	    break;
			case Left:
				turn_left_process(my_position,tmp_my_pos,goal_pos,goal_size,mask,_wall,_map,motion);
				break;
			case Rear:
				turn_rear_process(my_position,tmp_my_pos,goal_pos,goal_size,mask,_wall,_map,motion);
				break;
		}
		my_position = tmp_my_pos;
	}
	if(motion->motion_exeStatus_get() != error)
	{
		_wall->set_wall(my_position);
		motion->exe_Motion_straight( 45.0, search_st_param.param->acc, search_st_param.param->max_velo, 0.0f);
	}
	HAL_Delay(100);
	motion->Motion_end();
	_wall->goal_clear_vwall(goal_pos.x, goal_pos.y, goal_size);
	return my_position;
}


t_position Search::search_adachi_acc(	t_position start_pos,t_position goal_pos,int goal_size,wall_class *_wall,make_map *_map,Motion *motion)
{
	t_position tmp_my_pos = start_pos;
	t_position my_position = tmp_my_pos;

	adachi search_algolithm(_wall,_map);
	_wall->goal_set_vwall(goal_pos.x, goal_pos.y, goal_size);
	//IrSensTask *ir_sens = (_wall->return_irObj());
	uint8_t mask = 0x01;

	_map->init_map(goal_pos.x, goal_pos.y, goal_size);
	update_map(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map);

	motion->Motion_start();

	int direction;
	switch(search_priority)
	{
		case priority_first:
			direction = search_algolithm.get_next_dir(my_position, mask, &tmp_my_pos);
			break;
		case priority_second:
			direction = search_algolithm.get_next_dir2(my_position,goal_pos, mask, &tmp_my_pos);
			break;
	}

	switch(direction)
	{
		case Front:
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
			break;
		case Right:
			motion->exe_Motion_pivot_turn(DEG2RAD(-90.0f), -40.0*PI, -4.0*PI);
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
			break;
		case Left:
			motion->exe_Motion_pivot_turn(DEG2RAD(90.0f), 40.0*PI, 4.0*PI);
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);

			break;
		case Rear:
			motion->exe_Motion_pivot_turn(DEG2RAD(180.0f), 40.0*PI, 4.0*PI);
			updateMap_half_straight(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map,motion);
			break;

	}
	my_position = tmp_my_pos;

	while(i_am_goal(my_position, goal_pos, goal_size) != True)
	{
		if(goal_pos.x == 0 && goal_pos.y == 0)
		{
			if(return_search_time() >= search_limit_time)
			{
				full_search = False;
				mask = 0x03;
			}


			if(return_search_time() >= search_limit_time + (90*1000) )
			{
				break;
			}


		}

		if(motion->motion_exeStatus_get() == error)
		{
			break;
		}

		if(_wall->is_unknown(my_position.x, my_position.y) == True)
		{
				_wall->set_wall(my_position);
		}

		switch(search_priority)
		{
			case priority_first:
				direction = search_algolithm.get_next_dir(my_position, mask, &tmp_my_pos);
				break;
			case priority_second:
				direction = search_algolithm.get_next_dir2(my_position,goal_pos, mask, &tmp_my_pos);
				break;
		}


		uint8_t next_acc_flag = 0x00;
		t_position next_acc_pos = tmp_my_pos;
		int		next_acc_dir  = direction;
		if(_wall->is_unknown(tmp_my_pos.x, tmp_my_pos.y) == False)
		{
			if(i_am_goal(tmp_my_pos,goal_pos,goal_size) == False){
				next_acc_flag = 0x80;
				next_acc_dir  = search_algolithm.get_next_dir(tmp_my_pos, mask, &next_acc_pos);
				switch(search_priority)
				{
					case priority_first:
						next_acc_dir  = search_algolithm.get_next_dir(tmp_my_pos, mask, &next_acc_pos);
						break;
					case priority_second:
						next_acc_dir  = search_algolithm.get_next_dir2(tmp_my_pos,goal_pos,mask, &next_acc_pos);
						break;
				}
			}
		}

		switch(direction | next_acc_flag)
		{
			case Front:
				if(goal_pos.x == 0 && goal_pos.y == 0)
				{
					if(return_search_time() >= (5*60*1000))
					{
						search_st_param = st_param_280;
						param_L90_search = param_L90_search_280;
						param_R90_search = param_R90_search_280;
					}
					else if(return_search_time() >= (3*60*1000))
					{
						search_st_param = st_param_300;
						param_L90_search = param_L90_search_300;
						param_R90_search = param_R90_search_300;
					}
				}

				motion->Init_Motion_search_straight(90.0, 6.0, motion->return_vehicleObj()->ideal.velo.get(), search_st_param.param->max_velo);
				update_map(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map);
				motion->execute_Motion();
				break;
			case Front|0x80:
				if(goal_pos.x == 0 && goal_pos.y == 0)
				{
					if(return_search_time() >= (5*60*1000))
					{
						search_st_param = st_param_280;
						param_L90_search = param_L90_search_280;
						param_R90_search = param_R90_search_280;
					}
					else if(return_search_time() >= (3*60*1000))
					{
						search_st_param = st_param_300;
						param_L90_search = param_L90_search_300;
						param_R90_search = param_R90_search_300;
					}
				}
				if(next_acc_dir == Front)
					motion->Init_Motion_search_straight(90.0, 6.0, 0.60, 0.60f);
				else
					motion->Init_Motion_search_straight(90.0, 6.0, motion->return_vehicleObj()->ideal.velo.get(), search_st_param.param->max_velo);

				update_map(goal_pos.x, goal_pos.y, tmp_my_pos, goal_size, mask,_map);
				motion->execute_Motion();
				break;
			case Right:
			case (Right|0x80):
				turn_right_process(my_position,tmp_my_pos,goal_pos,goal_size,mask,_wall,_map,motion);
		  	    break;
			case Left:
			case (Left|0x80):
				turn_left_process(my_position,tmp_my_pos,goal_pos,goal_size,mask,_wall,_map,motion);
				break;
			case Rear:
			case (Rear|0x80):
				turn_rear_process(my_position,tmp_my_pos,goal_pos,goal_size,mask,_wall,_map,motion);
				break;
		}
		my_position = tmp_my_pos;
	}
	if(motion->motion_exeStatus_get() != error)
	{
		_wall->set_wall(my_position);
		//motion->exe_Motion_straight( 45.0, search_st_param.param->acc, search_st_param.param->max_velo, 0.0f);
		motion->exe_Motion_straight( 45.0, search_st_param.param->acc, motion->return_vehicleObj()->ideal.velo.get(), 0.0f);
	}
	HAL_Delay(100);
	motion->Motion_end();
	_wall->goal_clear_vwall(goal_pos.x, goal_pos.y, goal_size);
	return my_position;
}


