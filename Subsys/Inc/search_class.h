/*
 * search_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SEARCH_CLASS_H_
#define CPP_INC_SEARCH_CLASS_H_


#include "wall_class.h"
#include "make_map_class.h"
#include "adachi_class.h"

#include "../../Task/Inc/ctrl_task.h"
#include "../../Module/Inc/interrupt.h"

#define END_TIME_LIMIT (6*60*1000)

typedef enum
{
	priority_first = 0,
	priority_second = 1,
}t_search_priority;


class Search
{
	private:
		wall_class *wall_property;
		make_map   *map_property;
		Motion *motion;
		int32_t search_start_time;
		int32_t search_limit_time = END_TIME_LIMIT;

		t_exeStatus updateMap_half_straight	(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion);
		t_exeStatus updateMap_half_straight_and_stop(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion);
		t_exeStatus updateMap_straight		(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion);
		t_exeStatus updateMap_left_turn		(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion);
		t_exeStatus updateMap_right_turn	(int x, int y,t_position expand_end,int size,int mask,make_map *_map,Motion *motion);

		t_exeStatus turn_right_process( t_position my_position,t_position tmp_my_pos,t_position goal_pos,int goal_size,int mask,
										wall_class *_wall,make_map *_map,Motion *motion);
		t_exeStatus turn_left_process (	t_position my_position,t_position tmp_my_pos,t_position goal_pos,int goal_size,int mask,
										wall_class *_wall,make_map *_map,Motion *motion);
		t_exeStatus turn_rear_process (	t_position my_position,t_position tmp_my_pos,t_position goal_pos,int goal_size,int mask,
										wall_class *_wall,make_map *_map,Motion *motion);

		void update_map(int x, int y,t_position expand_end,int size,int mask,make_map *_map);

		t_bool full_search			= False;
		t_search_priority search_priority = priority_first;

		t_straight_param search_st_param = st_param_320;
		t_param param_L90_search = param_L90_search_320;
		t_param param_R90_search = param_R90_search_320;

	public:
		int32_t return_search_time()	{		return Interrupt::getInstance().return_time_count() - search_start_time;	    };
		void reset_search_time()		{		search_start_time = Interrupt::getInstance().return_time_count(); 		};

		int32_t return_search_limit_time()			{		return search_limit_time;	    };
		void set_search_limit_time(int32_t time)	{		search_limit_time = time; 		};

		t_position search_adachi	(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion );
		t_position search_adachi_acc(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion );

		t_position search_adachi_1(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_first;
			return search_adachi	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}
		t_position search_adachi_1_acc(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_first;
			return search_adachi_acc	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}


		t_position search_adachi_2(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= True;
			search_priority     = priority_first;
			return search_adachi	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}


		t_position search_adachi_2_acc(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= True;
			search_priority     = priority_first;
			//reset_search_time();
			return search_adachi_acc	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}


		t_position search_adachi_3(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_second;
			return search_adachi	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}

		t_position search_adachi_3_acc(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_second;
			//reset_search_time();
			return search_adachi_acc(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}

		void search_param_init()
		{
			search_st_param = st_param_320;
			param_L90_search = param_L90_search_320;
			param_R90_search = param_R90_search_320;
		}

		t_bool i_am_goal(int x,int y,int gx,int gy,int goal_size);
		t_bool i_am_goal(t_position pos,t_position g_pos,int goal_size);

};


#endif /* CPP_INC_SEARCH_CLASS_H_ */
