/*
 * search_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SEARCH_CLASS_H_
#define CPP_INC_SEARCH_CLASS_H_


#include "Subsys/Inc/wall_class.h"
#include "Subsys/Inc/make_map_class.h"
#include "Subsys/Inc/adachi_class.h"
#include "Subsys/Inc/virtual_wall_class.h"

#include "Task/Inc/ctrl_task.h"
#include "Module/Inc/interrupt.h"

#define END_TIME_LIMIT (6*60*1000)

typedef enum
{
	priority_first = 0,
	priority_second = 1,
}t_search_priority;

// Shared by the real Search loop and no-motion diagnostics.  Keeping this
// operation in one place prevents virtual-wall/map ordering from diverging.
void Search_UpdateMap(wall_class *wall,make_map *map,
		const t_virtual_wall_context& virtual_context,t_position expand_end,
		t_position map_goal,int map_goal_size,t_bool full_search,int mask);


class Search
{
	private:
		wall_class *wall_property;
		make_map   *map_property;
		Motion *motion;
		int32_t search_start_time;
		int32_t search_limit_time = END_TIME_LIMIT;
		t_position virtual_wall_maze_start = {0, 0, North};
		t_position virtual_wall_maze_goal = {MAZE_GOAL_X, MAZE_GOAL_Y, North};
		uint8_t virtual_wall_maze_goal_size = MAZE_GOAL_SIZE;
		t_virtual_branch_mode virtual_wall_branch_mode = VIRTUAL_BRANCH_OBSERVED_ONLY;
		t_position virtual_wall_mouse;

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

#if defined(MOUSE_A)
		t_straight_param search_st_param = st_param_370;
		t_param param_L90_search = param_L90_search_370;
		t_param param_R90_search = param_R90_search_370;
#else
		t_straight_param search_st_param = st_param_320;
		t_param param_L90_search = param_L90_search_320;
		t_param param_R90_search = param_R90_search_320;
#endif

	public:
		int32_t return_search_time()	{		return Interrupt::getInstance().return_time_count() - search_start_time;	    };
		void reset_search_time()		{		search_start_time = Interrupt::getInstance().return_time_count(); 		};

		int32_t return_search_limit_time()			{		return search_limit_time;	    };
		void set_search_limit_time(int32_t time)	{		search_limit_time = time; 		};
		void set_virtual_wall_protected_area(t_position maze_start,t_position maze_goal,uint8_t maze_goal_size)
		{
			virtual_wall_maze_start = maze_start;
			virtual_wall_maze_goal = maze_goal;
			virtual_wall_maze_goal_size = maze_goal_size;
		}
		void set_virtual_wall_branch_mode(t_virtual_branch_mode mode)
		{
			virtual_wall_branch_mode = mode;
		}

		t_position run_goal_search	(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion );
		t_position run_accelerated_goal_search(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion );

		t_position run_goal_search_first_priority(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_first;
			virtual_wall_branch_mode = VIRTUAL_BRANCH_OBSERVED_ONLY;
			return run_goal_search	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}
		t_position run_accelerated_goal_search_first_priority(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_first;
			virtual_wall_branch_mode = VIRTUAL_BRANCH_OBSERVED_ONLY;
			return run_accelerated_goal_search	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}


		t_position run_full_exploration(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= True;
			search_priority     = priority_first;
			virtual_wall_branch_mode = VIRTUAL_BRANCH_OBSERVED_ONLY;
			return run_goal_search	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}


		t_position run_accelerated_full_exploration(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= True;
			search_priority     = priority_first;
			virtual_wall_branch_mode = VIRTUAL_BRANCH_OBSERVED_ONLY;
			//reset_search_time();
			return run_accelerated_goal_search	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}

		t_position run_accelerated_pruned_full_exploration(t_position start_pos,t_position goal_pos,int goal_size,
					wall_class *_wall,make_map *_map,Motion *motion)
		{
			full_search = True;
			search_priority = priority_first;
			virtual_wall_branch_mode = VIRTUAL_BRANCH_UNKNOWN_OPEN;
			return run_accelerated_goal_search(start_pos,goal_pos,goal_size,_wall,_map,motion);
		}


		t_position run_goal_search_second_priority(	t_position start_pos,	t_position goal_pos,	int goal_size,
									wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_second;
			virtual_wall_branch_mode = VIRTUAL_BRANCH_OBSERVED_ONLY;
			return run_goal_search	(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}

		t_position run_accelerated_goal_search_second_priority(	t_position start_pos,	t_position goal_pos,	int goal_size,
										wall_class *_wall,		make_map *_map,			Motion *motion )
		{
			full_search			= False;
			search_priority     = priority_second;
			virtual_wall_branch_mode = VIRTUAL_BRANCH_OBSERVED_ONLY;
			//reset_search_time();
			return run_accelerated_goal_search(start_pos,goal_pos,goal_size,_wall,_map,motion );
		}

		void search_param_init()
		{
#if defined(MOUSE_A)
			search_st_param = st_param_370;
			param_L90_search = param_L90_search_370;
			param_R90_search = param_R90_search_370;
#else
			search_st_param = st_param_320;
			param_L90_search = param_L90_search_320;
			param_R90_search = param_R90_search_320;
#endif
		}

		t_bool i_am_goal(int x,int y,int gx,int gy,int goal_size);
		t_bool i_am_goal(t_position pos,t_position g_pos,int goal_size);

};


#endif /* CPP_INC_SEARCH_CLASS_H_ */
