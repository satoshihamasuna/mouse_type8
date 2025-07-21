/*
 * make_path.h
 *
 *  Created on: 2023/06/26
 *      Author: sato1
 */

#ifndef CPP_INC_MAKE_PATH_H_
#define CPP_INC_MAKE_PATH_H_

#include "wall_class.h"
#include "typedef.h"

#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controller.h"

#include "../../Params/run_param.h"

#include "../../Task/Inc/ctrl_task.h"
#include "../../Task/Inc/run_typedef.h"
#include "../../Task/Inc/sensing_task.h"
//#define DEBUG_MODE

typedef enum
{
	C_pos = 0,
	N_pos = 1,
	E_pos = 2,
}t_DijkstraWallPos;

typedef struct
{
	uint8_t x:6;
	uint8_t y:6;
	t_DijkstraWallPos NodePos:2;
}t_posDijkstra;

typedef struct
{
	t_posDijkstra parent_pos;
	uint16_t time;
	t_direction dir:4;
	uint8_t run_pt:5;
	t_bool determine:1;
}t_element;

typedef struct
{
	t_element Center;
	t_element North;
	t_element East;
}t_MapNodeWall;



class calcRunTime
{
	private:
		float turn_V90_time;
		float turn_Long90_time;
		float turn_Long180_time;
		float turn_in45_time;
		float turn_in135_time;
		float turn_out45_time;
		float turn_out135_time;
		const t_straight_param *const *st_set_mode;
		const t_straight_param *const *di_set_mode;
		uint16_t st_mode_size = 1;uint16_t di_mode_size = 1;
	public:
		void turn_time_set(const t_param *const *mode);
		void turn_time_set(const t_param *const*const *mode  ,uint16_t mode_size);
		void st_param_set(const t_straight_param *const *mode,uint16_t mode_size);
		void di_param_set(const t_straight_param *const *mode,uint16_t mode_size);
		uint16_t return_turn_time(t_run_pattern run_pt);
		uint16_t straight_time_set(float length);
		t_straight_param calc_end_straight_max_velo(float length);
		t_straight_param straight_base_velo()
		{
			t_straight_param return_param;
        	return_param.param 			  =	st_set_mode[0]->param;
        	return_param.sp_gain		  = st_set_mode[0]->sp_gain;
        	return_param.om_gain		  = st_set_mode[0]->om_gain;
			return return_param;
		}
		uint16_t diagonal_time_set(float length);
		t_straight_param calc_end_diagonal_max_velo(float length);
		t_straight_param diagonal_base_velo()
		{
			t_straight_param return_param;
        	return_param.param 			  =	di_set_mode[0]->param;
        	return_param.sp_gain		  = di_set_mode[0]->sp_gain;
        	return_param.om_gain		  = di_set_mode[0]->om_gain;
			return return_param;
		}
		calcRunTime()
		{
			turn_V90_time		= 2.0f;
			turn_Long90_time	= 2.0f;
			turn_Long180_time	= 2.0f;
			turn_in45_time		= 2.0f;
			turn_in135_time		= 2.0f;
			turn_out45_time		= 2.0f;
			turn_out135_time	= 2.0f;
			st_set_mode = st_mode_300_v0;
			di_set_mode = di_mode_300_v0;
			st_mode_size = 1;
			di_mode_size = 1;

		}
};

class Dijkstra:public calcRunTime
{
	private:
		void straight_expand(t_posDijkstra pos,t_direction m_dir);
		void diagonal_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_inR45_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_inL45_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_outR45_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_outL45_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_inR135_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_inL135_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_outR135_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_outL135_expand(t_posDijkstra pos,t_direction m_dir);
		void longturn_R90_expand(t_posDijkstra pos,t_direction m_dir);
		void longturn_L90_expand(t_posDijkstra pos,t_direction m_dir);
		void longturn_R180_expand(t_posDijkstra pos,t_direction m_dir);
		void longturn_L180_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_vR90_expand(t_posDijkstra pos,t_direction m_dir);
		void turn_vL90_expand(t_posDijkstra pos,t_direction m_dir);


		t_posDijkstra LocalPosDir2GlobWallPos_Center(t_posDijkstra glob_pos,t_direction glob_dir,t_local_dir LocalPos,t_local_dir LocalDir);
		t_posDijkstra LocalPosDir2GlobWallPos_WPos(t_posDijkstra glob_pos,t_direction glob_dir,t_local_dir LocalDir);
		t_posDijkstra SetNodePos(uint8_t _x,uint8_t _y,t_DijkstraWallPos _dpos);
		t_element SetNode(t_posDijkstra _parent,	uint16_t _time,		t_direction _dir
						 ,t_run_pattern _run_pt,		t_bool _determine);
		void set_determine(t_posDijkstra set_pos);

		t_element*  get_closure_inf(t_posDijkstra position);
		uint8_t get_wall_inf(t_posDijkstra position);
	public:
		t_MapNodeWall closure[MAZE_SIZE_X][MAZE_SIZE_Y];
		t_posDijkstra run_pos_buff[MAZE_SIZE];
		wall_class *wall_property;
		Dijkstra(wall_class *_wall_property)
		{
			wall_property = _wall_property;
		}
		void init_dijkstra_map();
		void start_node_setUp(t_posDijkstra start_pos,t_direction dir);
		t_bool is_goal_Dijkstra(t_posDijkstra check_pos,t_position goal_pos,uint8_t goal_size);
		t_posDijkstra conv_t_pos2t_posDijkstra(t_position pos,t_direction wall_pos);
		t_posDijkstra conv_t_pos2t_posDijkstra(int _x,int _y,t_direction wall_pos);
		t_posDijkstra min_search();
		t_posDijkstra make_path_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size);
		void check_run_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size);
		t_bool check_DijkstraPath(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size);

		void run_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,
						  const t_straight_param *const *st_mode,uint16_t size_st_mode,
						  const t_straight_param *const *di_mode,uint16_t size_di_mode,
						  const t_param *const *turn_mode,Motion *motion);
		void run_Dijkstra_suction(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,int suction,
								  const t_straight_param *const *st_mode,uint16_t size_st_mode,
								  const t_straight_param *const *di_mode,uint16_t size_di_mode,
								  const t_param *const *turn_mode , Motion *motion);
		void run_Dijkstra_suction_acc(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,int suction,
											  const t_straight_param *const *st_mode,uint16_t size_st_mode,
											  const t_straight_param *const *di_mode,uint16_t size_di_mode,
											  const t_param *const*const *turn_mode ,uint16_t size_turn_mode, Motion *motion);
		void expand(t_posDijkstra pos);
		t_posDijkstra last_expand(t_posDijkstra pos,t_direction m_dir,t_position goal_pos,uint8_t goal_size);
		uint16_t straight_section_num(t_posDijkstra s_pos,t_posDijkstra e_pos,t_direction dir);
		uint16_t diagonal_section_num(t_posDijkstra s_pos,t_posDijkstra e_pos,t_direction dir);
};

/*
class Dijkstra_type7: public Dijkstra,public Singleton<Dijkstra_type7>
{
public:
	Dijkstra_type7(wall_class *w = &wall_class_type7::getInstance()):Dijkstra(w){}
};
*/

#endif /* CPP_INC_MAKE_PATH_H_ */
