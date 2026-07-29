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

#include "singleton.h"
#include "controller.h"

#include "run_param.h"

#include "ctrl_task.h"
#include "run_typedef.h"
#include "sensing_task.h"
//#define DEBUG_MODE

#define DIJKSTRA_PATH_MAX 512

typedef enum
{
	C_pos = 0,
	N_pos = 1,
	E_pos = 2,
}t_DijkstraWallPos;

typedef struct
{
	uint16_t x:5;
	uint16_t y:5;
	uint16_t NodePos:2;
	uint16_t state_dir:2;
}t_posDijkstra;

typedef struct
{
	uint32_t parent_x:5;
	uint32_t parent_y:5;
	uint32_t parent_NodePos:2;
	uint32_t parent_state_dir:2;
	uint32_t determine:1;
	uint32_t time:17;
} __attribute__((packed)) t_element;

static_assert(sizeof(t_element) == 4, "t_element must remain 4 bytes");

typedef struct
{
	t_element Center[4];
	t_element North[4];
	t_element East[4];
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
		t_posDijkstra SetNodePos(uint8_t _x,uint8_t _y,t_DijkstraWallPos _dpos, t_direction _dir = North);
		t_element SetNode(t_posDijkstra _parent, uint32_t _time, t_bool _determine);
		t_posDijkstra get_parent(const t_element *element) const;
		t_run_pattern get_run_pattern(t_posDijkstra current) const;
		void set_determine(t_posDijkstra set_pos);
		uint16_t dijkstra_node_order(t_posDijkstra pos);
		void push_open_node(t_posDijkstra pos);
		t_bool pop_open_node(t_posDijkstra *pos);

		t_element*  get_closure_inf(t_posDijkstra position);
		uint8_t get_wall_inf(t_posDijkstra position);
		t_bool use_priority_queue;
	public:
		t_MapNodeWall closure[MAZE_SIZE_X][MAZE_SIZE_Y];
		t_posDijkstra run_pos_buff[DIJKSTRA_PATH_MAX];
		uint8_t run_pt_buff[DIJKSTRA_PATH_MAX];
		wall_class *wall_property;
		Dijkstra(wall_class *_wall_property)
		{
			wall_property = _wall_property;
			use_priority_queue = False;
		}
		void init_dijkstra_map();
		void start_node_setUp(t_posDijkstra start_pos,t_direction dir);
		t_bool is_goal_Dijkstra(t_posDijkstra check_pos,t_position goal_pos,uint8_t goal_size);
		t_posDijkstra conv_t_pos2t_posDijkstra(t_position pos,t_direction wall_pos);
		t_posDijkstra conv_t_pos2t_posDijkstra(int _x,int _y,t_direction wall_pos);
		t_posDijkstra min_search();
		t_posDijkstra Local2Global(t_posDijkstra base,t_direction base_dir,t_local_dir local_pos,t_local_dir local_dir);

		t_posDijkstra make_path_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size);
		t_posDijkstra make_path_Dijkstra_priority_queue(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size);
		void check_run_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,
								t_bool priority_queue = False);
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
class Dijkstra_type8: public Dijkstra,public Singleton<Dijkstra_type8>
{
public:
	Dijkstra_type8(wall_class *w = &wall_class_type8::getInstance()):Dijkstra(w){}
};
*/

#endif /* CPP_INC_MAKE_PATH_H_ */
