/*
 * makeMap_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_MAKE_MAP_CLASS_H_
#define CPP_INC_MAKE_MAP_CLASS_H_

#include "typedef.h"
#include "math_utils.h"
#include "queue.h"
#include "wall_class.h"

typedef struct{
	uint8_t st_x;
	uint8_t st_y;
	int8_t  par_x;
	int8_t  par_y;
	int16_t cost;
	int16_t cost_h;
}t_MapNode;

class make_map{
	public:
		make_map(wall_class *wall_property_,ring_queue<1024,t_MapNode> *maze_q_);
		wall_class *wall_property;
		ring_queue<1024,t_MapNode> *maze_q;
		uint16_t map[MAZE_SIZE_X][MAZE_SIZE_Y];
		void init_map(int x, int y,int goal_size);
		void expand(t_MapNode n,int mask);
		void expand_and_closeWall(t_MapNode n,int mask);
		void make_map_queue(int x, int y,t_position expand_end,int size,int mask);
		void make_map_queue_closeWall(int x = 0, int y = 0,int size = 1,int mask = 0x01);
		void make_map_queue_zenmen(int x, int y,t_position expand_end,int size,int mask);
		void Display();
};



#endif /* CPP_INC_MAKE_MAP_CLASS_H_ */
