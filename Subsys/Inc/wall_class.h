/*
 * wall_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_WALL_CLASS_H_
#define CPP_INC_WALL_CLASS_H_

#include "Component/Inc/typedef.h"
#include "Component/Inc/maze_typedef.h"
#include "Component/Inc/math_utils.h"
#include "Task/Inc/sensing_task.h"


typedef struct
{
	int8_t x;
	int8_t y;
	t_wall wall;
}t_history_wall;


class wall_history_class
{
	int16_t history_cnt;
	int16_t history_tail;
	public:
		t_history_wall history_wall[MAZE_SIZE];
		void history_init()
		{
			for(int i = 0; i < MAZE_SIZE;i++)
			{
				history_wall[i].x = -1;
				history_wall[i].y = -1;
				history_wall[i].wall.north = UNKNOWN;
				history_wall[i].wall.south = UNKNOWN;
				history_wall[i].wall.east = UNKNOWN;
				history_wall[i].wall.west = UNKNOWN;
			}
			history_cnt = 0;
			history_tail =- 1;
		}

		int16_t get_history_cnt()
		{
			return history_cnt;
		}

		int16_t get_history_tail()
		{
			return history_tail;
		}

		void history_set(int x,int y,t_wall wall)
		{
			if(history_cnt < MAZE_SIZE)
			{
				history_wall[history_tail + 1].x = x;
				history_wall[history_tail + 1].y = y;
				history_wall[history_tail + 1].wall.north = wall.north;
				history_wall[history_tail + 1].wall.south = wall.south;
				history_wall[history_tail + 1].wall.east = wall.east;
				history_wall[history_tail + 1].wall.west = wall.west;
				history_tail = history_tail + 1;
				history_cnt  = history_cnt + 1;
			}
		}

		void history_delete(int num)
		{
			if(num > history_cnt) num = history_cnt;
			for(int i = 0; i < num; i++)
			{
				if(history_tail == -1) break;
				history_wall[history_tail].x = -1;
				history_wall[history_tail].y = -1;
				history_wall[history_tail].wall.north = UNKNOWN;
				history_wall[history_tail].wall.south = UNKNOWN;
				history_wall[history_tail].wall.east = UNKNOWN;
				history_wall[history_tail].wall.west = UNKNOWN;
				history_tail = history_tail - 1;
				history_cnt  = history_cnt - 1;
			}
		}

		void history_indicate()
		{
			for(int i = 0; i <history_cnt;i++)
			{
				printf("%d:",i);
				printf("(x,y)->(%2d,%2d),",history_wall[i].x,history_wall[i].y);
				printf("(n,e,s,w)->(%2x,%2x,%2x,%2x)\n",
						history_wall[i].wall.north,history_wall[i].wall.east,history_wall[i].wall.south,history_wall[i].wall.west);
			}
		}

};


class wall_class
{
	IrSensTask *ir_sens;

	public:
		wall_history_class wall_history;
		wall_class(IrSensTask *ir_sens_)
		{
			ir_sens = ir_sens_;
		}
		IrSensTask *return_irObj() {return ir_sens;};
		t_wall wall[MAZE_SIZE_X][MAZE_SIZE_Y];
		// Keep inferred closures separate from sensor observations. They persist
		// across moves; edges touching protected cells are cleared selectively.
		t_wall virtual_wall[MAZE_SIZE_X][MAZE_SIZE_Y];
		void init_maze();
		void clear_virtual_wall();
		void clear_virtual_wall(uint16_t x,uint16_t y,t_direction dir);
		t_bool get_virtual_wall(uint16_t x,uint16_t y,t_direction dir) const;
		t_bool set_virtual_wall(uint16_t x,uint16_t y,t_direction dir);
		t_bool is_open(uint16_t x,uint16_t y,t_direction dir,int mask) const;
		void set_wall(t_position pos);
		t_bool is_unknown(uint16_t x,uint16_t y);
		void goal_set_vwall(int gx,int gy,int goal_size){
			if(goal_size == 3)
			{
				wall[gx+1][gy+1].north = wall[gx+1][gy+1].east = wall[gx+1][gy+1].south = wall[gx+1][gy+1].west = VWALL;
				wall[gx+1][gy+2].south  = wall[gx+2][gy+1].west = wall[gx+1][gy+0].north = wall[gx+0][gy+1].east = VWALL;
			}

		}
		void goal_clear_vwall(int gx,int gy,int goal_size){
			if(goal_size == 3)
			{
				wall[gx+1][gy+1].north = wall[gx+1][gy+1].east = wall[gx+1][gy+1].south = wall[gx+1][gy+1].west = NOWALL;
				wall[gx+1][gy+2].south = wall[gx+2][gy+1].west = wall[gx+1][gy+0].north = wall[gx+0][gy+1].east = NOWALL;
			}
		}
		t_wall_state get_WallState(t_position pos);
		void indicate_wall();
		void indicate_wall_binary();
		void history2wall_append();


};

/*
class wall_class_type8: public wall_class,public Singleton<wall_class_type8>
{
public:
	wall_class_type8(IrSensTask *ir = &IrSensTask_type8::getInstance()):wall_class(ir){}
};
*/

#endif /* CPP_INC_WALL_CLASS_H_ */
