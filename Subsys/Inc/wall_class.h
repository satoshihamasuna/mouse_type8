/*
 * wall_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_WALL_CLASS_H_
#define CPP_INC_WALL_CLASS_H_

#include "typedef.h"
#include "maze_typedef.h"
#include "math_utils.h"
#include "../../Task/Inc/sensing_task.h"


typedef struct
{
	int8_t x;
	int8_t y;
	t_wall wall;
}t_histry_wall;

class wall_histry_class
{
	int16_t histry_cnt;
	int16_t histry_tail;
	public:
		t_histry_wall histry_wall[MAZE_SIZE];
		void histry_init()
		{
			for(int i = 0; i < MAZE_SIZE;i++)
			{
				histry_wall[i].x = -1;
				histry_wall[i].y = -1;
				histry_wall[i].wall.north = UNKNOWN;
				histry_wall[i].wall.south = UNKNOWN;
				histry_wall[i].wall.east = UNKNOWN;
				histry_wall[i].wall.west = UNKNOWN;
			}
			histry_cnt = 0;
			histry_tail =- 1;
		}

		int16_t get_histry_cnt()
		{
			return histry_cnt;
		}

		int16_t get_histry_tail()
		{
			return histry_tail;
		}

		void histry_set(int x,int y,t_wall wall)
		{
			if(histry_cnt < MAZE_SIZE)
			{
				histry_wall[histry_tail + 1].x = x;
				histry_wall[histry_tail + 1].y = y;
				histry_wall[histry_tail + 1].wall.north = wall.north;
				histry_wall[histry_tail + 1].wall.south = wall.south;
				histry_wall[histry_tail + 1].wall.east = wall.east;
				histry_wall[histry_tail + 1].wall.west = wall.west;
				histry_tail = histry_tail + 1;
				histry_cnt  = histry_cnt + 1;
			}
		}

		void histry_delete(int num)
		{
			if(num > histry_cnt) num = histry_cnt;
			for(int i = 0; i < num; i++)
			{
				if(histry_tail == -1) break;
				histry_wall[histry_tail].x = -1;
				histry_wall[histry_tail].y = -1;
				histry_wall[histry_tail].wall.north = UNKNOWN;
				histry_wall[histry_tail].wall.south = UNKNOWN;
				histry_wall[histry_tail].wall.east = UNKNOWN;
				histry_wall[histry_tail].wall.west = UNKNOWN;
				histry_tail = histry_tail - 1;
				histry_cnt  = histry_cnt - 1;
			}
		}

		void histry_indicate()
		{
			for(int i = 0; i <histry_cnt;i++)
			{
				printf("%d:",i);
				printf("(x,y)->(%2d,%2d),",histry_wall[i].x,histry_wall[i].y);
				printf("(n,e,s,w)->(%2x,%2x,%2x,%2x)\n",
						histry_wall[i].wall.north,histry_wall[i].wall.east,histry_wall[i].wall.south,histry_wall[i].wall.west);
			}
		}
};

class wall_class
{
	IrSensTask *ir_sens;

	public:
		wall_histry_class wall_histry;
		wall_class(IrSensTask *ir_sens_)
		{
			ir_sens = ir_sens_;
		}
		IrSensTask *return_irObj() {return ir_sens;};
		t_wall wall[MAZE_SIZE_X][MAZE_SIZE_Y];
		void init_maze();
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
		void histry2wall_append();


};

/*
class wall_class_type7: public wall_class,public Singleton<wall_class_type7>
{
public:
	wall_class_type7(IrSensTask *ir = &IrSensTask_type7::getInstance()):wall_class(ir){}
};
*/

#endif /* CPP_INC_WALL_CLASS_H_ */
