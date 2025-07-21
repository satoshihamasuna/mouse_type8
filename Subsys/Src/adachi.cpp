/*
 * adachi.cpp
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */


#include "typedef.h"
#include "maze_typedef.h"
#include "adachi_class.h"

t_position pos_set(uint8_t _x,uint8_t _y ,t_direction _dir)
{
	t_position set_pos;
	set_pos.x = _x;
	set_pos.y = _y;
	set_pos.dir = _dir;
	return set_pos;
}

int adachi::get_priority(t_position mypos,t_position next_pos){	//そのマスの情報から、優先度を算出する

	int priority;	//優先度を記録する変数
	priority = 0;

	if(mypos.dir == next_pos.dir){				//行きたい方向が現在の進行方向と同じ場合
		priority = 2;
	}else if( ((4+mypos.dir-next_pos.dir)%4) == 2){		//行きたい方向が現在の進行方向と逆の場合
		priority = 0;
	}else{						//それ以外(左右どちらか)の場合
		priority = 1;
	}

	if(wall_property->is_unknown(next_pos.x,next_pos.y) == True){
		priority += 4;				//未探索の場合優先度をさらに付加
	}

	return priority;				//優先度を返す

}

int adachi::get_priority2(t_position mypos,t_position next_pos,t_position goal_pos){	//そのマスの情報から、優先度を算出する

	int priority;	//優先度を記録する変数
	priority = 0;

	if(mypos.dir == next_pos.dir){				//行きたい方向が現在の進行方向と同じ場合
		priority = 2;
	}else if( ((4+mypos.dir-next_pos.dir)%4) == 2){		//行きたい方向が現在の進行方向と逆の場合
		priority = 0;
	}else{						//それ以外(左右どちらか)の場合
		priority = 1;
	}

	if(wall_property->is_unknown(next_pos.x,next_pos.y) == True){

		 priority = 2*MAZE_SIZE
				   -(((goal_pos.x-next_pos.x)*(goal_pos.x-next_pos.x)
		            +(goal_pos.y-next_pos.y)*(goal_pos.y-next_pos.y)));				//未探索の場合優先度をさらに付加
	}

	return priority;				//優先度を返す

}


int adachi::get_next_dir(t_position mypos,int mask,t_position *glob_next_pos)
{
	int little,priority,tmp_priority;
	little = MAP_MAX_VALUE;

	priority = 0;

	if((wall_property->wall[mypos.x][mypos.y].north & mask) == NOWALL)
	{
		tmp_priority = get_priority(mypos, pos_set(mypos.x,mypos.y+1,North));
		if(map_property->map[mypos.x][mypos.y+1] < little)
		{
			little = map_property->map[mypos.x][mypos.y+1];
			*glob_next_pos = pos_set(mypos.x,mypos.y+1,North);
			/*
			glob_next_pos->x 	= mypos.x;
			glob_next_pos->y 	= mypos.y+1;
			glob_next_pos->dir	= North;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x][mypos.y+1] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x,mypos.y+1,North);
				/*
				glob_next_pos->x 	= mypos.x;
				glob_next_pos->y 	= mypos.y+1;
				glob_next_pos->dir	= North;*/
				priority = tmp_priority;
			}
		}
	}

	if((wall_property->wall[mypos.x][mypos.y].east & mask) == NOWALL)
	{
		tmp_priority = get_priority(mypos, pos_set(mypos.x+1,mypos.y,East));
		if(map_property->map[mypos.x+1][mypos.y] < little)
		{
			little = map_property->map[mypos.x+1][mypos.y];
			*glob_next_pos = pos_set(mypos.x+1,mypos.y,East);
			/*
			glob_next_pos->x 	= mypos.x+1;
			glob_next_pos->y 	= mypos.y;
			glob_next_pos->dir	= East;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x+1][mypos.y] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x+1,mypos.y,East);
				/*
				glob_next_pos->x 	= mypos.x+1;
				glob_next_pos->y 	= mypos.y;
				glob_next_pos->dir	= East;
				*/
				priority = tmp_priority;
			}
		}
	}

	if((wall_property->wall[mypos.x][mypos.y].south & mask) == NOWALL)
	{
		tmp_priority = get_priority(mypos, pos_set(mypos.x,mypos.y-1,South));
		if(map_property->map[mypos.x][mypos.y-1] < little)
		{
			little= map_property->map[mypos.x][mypos.y-1];
			*glob_next_pos = pos_set(mypos.x,mypos.y-1,South);
			/*
			glob_next_pos->x 	= mypos.x;
			glob_next_pos->y 	= mypos.y-1;
			glob_next_pos->dir	= South;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x][mypos.y-1] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x,mypos.y-1,South);
				/*
				glob_next_pos->x 	= mypos.x;
				glob_next_pos->y 	= mypos.y-1;
				glob_next_pos->dir	= South;
				*/
				priority = tmp_priority;
			}
		}
	}

	if((wall_property->wall[mypos.x][mypos.y].west & mask) == NOWALL)
	{
		tmp_priority = get_priority(mypos, pos_set(mypos.x-1,mypos.y,West));
		if(map_property->map[mypos.x-1][mypos.y] < little)
		{
			little = map_property->map[mypos.x-1][mypos.y];
			*glob_next_pos = pos_set(mypos.x-1,mypos.y,West);
			/*
			glob_next_pos->x 	= mypos.x-1;
			glob_next_pos->y 	= mypos.y;
			glob_next_pos->dir	= West;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x-1][mypos.y] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x-1,mypos.y,West);
				/*
				glob_next_pos->x 	= mypos.x-1;
				glob_next_pos->y 	= mypos.y;
				glob_next_pos->dir	= West;
				*/
				priority = tmp_priority;
			}
		}
	}

	return ((int)((4+glob_next_pos->dir - mypos.dir)%4));
}

int adachi::get_next_dir2(t_position mypos,t_position goal_pos,int mask,t_position *glob_next_pos)
{
	int little,priority,tmp_priority;
	little = MAP_MAX_VALUE;

	priority = 0;

	if((wall_property->wall[mypos.x][mypos.y].north & mask) == NOWALL)
	{
		tmp_priority = get_priority2(mypos, pos_set(mypos.x,mypos.y+1,North),goal_pos);
		if(map_property->map[mypos.x][mypos.y+1] < little)
		{
			little = map_property->map[mypos.x][mypos.y+1];
			*glob_next_pos = pos_set(mypos.x,mypos.y+1,North);
			/*
			glob_next_pos->x 	= mypos.x;
			glob_next_pos->y 	= mypos.y+1;
			glob_next_pos->dir	= North;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x][mypos.y+1] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x,mypos.y+1,North);
				/*
				glob_next_pos->x 	= mypos.x;
				glob_next_pos->y 	= mypos.y+1;
				glob_next_pos->dir	= North;*/
				priority = tmp_priority;
			}
		}
	}

	if((wall_property->wall[mypos.x][mypos.y].east & mask) == NOWALL)
	{
		tmp_priority = get_priority2(mypos, pos_set(mypos.x+1,mypos.y,East),goal_pos);
		if(map_property->map[mypos.x+1][mypos.y] < little)
		{
			little = map_property->map[mypos.x+1][mypos.y];
			*glob_next_pos = pos_set(mypos.x+1,mypos.y,East);
			/*
			glob_next_pos->x 	= mypos.x+1;
			glob_next_pos->y 	= mypos.y;
			glob_next_pos->dir	= East;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x+1][mypos.y] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x+1,mypos.y,East);
				/*
				glob_next_pos->x 	= mypos.x+1;
				glob_next_pos->y 	= mypos.y;
				glob_next_pos->dir	= East;
				*/
				priority = tmp_priority;
			}
		}
	}

	if((wall_property->wall[mypos.x][mypos.y].south & mask) == NOWALL)
	{
		tmp_priority = get_priority2(mypos, pos_set(mypos.x,mypos.y-1,South),goal_pos);
		if(map_property->map[mypos.x][mypos.y-1] < little)
		{
			little= map_property->map[mypos.x][mypos.y-1];
			*glob_next_pos = pos_set(mypos.x,mypos.y-1,South);
			/*
			glob_next_pos->x 	= mypos.x;
			glob_next_pos->y 	= mypos.y-1;
			glob_next_pos->dir	= South;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x][mypos.y-1] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x,mypos.y-1,South);
				/*
				glob_next_pos->x 	= mypos.x;
				glob_next_pos->y 	= mypos.y-1;
				glob_next_pos->dir	= South;
				*/
				priority = tmp_priority;
			}
		}
	}

	if((wall_property->wall[mypos.x][mypos.y].west & mask) == NOWALL)
	{
		tmp_priority = get_priority2(mypos, pos_set(mypos.x-1,mypos.y,West),goal_pos);
		if(map_property->map[mypos.x-1][mypos.y] < little)
		{
			little = map_property->map[mypos.x-1][mypos.y];
			*glob_next_pos = pos_set(mypos.x-1,mypos.y,West);
			/*
			glob_next_pos->x 	= mypos.x-1;
			glob_next_pos->y 	= mypos.y;
			glob_next_pos->dir	= West;
			*/
			priority = tmp_priority;
		}
		else if(map_property->map[mypos.x-1][mypos.y] == little)
		{
			if(priority < tmp_priority)
			{
				*glob_next_pos = pos_set(mypos.x-1,mypos.y,West);
				/*
				glob_next_pos->x 	= mypos.x-1;
				glob_next_pos->y 	= mypos.y;
				glob_next_pos->dir	= West;
				*/
				priority = tmp_priority;
			}
		}
	}

	return ((int)((4+glob_next_pos->dir - mypos.dir)%4));
}
