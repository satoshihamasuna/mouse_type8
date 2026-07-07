/*
 * make_map.c
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */


#include "queue.h"
#include "make_map_class.h"
#include "typedef.h"
#include "maze_typedef.h"
#include "math_utils.h"



t_MapNode node_set(int16_t st_x,int16_t st_y,int16_t par_x,int16_t par_y,int16_t cost,int16_t cost_h){
	t_MapNode n;
	n.st_x = st_x;		n.st_y = st_y;
	n.par_x = par_x;	n.par_y = par_y;
	n.cost = cost;		n.cost_h = cost_h;
	return n;
}

make_map::make_map(wall_class *wall_property_,ring_queue<1024,t_MapNode> *maze_q_)
{
	wall_property = wall_property_;
	maze_q = maze_q_;
}

void make_map::init_map(int x, int y,int goal_size){
	for( int i = 0; i < MAZE_SIZE_X ; i++ ){
		for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			map[i][j] = MAZE_SIZE;


		}
	}

	for(int i = 0;i < goal_size;i++){
		for(int j = 0;j < goal_size;j++){
			map[x+i][y+j] = 0;
		}
	}
}


void make_map::expand(t_MapNode n,int mask){

	int16_t par_x = n.st_x;
	int16_t par_y = n.st_y;


	if(n.st_y < MAZE_SIZE_Y-1)					//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
		{
			if(map[n.st_x][n.st_y+1] == MAZE_SIZE)			//まだ値が入っていなければ
			{
				map[n.st_x][n.st_y+1] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x,n.st_y+1,par_x,par_y,map[n.st_x][n.st_y+1],0));
			}
		}
	}

	if(n.st_x < MAZE_SIZE_X-1)					//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].east & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x+1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x+1][n.st_y] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x+1,n.st_y,par_x,par_y,map[n.st_x+1][n.st_y],0));
			}
		}
	}

	if(n.st_y > 0)						//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].south & mask) == NOWALL)	//壁がなければ
		{
			if(map[n.st_x][n.st_y-1] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x][n.st_y-1] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x,n.st_y-1,par_x,par_y,map[n.st_x][n.st_y-1],0));
			}
		}
	}

	if(n.st_x > 0)						//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].west & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x-1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x-1][n.st_y] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x-1,n.st_y,par_x,par_y,map[n.st_x-1][n.st_y],0));
			}

		}
	}

}


void make_map::expand_and_closeWall(t_MapNode n,int mask){
	int16_t par_x = n.st_x;
	int16_t par_y = n.st_y;

	t_bool CloseWallFlag = True;

	if(n.st_y < MAZE_SIZE_Y-1)					//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
		{
			if(map[n.st_x][n.st_y+1] == MAZE_SIZE)			//まだ値が入っていなければ
			{
				map[n.st_x][n.st_y+1] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x,n.st_y+1,par_x,par_y,map[n.st_x][n.st_y+1],0));
				CloseWallFlag = False;
			}
			else if (!(n.st_x == n.par_x &&  n.st_y+1 == n.par_y))
			{
				CloseWallFlag = False;
			}
		}
	}

	if(n.st_x < MAZE_SIZE_X-1)					//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].east & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x+1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x+1][n.st_y] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x+1,n.st_y,par_x,par_y,map[n.st_x+1][n.st_y],0));
				CloseWallFlag = False;
			}
			else if (!(n.st_x+1 == n.par_x &&  n.st_y == n.par_y))
			{
				CloseWallFlag = False;
			}
		}
	}

	if(n.st_y > 0)						//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].south & mask) == NOWALL)	//壁がなければ
		{
			if(map[n.st_x][n.st_y-1] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x][n.st_y-1] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x,n.st_y-1,par_x,par_y,map[n.st_x][n.st_y-1],0));
				CloseWallFlag = False;
			}
			else if (!(n.st_x == n.par_x &&  n.st_y-1 == n.par_y))
			{
				CloseWallFlag = False;
			}
		}
	}

	if(n.st_x > 0)						//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].west & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x-1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x-1][n.st_y] = n.cost + 1;	//値を代入
				maze_q->push(node_set(n.st_x-1,n.st_y,par_x,par_y,map[n.st_x-1][n.st_y],0));
				CloseWallFlag = False;
			}
			else if (!(n.st_x-1 == n.par_x &&  n.st_y == n.par_y))
			{
				CloseWallFlag = False;
			}

		}
	}

	if(CloseWallFlag)
	{
		if(n.st_y < MAZE_SIZE_Y-1)
		{
			if( wall_property->wall[n.st_x][n.st_y].north == UNKNOWN)		//壁がなければ
			{
				if(n.st_x == n.par_x &&  n.st_y+1 == n.par_y)
				{
					wall_property->wall[n.st_x][n.st_y].north   = VWALL;
					wall_property->wall[n.st_x][n.st_y+1].south = VWALL;
				}
			}
		}
		if(n.st_x < MAZE_SIZE_X-1)
		{
			if( wall_property->wall[n.st_x][n.st_y].east == UNKNOWN)		//壁がなければ
			{
				if(n.st_x+1 == n.par_x &&  n.st_y == n.par_y)
				{
					wall_property->wall[n.st_x][n.st_y].east   = VWALL;
					wall_property->wall[n.st_x+1][n.st_y].west = VWALL;
				}
			}
		}
		if(n.st_y > 0)
		{
			if( wall_property->wall[n.st_x][n.st_y].south == UNKNOWN)	//壁がなければ
			{
				if(n.st_x == n.par_x &&  n.st_y-1 == n.par_y)
				{
					wall_property->wall[n.st_x][n.st_y].south   = VWALL;
					wall_property->wall[n.st_x][n.st_y-1].north = VWALL;
				}
			}
		}
		if(n.st_x > 0)
		{
			if( wall_property->wall[n.st_x][n.st_y].west == UNKNOWN)
			{
				if(n.st_x-1 == n.par_x &&  n.st_y == n.par_y)
				{
					wall_property->wall[n.st_x][n.st_y].west   = VWALL;
					wall_property->wall[n.st_x-1][n.st_y].east = VWALL;
				}
			}
		}
	}

}



void make_map::make_map_queue_zenmen(int x, int y,t_position expand_end,int size,int mask)
{
	//queueの初期化
		maze_q->queue_reset();

		for( int i = 0; i < MAZE_SIZE_X ; i++ ){
			for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
				if(wall_property->is_unknown(i, j) == True)
				{
					map[i][j] = 0;
					maze_q->push(node_set(i,j,-1,-1,0,0));
				}
				else
				{
					map[i][j] = MAZE_SIZE;
				}
			}
		}

	    t_MapNode n;
		while(maze_q->queue_length() != 0){
			n = maze_q->pop();
			expand(n,mask);
			if(expand_end.x == n.st_x && expand_end.y == n.st_y)
				break;
		}

		if(map[expand_end.x][expand_end.y] == MAZE_SIZE)
		{
			make_map_queue(x, y, expand_end, size, mask);
		}
}

void make_map::make_map_queue(int x, int y,t_position expand_end,int size,int mask)
{
	//mapの初期化
		init_map(x,y,size);
	//queueの初期化
		maze_q->queue_reset();

		for(int i = 0;i < size;i++){
			for(int j = 0;j < size;j++){
				maze_q->push(node_set(x+i,y+j,-1,-1,0,0));
			}
		}
	    t_MapNode n;
		while(maze_q->queue_length() != 0){
			n = maze_q->pop();
			expand(n,mask);
			if(expand_end.x == n.st_x && expand_end.y == n.st_y)
				break;

		}
}


void make_map::make_map_queue_closeWall(int x, int y,int size,int mask)
{
		//mapの初期化
		init_map(x,y,size);
		//queueの初期化
		maze_q->queue_reset();

		for(int i = 0;i < size;i++){
			for(int j = 0;j < size;j++){
				maze_q->push(node_set(x+i,y+j,-1,-1,0,0));
			}
		}

	    t_MapNode n;
		while(maze_q->queue_length() != 0){
			n = maze_q->pop();
			expand_and_closeWall(n,mask);
		}
}

void make_map::Display()
{
	for( int y = MAZE_SIZE_Y - 1 ; y >= 0 ; y-- ){
		for(int x = 0; x < MAZE_SIZE_X ; x++ ){
			if(wall_property->wall[x][y].north == WALL)// || wall_property->wall[x][y].north == VWALL)
			{	printf("+---");	HAL_Delay(10);	}
			else if(wall_property->wall[x][y].north == VWALL)
			{
				printf("\x1b[31m");
				printf("+---");	HAL_Delay(10);
				printf("\x1b[39m");
			}
			else							{	printf("+   "); HAL_Delay(10);	}
			//if(x == MAZE_SIZE_X - 1)		{	printf("+\n");	HAL_Delay(5);	}
		}
		printf("+\n");	HAL_Delay(10);

		for(int x = 0; x < MAZE_SIZE_X ; x++ ){
			if(wall_property->wall[x][y].west == WALL) //|| wall_property->wall[x][y].west == VWALL)
			{	printf("|%3x",map[x][y]);	HAL_Delay(10);	}
			else if(wall_property->wall[x][y].west == VWALL)
			{
				printf("\x1b[31m");
				printf("|%3x",map[x][y]);	HAL_Delay(10);
				printf("\x1b[39m");
			}
			else							{	printf(" %3x",map[x][y]);	HAL_Delay(10);	}
			//if(x == MAZE_SIZE_X - 1)		{	printf("|\n");				HAL_Delay(5);	}
		}
		printf("|\n");				HAL_Delay(5);
	}
	for(int x = 0; x < MAZE_SIZE_X ; x++)	{	printf("+---"); HAL_Delay(5);	}	printf("+\n");
}
