/*
 * make_path_expand.cpp
 *
 *  Created on: 2023/06/30
 *      Author: sato1
 */

#include "queue.h"
#include "make_map_class.h"
#include "make_path.h"
#include "typedef.h"
#include "math_utils.h"

#define DIR_TURN_NEWS_R90(x) ((x + 1 + 4) % 4)
#define DIR_TURN_NEWS_L90(x) ((x - 1 + 4) % 4)
#define DIR_TURN_NEWS_R45(x) ((x + 4 + 8) % 8)
#define DIR_TURN_NEWS_L45(x) (((x + 3) % 4 + 4) % 8)
#define DIR_TURN_NEWS_R135(x) (DIR_TURN_NEWS_R45(DIR_TURN_NEWS_R90(x)))
#define DIR_TURN_NEWS_L135(x) (DIR_TURN_NEWS_L45(DIR_TURN_NEWS_L90(x)))
#define DIR_TURN_NEWS_R180(x) (DIR_TURN_NEWS_R90(DIR_TURN_NEWS_R90(x)))
#define DIR_TURN_NEWS_L180(x) (DIR_TURN_NEWS_L90(DIR_TURN_NEWS_L90(x)))

#define DIR_TURN_DIAG_R45(x) ((x - 3 + 4)%4)
#define DIR_TURN_DIAG_L45(x) ((x - 4 + 4)%4)
#define DIR_TURN_DIAG_R90(x)  (DIR_TURN_NEWS_R45((DIR_TURN_DIAG_R45(x))))
#define DIR_TURN_DIAG_L90(x)  (DIR_TURN_NEWS_L45((DIR_TURN_DIAG_L45(x))))
#define DIR_TURN_DIAG_R135(x) (DIR_TURN_NEWS_R90((DIR_TURN_DIAG_R45(x))))
#define DIR_TURN_DIAG_L135(x) (DIR_TURN_NEWS_L90((DIR_TURN_DIAG_L45(x))))
#define DIR_TURN_DIAG_R180(x) (DIR_TURN_DIAG_R90((DIR_TURN_DIAG_R90(x))))
#define DIR_TURN_DIAG_L180(x) (DIR_TURN_DIAG_L90((DIR_TURN_DIAG_L90(x))))


t_posDijkstra Dijkstra::LocalPosDir2GlobWallPos_Center(t_posDijkstra glob_pos,t_direction glob_dir,t_local_dir LocalPos,t_local_dir LocalDir)
{
	t_posDijkstra return_glob_pos = glob_pos;
	t_direction return_glob_dir = Dir_None;
	if(glob_pos.NodePos == C_pos)
	{
		switch((int)glob_dir)
		{
			case North:
				return_glob_pos.y = (LocalPos == Front || LocalPos == Right_Front || LocalPos == Left_Front)
									? return_glob_pos.y + 1 : return_glob_pos.y;

				return_glob_pos.y = (LocalPos == Rear  || LocalPos == Right_Rear  || LocalPos == Left_Rear)
									? return_glob_pos.y - 1 : return_glob_pos.y;

				return_glob_pos.x = (LocalPos == Right || LocalPos == Right_Front || LocalPos == Right_Rear)
									? return_glob_pos.x + 1 : return_glob_pos.x;

				return_glob_pos.x = (LocalPos == Left  || LocalPos == Left_Front  || LocalPos == Left_Rear)
									? return_glob_pos.x - 1 : return_glob_pos.x;
				if(LocalDir != None) return_glob_dir   = (t_direction)((glob_dir+ (int)LocalDir) % 4);
				break;
			case East:
				return_glob_pos.x = (LocalPos == Front || LocalPos == Right_Front || LocalPos == Left_Front)
									? return_glob_pos.x + 1 : return_glob_pos.x;

				return_glob_pos.x = (LocalPos == Rear  || LocalPos == Right_Rear  || LocalPos == Left_Rear)
									? return_glob_pos.x - 1 : return_glob_pos.x;

				return_glob_pos.y = (LocalPos == Right || LocalPos == Right_Front || LocalPos == Right_Rear)
									? return_glob_pos.y - 1 : return_glob_pos.y;

				return_glob_pos.y = (LocalPos == Left  || LocalPos == Left_Front  || LocalPos == Left_Rear)
									? return_glob_pos.y + 1 : return_glob_pos.y;
				if(LocalDir != None) return_glob_dir   = (t_direction)((glob_dir+ (int)LocalDir) % 4);
				break;
			case South:
				return_glob_pos.y = (LocalPos == Front || LocalPos == Right_Front || LocalPos == Left_Front)
									? return_glob_pos.y - 1 : return_glob_pos.y;

				return_glob_pos.y = (LocalPos == Rear  || LocalPos == Right_Rear  || LocalPos == Left_Rear)
									? return_glob_pos.y + 1 : return_glob_pos.y;

				return_glob_pos.x = (LocalPos == Right || LocalPos == Right_Front || LocalPos == Right_Rear)
									? return_glob_pos.x - 1 : return_glob_pos.x;

				return_glob_pos.x = (LocalPos == Left  || LocalPos == Left_Front  || LocalPos == Left_Rear)
									? return_glob_pos.x + 1 : return_glob_pos.x;
				if(LocalDir != None) return_glob_dir   = (t_direction)((glob_dir+ (int)LocalDir) % 4);
				break;
			case West:
				return_glob_pos.x = (LocalPos == Front || LocalPos == Right_Front || LocalPos == Left_Front)
									? return_glob_pos.x - 1 : return_glob_pos.x;

				return_glob_pos.x = (LocalPos == Rear  || LocalPos == Right_Rear  || LocalPos == Left_Rear)
									? return_glob_pos.x + 1 : return_glob_pos.x;

				return_glob_pos.y = (LocalPos == Right || LocalPos == Right_Front || LocalPos == Right_Rear)
									? return_glob_pos.y + 1 : return_glob_pos.y;

				return_glob_pos.y = (LocalPos == Left  || LocalPos == Left_Front  || LocalPos == Left_Rear)
									? return_glob_pos.y - 1 : return_glob_pos.y;
				if(LocalDir != None) return_glob_dir   = (t_direction)((glob_dir+ (int)LocalDir) % 4);
				break;
			default:
				break;

		}
		return_glob_pos = conv_t_pos2t_posDijkstra(return_glob_pos.x,return_glob_pos.y, return_glob_dir);
	}
	return return_glob_pos;
}

t_posDijkstra Dijkstra::LocalPosDir2GlobWallPos_WPos(t_posDijkstra glob_pos,t_direction glob_dir,t_local_dir LocalDir)
{
	t_posDijkstra return_glob_pos = glob_pos;
	t_direction return_glob_dir = glob_dir;
	switch((int)glob_dir)
	{
		case NorthEast:
			if(glob_pos.NodePos == N_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x - 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = West;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = North;
						break;
					case None:
						break;
				}
			}
			else if(glob_pos.NodePos == E_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y - 1;
						return_glob_dir = East;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case None:
						break;
				}
			}
			break;

		case SouthEast:
			if(glob_pos.NodePos == N_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x - 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = West;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = North;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case None:
						break;
				}
			}
			else if(glob_pos.NodePos == E_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y - 1;
						return_glob_dir = East;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case None:
						break;
				}
			}

			break;

		case SouthWest:
			if(glob_pos.NodePos == N_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x - 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = West;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = North;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case None:
						break;
				}
			}
			else if(glob_pos.NodePos == E_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y - 1;
						return_glob_dir = East;
						break;
					case None:
						break;
				}
			}
			break;

		case NorthWest:
			if(glob_pos.NodePos == N_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = West;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = North;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x - 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case None:
						break;
				}
			}
			else if(glob_pos.NodePos == E_pos)
			{
				switch(LocalDir)
				{
					case Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Right_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 1;
						return_glob_dir = East;
						break;
					case Right:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = North;
						break;
					case Right_Rear:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = East;
						break;
					case Rear:
						return_glob_pos.x = return_glob_pos.x + 1;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Left_Rear:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y - 1;
						return_glob_dir = East;
						break;
					case Left:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = South;
						break;
					case Left_Front:
						return_glob_pos.x = return_glob_pos.x + 0;
						return_glob_pos.y = return_glob_pos.y + 0;
						return_glob_dir = West;
						break;
					case None:
						break;
				}
			}
			break;
		default:
			break;

	}
	if(glob_pos.NodePos == E_pos || glob_pos.NodePos == N_pos)
		return_glob_pos = conv_t_pos2t_posDijkstra(return_glob_pos.x,return_glob_pos.y, return_glob_dir);

	return return_glob_pos;
}

t_element* Dijkstra::get_closure_inf(t_posDijkstra position)
{
	switch(position.NodePos)
	{
		case N_pos:
			return &(closure[position.x][position.y].North);
		case C_pos:
			return &(closure[position.x][position.y].Center);
		case E_pos:
			return &(closure[position.x][position.y].East);
	}
	return nullptr;
}

uint8_t Dijkstra::get_wall_inf(t_posDijkstra position)
{
	int return_wall_inf = WALL;
	int mask = 0x03;
	switch(position.NodePos)
	{
		case N_pos:
			if((0 <= position.y && position.y < (MAZE_SIZE_Y - 1) )
			&& (0 <= position.x && position.x < (MAZE_SIZE_X    ) ) )
			{
				if((wall_property->wall[position.x][position.y].north & mask) == NOWALL)
				{
					return_wall_inf = NOWALL;
				}
			}
			break;
		case E_pos:
			if((0 <= position.y && position.y < (MAZE_SIZE_Y     ) )
			&& (0 <= position.x && position.x < (MAZE_SIZE_X - 1 ) ))
			{
				if((wall_property->wall[position.x][position.y].east & mask) == NOWALL)
				{
					return_wall_inf = NOWALL;
				}
			}
			break;
			break;
		case C_pos:
			break;
	}
	return return_wall_inf;
}

void Dijkstra::straight_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = m_dir;
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra next_pos = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, None);
	for(int i = 1;; i++)
	{
		int time  = (*get_closure_inf(pos)) .time + straight_time_set(SECTION*i);
		if(get_wall_inf(pos1) == NOWALL)
		{
				if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
				{
					(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Straight, False);
					#ifdef DEBUG_MODE
					printf("Straight_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
					HAL_Delay(10);
					#endif
				}
		}
		else
		{
			break;
		}
		pos1 = LocalPosDir2GlobWallPos_Center(next_pos, m_dir, Front, Rear);
		next_pos = LocalPosDir2GlobWallPos_Center(next_pos, m_dir, Front, None);
	}

}

void Dijkstra::diagonal_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = m_dir;
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
	t_posDijkstra next_pos= pos1;
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_WPos(next_pos, next_dir, Front);
	for(int i = 1;; i++)
	{
		int time = (*get_closure_inf(pos)) .time + diagonal_time_set(DIAG_SECTION*i);
		if(get_wall_inf(pos1) == NOWALL && get_wall_inf(pos2) == NOWALL)
		{
			if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
			{
				(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Diagonal, False);
				#ifdef DEBUG_MODE
				printf("Diagonal_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
				HAL_Delay(10);
				#endif
			}
		}
		else
		{
			break;
		}
		pos1 = pos2;
		next_pos = pos1;
		pos2 = LocalPosDir2GlobWallPos_WPos(next_pos, next_dir, Front);
	}
}

void Dijkstra::turn_inR45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R45((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Right);
	t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right_Front, Front);
	t_posDijkstra next_pos = pos2;
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL)
	{
		int time = return_turn_time(Turn_in_R45) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_in_R45, False);
			#ifdef DEBUG_MODE
			printf("turn_inR45_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}

}

void Dijkstra::turn_inL45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L45((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Left);
	t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left_Front, Front);
	t_posDijkstra next_pos = pos2;
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL)
	{
		int time = return_turn_time(Turn_in_L45) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_in_L45, False);
			#ifdef DEBUG_MODE
			printf("turn_inL45_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::turn_outR45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_R45((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
	t_posDijkstra next_pos = SetNodePos(pos.x, pos.y, C_pos);
	switch(m_dir)
	{
		//Epos
		case NorthWest:
			next_pos.x = pos.x + 0;
			next_pos.y = pos.y + 1;
			if(pos.NodePos != E_pos) return;
			break;
		case SouthEast:
			next_pos.x = pos.x + 1;
			next_pos.y = pos.y - 1;
			if(pos.NodePos != E_pos) return;
			break;
		//Npos
		case NorthEast:
			next_pos.y = pos.y + 1;
			next_pos.x = pos.x + 1;
			if(pos.NodePos != N_pos) return;
			break;
		case SouthWest:
			next_pos.y = pos.y + 0;
			next_pos.x = pos.x - 1 ;
			if(pos.NodePos != N_pos) return;
			break;
		default:
			break;
	}
 	next_pos = SetNodePos(next_pos.x, next_pos.y, C_pos);
	if(get_wall_inf(pos1) == NOWALL)
	{
		int time = return_turn_time(Turn_out_R45) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_out_R45, False);
			#ifdef DEBUG_MODE
			printf("turn_outR45_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::turn_outL45_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_L45((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
	t_posDijkstra next_pos = SetNodePos(pos.x, pos.y, C_pos);
	switch(m_dir)
	{
		//N_pos
		case NorthWest:
			next_pos.x = pos.x - 1;
			next_pos.y = pos.y + 1;
			if(pos.NodePos != N_pos) return;
			break;
		case SouthEast:
			next_pos.x = pos.x + 1;
			next_pos.y = pos.y + 0;
			if(pos.NodePos != N_pos) return;
			break;
		//E_pos
		case NorthEast:
			next_pos.y = pos.y + 1;
			next_pos.x = pos.x + 1;
			if(pos.NodePos != E_pos) return;
			break;
		case SouthWest:
			next_pos.y = pos.y - 1;
			next_pos.x = pos.x + 0;
			if(pos.NodePos != E_pos) return;
			break;
		default:
			return;
			break;
	}
 	next_pos = SetNodePos(next_pos.x, next_pos.y, C_pos);
	if(get_wall_inf(pos1) == NOWALL )
	{
		int time = return_turn_time(Turn_out_L45) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_out_L45, False);
			#ifdef DEBUG_MODE
			printf("turn_outL45_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::turn_inR135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R135((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Right);
	t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right_Front, Rear);
	t_posDijkstra pos4 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right, Right);
	t_posDijkstra next_pos = pos3;
	#ifdef DEBUG_MODE
	printf("turn_inR135_expand_check->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
	printf("turn_inR135_expand_check->1:%2d,2:%2d,3:%2d\n",get_wall_inf(pos1),get_wall_inf(pos2),get_wall_inf(pos3));
	HAL_Delay(10);
	#endif
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL && get_wall_inf(pos4) == NOWALL)
	{
		#ifdef DEBUG_MODE
		printf("turn_inR135_expand->wallOK\n");
		HAL_Delay(10);
		#endif
		uint16_t time = return_turn_time(Turn_in_R135) + (*get_closure_inf(pos)).time;
		#ifdef DEBUG_MODE
		printf("time:%d,next_det:%d,nex_time:%d\n",time,(int)((*get_closure_inf(pos)) .determine),(int)((*get_closure_inf(pos)) .time));
		printf("time:%d,next_det:%d,nex_time:%d\n",time,(int)((*get_closure_inf(next_pos)).determine),(int)((*get_closure_inf(next_pos)) .time));
		printf("time:%d,next_det:%d,nex_time:%d\n",time,(int)(closure[1][0].North.determine),(int)(closure[1][0].North.time));
		HAL_Delay(10);
		#endif
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{

			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_in_R135, False);
			#ifdef DEBUG_MODE
			printf("turn_inR135_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::turn_inL135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L135((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Left);
	t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left_Front, Rear);
	t_posDijkstra pos4 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left, Left);
	t_posDijkstra next_pos = pos3;
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL && get_wall_inf(pos4) == NOWALL)
	{
		int time = return_turn_time(Turn_in_L135) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_in_L135, False);
			#ifdef DEBUG_MODE
			printf("turn_inL135_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::turn_outR135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_R135((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Right_Front);
	t_posDijkstra next_pos = SetNodePos(pos.x, pos.y, C_pos);
	switch(m_dir)
	{
		//Epos
		case NorthWest:
			next_pos.x = pos.x + 1;
			next_pos.y = pos.y + 1;
			if(pos.NodePos != E_pos) return;
			break;
		case SouthEast:
			next_pos.x = pos.x + 0;
			next_pos.y = pos.y - 1;
			if(pos.NodePos != E_pos) return;
			break;
		//Npos
		case NorthEast:
			next_pos.y = pos.y + 0;
			next_pos.x = pos.x + 1;
			if(pos.NodePos != N_pos) return;
			break;
		case SouthWest:
			next_pos.y = pos.y + 1;
			next_pos.x = pos.x - 1 ;
			if(pos.NodePos != N_pos) return;
			break;
		default:
			return;
			break;
	}
 	next_pos = SetNodePos(next_pos.x, next_pos.y, C_pos);
	if(get_wall_inf(pos1) == NOWALL && get_wall_inf(pos2) == NOWALL)
	{
		int time = return_turn_time(Turn_out_R135) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_out_R135, False);
			#ifdef DEBUG_MODE
			printf("turn_outR135_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}

}

void Dijkstra::turn_outL135_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_L135((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Left_Front);
	t_posDijkstra next_pos = SetNodePos(pos.x, pos.y, C_pos);
	switch(m_dir)
	{
		//N_pos
		case NorthWest:
			next_pos.x = pos.x - 1;
			next_pos.y = pos.y + 0;
			if(pos.NodePos != N_pos) return;
			break;
		case SouthEast:
			next_pos.x = pos.x + 1;
			next_pos.y = pos.y + 1;
			if(pos.NodePos != N_pos) return;
			break;
		//E_pos
		case NorthEast:
			next_pos.y = pos.y + 1;
			next_pos.x = pos.x + 0;
			if(pos.NodePos != E_pos) return;
			break;
		case SouthWest:
			next_pos.y = pos.y - 1;
			next_pos.x = pos.x + 1;
			if(pos.NodePos != E_pos) return;
			break;
		default:
			return;
			break;
	}
 	next_pos = SetNodePos(next_pos.x, next_pos.y, C_pos);
	if(get_wall_inf(pos1) == NOWALL && get_wall_inf(pos2) == NOWALL )
	{
		int time = return_turn_time(Turn_out_L135)  + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_out_L135, False);
			#ifdef DEBUG_MODE
			printf("turn_outL135_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::longturn_R90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R90((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Right);
	//t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right_Front, Right);
	//t_posDijkstra next_pos = SetNodePos(pos3.x,pos3.y,C_pos);
	t_posDijkstra next_pos = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right_Front, None);
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL)
	{
		int time = return_turn_time(Long_turnR90) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Long_turnR90, False);
			#ifdef DEBUG_MODE
			printf("longturn_R90_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::longturn_L90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L90((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Left);
	//t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left_Front, Left);
	//t_posDijkstra next_pos = SetNodePos(pos3.x,pos3.y,C_pos);
	t_posDijkstra next_pos = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left_Front, None);
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL)
	{
		int time = return_turn_time(Long_turnL180) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Long_turnL90, False);
			#ifdef DEBUG_MODE
			printf("longturn_L90_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::longturn_R180_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_R180((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Right);
	t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right_Front, Rear);
	//t_posDijkstra pos4 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right, Rear);
	//t_posDijkstra next_pos = SetNodePos(pos4.x,pos4.y,C_pos);
	t_posDijkstra next_pos = LocalPosDir2GlobWallPos_Center(pos, m_dir, Right, None);
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL)
	{
		int time = return_turn_time(Long_turnR180) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Long_turnR180, False);
			#ifdef DEBUG_MODE
			printf("longturn_R180_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::longturn_L180_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_NEWS_L180((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Left);
	t_posDijkstra pos3 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left_Front, Rear);
	//t_posDijkstra pos4 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left, None);
	t_posDijkstra next_pos = LocalPosDir2GlobWallPos_Center(pos, m_dir, Left, None);
	if(get_wall_inf(pos1) == NOWALL  && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL)
	{
		int time = return_turn_time(Long_turnL180) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Long_turnL180, False);
			#ifdef DEBUG_MODE
			printf("longturn_L180_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::turn_vR90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_R90((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Right_Front);
	t_posDijkstra next_pos = pos2;
	t_posDijkstra pos3 =  LocalPosDir2GlobWallPos_WPos(next_pos,next_dir,Front);
	switch(m_dir)
	{
		//E_pos
		case NorthWest:
		case SouthEast:
			if(pos.NodePos != E_pos) return;
			break;
		//N_pos
		case NorthEast:
		case SouthWest:
			if(pos.NodePos != N_pos) return;
			break;
		default:
			return;
			break;
	}
	if(get_wall_inf(pos1) == NOWALL && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL)
	{
		int time = return_turn_time(Turn_RV90) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_RV90, False);
			#ifdef DEBUG_MODE
			printf("turn_vR90_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}

void Dijkstra::turn_vL90_expand(t_posDijkstra pos,t_direction m_dir)
{
	t_direction next_dir = (t_direction)DIR_TURN_DIAG_L90((int)m_dir);
	t_posDijkstra pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
	t_posDijkstra pos2 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Left_Front);
	t_posDijkstra next_pos = pos2;
	t_posDijkstra pos3 =  LocalPosDir2GlobWallPos_WPos(next_pos,next_dir,Front);
	switch(m_dir)
	{
		//N_pos
		case NorthWest:
		case SouthEast:
			if(pos.NodePos != N_pos) return;
			break;
		//E_pos
		case NorthEast:
		case SouthWest:
			if(pos.NodePos != E_pos) return;
			break;
		default:
			return;
			break;
	}

	if(get_wall_inf(pos1) == NOWALL && get_wall_inf(pos2) == NOWALL && get_wall_inf(pos3) == NOWALL)
	{
		int time = return_turn_time(Turn_LV90) + (*get_closure_inf(pos)) .time;
		if((*get_closure_inf(next_pos)) .determine == False && (*get_closure_inf(next_pos)) .time >= time)
		{
			(*get_closure_inf(next_pos)) = SetNode(pos, time, next_dir, Turn_LV90, False);
			#ifdef DEBUG_MODE
			printf("turn_vL90_expand_Set->x:%2d,y:%2d,d:%2d\n",next_pos.x,next_pos.y,next_pos.NodePos);
			HAL_Delay(10);
			#endif
		}
	}
}
