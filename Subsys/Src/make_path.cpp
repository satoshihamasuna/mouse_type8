/*
 * make_path.cpp
 *
 *  Created on: 2023/06/26
 *      Author: sato1
 */

#include "peripheral.h"
#include "typedef.h"
#include "math_utils.h"

#include "../../Component/Inc/controller.h"

#include "../../Task/Inc/run_typedef.h"
#include "../../Task/Inc/ctrl_task.h"

#include "../../Module/Inc/log_data.h"

#include "../Inc/make_path.h"

#define DIJKSTRA_MAX_TIME (65535-1)

#define DIJKSTRA_NODE_NUM (MAZE_SIZE_X * MAZE_SIZE_Y * 3 * 4)

static uint16_t dijkstra_open_heap[DIJKSTRA_NODE_NUM];
static int16_t dijkstra_heap_index[DIJKSTRA_NODE_NUM];
static int16_t dijkstra_heap_tail = -1;

static t_posDijkstra dijkstra_id_to_pos(uint16_t id)
{
	t_posDijkstra pos;
	pos.state_dir = id % 4;
	id /= 4;
	uint16_t cell = id / 3;
	pos.x = cell / MAZE_SIZE_Y;
	pos.y = cell % MAZE_SIZE_Y;
	pos.NodePos = (t_DijkstraWallPos)(id % 3);
	return pos;
}

static void dijkstra_open_queue_init()
{
	dijkstra_heap_tail = -1;
	for(int i = 0;i < DIJKSTRA_NODE_NUM;i++)
	{
		dijkstra_heap_index[i] = -1;
	}
}


void log_enable()
{
	  LogData::getInstance().data_count = 0;
	  LogData::getInstance().log_enable = True;
}

void log_disable()
{
	  LogData::getInstance().data_count = 0;
	  LogData::getInstance().log_enable = False;
}

t_posDijkstra Dijkstra::conv_t_pos2t_posDijkstra(t_position pos,t_direction wall_pos)
{
	t_posDijkstra position;
	switch(wall_pos)
	{
		case North:
			position = SetNodePos(pos.x,pos.y,N_pos);
			break;
		case East:
			position = SetNodePos(pos.x,pos.y,E_pos);
			break;
		case South:
			position = SetNodePos(pos.x,pos.y-1,N_pos);
			break;
		case West :
			position = SetNodePos(pos.x-1,pos.y,E_pos);
			break;
		case Dir_None:
		default:
			position = SetNodePos(pos.x,pos.y,C_pos);
			break;
	}
	position.state_dir = (((uint8_t)pos.dir) >> 1) & 0x03;
	return position;
}


t_posDijkstra Dijkstra::conv_t_pos2t_posDijkstra(int _x,int _y,t_direction wall_pos)
{
	t_posDijkstra position;
	switch(wall_pos)
	{
		case North:
			position = SetNodePos(_x,_y,N_pos);
			break;
		case East:
			position = SetNodePos(_x,_y,E_pos);
			break;
		case South:
			position = SetNodePos(_x,_y-1,N_pos);
			break;
		case West :
			position = SetNodePos(_x-1,_y,E_pos);
			break;
		case Dir_None:
		default:
			position = SetNodePos(_x,_y,C_pos);
			break;
	}
	return position;
}

t_posDijkstra Dijkstra::SetNodePos(uint8_t _x,uint8_t _y,t_DijkstraWallPos _dpos, t_direction _dir)
{
	t_posDijkstra pos;
	pos.x = _x;
	pos.y = _y;
	pos.NodePos = _dpos;
	pos.state_dir = (((uint8_t)_dir) >> 1) & 0x03;
	return pos;
}

t_element Dijkstra::SetNode(t_posDijkstra _parent,	uint16_t _time,		t_direction _dir
				 ,t_run_pattern _run_pt,		t_bool _determine)
{
	t_element node;
	node.parent_pos = _parent;
	node.time		= _time;
	node.dir		= _dir;
	node.run_pt		= _run_pt;
	node.determine  = _determine;
	return node;
}

void Dijkstra::init_dijkstra_map()
{
	for(int i = 0;i < MAZE_SIZE_X;i++)
	{
		for(int j = 0;j < MAZE_SIZE_Y;j++)
		{
			for(int d = 0; d < 3;d++)
			{
				for(int state = 0; state < 4; state++) switch(d)
				{
					case C_pos:
						closure[i][j].Center[state] = SetNode(SetNodePos(i,j,C_pos,(t_direction)(state*2)), DIJKSTRA_MAX_TIME, Dir_None, No_run, False);
						break;
					case N_pos:
						closure[i][j].North[state] = SetNode(SetNodePos(i,j,N_pos,(t_direction)(state*2+1)), DIJKSTRA_MAX_TIME, Dir_None, No_run, False);
						break;
					case E_pos:
						closure[i][j].East[state] = SetNode(SetNodePos(i,j,E_pos,(t_direction)(state*2+1)), DIJKSTRA_MAX_TIME, Dir_None, No_run, False);
						break;
				}
			}
		}
	}
}

void Dijkstra::start_node_setUp(t_posDijkstra start_pos,t_direction dir)
{
	start_pos.state_dir = (((uint8_t)dir) >> 1) & 0x03;
	switch(start_pos.NodePos)
	{
		case C_pos:
			closure[start_pos.x][start_pos.y].Center[start_pos.state_dir] = SetNode(start_pos, 0, dir, No_run, False);
			break;
		case N_pos:
			closure[start_pos.x][start_pos.y].North[start_pos.state_dir] = SetNode(start_pos, 0, dir, No_run, False);
			break;
		case E_pos:
			closure[start_pos.x][start_pos.y].East[start_pos.state_dir] = SetNode(start_pos, 0, dir, No_run, False);
			break;
	}
}

t_bool Dijkstra::is_goal_Dijkstra(t_posDijkstra check_pos,t_position goal_pos,uint8_t goal_size)
{
	if(goal_pos.x <= check_pos.x  && check_pos.x < (goal_pos.x + goal_size))
	{
		if(goal_pos.y <= check_pos.y  && check_pos.y < (goal_pos.y + goal_size))
		{
			return True;
		}
	}

	if(check_pos.x == (goal_pos.x - 1) && check_pos.NodePos == E_pos)
	{
		if(goal_pos.y <= check_pos.y  && check_pos.y < (goal_pos.y + goal_size))
		{
			return True;
		}
	}


	if(check_pos.y == (goal_pos.y - 1) && check_pos.NodePos == N_pos)
	{
		if(goal_pos.x <= check_pos.x  && check_pos.x < (goal_pos.x + goal_size))
		{
			return True;
		}
	}
	return False;
}

void Dijkstra::set_determine(t_posDijkstra set_pos)
{
	get_closure_inf(set_pos)->determine = True;
}

uint16_t Dijkstra::dijkstra_node_order(t_posDijkstra pos)
{
	return (((uint16_t)pos.x * MAZE_SIZE_Y + (uint16_t)pos.y) * 3 + (uint16_t)pos.NodePos) * 4 + pos.state_dir;
}

static uint16_t dijkstra_node_time(Dijkstra *dijkstra, uint16_t id)
{
	t_posDijkstra pos = dijkstra_id_to_pos(id);
	switch(pos.NodePos)
	{
		case N_pos:
			return dijkstra->closure[pos.x][pos.y].North[pos.state_dir].time;
		case C_pos:
			return dijkstra->closure[pos.x][pos.y].Center[pos.state_dir].time;
		case E_pos:
			return dijkstra->closure[pos.x][pos.y].East[pos.state_dir].time;
		default:
			return DIJKSTRA_MAX_TIME;
	}
}

static t_bool dijkstra_heap_less(Dijkstra *dijkstra, uint16_t a, uint16_t b)
{
	uint16_t a_time = dijkstra_node_time(dijkstra, a);
	uint16_t b_time = dijkstra_node_time(dijkstra, b);
	if(a_time != b_time)
	{
		return (a_time < b_time) ? True : False;
	}
	return (a < b) ? True : False;
}

static void dijkstra_heap_swap(int16_t a, int16_t b)
{
	uint16_t temp = dijkstra_open_heap[a];
	dijkstra_open_heap[a] = dijkstra_open_heap[b];
	dijkstra_open_heap[b] = temp;
	dijkstra_heap_index[dijkstra_open_heap[a]] = a;
	dijkstra_heap_index[dijkstra_open_heap[b]] = b;
}

static void dijkstra_heap_sift_up(Dijkstra *dijkstra, int16_t index)
{
	while(index > 0)
	{
		int16_t parent = (index - 1) / 2;
		if(dijkstra_heap_less(dijkstra, dijkstra_open_heap[index], dijkstra_open_heap[parent]) == True)
		{
			dijkstra_heap_swap(index, parent);
			index = parent;
		}
		else
		{
			break;
		}
	}
}

static void dijkstra_heap_sift_down(Dijkstra *dijkstra, int16_t index)
{
	while(1)
	{
		int16_t left = 2 * index + 1;
		int16_t right = 2 * index + 2;
		int16_t best = index;

		if(left <= dijkstra_heap_tail
		&& dijkstra_heap_less(dijkstra, dijkstra_open_heap[left], dijkstra_open_heap[best]) == True)
		{
			best = left;
		}
		if(right <= dijkstra_heap_tail
		&& dijkstra_heap_less(dijkstra, dijkstra_open_heap[right], dijkstra_open_heap[best]) == True)
		{
			best = right;
		}

		if(best == index)
		{
			break;
		}
		dijkstra_heap_swap(index, best);
		index = best;
	}
}

void Dijkstra::push_open_node(t_posDijkstra pos)
{
	uint16_t id = dijkstra_node_order(pos);
	if((*get_closure_inf(pos)).determine == True)
	{
		return;
	}

	if(dijkstra_heap_index[id] >= 0)
	{
		dijkstra_heap_sift_up(this, dijkstra_heap_index[id]);
		return;
	}
	if(dijkstra_heap_tail + 1 >= DIJKSTRA_NODE_NUM)
	{
		return;
	}

	dijkstra_heap_tail++;
	dijkstra_open_heap[dijkstra_heap_tail] = id;
	dijkstra_heap_index[id] = dijkstra_heap_tail;
	dijkstra_heap_sift_up(this, dijkstra_heap_tail);
}

t_bool Dijkstra::pop_open_node(t_posDijkstra *pos)
{
	if(dijkstra_heap_tail < 0)
	{
		return False;
	}

	uint16_t id = dijkstra_open_heap[0];
	dijkstra_heap_index[id] = -1;
	dijkstra_open_heap[0] = dijkstra_open_heap[dijkstra_heap_tail];
	dijkstra_heap_tail--;
	if(dijkstra_heap_tail >= 0)
	{
		dijkstra_heap_index[dijkstra_open_heap[0]] = 0;
		dijkstra_heap_sift_down(this, 0);
	}

	*pos = dijkstra_id_to_pos(id);
	return True;
}

t_posDijkstra Dijkstra::min_search()
{
	t_posDijkstra min_pos = SetNodePos(0, 0, C_pos);
	uint16_t min_time = DIJKSTRA_MAX_TIME;
	for(int i = 0;i < MAZE_SIZE_X;i++)
	{
		for(int j = 0;j < MAZE_SIZE_Y;j++)
		{
			for(int state = 0; state < 4; state++)
			{
				for(int node = 0; node < 3; node++)
				{
					t_direction dir = (t_direction)(state * 2 + (node == C_pos ? 0 : 1));
					t_posDijkstra candidate = SetNodePos(i, j, (t_DijkstraWallPos)node, dir);
					t_element *element = get_closure_inf(candidate);
					if(element->time < min_time && element->determine == False)
					{
						min_pos = candidate;
						min_time = element->time;
					}
				}
			}
		}
	}
	return min_pos;
}

t_posDijkstra Dijkstra::make_path_Dijkstra_priority_queue(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size)
{
	t_posDijkstra min_pos = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	init_dijkstra_map();
	dijkstra_open_queue_init();
	use_priority_queue = True;

	start_node_setUp(min_pos, start_pos.dir);
	push_open_node(min_pos);

	for(int i = 0; i < DIJKSTRA_NODE_NUM; i++)
	{
		if(pop_open_node(&min_pos) == False)
		{
			break;
		}

		#ifdef DEBUG_MODE
		printf("minimum->%d,%d,%d\n",min_pos.x,min_pos.y,min_pos.NodePos);
		#endif
		set_determine(min_pos);

		if(is_goal_Dijkstra(min_pos, goal_pos, goal_size))
		{
			t_direction pos_dir = (*get_closure_inf(min_pos)).dir;
			min_pos = last_expand(min_pos,pos_dir ,goal_pos, goal_size);
			break;
		}
		expand(min_pos);
	}

	use_priority_queue = False;
	return min_pos;
}

t_posDijkstra Dijkstra::make_path_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size)
{
	t_posDijkstra min_pos;
	init_dijkstra_map();
	start_node_setUp(conv_t_pos2t_posDijkstra(start_pos, start_wallPos), start_pos.dir);
	use_priority_queue = False;
	for(int i = 0; i < DIJKSTRA_NODE_NUM;i++)
	{
		min_pos = min_search();
		//set_determine
		#ifdef DEBUG_MODE
		printf("minimum->%d,%d,%d\n",min_pos.x,min_pos.y,min_pos.NodePos);
		#endif
		set_determine(min_pos);

		if(is_goal_Dijkstra(min_pos, goal_pos, goal_size))
		{
			//last_expand(min_pos,goal_pos,(int)goal_size);
			t_direction pos_dir = (*get_closure_inf(min_pos)).dir;
			min_pos = last_expand(min_pos,pos_dir ,goal_pos, goal_size);
			break;
		}
		expand(min_pos);
	}
	return min_pos;
}

void Dijkstra::expand(t_posDijkstra pos)
{
	t_direction pos_dir = (*get_closure_inf(pos)).dir;
	switch(pos.NodePos)
	{
		case N_pos:
		case E_pos:
			diagonal_expand(pos,pos_dir);
			turn_outR45_expand(pos,pos_dir);
			turn_outL45_expand(pos,pos_dir);
			turn_outR135_expand(pos,pos_dir);
			turn_outL135_expand(pos,pos_dir);
			turn_vR90_expand(pos,pos_dir);
			turn_vL90_expand(pos,pos_dir);
			break;
		case C_pos:
			straight_expand(pos, pos_dir);
			turn_inR135_expand(pos, pos_dir);
			turn_inL135_expand(pos, pos_dir);
			turn_inR45_expand(pos, pos_dir);
			turn_inL45_expand(pos, pos_dir);
			//turn_inR45_expand(pos, pos_dir);
			//turn_inL45_expand(pos, pos_dir);
			longturn_R90_expand(pos, pos_dir);
			longturn_L90_expand(pos, pos_dir);
			longturn_R180_expand(pos, pos_dir);
			longturn_L180_expand(pos, pos_dir);
			break;
	}
}

t_posDijkstra Dijkstra::last_expand(t_posDijkstra pos,t_direction m_dir,t_position goal_pos,uint8_t goal_size)
{
	t_posDijkstra  last_pos = pos;
	t_direction next_dir = m_dir;
	t_posDijkstra pos1 = pos;
	t_posDijkstra pos2 = pos;
	t_posDijkstra next_pos = pos;
	switch(last_pos.NodePos)
	{
		case N_pos:
		case E_pos:
			pos1 = LocalPosDir2GlobWallPos_WPos(pos, m_dir, Front);
			next_pos= pos1;
			pos2 = LocalPosDir2GlobWallPos_WPos(next_pos, next_dir, Front);
			for(int i = 1;; i++)
			{
				int time = (*get_closure_inf(pos)) .time + diagonal_time_set(DIAG_SECTION*i);
				if(get_wall_inf(pos1) == NOWALL && get_wall_inf(pos2) == NOWALL && is_goal_Dijkstra(next_pos, goal_pos, goal_size) == True)
				{
					next_pos.state_dir = (((uint8_t)next_dir) >> 1) & 0x03;
					if((*get_closure_inf(next_pos)) .determine == False )//&& (*get_closure_inf(next_pos)) .time >= time)
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
				last_pos = next_pos;
				pos1 = pos2;
				next_pos = pos1;
				pos2 = LocalPosDir2GlobWallPos_WPos(next_pos, next_dir, Front);
			}
			break;
		case C_pos:
			pos1 = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, Rear);
			next_pos = LocalPosDir2GlobWallPos_Center(pos, m_dir, Front, None);
			for(int i = 1;; i++)
			{
				int time  = (*get_closure_inf(pos)) .time + straight_time_set(SECTION*i);
				if(get_wall_inf(pos1) == NOWALL && is_goal_Dijkstra(next_pos, goal_pos, goal_size) == True)
				{
						next_pos.state_dir = (((uint8_t)next_dir) >> 1) & 0x03;
						if((*get_closure_inf(next_pos)) .determine == False )
						{
							/*
							if((*get_closure_inf(pos)).run_pt == Straight)
							{
								parent = (*get_closure_inf(pos)).parent_pos;
							}
							*/
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
				last_pos = next_pos;
				pos1 = LocalPosDir2GlobWallPos_Center(next_pos, m_dir, Front, Rear);
				next_pos = LocalPosDir2GlobWallPos_Center(next_pos, m_dir, Front, None);
			}
			break;
	}
	return last_pos;
}

uint16_t Dijkstra::straight_section_num(t_posDijkstra s_pos,t_posDijkstra e_pos,t_direction dir)
{
	switch(dir)
	{
		case North:
		case South:
			return ABS(s_pos.y - e_pos.y);
		case East:
		case West:
			return ABS(s_pos.x - e_pos.x);
		default:
			return 0;
	}
	return 0;
}

uint16_t Dijkstra::diagonal_section_num(t_posDijkstra s_pos,t_posDijkstra e_pos,t_direction dir)
{
	uint16_t count = 0;
	t_posDijkstra pos = s_pos;
	switch(dir)
	{
		case NorthWest:
		case NorthEast:
		case SouthEast:
		case SouthWest:
			for(count = 1;;count++)
			{
				pos = LocalPosDir2GlobWallPos_WPos(pos, dir, Front);
				if(pos.x == e_pos.x && pos.y == e_pos.y && pos.NodePos == e_pos.NodePos)
				{
					break;
				}
			}
			return count;
		default:
			return 0;
	}
	return count;
}

t_bool Dijkstra::check_DijkstraPath(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size)
{
	t_posDijkstra last_pos = make_path_Dijkstra_priority_queue(start_pos, start_wallPos, goal_pos, goal_size);
	return is_goal_Dijkstra(last_pos,goal_pos, goal_size);
}

void Dijkstra::check_run_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,
		t_bool priority_queue)
{
	t_posDijkstra last_pos;
	if(priority_queue == True)
	{
		last_pos = make_path_Dijkstra_priority_queue(start_pos, start_wallPos, goal_pos, goal_size);
	}
	else
	{
		last_pos = make_path_Dijkstra(start_pos, start_wallPos, goal_pos, goal_size);
	}
	if(is_goal_Dijkstra(last_pos, goal_pos, goal_size) != True)
	{
		printf("DIJKSTRA_RESULT NO_PATH last=(%2d,%2d,%2d) time:%d\r\n",
				last_pos.x,
				last_pos.y,
				last_pos.NodePos,
				(*get_closure_inf(last_pos)).time);
		return;
	}
	printf("DIJKSTRA_RESULT GOAL last=(%2d,%2d,%2d) time:%d\r\n",
			last_pos.x,
			last_pos.y,
			last_pos.NodePos,
			(*get_closure_inf(last_pos)).time);
	t_posDijkstra tmp_pos = last_pos;
	t_posDijkstra start = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	int tail = 0;;
	for(int i = 0; i < DIJKSTRA_PATH_MAX; i++)
	{
		#ifdef DEBUG_MODE
			printf("x:%2d,y:%2d,d:%2d->",tmp_pos.x,tmp_pos.y,tmp_pos.NodePos);
		#endif
		switch((*get_closure_inf(tmp_pos)).run_pt)
		{
			#ifdef DEBUG_MODE
			case No_run: 			printf("No_run\n"); 			break;
			case Straight:	 		printf("Straight\n"); 			break;
			case Diagonal: 			printf("Diagonal\n"); 			break;
			case Long_turnR90: 		printf("Long_turnR90\n"); 		break;
			case Long_turnL90: 		printf("Long_turnL90\n"); 		break;
			case Long_turnR180: 	printf("Long_turnR180\n"); 		break;
			case Long_turnL180: 	printf("Long_turnL180\n"); 		break;
			case Turn_in_R45: 		printf("Turn_in_R45\n"); 		break;
			case Turn_in_L45: 		printf("Turn_in_L45\n"); 		break;
			case Turn_out_R45: 		printf("Turn_out_R45\n"); 		break;
			case Turn_out_L45: 		printf("Turn_out_L45\n"); 		break;
			case Turn_in_R135: 		printf("Turn_in_R135\n"); 		break;
			case Turn_in_L135: 		printf("Turn_in_L135\n"); 		break;
			case Turn_out_R135: 	printf("Turn_out_R135\n"); 		break;
			case Turn_out_L135: 	printf("Turn_out_L135\n"); 		break;
			case Turn_RV90: 		printf("Turn_RV90\n"); 			break;
			case Turn_LV90: 		printf("Turn_LV90\n"); 			break;
			case Diagonal_R: 		printf("Diagonal_R\n"); 		break;
			case Diagonal_L: 		printf("Diagonal_L\n"); 		break;
			case Search_st_section: printf("Search_st_section\n"); 	break;
			case Search_st_half: 	printf("Search_st_half\n"); 	break;
			case Pivot_turn_R: 		printf("Pivot_turn_R\n"); 		break;
			case Pivot_turn_L: 		printf("Pivot_turn_L\n"); 		break;
			case Search_slalom_R: 	printf("Search_slalom_R\n"); 	break;
			case Search_slalom_L: 	printf("Search_slalom_L\n"); 	break;
			case run_brake: 		printf("run_brake\n"); 			break;
			case motor_free: 		printf("motor_free\n"); 		break;
			case Fix_wall: 			printf("Fix_wall\n"); 			break;
			#endif
			default :
				break;
		}
		run_pos_buff[i] = tmp_pos;
		tmp_pos = (*get_closure_inf(tmp_pos)).parent_pos;
		if(tmp_pos.x == start.x && tmp_pos.y == start.y && tmp_pos.NodePos == start.NodePos && tmp_pos.state_dir == start.state_dir)
		{
			tail = i;
			break;
		}
	}

	#ifdef DEBUG_MODE
		printf("\nstart\n");
	#endif

	for(int i = tail ; i >= 0;i--)
	{
		//#ifdef DEBUG_MODE
			printf("x:%2d,y:%2d,d:%2d,mdir:%2d,time:%d->",run_pos_buff[i].x,run_pos_buff[i].y,run_pos_buff[i].NodePos,(*get_closure_inf(run_pos_buff[i])).dir,(*get_closure_inf(run_pos_buff[i])).time);
		//#endif
		switch((*get_closure_inf(run_pos_buff[i])).run_pt)
		{
			//#ifdef DEBUG_MODE
			case No_run: 			printf("No_run\n"); 			break;
			case Straight:
				printf("count->%2d",straight_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir));
				printf("Straight\n"); 			break;
			case Diagonal:
				printf("count->%2d",diagonal_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir));
				printf("Diagonal\n"); 			break;
			case Long_turnR90: 		printf("Long_turnR90\n"); 		break;
			case Long_turnL90: 		printf("Long_turnL90\n"); 		break;
			case Long_turnR180: 	printf("Long_turnR180\n"); 		break;
			case Long_turnL180: 	printf("Long_turnL180\n"); 		break;
			case Turn_in_R45: 		printf("Turn_in_R45\n"); 		break;
			case Turn_in_L45: 		printf("Turn_in_L45\n"); 		break;
			case Turn_out_R45: 		printf("Turn_out_R45\n"); 		break;
			case Turn_out_L45: 		printf("Turn_out_L45\n"); 		break;
			case Turn_in_R135: 		printf("Turn_in_R135\n"); 		break;
			case Turn_in_L135: 		printf("Turn_in_L135\n"); 		break;
			case Turn_out_R135: 	printf("Turn_out_R135\n"); 		break;
			case Turn_out_L135: 	printf("Turn_out_L135\n"); 		break;
			case Turn_RV90: 		printf("Turn_RV90\n"); 			break;
			case Turn_LV90: 		printf("Turn_LV90\n"); 			break;
			//case Diagonal_R: 		printf("Diagonal_R\n"); 		break;
			//case Diagonal_L: 		printf("Diagonal_L\n"); 		break;
			case Search_st_section: printf("Search_st_section\n"); 	break;
			case Search_st_half: 	printf("Search_st_half\n"); 	break;
			case Pivot_turn_R: 		printf("Pivot_turn_R\n"); 		break;
			case Pivot_turn_L: 		printf("Pivot_turn_L\n"); 		break;
			case Search_slalom_R: 	printf("Search_slalom_R\n"); 	break;
			case Search_slalom_L: 	printf("Search_slalom_L\n"); 	break;
			case run_brake: 		printf("run_brake\n"); 			break;
			case motor_free: 		printf("motor_free\n"); 		break;
			case Fix_wall: 			printf("Fix_wall\n"); 			break;
			//#endif
			default :
				break;
		}
	}
}

void Dijkstra::run_Dijkstra(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,
				  const t_straight_param *const *st_mode,uint16_t size_st_mode,
				  const t_straight_param *const *di_mode,uint16_t size_di_mode,
				  const t_param *const *turn_mode , Motion *motion)
{
	turn_time_set(turn_mode);
	st_param_set(st_mode, size_st_mode);
	di_param_set(di_mode, size_di_mode);

	//t_posDijkstra last_pos = make_path_Dijkstra(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra last_pos = make_path_Dijkstra_priority_queue(start_pos, start_wallPos, goal_pos, goal_size);
	
	t_posDijkstra tmp_pos = last_pos;
	t_posDijkstra start = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	t_straight_param st_parameter ;

	int tail = 0;
	for(int i = 0; i < DIJKSTRA_PATH_MAX; i++)
	{
		run_pos_buff[i] = tmp_pos;
		tmp_pos = (*get_closure_inf(tmp_pos)).parent_pos;
		if(tmp_pos.x == start.x && tmp_pos.y == start.y && tmp_pos.NodePos == start.NodePos && tmp_pos.state_dir == start.state_dir)
		{
			tail = i;
			break;
		}
	}
	log_enable();
	motion->Motion_start();
	motion->exe_Motion_straight(  16.10-3.0, straight_base_velo().param->acc, straight_base_velo().param->max_velo, straight_base_velo().param->max_velo);

	uint16_t section_count = 0;
	for(int i = tail ; i >= 0;i--)
	{
		switch((*get_closure_inf(run_pos_buff[i])).run_pt)
		{
			//#ifdef DEBUG_MODE
			case No_run: 	break;
			case Straight:
				section_count = straight_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_straight_max_velo(SECTION * section_count);
				if(i == 0)
					motion->exe_Motion_straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f,st_parameter.sp_gain,st_parameter.om_gain);
				else
					motion->exe_Motion_straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, straight_base_velo().param->max_velo,st_parameter.sp_gain,st_parameter.om_gain);
				break;
			case Diagonal:
				section_count = diagonal_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_diagonal_max_velo(DIAG_SECTION * section_count);
				if(i == 0)
					motion->exe_Motion_diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f,st_parameter.sp_gain,st_parameter.om_gain);
				else
					motion->exe_Motion_diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, diagonal_base_velo().param->max_velo,st_parameter.sp_gain,st_parameter.om_gain);
				break;
			case Long_turnR90:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnR90],(t_run_pattern)(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnL90:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnL90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnR180:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnR180],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnL180:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnL180],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_R45:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_R45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_L45:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_L45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_R45:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_R45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_L45:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_L45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_R135:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_R135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_L135:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_L135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_R135:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_R135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_L135:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_L135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_RV90:
				motion->exe_Motion_turn_v90(  turn_mode[Turn_RV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_LV90:
				motion->exe_Motion_turn_v90(  turn_mode[Turn_LV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			//case Diagonal_R: 		break;
			//case Diagonal_L: 		break;
			case Search_st_section: break;
			case Search_st_half: 	break;
			case Pivot_turn_R: 		break;
			case Pivot_turn_L: 		break;
			case Search_slalom_R: 	break;
			case Search_slalom_L: 	break;
			case run_brake: 		break;
			case motor_free: 		break;
			case Fix_wall: 			break;
			//#endif
			default :
				break;
		}
		//while(controll_task::getInstance().run_task !=No_run);
		if(motion->motion_exeStatus_get() == error)
		{
			break;
		}
	}
	log_disable();
	motion->Motion_end();
	HAL_Delay(200);
	FAN_Motor_SetDuty(0);;
	HAL_Delay(200);
}

void Dijkstra::run_Dijkstra_suction(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,int suction,
									  const t_straight_param *const *st_mode,uint16_t size_st_mode,
									  const t_straight_param *const *di_mode,uint16_t size_di_mode,
									  const t_param *const *turn_mode , Motion *motion)
{
	turn_time_set(turn_mode);
	st_param_set(st_mode, size_st_mode);
	di_param_set(di_mode, size_di_mode);

	//t_posDijkstra last_pos = make_path_Dijkstra(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra last_pos = make_path_Dijkstra_priority_queue(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra tmp_pos = last_pos;
	t_posDijkstra start = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	t_straight_param st_parameter ;

	int tail = 0;
	for(int i = 0; i < DIJKSTRA_PATH_MAX; i++)
	{
		run_pos_buff[i] = tmp_pos;
		tmp_pos = (*get_closure_inf(tmp_pos)).parent_pos;
		if(tmp_pos.x == start.x && tmp_pos.y == start.y && tmp_pos.NodePos == start.NodePos && tmp_pos.state_dir == start.state_dir)
		{
			tail = i;
			break;
		}
	}

	motion->Motion_start();
	/*
	motion->Init_Motion_suction_start( suction/5*3+200);
	for(int i = 5; i <= suction; i = i + 5)
	{
		FAN_Motor_SetDuty(i);;
		HAL_Delay(3);
	}
	motion->execute_Motion();
	*/
	float suction_value = suction/1000.0f*8.40;
	int stay_time 	= (int)(suction_value/SUCTION_ACC) + 300;
	motion->exe_Motion_suction_start(suction/1000.0f*8.40, stay_time,straight_base_velo().sp_gain,straight_base_velo().om_gain);
	//motion->Motion_start();
	log_enable();
	motion->exe_Motion_straight(  16.10-3.0, straight_base_velo().param->acc, straight_base_velo().param->max_velo, straight_base_velo().param->max_velo,straight_base_velo().sp_gain,straight_base_velo().om_gain);

	uint16_t section_count = 0;
	for(int i = tail ; i >= 0;i--)
	{
		switch((*get_closure_inf(run_pos_buff[i])).run_pt)
		{
			//#ifdef DEBUG_MODE
			case No_run: 	break;
			case Straight:
				section_count = straight_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_straight_max_velo(SECTION * section_count);
				if(i == 0)
					motion->exe_Motion_straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f,st_parameter.sp_gain,st_parameter.om_gain);
				else
					motion->exe_Motion_straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, straight_base_velo().param->max_velo,st_parameter.sp_gain,st_parameter.om_gain);
				break;
			case Diagonal:
				section_count = diagonal_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_diagonal_max_velo(DIAG_SECTION * section_count);
				if(i == 0)
					motion->exe_Motion_diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f,st_parameter.sp_gain,st_parameter.om_gain);
				else
					motion->exe_Motion_diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, diagonal_base_velo().param->max_velo,st_parameter.sp_gain,st_parameter.om_gain);
				break;
			case Long_turnR90:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnR90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnL90:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnL90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnR180:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnR180],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnL180:
				motion->exe_Motion_long_turn(  turn_mode[Long_turnL180],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_R45:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_R45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_L45:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_L45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_R45:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_R45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_L45:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_L45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_R135:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_R135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_L135:
				motion->exe_Motion_turn_in(  turn_mode[Turn_in_L135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_R135:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_R135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_L135:
				motion->exe_Motion_turn_out(  turn_mode[Turn_out_L135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_RV90:
				motion->exe_Motion_turn_v90(  turn_mode[Turn_RV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_LV90:
				motion->exe_Motion_turn_v90(  turn_mode[Turn_LV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			//case Diagonal_R: 		break;
			//case Diagonal_L: 		break;
			case Search_st_section: break;
			case Search_st_half: 	break;
			case Pivot_turn_R: 		break;
			case Pivot_turn_L: 		break;
			case Search_slalom_R: 	break;
			case Search_slalom_L: 	break;
			case run_brake: 		break;
			case motor_free: 		break;
			case Fix_wall: 			break;
			//#endif
			default :
				break;
		}
		//while(controll_task::getInstance().run_task !=No_run);
		if(motion->motion_exeStatus_get() == error)
		{
			break;
		}
	}

	log_disable();
	motion->Motion_end();
	HAL_Delay(200);
	FAN_Motor_SetDuty(0);;
	HAL_Delay(200);
}

void Dijkstra::run_Dijkstra_suction_acc(t_position start_pos,t_direction start_wallPos,t_position goal_pos,uint8_t goal_size,int suction,
									  const t_straight_param *const *st_mode,uint16_t size_st_mode,
									  const t_straight_param *const *di_mode,uint16_t size_di_mode,
									  const t_param *const*const *turn_mode ,uint16_t size_turn_mode, Motion *motion)
{
	turn_time_set(turn_mode,size_turn_mode);
	st_param_set(st_mode, size_st_mode);
	di_param_set(di_mode, size_di_mode);

	//t_posDijkstra last_pos = make_path_Dijkstra(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra last_pos = make_path_Dijkstra_priority_queue(start_pos, start_wallPos, goal_pos, goal_size);
	t_posDijkstra tmp_pos = last_pos;
	t_posDijkstra start = conv_t_pos2t_posDijkstra(start_pos, start_wallPos);
	t_straight_param st_parameter ;

	uint8_t turn_select[MAZE_SIZE];

	int tail = 0;
	for(int i = 0; i < DIJKSTRA_PATH_MAX; i++)
	{
		run_pos_buff[i] = tmp_pos;
		turn_select [i] = 0;
		tmp_pos = (*get_closure_inf(tmp_pos)).parent_pos;

		for(int turn_cnt = 0; turn_cnt < size_turn_mode;turn_cnt++)
		{
			if(turn_mode[turn_cnt][(*get_closure_inf(run_pos_buff[i])).run_pt] != NULL)
			{
				turn_select [i] = turn_cnt;
			}

		}

		if( i > 0)
		{
			if((*get_closure_inf(run_pos_buff[i])).run_pt == Turn_out_R45 && (*get_closure_inf(run_pos_buff[i-1])).run_pt == Turn_in_R45)
			{
				for(int turn_cnt = 0; turn_cnt < size_turn_mode;turn_cnt++)
				{
					if(turn_mode[turn_cnt][Long_turn_RV90] != NULL)
					{
						turn_select [i] = turn_cnt;
						turn_select [i-1] = turn_cnt;
						(get_closure_inf(run_pos_buff[i]))->run_pt = Long_turn_RV90;
						(get_closure_inf(run_pos_buff[i-1]))->run_pt = Long_turn_RV90;
					}

				}
			}

			if((*get_closure_inf(run_pos_buff[i])).run_pt == Turn_out_L45 && (*get_closure_inf(run_pos_buff[i-1])).run_pt == Turn_in_L45)
			{
				for(int turn_cnt = 0; turn_cnt < size_turn_mode;turn_cnt++)
				{
					if(turn_mode[turn_cnt][Long_turn_LV90] != NULL)
					{
						turn_select [i] = turn_cnt;
						turn_select [i-1] = turn_cnt;
						(get_closure_inf(run_pos_buff[i]))->run_pt = Long_turn_LV90;
						(get_closure_inf(run_pos_buff[i-1]))->run_pt = Long_turn_LV90;

					}

				}
			}

		}


		if(tmp_pos.x == start.x && tmp_pos.y == start.y && tmp_pos.NodePos == start.NodePos && tmp_pos.state_dir == start.state_dir)
		{
			tail = i;
			break;
		}
	}

	motion->Motion_start();
	float suction_value = suction/1000.0f*8.40;
	int stay_time 	= (int)(suction_value/SUCTION_ACC) + 300;
	motion->exe_Motion_suction_start(suction/1000.0f*8.40, stay_time,straight_base_velo().sp_gain,straight_base_velo().om_gain);
	//motion->Motion_start();
	log_enable();
	motion->exe_Motion_straight(  16.10-3.0, straight_base_velo().param->acc, straight_base_velo().param->max_velo, straight_base_velo().param->max_velo,straight_base_velo().sp_gain,straight_base_velo().om_gain);

	uint16_t section_count = 0;
	float end_velo = straight_base_velo().param->max_velo;
	for(int i = tail ; i >= 0;i--)
	{

		end_velo = straight_base_velo().param->max_velo;
		if(i > 0)
		{

			if(!((*get_closure_inf(run_pos_buff[i-1])).run_pt == Straight || (*get_closure_inf(run_pos_buff[i-1])).run_pt == Diagonal ))
			{
				if(turn_mode[turn_select[i-1]][(*get_closure_inf(run_pos_buff[i-1])).run_pt] != NULL)
					end_velo = turn_mode[turn_select[i-1]][(*get_closure_inf(run_pos_buff[i-1])).run_pt]->param->velo;
			}

			if((*get_closure_inf(run_pos_buff[i-1])).run_pt == Straight || (*get_closure_inf(run_pos_buff[i-1])).run_pt == Diagonal )
			{
				if(turn_mode[turn_select[i]][(*get_closure_inf(run_pos_buff[i])).run_pt] != NULL)
					end_velo = turn_mode[turn_select[i]][(*get_closure_inf(run_pos_buff[i])).run_pt]->param->velo;
			}

			if(i > 1 && ((*get_closure_inf(run_pos_buff[i])).run_pt == Long_turn_RV90 || (*get_closure_inf(run_pos_buff[i])).run_pt == Long_turn_LV90 ))
			{
				if(!((*get_closure_inf(run_pos_buff[i-2])).run_pt == Straight || (*get_closure_inf(run_pos_buff[i-2])).run_pt == Diagonal ))
				{
					if(turn_mode[turn_select[i-2]][(*get_closure_inf(run_pos_buff[i-2])).run_pt] != NULL)
						end_velo = turn_mode[turn_select[i-2]][(*get_closure_inf(run_pos_buff[i-2])).run_pt]->param->velo;
				}

				if((*get_closure_inf(run_pos_buff[i-2])).run_pt == Straight || (*get_closure_inf(run_pos_buff[i-2])).run_pt == Diagonal )
				{
					if(turn_mode[turn_select[i]][(*get_closure_inf(run_pos_buff[i])).run_pt] != NULL)
						end_velo = turn_mode[turn_select[i]][(*get_closure_inf(run_pos_buff[i])).run_pt]->param->velo;
				}
			}
		}

		switch((*get_closure_inf(run_pos_buff[i])).run_pt)
		{
			//#ifdef DEBUG_MODE
			case No_run: 	break;
			case Straight:
				section_count = straight_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_straight_max_velo(SECTION * section_count);
				if(i == 0)
					motion->exe_Motion_straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f,st_parameter.sp_gain,st_parameter.om_gain);
				else
					if(st_parameter.param->max_velo >= end_velo)
						motion->exe_Motion_straight(  SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, end_velo,st_parameter.sp_gain,st_parameter.om_gain);
					else
						motion->exe_Motion_straight(  SECTION * section_count, st_parameter.param->acc, end_velo					, end_velo,st_parameter.sp_gain,st_parameter.om_gain);
				break;
			case Diagonal:
				section_count = diagonal_section_num((*get_closure_inf(run_pos_buff[i])).parent_pos, run_pos_buff[i], (*get_closure_inf(run_pos_buff[i])).dir);
				st_parameter =  calc_end_diagonal_max_velo(DIAG_SECTION * section_count);
				if(i == 0)
					motion->exe_Motion_diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, 0.0f,st_parameter.sp_gain,st_parameter.om_gain);
				else
					if(st_parameter.param->max_velo >= end_velo)
						motion->exe_Motion_diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, st_parameter.param->max_velo, end_velo,st_parameter.sp_gain,st_parameter.om_gain);
					else
						motion->exe_Motion_diagonal(  DIAG_SECTION * section_count, st_parameter.param->acc, end_velo					 , end_velo,st_parameter.sp_gain,st_parameter.om_gain);
				break;
			case Long_turnR90:
				if (i == tail) 	motion->exe_Motion_long_turn(  turn_mode[turn_select[0]][Long_turnR90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				else			motion->exe_Motion_long_turn(  turn_mode[turn_select[i]][Long_turnR90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnL90:
				motion->exe_Motion_long_turn(  turn_mode[turn_select[i]][Long_turnL90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnR180:
				motion->exe_Motion_long_turn(  turn_mode[turn_select[i]][Long_turnR180],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turnL180:
				motion->exe_Motion_long_turn(  turn_mode[turn_select[i]][Long_turnL180],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_R45:
				if (i == tail) 	motion->exe_Motion_turn_in(  turn_mode[turn_select[0]][Turn_in_R45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				else			motion->exe_Motion_turn_in(  turn_mode[turn_select[i]][Turn_in_R45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_L45:
				motion->exe_Motion_turn_in(  turn_mode[turn_select[i]][Turn_in_L45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_R45:
				motion->exe_Motion_turn_out(  turn_mode[turn_select[i]][Turn_out_R45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_L45:
				motion->exe_Motion_turn_out(  turn_mode[turn_select[i]][Turn_out_L45],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Long_turn_RV90:
				motion->exe_Motion_long_turn_v90(  turn_mode[turn_select[i]][Long_turn_RV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				i--;
				break;
			case Long_turn_LV90:
				motion->exe_Motion_long_turn_v90(  turn_mode[turn_select[i]][Long_turn_LV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				i--;
				break;
			case Turn_in_R135:
				if (i == tail) 	motion->exe_Motion_turn_in(  turn_mode[turn_select[0]][Turn_in_R135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				else			motion->exe_Motion_turn_in(  turn_mode[turn_select[i]][Turn_in_R135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_in_L135:
				motion->exe_Motion_turn_in(  turn_mode[turn_select[i]][Turn_in_L135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_R135:
				motion->exe_Motion_turn_out(  turn_mode[turn_select[i]][Turn_out_R135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_out_L135:
				motion->exe_Motion_turn_out(  turn_mode[turn_select[i]][Turn_out_L135],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_RV90:
				motion->exe_Motion_turn_v90(  turn_mode[turn_select[i]][Turn_RV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
			case Turn_LV90:
				motion->exe_Motion_turn_v90(  turn_mode[turn_select[i]][Turn_LV90],(t_run_pattern)(*get_closure_inf(run_pos_buff[i])).run_pt,end_velo,straight_base_velo().param->acc,straight_base_velo().sp_gain,straight_base_velo().om_gain);
				break;
//			case Diagonal_R: 		break;
//			case Diagonal_L: 		break;
			case Search_st_section: break;
			case Search_st_half: 	break;
			case Pivot_turn_R: 		break;
			case Pivot_turn_L: 		break;
			case Search_slalom_R: 	break;
			case Search_slalom_L: 	break;
			case run_brake: 		break;
			case motor_free: 		break;
			case Fix_wall: 			break;
			//#endif
			default :
				break;
		}
		//while(controll_task::getInstance().run_task !=No_run);
		if(motion->motion_exeStatus_get() == error)
		{
			break;
		}
	}

	log_disable();
	motion->Motion_end();
	HAL_Delay(200);
	FAN_Motor_SetDuty(0);;
	HAL_Delay(200);
}
