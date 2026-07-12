#include "../Inc/demo_util.h"

#include "make_map_class.h"
#include "queue.h"

namespace DemoUtil
{
t_position run_search(Search *search, search_type_t type,
	t_position start, t_position goal, int goal_size,
	wall_class *wall, Motion *motion)
{
	ring_queue<1024, t_MapNode> maze_q;
	make_map map_data(wall, &maze_q);

	switch(type) {
	case SEARCH_1:
		return search->search_adachi_1(start, goal, goal_size, wall, &map_data, motion);
	case SEARCH_1_ACC:
		return search->search_adachi_1_acc(start, goal, goal_size, wall, &map_data, motion);
	case SEARCH_2_ACC:
		return search->search_adachi_2_acc(start, goal, goal_size, wall, &map_data, motion);
	case SEARCH_3_ACC:
		return search->search_adachi_3_acc(start, goal, goal_size, wall, &map_data, motion);
	default:
		return start;
	}
}
}
