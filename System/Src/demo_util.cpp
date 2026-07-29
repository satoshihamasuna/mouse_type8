#include "System/Inc/demo_util.h"

#include "Subsys/Inc/make_map_class.h"
#include "Component/Inc/queue.h"

namespace DemoUtil
{
t_position run_search(Search *search, search_type_t type,
	t_position start, t_position goal, int goal_size,
	wall_class *wall, Motion *motion)
{
	ring_queue<1024, t_MapNode> maze_q;
	make_map map_data(wall, &maze_q);

	switch(type) {
	case GOAL_SEARCH_FIRST_PRIORITY:
		return search->run_goal_search_first_priority(start, goal, goal_size, wall, &map_data, motion);
	case ACCELERATED_GOAL_SEARCH_FIRST_PRIORITY:
		return search->run_accelerated_goal_search_first_priority(start, goal, goal_size, wall, &map_data, motion);
	case ACCELERATED_FULL_EXPLORATION:
		return search->run_accelerated_full_exploration(start, goal, goal_size, wall, &map_data, motion);
	case ACCELERATED_PRUNED_FULL_EXPLORATION:
		return search->run_accelerated_pruned_full_exploration(start, goal, goal_size, wall, &map_data, motion);
	case ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY:
		return search->run_accelerated_goal_search_second_priority(start, goal, goal_size, wall, &map_data, motion);
	default:
		return start;
	}
}
}
