#ifndef INC_DEMO_UTIL_H_
#define INC_DEMO_UTIL_H_

#include "Subsys/Inc/search_class.h"

namespace DemoUtil
{
	enum search_type_t {
		GOAL_SEARCH_FIRST_PRIORITY,
		ACCELERATED_GOAL_SEARCH_FIRST_PRIORITY,
		ACCELERATED_FULL_EXPLORATION,
		ACCELERATED_PRUNED_FULL_EXPLORATION,
		ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY
	};

	t_position run_search(Search *search, search_type_t type,
		t_position start, t_position goal, int goal_size,
		wall_class *wall, Motion *motion);
}

#endif /* INC_DEMO_UTIL_H_ */
