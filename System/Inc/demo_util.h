#ifndef INC_DEMO_UTIL_H_
#define INC_DEMO_UTIL_H_

#include "search_class.h"

namespace DemoUtil
{
	enum search_type_t {
		SEARCH_1,
		SEARCH_1_ACC,
		SEARCH_2_ACC,
		SEARCH_2_PRUNE_ACC,
		SEARCH_3_ACC
	};

	t_position run_search(Search *search, search_type_t type,
		t_position start, t_position goal, int goal_size,
		wall_class *wall, Motion *motion);
}

#endif /* INC_DEMO_UTIL_H_ */
