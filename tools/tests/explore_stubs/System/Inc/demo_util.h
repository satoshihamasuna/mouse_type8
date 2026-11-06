#ifndef TEST_EXPLORE_DEMO_UTIL_H_
#define TEST_EXPLORE_DEMO_UTIL_H_
#include <stdint.h>
#include "Component/Inc/typedef.h"
#include "Core/Inc/main.h"
#include "Task/Inc/run_typedef.h"
enum t_direction { North = 0, Dir_None = 4 };
struct t_position { uint8_t x, y; t_direction dir; };
#define MAZE_GOAL_X 7
#define MAZE_GOAL_Y 7
#define MAZE_GOAL_SIZE 2
#define MAZE_SIZE_X 32
#define MAZE_SIZE_Y 32
extern int test_sensor, test_motion_calls, test_motion_ends;
extern bool test_stop_during_motion;
class IrSensTask { public: int IrSensor_Avg() { return test_sensor; } };
class Motion
{
public:
	IrSensTask ir;
	t_exeStatus status = complete;
	IrSensTask *return_irObj() { return &ir; }
	t_exeStatus motion_exeStatus_get() { return status; }
	void Motion_end() { ++test_motion_ends; }
};
class CtrlTask_type8 : public Motion
{
public:
	static CtrlTask_type8 &getInstance() { static CtrlTask_type8 instance; return instance; }
};
class wall_class
{
public:
	explicit wall_class(IrSensTask *) {}
	void init_maze() {}
	void history2wall_append() {}
	struct History { void history_init() {} } wall_history;
};
class Search
{
public:
	void set_virtual_wall_protected_area(t_position, t_position, int) {}
	void set_search_limit_time(int32_t) {}
	void search_param_init() {}
	void reset_search_time() {}
	t_bool i_am_goal(t_position p, t_position g, int size)
	{ return (p.x >= g.x && p.y >= g.y && p.x < g.x + size && p.y < g.y + size) ? True : False; }
};
namespace DemoUtil
{
enum search_type_t { GOAL_SEARCH_FIRST_PRIORITY, ACCELERATED_GOAL_SEARCH_FIRST_PRIORITY };
t_position run_search(Search *, search_type_t, t_position, t_position, int, wall_class *, Motion *);
}
#endif
