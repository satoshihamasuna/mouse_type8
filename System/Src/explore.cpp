#include "System/Inc/explore.h"
#include "System/Inc/demo_util.h"
#include "Module/Inc/log_data.h"
#include "Module/Inc/log_stream.h"
#include "Module/Inc/remote_explore.h"
#include "Peripheral/Inc/battery.h"
#include "Peripheral/Inc/peripheral.h"
#include "Peripheral/Inc/ir_sensor_activity.h"
#include "Subsys/Inc/search_turn_diagnostics.h"
#include "core/ntlibc.h"
#include <stdio.h>

namespace
{
bool maze_ready = false;
wall_class *explore_maze()
{
	static wall_class wall(CtrlTask_type8::getInstance().return_irObj());
	return &wall;
}

const char *result_name()
{
	switch(RemoteExplore::stop_reason())
	{
	case RemoteExplore::USER_STOP: return "stopped";
	case RemoteExplore::LINK_TIMEOUT: return "link_timeout";
	case RemoteExplore::TIME_LIMIT: return "time_limit";
	default: return "motion_error";
	}
}

bool stopped() { return RemoteExplore::should_stop(HAL_GetTick()); }
}

wall_class *explore_completed_maze() { return maze_ready ? explore_maze() : nullptr; }

int shell_explore(int argc, char **argv)
{
	if(argc == 2 && ntlibc_strcmp(argv[1], "show") == 0)
	{
#if defined(MOUSE_A)
		const char *mouse = "A";
#else
		const char *mouse = "B";
#endif
		printf("EXPLORE_CAP protocol:1 mouse:%s goal_x:%d goal_y:%d goal_size:%d max_x:%d max_y:%d state:%s turn_diag:1 turn_cov:1\r\n",
			mouse, MAZE_GOAL_X, MAZE_GOAL_Y, MAZE_GOAL_SIZE, MAZE_SIZE_X - 1, MAZE_SIZE_Y - 1,
			RemoteExplore::running() ? "running" : "idle");
		return 0;
	}
	int gx, gy, size, seconds;
	if(argc != 8 || ntlibc_strcmp(argv[1], "start") != 0 ||
	   sscanf(argv[2], "%d", &gx) != 1 || sscanf(argv[3], "%d", &gy) != 1 ||
	   sscanf(argv[4], "%d", &size) != 1 || sscanf(argv[7], "%d", &seconds) != 1 ||
	   gx < 0 || gy < 0 || gx >= MAZE_SIZE_X || gy >= MAZE_SIZE_Y ||
	   size <= 0 || size > MAZE_SIZE_X || size > MAZE_SIZE_Y ||
	   gx + size > MAZE_SIZE_X || gy + size > MAZE_SIZE_Y ||
	   (gx == 0 && gy == 0) || seconds < 1 || seconds > 3600 ||
	   (ntlibc_strcmp(argv[5], "plain") != 0 && ntlibc_strcmp(argv[5], "acc") != 0) ||
	   (ntlibc_strcmp(argv[6], "outward") != 0 && ntlibc_strcmp(argv[6], "roundtrip") != 0))
	{
		printf("EXPLORE_ERROR invalid_argument\r\n");
		return -1;
	}
	if(Battery_IsVoltageError(Battery_GetAverageVoltage()))
	{
		printf("EXPLORE_ERROR low_battery\r\n");
		return -1;
	}
	if(!RemoteExplore::begin(HAL_GetTick(), static_cast<uint32_t>(seconds) * 1000U))
	{
		printf("EXPLORE_ERROR not_armed_or_no_stream\r\n");
		return -1;
	}
	Motion *motion = &CtrlTask_type8::getInstance();
	SearchTurnDiagnostics::instance().begin();
	IrSensTask *ir = motion->return_irObj();
	// Reuse static storage, rather than adding a maze to the foreground stack.
	wall_class &wall = *explore_maze();
	maze_ready = false;
	wall.init_maze();
	wall.wall_history.history_init();
	Search search;
	t_position start = {0, 0, North};
	t_position goal = {static_cast<uint8_t>(gx), static_cast<uint8_t>(gy), North};
	t_position position = start;
	search.set_virtual_wall_protected_area(start, goal, size);
	search.set_search_limit_time(static_cast<int32_t>(seconds) * 1000);
	search.search_param_init();
	search.reset_search_time();
	const bool accelerated = ntlibc_strcmp(argv[5], "acc") == 0;
	const bool roundtrip = ntlibc_strcmp(argv[6], "roundtrip") == 0;
	const auto type = accelerated ? DemoUtil::ACCELERATED_GOAL_SEARCH_FIRST_PRIORITY : DemoUtil::GOAL_SEARCH_FIRST_PRIORITY;
	printf("EXPLORE_WAIT_SENSOR goal:%d,%d,%d motion:%s route:%s\r\n", gx, gy, size, argv[5], argv[6]);
	{
		SensorHandWait hand_wait(true);
		while(ir->IrSensor_Avg() < 2000 && !stopped()) HAL_Delay(1);
	}
	for(int i = 0; i < 11 && !stopped(); ++i)
	{
		Indicate_LED(i % 2 ? 0x00 : 0x3f);
		HAL_Delay(50);
	}
	const char *result = nullptr;
	if(!stopped())
	{
		LogData::getInstance().reset_log();
		LogData::getInstance().log_enable = True;
		printf("EXPLORE_PHASE name:outward\r\n");
		position = DemoUtil::run_search(&search, type, start, goal, size, &wall, motion);
		if(!stopped() && motion->motion_exeStatus_get() != error && search.i_am_goal(position, goal, size) == True)
		{
			result = "complete";
			if(roundtrip)
			{
				printf("EXPLORE_PHASE name:return\r\n");
				position = DemoUtil::run_search(&search, type, position, start, 1, &wall, motion);
				if(stopped() || motion->motion_exeStatus_get() == error || search.i_am_goal(position, start, 1) != True)
					result = nullptr;
			}
		}
	}
	if(result == nullptr) result = result_name();
	maze_ready = ntlibc_strcmp(result, "complete") == 0;
	LogData::getInstance().log_enable = False;
	motion->Motion_end();
	RemoteExplore::finish();
	Indicate_LED(0x30);
	SearchTurnDiagnostics::instance().finish_and_print();
	printf("EXPLORE_END result:%s x:%d y:%d dir:%d\r\n", result, position.x, position.y, position.dir);
	return ntlibc_strcmp(result, "complete") == 0 ? 0 : -1;
}
