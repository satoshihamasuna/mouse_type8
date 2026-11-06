#include "System/Inc/shortest_run.h"
#include "System/Inc/explore.h"
#include "Subsys/Inc/make_path.h"
#include "Module/Inc/flash.h"
#include "Module/Inc/log_data.h"
#include "Module/Inc/remote_explore.h"
#include "Peripheral/Inc/battery.h"
#include "Peripheral/Inc/peripheral.h"
#include "core/ntlibc.h"
#include <stdio.h>

namespace
{
bool stopped() { return RemoteExplore::should_stop(HAL_GetTick()); }

const char *stop_result()
{
	switch(RemoteExplore::stop_reason())
	{
	case RemoteExplore::USER_STOP: return "stopped";
	case RemoteExplore::LINK_TIMEOUT: return "link_timeout";
	case RemoteExplore::TIME_LIMIT: return "time_limit";
	default: return "motion_error";
	}
}

// Bound parsing to reject trailing text, overflow and negative input.
bool parse_uint(const char *text, int *value)
{
	if(*text == '\0') return false;
	int parsed = 0;
	for(; *text; ++text)
	{
		if(*text < '0' || *text > '9' || parsed > 3600) return false;
		parsed = parsed * 10 + (*text - '0');
	}
	*value = parsed;
	return parsed <= 3600;
}
}

int shell_shortest(int argc, char **argv)
{
	if(argc == 2 && ntlibc_strcmp(argv[1], "show") == 0)
	{
#if defined(MOUSE_A)
		const char *mouse = "A";
#else
		const char *mouse = "B";
#endif
		printf("SHORTEST_CAP protocol:1 mouse:%s goal_x:%d goal_y:%d goal_size:%d max_x:%d max_y:%d state:%s ram_maze:%d profiles:",
			mouse, MAZE_GOAL_X, MAZE_GOAL_Y, MAZE_GOAL_SIZE, MAZE_SIZE_X - 1, MAZE_SIZE_Y - 1,
			RemoteExplore::running() ? "running" : "idle", explore_completed_maze() != nullptr ? 1 : 0);
		for(uint16_t i = 0; i < run_config_count; ++i)
			printf("%s%s", i == 0 ? "" : ",", run_config_table[i]->key);
		printf("\r\n");
		return 0;
	}
	int gx, gy, size, seconds;
	if(argc != 8 || ntlibc_strcmp(argv[1], "start") != 0 ||
	   !parse_uint(argv[2], &gx) || !parse_uint(argv[3], &gy) ||
	   !parse_uint(argv[4], &size) || !parse_uint(argv[7], &seconds) ||
	   gx >= MAZE_SIZE_X || gy >= MAZE_SIZE_Y || size < 1 ||
	   gx + size > MAZE_SIZE_X || gy + size > MAZE_SIZE_Y ||
	   (gx == 0 && gy == 0) || seconds < 1 ||
	   (ntlibc_strcmp(argv[6], "ram") != 0 && ntlibc_strcmp(argv[6], "flash") != 0))
	{
		printf("SHORTEST_ERROR invalid_argument\r\n");
		return -1;
	}
	const t_run_config *config = find_run_config(argv[5]);
	if(config == nullptr)
	{
		printf("SHORTEST_ERROR unknown_profile\r\n");
		return -1;
	}
	if(Battery_IsVoltageError(Battery_GetAverageVoltage()))
	{
		printf("SHORTEST_ERROR low_battery\r\n");
		return -1;
	}
	if(!RemoteExplore::begin(HAL_GetTick(), static_cast<uint32_t>(seconds) * 1000U))
	{
		printf("SHORTEST_ERROR not_armed_or_no_stream\r\n");
		return -1;
	}
	Motion *motion = &CtrlTask_type8::getInstance();
	wall_class *wall = explore_completed_maze();
	if(ntlibc_strcmp(argv[6], "flash") == 0)
	{
		// Keep the latest exploration map intact when trying a saved maze.
		static wall_class saved_wall(motion->return_irObj());
		saved_wall.init_maze();
		read_save_data(&saved_wall);
		saved_wall.history2wall_append();
		wall = &saved_wall;
	}
	const char *result = "missing_maze";
	if(wall != nullptr)
	{
		// Same foreground stack allocation as the existing on-board shortest run.
		Dijkstra path(wall);
		path.abort_requested = stopped;
		path.run_config_set(config);
		t_position start = {0, 0, North};
		t_position goal = {static_cast<uint8_t>(gx), static_cast<uint8_t>(gy), North};
		printf("SHORTEST_PHASE name:planning profile:%s maze:%s\r\n", config->key, argv[6]);
		if(path.check_DijkstraPath(start, Dir_None, goal, size) != True)
			result = "no_path";
		else if(!stopped())
		{
			printf("SHORTEST_WAIT_SENSOR goal:%d,%d,%d profile:%s maze:%s\r\n", gx, gy, size, config->key, argv[6]);
			while(motion->return_irObj()->IrSensor_Avg() < 2000 && !stopped()) HAL_Delay(1);
			for(int i = 0; i < 11 && !stopped(); ++i)
			{
				Indicate_LED(i % 2 ? 0x00 : 0x3f);
				HAL_Delay(50);
			}
			if(!stopped())
			{
				LogData::getInstance().reset_log();
				LogData::getInstance().log_enable = True;
				printf("SHORTEST_PHASE name:running\r\n");
				switch(config->type)
				{
				case RUN_CONFIG_PLAIN: path.run_Dijkstra(start, Dir_None, goal, size, config, motion); break;
				case RUN_CONFIG_SUCTION: path.run_Dijkstra_suction(start, Dir_None, goal, size, config, motion); break;
				case RUN_CONFIG_VARIABLE_TURN: path.run_Dijkstra_suction_acc(start, Dir_None, goal, size, config, motion); break;
				}
				result = motion->motion_exeStatus_get() == error ? "motion_error" : "complete";
			}
		}
	}
	if(stopped()) result = stop_result();
	LogData::getInstance().log_enable = False;
	motion->Motion_end();
	RemoteExplore::finish();
	Indicate_LED(0x30);
	// Endpoint is the commanded goal, not an independently measured position.
	printf("SHORTEST_END result:%s goal_x:%d goal_y:%d goal_size:%d profile:%s maze:%s\r\n",
		result, gx, gy, size, config->key, argv[6]);
	return ntlibc_strcmp(result, "complete") == 0 ? 0 : -1;
}
