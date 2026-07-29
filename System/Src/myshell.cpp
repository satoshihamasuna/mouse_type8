/*
 * myshell.cpp
 *
 *  Created on: 2024/03/09
 *      Author: sato1
 */

#include "core/ntshell.h"
#include "core/ntlibc.h"
#include "util/ntopt.h"
#include "myshell.h"
#include "communicate.h"
#include "log_data.h"
#include "flash.h"
#include "interrupt.h"
#include "make_path.h"
#include "adachi_class.h"
#include "search_class.h"
#include "virtual_wall_class.h"
#include "typedef.h"
#include "sensing_task.h"
#include "ctrl_task.h"
#include "run_param.h"
#include "battery.h"

#include <stdio.h>

typedef int (*USRCMDFUNC)(int argc, char **argv);

static int user_callback(const char *text, void *extobj);

static int usrcmd_execute(const char *text);
static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);

//コマンド一覧
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);
static int usrcmd_disp(int argc, char **argv);
static int usrcmd_end(int argc, char **argv);
static int usrcmd_debug(int argc, char **argv);
static int usrcmd_turnpattern(int argc, char **argv);
static int usrcmd_log(int argc, char **argv);
static int usrcmd_load(int argc, char **argv);
static int usrcmd_path(int argc, char **argv);
static int usrcmd_search(int argc, char **argv);
typedef struct {
	const char *cmd;
	const char *desc;
    USRCMDFUNC func;
} cmd_table_t;

static const cmd_table_t cmdlist[] = {
    { "help", "This is a description text string for help command.", usrcmd_help },
    { "info", "This is a description text string for info command.", usrcmd_info },
	{ "disp", "This is a description text string for disp command.", usrcmd_disp },
	{ "end",  "This is a description text string for end command.", usrcmd_end },
	{ "debug","This is a description text string for debug command.", usrcmd_debug },
	{ "turnpattern", "Build and execute a turn pattern (max 20).", usrcmd_turnpattern },
	{ "log"  ,"This is a description text string for debug command.", usrcmd_log },
	{ "load" ,"load saved maze data from flash.", usrcmd_load },
	{ "path" ,"check path generation.", usrcmd_path },
	{ "search" ,"replay search/map/virtual-wall logic without motion.", usrcmd_search }
};

static ntshell_t nts;

t_bool	shell_end_flag;
static t_bool shell_wall_data_ready = False;
static uint8_t shell_goal_x = MAZE_GOAL_X;
static uint8_t shell_goal_y = MAZE_GOAL_Y;
static uint8_t shell_goal_size = MAZE_GOAL_SIZE;
static t_bool shell_replay_ready = False;
static t_position shell_replay_maze_start = {0, 0, North};

static wall_class *shell_wall_data(void)
{
	static wall_class wall_data(&IrSensTask_type8::getInstance());
	return &wall_data;
}

static void shell_read_save_data(wall_class *wall_data)
{
	wall_data->init_maze();
	read_save_data(wall_data);
	wall_data->history2wall_append();
	shell_wall_data_ready = True;
	shell_goal_x = MAZE_GOAL_X;
	shell_goal_y = MAZE_GOAL_Y;
	shell_goal_size = MAZE_GOAL_SIZE;
}

static void shell_receive_maze_binary(wall_class *wall_data)
{
	wall_data->init_maze();
	for( int y = MAZE_SIZE_Y - 1 ; y >= 0 ; y-- ){
		for(int x = 0; x < MAZE_SIZE_X ; x++ ){
			uint8_t data = Communicate_RxPopData();
			wall_data->wall[x][y].north = (t_wall_state)(data & 0x03);
			wall_data->wall[x][y].east  = (t_wall_state)((data >> 2) & 0x03);
			wall_data->wall[x][y].south = (t_wall_state)((data >> 4) & 0x03);
			wall_data->wall[x][y].west  = (t_wall_state)((data >> 6) & 0x03);
		}
	}
	wall_data->wall_history.history_init();
	shell_wall_data_ready = True;
}

static t_bool shell_set_goal(int x, int y, int size)
{
	if(size <= 0) {
		return False;
	}
	if(x < 0 || y < 0) {
		return False;
	}
	if((x + size) > MAZE_SIZE_X || (y + size) > MAZE_SIZE_Y) {
		return False;
	}
	shell_goal_x = (uint8_t)x;
	shell_goal_y = (uint8_t)y;
	shell_goal_size = (uint8_t)size;
	return True;
}

static wall_class *shell_replay_wall_data(void)
{
	static wall_class wall_data(&IrSensTask_type8::getInstance());
	return &wall_data;
}

static make_map *shell_search_map_data(wall_class *wall_data)
{
	static ring_queue<1024,t_MapNode> maze_q;
	static make_map map_data(shell_wall_data(), &maze_q);
	maze_q.queue_reset();
	map_data.wall_property = wall_data;
	return &map_data;
}

static const t_direction shell_search_dirs[4] = {North, East, South, West};

static const char *shell_search_direction_name(t_direction dir)
{
	switch(dir) {
		case North: return "N";
		case East: return "E";
		case South: return "S";
		case West: return "W";
		default: return "NONE";
	}
}

static t_direction shell_search_parse_direction(const char *text)
{
	if(ntlibc_strcmp(text, "N") == 0 || ntlibc_strcmp(text, "0") == 0) return North;
	if(ntlibc_strcmp(text, "E") == 0 || ntlibc_strcmp(text, "2") == 0) return East;
	if(ntlibc_strcmp(text, "S") == 0 || ntlibc_strcmp(text, "4") == 0) return South;
	if(ntlibc_strcmp(text, "W") == 0 || ntlibc_strcmp(text, "6") == 0) return West;
	return Dir_None;
}

static t_bool shell_search_is_goal(t_position pos)
{
	return (pos.x >= shell_goal_x && pos.x < shell_goal_x + shell_goal_size &&
		pos.y >= shell_goal_y && pos.y < shell_goal_y + shell_goal_size) ? True : False;
}

static t_bool shell_search_is_goal(t_position pos,t_position goal,int goal_size)
{
	return (pos.x >= goal.x && pos.x < goal.x + goal_size &&
		pos.y >= goal.y && pos.y < goal.y + goal_size) ? True : False;
}

static uint16_t shell_search_map_value(const make_map *map_data,int x,int y)
{
	if(x < 0 || x >= MAZE_SIZE_X || y < 0 || y >= MAZE_SIZE_Y) return MAZE_SIZE;
	return map_data->map[x][y];
}

static t_bool shell_search_neighbor(t_position pos,t_direction dir,int *nx,int *ny)
{
	*nx = pos.x;
	*ny = pos.y;
	switch(dir) {
		case North: ++*ny; break;
		case East: ++*nx; break;
		case South: --*ny; break;
		case West: --*nx; break;
		default: return False;
	}
	return (*nx >= 0 && *nx < MAZE_SIZE_X && *ny >= 0 && *ny < MAZE_SIZE_Y) ? True : False;
}

static t_bool shell_search_has_candidate(wall_class *wall_data,make_map *map_data,t_position pos,int mask)
{
	for(int i = 0; i < 4; i++) {
		int nx, ny;
		if(shell_search_neighbor(pos, shell_search_dirs[i], &nx, &ny) != True) continue;
		if(wall_data->is_open(pos.x, pos.y, shell_search_dirs[i], mask) != True) continue;
		if(map_data->map[nx][ny] < MAP_MAX_VALUE) return True;
	}
	return False;
}

static int shell_search_virtual_count(const wall_class *wall_data)
{
	int count = 0;
	for(int x = 0; x < MAZE_SIZE_X; x++) {
		for(int y = 0; y < MAZE_SIZE_Y; y++) {
			if(y + 1 < MAZE_SIZE_Y && wall_data->get_virtual_wall(x, y, North) == True) count++;
			if(x + 1 < MAZE_SIZE_X && wall_data->get_virtual_wall(x, y, East) == True) count++;
		}
	}
	return count;
}

static void shell_search_update_map(wall_class *wall_data,make_map *map_data,
		t_position maze_start,t_position protected_mouse,t_position expand_end,
		t_bool full_search,t_virtual_branch_mode branch_mode,int mask)
{
	t_position maze_goal = {shell_goal_x, shell_goal_y, North};
	t_virtual_wall_context context = {maze_start, protected_mouse,
		maze_goal, shell_goal_size, branch_mode};
	Search_UpdateMap(wall_data, map_data, context, expand_end, maze_goal,
		shell_goal_size, full_search, mask);
}

static t_wall_state shell_search_wall_state(const wall_class *wall_data,int x,int y,t_direction dir)
{
	if(x < 0 || x >= MAZE_SIZE_X || y < 0 || y >= MAZE_SIZE_Y) return WALL;
	switch(dir) {
		case North: return (t_wall_state)wall_data->wall[x][y].north;
		case East: return (t_wall_state)wall_data->wall[x][y].east;
		case South: return (t_wall_state)wall_data->wall[x][y].south;
		case West: return (t_wall_state)wall_data->wall[x][y].west;
		default: return WALL;
	}
}

static void shell_search_write_wall_state(wall_class *wall_data,int x,int y,
		t_direction dir,t_wall_state state)
{
	if(x < 0 || x >= MAZE_SIZE_X || y < 0 || y >= MAZE_SIZE_Y) return;
	switch(dir) {
		case North: wall_data->wall[x][y].north = state; break;
		case East: wall_data->wall[x][y].east = state; break;
		case South: wall_data->wall[x][y].south = state; break;
		case West: wall_data->wall[x][y].west = state; break;
		default: break;
	}
}

static t_direction shell_search_opposite(t_direction dir)
{
	switch(dir) {
		case North: return South;
		case East: return West;
		case South: return North;
		case West: return East;
		default: return Dir_None;
	}
}

static void shell_search_write_observation(wall_class *wall_data,int x,int y,
		t_direction dir,t_wall_state observed)
{
	const t_wall_state old_state = shell_search_wall_state(wall_data, x, y, dir);
	const t_wall_state stored = (old_state == UNKNOWN || old_state == observed) ? observed : VWALL;
	shell_search_write_wall_state(wall_data, x, y, dir, stored);
	int nx, ny;
	t_position pos = {(uint8_t)x, (uint8_t)y, dir};
	if(shell_search_neighbor(pos, dir, &nx, &ny) == True) {
		shell_search_write_wall_state(wall_data, nx, ny, shell_search_opposite(dir), stored);
	}
}

// Reproduce set_wall() using the uploaded snapshot as the sensor truth.  The
// write/contradiction and reciprocal-edge rules match wall_class::set_wall().
static t_bool shell_search_sense_from_truth(wall_class *wall_data,
		const wall_class *truth,t_position pos)
{
	const t_bool append_history = wall_data->is_unknown(pos.x, pos.y);
	t_bool complete_truth = True;
	for(int i = 0; i < 4; i++) {
		const t_wall_state observed = shell_search_wall_state(truth, pos.x, pos.y,
			shell_search_dirs[i]);
		if(observed == UNKNOWN) {
			complete_truth = False;
			continue;
		}
		shell_search_write_observation(wall_data, pos.x, pos.y,
			shell_search_dirs[i], observed);
	}
	if(append_history == True) {
		wall_data->wall_history.history_set(pos.x, pos.y, wall_data->wall[pos.x][pos.y]);
	}
	return complete_truth;
}

static void shell_search_replay_dump(wall_class *wall_data,const make_map *map_data)
{
	printf("REPLAY_DUMP_START\r\n");
	for(int x = 0; x < MAZE_SIZE_X; x++) {
		for(int y = 0; y < MAZE_SIZE_Y; y++) {
			if(y + 1 < MAZE_SIZE_Y && wall_data->get_virtual_wall(x, y, North) == True) {
				printf("REPLAY_VIRTUAL x:%d y:%d dir:N\r\n", x, y);
			}
			if(x + 1 < MAZE_SIZE_X && wall_data->get_virtual_wall(x, y, East) == True) {
				printf("REPLAY_VIRTUAL x:%d y:%d dir:E\r\n", x, y);
			}
		}
	}
	for(int y = MAZE_SIZE_Y - 1; y >= 0; y--) {
		printf("REPLAY_MAP y:%d values:", y);
		for(int x = 0; x < MAZE_SIZE_X; x++) {
			printf("%d%s", (int)map_data->map[x][y], x + 1 < MAZE_SIZE_X ? "," : "");
		}
		printf("\r\n");
	}
	printf("REPLAY_DUMP_END virtual_edges:%d map_rows:%d history:%d\r\n",
		shell_search_virtual_count(wall_data), MAZE_SIZE_Y,
		wall_data->wall_history.get_history_cnt());
}

static void shell_search_dump(wall_class *wall_data,const make_map *map_data)
{
	printf("SEARCH_DUMP_START\r\n");
	for(int x = 0; x < MAZE_SIZE_X; x++) {
		for(int y = 0; y < MAZE_SIZE_Y; y++) {
			if(y + 1 < MAZE_SIZE_Y && wall_data->get_virtual_wall(x, y, North) == True) {
				printf("SEARCH_VIRTUAL x:%d y:%d dir:N\r\n", x, y);
			}
			if(x + 1 < MAZE_SIZE_X && wall_data->get_virtual_wall(x, y, East) == True) {
				printf("SEARCH_VIRTUAL x:%d y:%d dir:E\r\n", x, y);
			}
		}
	}
	for(int y = MAZE_SIZE_Y - 1; y >= 0; y--) {
		printf("SEARCH_MAP y:%d values:", y);
		for(int x = 0; x < MAZE_SIZE_X; x++) {
			printf("%d%s", (int)map_data->map[x][y], x + 1 < MAZE_SIZE_X ? "," : "");
		}
		printf("\r\n");
	}
	printf("SEARCH_DUMP_END virtual_edges:%d map_rows:%d\r\n",
		shell_search_virtual_count(wall_data), MAZE_SIZE_Y);
}

/* ---------------------------------------------------------------
	help と info
--------------------------------------------------------------- */
static int usrcmd_help(int argc, char **argv)
{
    const cmd_table_t *p = &cmdlist[0];
    for (uint16_t i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        printf("  %s", p->cmd);
        printf("\t:");
        printf("  %s", p->desc);
        printf("\r\n");
        p++;
    }
    return 0;
}

static int usrcmd_info(int argc, char **argv)
{
    if (argc != 2) {
    	printf("info sys\r\n");
    	printf("info ver\r\n");
    	printf("info battery\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "sys") == 0) {
    	printf("prototype8\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "ver") == 0) {
    	printf("Version 0.0.0\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "battery") == 0) {
    	const float voltage = Battery_GetAverageVoltage();
    	const float limit = Battery_GetLimitVoltage();
    	printf("BATTERY voltage_mv:%ld limit_mv:%ld status:%s\r\n",
				(long)(voltage * 1000.0f), (long)(limit * 1000.0f),
				Battery_IsVoltageError(voltage) ? "ERROR" : "OK");
        return 0;
    }
    printf("Unknown sub command found\r\n");
    return -1;
}


static int usrcmd_disp(int argc, char **argv)
{
    if (argc != 2) {
    	printf("disp maze\r\n");
    	printf("disp maze_bin\r\n");
    	printf("disp history\r\n");
    	printf("disp log\r\n");
    	printf("disp log_bin\r\n");
    	return 0;
    }
    if (ntlibc_strcmp(argv[1], "maze") == 0) {
    	wall_class *wall_data = shell_wall_data();
    	shell_read_save_data(wall_data);
    	printf("MAZE_START\r\n");
    	wall_data->indicate_wall();
    	printf("MAZE_END\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "maze_bin") == 0) {
    	wall_class *wall_data = shell_wall_data();
    	shell_read_save_data(wall_data);
    	printf("MAZE_BIN_START\r\n");
    	wall_data->indicate_wall_binary();
    	printf("\r\nMAZE_BIN_END\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "history") == 0) {
    	wall_class *wall_data = shell_wall_data();
    	shell_read_save_data(wall_data);
    	wall_data->wall_history.history_indicate();
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "log") == 0) {
    	LogData::getInstance().indicate_data();
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "log_bin") == 0) {
    	LogData::getInstance().indicate_data_binary();
        return 0;
    }
    printf("Unknown sub command found\r\n");
    return -1;
}

static int usrcmd_load(int argc, char **argv)
{
    if (argc < 2) {
    	printf("load save\r\n");
    	printf("load maze_bin\r\n");
    	printf("load goal x y size\r\n");
    	return 0;
    }
    if (ntlibc_strcmp(argv[1], "save") == 0) {
    	wall_class *wall_data = shell_wall_data();
    	shell_read_save_data(wall_data);
    	printf("read_save_data done. history_cnt:%d\r\n", wall_data->wall_history.get_history_cnt());
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "maze_bin") == 0) {
    	wall_class *wall_data = shell_wall_data();
    	printf("MAZE_BIN_READY %d\r\n", MAZE_SIZE_X * MAZE_SIZE_Y);
    	shell_receive_maze_binary(wall_data);
    	printf("MAZE_BIN_LOAD_DONE\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "goal") == 0) {
    	if(argc != 5) {
    		printf("load goal x y size\r\n");
    		return 0;
    	}
    	int x = ntlibc_atoi(argv[2]);
    	int y = ntlibc_atoi(argv[3]);
    	int size = ntlibc_atoi(argv[4]);
    	if(shell_set_goal(x, y, size) != True) {
    		printf("GOAL_SET_ERROR\r\n");
    		return -1;
    	}
    	printf("GOAL_SET_DONE x:%d y:%d size:%d\r\n", shell_goal_x, shell_goal_y, shell_goal_size);
        return 0;
    }
    printf("Unknown sub command found\r\n");
    return -1;
}

static int usrcmd_path(int argc, char **argv)
{
    if (argc != 2) {
    	printf("path dijkstra\r\n");
		printf("path dijkstra_queue\r\n");
    	return 0;
    }
	t_bool priority_queue = False;
	if (ntlibc_strcmp(argv[1], "dijkstra_queue") == 0) {
		priority_queue = True;
	}
	if (ntlibc_strcmp(argv[1], "dijkstra") == 0 || priority_queue == True) {
    	wall_class *wall_data = shell_wall_data();
    	if(shell_wall_data_ready != True) {
    		shell_read_save_data(wall_data);
    	}

	Dijkstra run_path(wall_data);
    	t_position start, goal;
    	start.x = start.y = 0;
    	start.dir = North;
    	goal.x = shell_goal_x;
    	goal.y = shell_goal_y;

	run_path.st_param_set(st_mode_1000_v0, sizeof(st_mode_1000_v0) / sizeof(st_mode_1000_v0[0]));
	run_path.di_param_set(di_mode_1000_v0, sizeof(di_mode_1000_v0) / sizeof(di_mode_1000_v0[0]));
	run_path.turn_time_set(mode_1000);
    	printf("DIJKSTRA_GOAL x:%d y:%d size:%d\r\n", shell_goal_x, shell_goal_y, shell_goal_size);
		printf("DIJKSTRA_MODE PRIORITY_QUEUE%s\r\n", priority_queue == True ? "" : " (COMPAT_ALIAS)");
	printf("DIJKSTRA_START\r\n");
	run_path.check_run_Dijkstra(start, Dir_None, goal, shell_goal_size, True);
    	printf("DIJKSTRA_END\r\n");
        return 0;
    }
    printf("Unknown sub command found\r\n");
    return -1;
}

static int shell_search_select(adachi *algorithm,t_bool priority_second,
		t_position current,t_position goal,int mask,t_position *next)
{
	return priority_second == True ?
		algorithm->select_goal_aware_direction(current, goal, mask, next) :
		algorithm->select_next_direction(current, mask, next);
}

static int shell_search_replay(t_bool reset,t_bool accelerated,t_position start,
		t_position goal,int goal_size,t_bool full_search,t_bool priority_second,
		t_virtual_branch_mode branch_mode,int mask,int max_steps)
{
	wall_class *truth = shell_wall_data();
	if(shell_wall_data_ready != True) shell_read_save_data(truth);
	wall_class *wall_data = shell_replay_wall_data();
	make_map *map_data = shell_search_map_data(wall_data);
	if(reset == True) {
		wall_data->init_maze();
		shell_replay_maze_start = start;
		shell_replay_ready = True;
	} else if(shell_replay_ready != True) {
		printf("REPLAY_ERROR keep_without_reset\r\n");
		return -1;
	}

	map_data->init_map(goal.x, goal.y, goal_size);
	wall_data->clear_virtual_wall();
	t_position maze_goal = {shell_goal_x, shell_goal_y, North};
	t_virtual_wall_context context = {shell_replay_maze_start, start,
		maze_goal, shell_goal_size, branch_mode};
	Search_UpdateMap(wall_data, map_data, context, start, goal, goal_size,
		full_search, mask);
	adachi algorithm(wall_data, map_data);
	t_position current = start;
	int steps = 0;
	int sensed_cells = 0;
	const char *result = "max_steps";

	printf("REPLAY_START state:%s motion:%s start:%d,%d,%s goal:%d,%d,%d "
		"maze_start:%d,%d maze_goal:%d,%d,%d mode:%s "
		"priority:%s mask:%d max_steps:%d\r\n",
		reset == True ? "reset" : "keep", accelerated == True ? "acc" : "plain",
		start.x, start.y, shell_search_direction_name(start.dir),
		goal.x, goal.y, goal_size,
		shell_replay_maze_start.x, shell_replay_maze_start.y,
		maze_goal.x, maze_goal.y, shell_goal_size,
		branch_mode == VIRTUAL_BRANCH_UNKNOWN_OPEN ? "full_prune" :
		(full_search == True ? "full" : "goal"),
		priority_second == True ? "second" : "first", mask, max_steps);

	while(steps < max_steps) {
		if(shell_search_is_goal(current, goal, goal_size) == True) {
			result = "goal";
			break;
		}

		int sensed = 0;
		// run_goal_search() senses every arrived cell. run_accelerated_goal_search() only
		// senses it while some edge is unknown. Neither senses the start before
		// selecting the first move.
		if(steps > 0 && (accelerated != True || wall_data->is_unknown(current.x, current.y) == True)) {
			if(shell_search_sense_from_truth(wall_data, truth, current) != True) {
				printf("REPLAY_ERROR truth_unknown index:%d pos:%d,%d,%s\r\n",
					steps, current.x, current.y, shell_search_direction_name(current.dir));
				result = "truth_unknown";
				break;
			}
			sensed = 1;
			sensed_cells++;
		}

		int map_values[4];
		int wall_values[4];
		int virtual_values[4];
		const int self_value = (int)map_data->map[current.x][current.y];
		for(int i = 0; i < 4; i++) {
			int nx, ny;
			map_values[i] = shell_search_neighbor(current, shell_search_dirs[i], &nx, &ny) == True ?
				(int)shell_search_map_value(map_data, nx, ny) : MAZE_SIZE;
			wall_values[i] = (int)shell_search_wall_state(wall_data, current.x, current.y,
				shell_search_dirs[i]);
			virtual_values[i] = wall_data->get_virtual_wall(current.x, current.y,
				shell_search_dirs[i]) == True ? 1 : 0;
		}

		if(shell_search_has_candidate(wall_data, map_data, current, mask) != True) {
			printf("REPLAY_STEP index:%d pos:%d,%d,%s self:%d map:%d,%d,%d,%d "
				"wall:%d,%d,%d,%d vwall:%d,%d,%d,%d sensed:%d next:%d,%d,NONE "
				"local:%d truth:%d selected_vwall:0 next_acc:%d virtual_edges:%d\r\n",
				steps, current.x, current.y, shell_search_direction_name(current.dir),
				self_value,
				map_values[0], map_values[1], map_values[2], map_values[3],
				wall_values[0], wall_values[1], wall_values[2], wall_values[3],
				virtual_values[0], virtual_values[1], virtual_values[2], virtual_values[3],
				sensed, current.x, current.y, None, UNKNOWN, None,
				shell_search_virtual_count(wall_data));
			result = "no_candidate";
			break;
		}

		t_position next = current;
		const int local_dir = shell_search_select(&algorithm, priority_second,
			current, goal, mask, &next);
		if(next.x >= MAZE_SIZE_X || next.y >= MAZE_SIZE_Y ||
			(next.dir != North && next.dir != East && next.dir != South && next.dir != West)) {
			printf("REPLAY_ERROR invalid_next index:%d\r\n", steps);
			result = "invalid_next";
			break;
		}

		int next_acc_dir = None;
		if(accelerated == True && wall_data->is_unknown(next.x, next.y) == False &&
			shell_search_is_goal(next, goal, goal_size) != True) {
			t_position next_acc = next;
			next_acc_dir = shell_search_select(&algorithm, priority_second,
				next, goal, mask, &next_acc);
		}

		const t_wall_state truth_selected = shell_search_wall_state(truth,
			current.x, current.y, next.dir);
		// Same order as every Search motion helper:
		// Init_Motion_* -> update_map -> execute_Motion(no-op here).
		context.mouse = current;
		Search_UpdateMap(wall_data, map_data, context, next, goal, goal_size,
			full_search, mask);
		const int selected_virtual = wall_data->get_virtual_wall(current.x,
			current.y, next.dir) == True ? 1 : 0;

		printf("REPLAY_STEP index:%d pos:%d,%d,%s self:%d map:%d,%d,%d,%d "
			"wall:%d,%d,%d,%d vwall:%d,%d,%d,%d sensed:%d next:%d,%d,%s "
			"local:%d truth:%d selected_vwall:%d next_acc:%d virtual_edges:%d\r\n",
			steps, current.x, current.y, shell_search_direction_name(current.dir),
			self_value,
			map_values[0], map_values[1], map_values[2], map_values[3],
			wall_values[0], wall_values[1], wall_values[2], wall_values[3],
			virtual_values[0], virtual_values[1], virtual_values[2], virtual_values[3],
			sensed, next.x, next.y, shell_search_direction_name(next.dir), local_dir,
			(int)truth_selected, selected_virtual, next_acc_dir,
			shell_search_virtual_count(wall_data));

		if(truth_selected == WALL || truth_selected == VWALL) {
			result = "truth_collision";
			break;
		}
		if(truth_selected == UNKNOWN) {
			result = "truth_unknown";
			break;
		}
		if(selected_virtual != 0) {
			result = "selected_edge_closed";
			break;
		}

		// execute_Motion() is intentionally a no-op. Arrival happens here.
		current = next;
		steps++;
	}

	if(shell_search_is_goal(current, goal, goal_size) == True) result = "goal";
	if(ntlibc_strcmp(result, "goal") == 0) {
		// Both Search variants call set_wall() once more at the goal.
		if(shell_search_sense_from_truth(wall_data, truth, current) == True) sensed_cells++;
		else result = "truth_unknown";
	}
	printf("REPLAY_END result:%s steps:%d final:%d,%d,%s sensed:%d history:%d virtual_edges:%d\r\n",
		result, steps, current.x, current.y, shell_search_direction_name(current.dir),
		sensed_cells, wall_data->wall_history.get_history_cnt(),
		shell_search_virtual_count(wall_data));
	shell_search_replay_dump(wall_data, map_data);
	return ntlibc_strcmp(result, "goal") == 0 ? 0 : -1;
}

static int usrcmd_search(int argc, char **argv)
{
	if(argc == 14 && ntlibc_strcmp(argv[1], "replay") == 0) {
		const t_bool reset = ntlibc_strcmp(argv[2], "reset") == 0 ? True : False;
		const t_bool keep = ntlibc_strcmp(argv[2], "keep") == 0 ? True : False;
		const t_bool accelerated = ntlibc_strcmp(argv[3], "acc") == 0 ? True : False;
		const t_bool plain = ntlibc_strcmp(argv[3], "plain") == 0 ? True : False;
		const int start_x = ntlibc_atoi(argv[4]);
		const int start_y = ntlibc_atoi(argv[5]);
		const t_direction start_dir = shell_search_parse_direction(argv[6]);
		const int goal_x = ntlibc_atoi(argv[7]);
		const int goal_y = ntlibc_atoi(argv[8]);
		const int goal_size = ntlibc_atoi(argv[9]);
		const t_bool prune_full = ntlibc_strcmp(argv[10], "full_prune") == 0 ? True : False;
		const t_bool full_search = (ntlibc_strcmp(argv[10], "full") == 0 || prune_full == True) ? True : False;
		const t_bool goal_mode = ntlibc_strcmp(argv[10], "goal") == 0 ? True : False;
		const t_bool priority_second = ntlibc_strcmp(argv[11], "second") == 0 ? True : False;
		const t_bool priority_first = ntlibc_strcmp(argv[11], "first") == 0 ? True : False;
		const int mask = ntlibc_atoi(argv[12]);
		const int max_steps = ntlibc_atoi(argv[13]);
		if((reset != True && keep != True) || (accelerated != True && plain != True) ||
			start_x < 0 || start_x >= MAZE_SIZE_X || start_y < 0 || start_y >= MAZE_SIZE_Y ||
			start_dir == Dir_None || goal_x < 0 || goal_y < 0 || goal_size <= 0 ||
			goal_x + goal_size > MAZE_SIZE_X || goal_y + goal_size > MAZE_SIZE_Y ||
			(full_search != True && goal_mode != True) ||
			(priority_second != True && priority_first != True) ||
			(mask != 0x01 && mask != 0x03) || max_steps <= 0 || max_steps > MAZE_SIZE * 4) {
			printf("REPLAY_ERROR invalid_argument\r\n");
			return -1;
		}
		t_position start = {(uint8_t)start_x, (uint8_t)start_y, start_dir};
		t_position goal = {(uint8_t)goal_x, (uint8_t)goal_y, North};
		return shell_search_replay(reset, accelerated, start, goal, goal_size,
			full_search, priority_second,
			prune_full == True ? VIRTUAL_BRANCH_UNKNOWN_OPEN : VIRTUAL_BRANCH_OBSERVED_ONLY,
			mask, max_steps);
	}
	if(argc != 9 || ntlibc_strcmp(argv[1], "run") != 0) {
		printf("search run start_x start_y N|E|S|W goal|full|full_prune first|second mask max_steps\r\n");
		printf("search replay reset|keep plain|acc start_x start_y N|E|S|W goal_x goal_y goal_size goal|full|full_prune first|second mask max_steps\r\n");
		return 0;
	}

	const int start_x = ntlibc_atoi(argv[2]);
	const int start_y = ntlibc_atoi(argv[3]);
	const t_direction start_dir = shell_search_parse_direction(argv[4]);
	const t_bool prune_full = ntlibc_strcmp(argv[5], "full_prune") == 0 ? True : False;
	const t_bool full_search = (ntlibc_strcmp(argv[5], "full") == 0 || prune_full == True) ? True : False;
	const t_bool goal_mode = ntlibc_strcmp(argv[5], "goal") == 0 ? True : False;
	const t_bool priority_second = ntlibc_strcmp(argv[6], "second") == 0 ? True : False;
	const t_bool priority_first = ntlibc_strcmp(argv[6], "first") == 0 ? True : False;
	const int mask = ntlibc_atoi(argv[7]);
	const int max_steps = ntlibc_atoi(argv[8]);

	if(start_x < 0 || start_x >= MAZE_SIZE_X || start_y < 0 || start_y >= MAZE_SIZE_Y ||
		start_dir == Dir_None || (full_search != True && goal_mode != True) ||
		(priority_first != True && priority_second != True) ||
		(mask != 0x01 && mask != 0x03) || max_steps <= 0 || max_steps > MAZE_SIZE * 4) {
		printf("SEARCH_ERROR invalid_argument\r\n");
		return -1;
	}

	wall_class *wall_data = shell_wall_data();
	if(shell_wall_data_ready != True) {
		shell_read_save_data(wall_data);
	}

	make_map *map_data = shell_search_map_data(wall_data);
	adachi search_algorithm(wall_data, map_data);
	t_position start = {(uint8_t)start_x, (uint8_t)start_y, start_dir};
	t_position current = start;
	t_position goal = {shell_goal_x, shell_goal_y, North};
	wall_data->clear_virtual_wall();
	shell_search_update_map(wall_data, map_data, start, current, current,
		full_search,
		prune_full == True ? VIRTUAL_BRANCH_UNKNOWN_OPEN : VIRTUAL_BRANCH_OBSERVED_ONLY,
		mask);

	printf("SEARCH_RUN_START start:%d,%d,%s goal:%d,%d,%d mode:%s priority:%s mask:%d max_steps:%d\r\n",
		start.x, start.y, shell_search_direction_name(start.dir),
		shell_goal_x, shell_goal_y, shell_goal_size,
		prune_full == True ? "full_prune" : (full_search == True ? "full" : "goal"),
		priority_second == True ? "second" : "first", mask, max_steps);

	const char *result = "max_steps";
	int steps = 0;
	while(steps < max_steps) {
		if(shell_search_is_goal(current) == True) {
			result = "goal";
			break;
		}

		int map_values[4];
		int wall_values[4];
		int virtual_values[4];
		const int self_value = (int)map_data->map[current.x][current.y];
		for(int i = 0; i < 4; i++) {
			int nx, ny;
			map_values[i] = shell_search_neighbor(current, shell_search_dirs[i], &nx, &ny) == True ?
				(int)shell_search_map_value(map_data, nx, ny) : MAZE_SIZE;
			t_position edge = current;
			edge.dir = shell_search_dirs[i];
			wall_values[i] = (int)wall_data->get_WallState(edge);
			virtual_values[i] = wall_data->get_virtual_wall(current.x, current.y,
				shell_search_dirs[i]) == True ? 1 : 0;
		}

		if(shell_search_has_candidate(wall_data, map_data, current, mask) != True) {
			printf("SEARCH_STEP index:%d pos:%d,%d,%s self:%d map:%d,%d,%d,%d wall:%d,%d,%d,%d "
				"vwall:%d,%d,%d,%d next:%d,%d,NONE local:%d found:0 selected_vwall:0 virtual_edges:%d\r\n",
				steps, current.x, current.y, shell_search_direction_name(current.dir),
				self_value,
				map_values[0], map_values[1], map_values[2], map_values[3],
				wall_values[0], wall_values[1], wall_values[2], wall_values[3],
				virtual_values[0], virtual_values[1], virtual_values[2], virtual_values[3],
				current.x, current.y, None, shell_search_virtual_count(wall_data));
			result = "no_candidate";
			break;
		}

		t_position next = current;
		const int local_dir = priority_second == True ?
			search_algorithm.select_goal_aware_direction(current, goal, mask, &next) :
			search_algorithm.select_next_direction(current, mask, &next);
		if(next.x >= MAZE_SIZE_X || next.y >= MAZE_SIZE_Y ||
			(next.dir != North && next.dir != East && next.dir != South && next.dir != West)) {
			printf("SEARCH_ERROR invalid_next index:%d\r\n", steps);
			result = "invalid_next";
			break;
		}

		shell_search_update_map(wall_data, map_data, start, current, next,
			full_search,
			prune_full == True ? VIRTUAL_BRANCH_UNKNOWN_OPEN : VIRTUAL_BRANCH_OBSERVED_ONLY,
			mask);
		const int selected_virtual = wall_data->get_virtual_wall(current.x, current.y, next.dir) == True ? 1 : 0;
		printf("SEARCH_STEP index:%d pos:%d,%d,%s self:%d map:%d,%d,%d,%d wall:%d,%d,%d,%d "
			"vwall:%d,%d,%d,%d next:%d,%d,%s local:%d found:1 selected_vwall:%d virtual_edges:%d\r\n",
			steps, current.x, current.y, shell_search_direction_name(current.dir),
			self_value,
			map_values[0], map_values[1], map_values[2], map_values[3],
			wall_values[0], wall_values[1], wall_values[2], wall_values[3],
			virtual_values[0], virtual_values[1], virtual_values[2], virtual_values[3],
			next.x, next.y, shell_search_direction_name(next.dir), local_dir,
			selected_virtual, shell_search_virtual_count(wall_data));

		if(selected_virtual != 0) {
			result = "selected_edge_closed";
			break;
		}
		current = next;
		steps++;
	}

	if(shell_search_is_goal(current) == True) result = "goal";
	printf("SEARCH_RUN_END result:%s steps:%d final:%d,%d,%s virtual_edges:%d\r\n",
		result, steps, current.x, current.y, shell_search_direction_name(current.dir),
		shell_search_virtual_count(wall_data));
	shell_search_dump(wall_data, map_data);
	return ntlibc_strcmp(result, "goal") == 0 ? 0 : -1;
}

static int usrcmd_end(int argc, char **argv)
{
    if (argc != 2) {
    	printf("end exe\r\n");
    	return 0;
    }
    if (ntlibc_strcmp(argv[1], "exe") == 0) {
    	shell_end_flag = True;
    	printf("shell_end!\r\n");
        return 0;
    }

    printf("Unknown sub command found\r\n");
    return -1;
}

static int usrcmd_log(int argc, char **argv)
{
    if (argc != 2) {
		printf("log init\r\n");
		printf("log mode0\r\n");
		printf("log mode1\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "init") == 0) {
		LogData::getInstance().init_log();
		printf("LOG_INIT_DONE\r\n");
		return 0;
	}
    if (ntlibc_strcmp(argv[1], "mode0") == 0) {
    	LogData::getInstance().set_logmode(0);
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "mode1") == 0) {
        	LogData::getInstance().set_logmode(1);
            return 0;
    }

    printf("Unknown sub command found\r\n");
    return -1;
}


typedef struct {
	float distance;
	float acc;
	float max_velo;
	float end_velo;
	t_pid_gain sp_gain;
	t_pid_gain om_gain;
	t_ff_gain ff_gain;
	t_bool suction_enable;
	int suction_duty;
} shell_debug_param_t;

static shell_debug_param_t shell_debug_straight_param = {
	90.0f * 4.0f, 6.5f, 0.7f, 0.0f,
	{2.0f, 0.05f, 0.0f}, {0.1f, 0.01f, 0.0f},
	{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_OM_ACCEL_COEF, FF_OM_BIAS_COEF}, False, 650
};
static shell_debug_param_t shell_debug_diagonal_param = {
	63.63f * 6.0f, 6.5f, 0.7f, 0.0f,
	{2.0f, 0.05f, 0.0f}, {0.1f, 0.01f, 0.0f},
	{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_OM_ACCEL_COEF, FF_OM_BIAS_COEF}, False, 650
};

typedef struct {
	t_turn_param_table table;
	t_pid_gain sp_gain;
	t_pid_gain om_gain;
	t_ff_gain ff_gain;
	t_param param;
	t_bool initialized;
	t_bool suction_enable;
	int suction_duty;
	int preset_speed;
	int turn_count;
	float pre_accel;
} shell_debug_turn_param_t;

static shell_debug_turn_param_t shell_debug_turn_params[Turn_LV90 + 1];

typedef struct {
	const char *name;
	t_run_pattern pattern;
} shell_debug_turn_name_t;

static const shell_debug_turn_name_t shell_debug_turn_names[] = {
	{"long_r90", Long_turnR90}, {"long_l90", Long_turnL90},
	{"long_r180", Long_turnR180}, {"long_l180", Long_turnL180},
	{"in_r45", Turn_in_R45}, {"in_l45", Turn_in_L45},
	{"out_r45", Turn_out_R45}, {"out_l45", Turn_out_L45},
	{"in_r135", Turn_in_R135}, {"in_l135", Turn_in_L135},
	{"out_r135", Turn_out_R135}, {"out_l135", Turn_out_L135},
	{"r_v90", Turn_RV90}, {"l_v90", Turn_LV90}
};

#define SHELL_TURNPATTERN_MAX 20

typedef struct {
	t_run_pattern turns[SHELL_TURNPATTERN_MAX];
	uint8_t count;
	int preset_speed;
	float turn_velo;
	float pre_accel;
	float post_accel;
	t_bool suction_enable;
	int suction_duty;
} shell_turnpattern_t;

static shell_turnpattern_t shell_turnpattern = {
	{No_run}, 0, 700, 0.7f, 6.5f, 6.5f, False, 650
};

static const char *shell_debug_turn_name(t_run_pattern pattern)
{
	for (uint16_t i = 0; i < sizeof(shell_debug_turn_names) / sizeof(shell_debug_turn_names[0]); i++) {
		if (shell_debug_turn_names[i].pattern == pattern) return shell_debug_turn_names[i].name;
	}
	return "unknown";
}

// 0: straight, 1: diagonal, -1: unsupported
static int shell_turnpattern_input_state(t_run_pattern pattern)
{
	switch (pattern) {
	case Long_turnR90: case Long_turnL90: case Long_turnR180: case Long_turnL180:
	case Turn_in_R45: case Turn_in_L45: case Turn_in_R135: case Turn_in_L135:
		return 0;
	case Turn_out_R45: case Turn_out_L45: case Turn_out_R135: case Turn_out_L135:
	case Turn_RV90: case Turn_LV90:
		return 1;
	default:
		return -1;
	}
}

static int shell_turnpattern_output_state(t_run_pattern pattern)
{
	switch (pattern) {
	case Turn_in_R45: case Turn_in_L45: case Turn_in_R135: case Turn_in_L135:
	case Turn_RV90: case Turn_LV90:
		return 1;
	case Long_turnR90: case Long_turnL90: case Long_turnR180: case Long_turnL180:
	case Turn_out_R45: case Turn_out_L45: case Turn_out_R135: case Turn_out_L135:
		return 0;
	default:
		return -1;
	}
}

static t_bool shell_turnpattern_validate(void)
{
	if (shell_turnpattern.count == 0 || shell_turnpattern.count > SHELL_TURNPATTERN_MAX) return False;
	int state = shell_turnpattern_input_state(shell_turnpattern.turns[0]);
	if (state < 0) return False;
	for (uint8_t i = 0; i < shell_turnpattern.count; i++) {
		if (shell_turnpattern_input_state(shell_turnpattern.turns[i]) != state) return False;
		state = shell_turnpattern_output_state(shell_turnpattern.turns[i]);
		if (state < 0) return False;
	}
	return True;
}

static t_bool shell_parse_float(const char *text, float *value)
{
	if (text == NULL || value == NULL || *text == '\0') return False;

	const char *p = text;
	float sign = 1.0f;
	if (*p == '-' || *p == '+') {
		if (*p == '-') sign = -1.0f;
		p++;
	}

	float result = 0.0f;
	int digits = 0;
	while (*p >= '0' && *p <= '9') {
		result = result * 10.0f + (float)(*p - '0');
		p++;
		digits++;
	}
	if (*p == '.') {
		float place = 0.1f;
		p++;
		while (*p >= '0' && *p <= '9') {
			result += (float)(*p - '0') * place;
			place *= 0.1f;
			p++;
			digits++;
		}
	}
	if (digits == 0) return False;

	int exponent = 0;
	if (*p == 'e' || *p == 'E') {
		p++;
		int exponent_sign = 1;
		if (*p == '-' || *p == '+') {
			if (*p == '-') exponent_sign = -1;
			p++;
		}
		int exponent_digits = 0;
		while (*p >= '0' && *p <= '9') {
			exponent = exponent * 10 + (*p - '0');
			p++;
			exponent_digits++;
			if (exponent > 38) return False;
		}
		if (exponent_digits == 0) return False;
		exponent *= exponent_sign;
	}
	if (*p != '\0') return False;

	while (exponent > 0) { result *= 10.0f; exponent--; }
	while (exponent < 0) { result *= 0.1f; exponent++; }
	*value = sign * result;
	return True;
}

static void shell_debug_wait_sensor(IrSensTask *irsens)
{
	// The board has six LEDs. Return_LED_Status() therefore returns 0x00-0x3f.
	const uint8_t wait_led = 0x3f;
	uint32_t time = Interrupt::getInstance().return_time_count();
	while (irsens->IrSensor_Avg() < 2000) {
		uint32_t current_time = Interrupt::getInstance().return_time_count();
		if ((current_time - time) > 400) {
			time = current_time;
			Indicate_LED((Return_LED_Status() != wait_led) ? wait_led : 0x00);
		}
		HAL_Delay(1);
	}
	Indicate_LED(wait_led);
}

static t_run_pattern shell_debug_turn_pattern(const char *name)
{
	for (uint16_t i = 0; i < sizeof(shell_debug_turn_names) / sizeof(shell_debug_turn_names[0]); i++) {
		if (ntlibc_strcmp(name, shell_debug_turn_names[i].name) == 0) {
			return shell_debug_turn_names[i].pattern;
		}
	}
	return No_run;
}

static const t_param *const *shell_debug_turn_mode(int speed)
{
	switch (speed) {
	case 300: return mode_300;
	case 500: return mode_500;
	case 700: return mode_700;
	case 1000: return mode_1000;
	case 1200: return mode_1200;
	case 1400: return mode_1400;
	case 1500: return mode_1500;
	case 1600: return mode_1600;
	case 1800: return mode_1800;
	case 2000: return mode_2000;
	default: return NULL;
	}
}

static float shell_debug_turn_default_pre_accel(t_run_pattern pattern)
{
	switch (pattern) {
	case Turn_out_R45:
	case Turn_out_L45:
	case Turn_out_R135:
	case Turn_out_L135:
	case Turn_RV90:
	case Turn_LV90:
		return shell_debug_diagonal_param.acc;
	default:
		return shell_debug_straight_param.acc;
	}
}

static shell_debug_turn_param_t *shell_debug_turn_get(t_run_pattern pattern, int preset_speed, t_bool reset)
{
	const t_param *const *turn_mode = shell_debug_turn_mode(preset_speed);
	if (pattern < Long_turnR90 || pattern > Turn_LV90 || turn_mode == NULL || turn_mode[pattern] == NULL) {
		return NULL;
	}
	shell_debug_turn_param_t *debug = &shell_debug_turn_params[pattern];
	if (debug->initialized != True || reset == True) {
		debug->table = *turn_mode[pattern]->param;
		debug->sp_gain = *turn_mode[pattern]->sp_gain;
		debug->om_gain = *turn_mode[pattern]->om_gain;
		debug->ff_gain = *(turn_mode[pattern]->ff_gain != nullptr ? turn_mode[pattern]->ff_gain : &ff_gain_default);
		debug->param.param = &debug->table;
		debug->param.sp_gain = &debug->sp_gain;
		debug->param.om_gain = &debug->om_gain;
		debug->param.ff_gain = &debug->ff_gain;
		debug->suction_enable = False;
		debug->suction_duty = 650;
		debug->preset_speed = preset_speed;
		debug->turn_count = 1;
		debug->pre_accel = shell_debug_turn_default_pre_accel(pattern);
		debug->initialized = True;
	}
	return debug;
}

static void shell_debug_turn_show(const char *name, const shell_debug_turn_param_t *debug)
{
	printf("DEBUG_TURN_PARAM_X1000 %s velo:%ld r_min:%ld Lstart:%ld Lend:%ld degree:%ld correction:%ld pre_accel:%ld "
		   "sp_u:%ld,%ld,%ld om_u:%ld,%ld,%ld ff_u:%ld,%ld,%ld,%ld,%ld,%ld,%ld jerk_n:%ld turn_sp_n:%ld,%ld suction:%d duty:%d preset:%d count:%d\r\n",
		   name, (long)(debug->table.velo * 1000.0f), (long)(debug->table.r_min * 1000.0f),
		   (long)(debug->table.Lstart * 1000.0f), (long)(debug->table.Lend * 1000.0f),
		   (long)(debug->table.degree * 1000.0f), (long)(debug->table.degree_correction * 1000.0f),
		   (long)(debug->pre_accel * 1000.0f),
		   (long)(debug->sp_gain.Kp * 1000000.0f), (long)(debug->sp_gain.Ki * 1000000.0f),
		   (long)(debug->sp_gain.Kd * 1000000.0f), (long)(debug->om_gain.Kp * 1000000.0f),
		   (long)(debug->om_gain.Ki * 1000000.0f), (long)(debug->om_gain.Kd * 1000000.0f),
		   (long)(debug->ff_gain.sp_velo * 1000000.0f), (long)(debug->ff_gain.sp_accel * 1000000.0f),
		   (long)(debug->ff_gain.sp_bias * 1000000.0f), (long)(debug->ff_gain.om_velo * 1000000.0f),
		   (long)(debug->ff_gain.om_accel * 1000000.0f), (long)(debug->ff_gain.om_decel * 1000000.0f),
		   (long)(debug->ff_gain.om_bias * 1000000.0f), (long)(debug->ff_gain.om_jerk * 1000000000.0f),
		   (long)(debug->ff_gain.sp_turn_accel_mag * 1000000000.0f),
		   (long)(debug->ff_gain.sp_turn_velo_sq * 1000000000.0f),
		   debug->suction_enable, debug->suction_duty, debug->preset_speed, debug->turn_count);
}

static void shell_debug_suction_start(Motion *motion, t_bool enable, int duty)
{
	if (enable != True) return;
	float suction_voltage = duty / 1000.0f * 7.20f;
	int stay_time = (int)(suction_voltage / SUCTION_ACC) + 300;
	motion->exe_Motion_suction_start(suction_voltage, stay_time);
}

static float shell_debug_accel_in_length(float target_velo, float length_mm, float configured_accel)
{
	if (target_velo <= 0.0f || length_mm <= 0.0f) return configured_accel;
	const float required_accel = target_velo * target_velo / (2.0f * length_mm / 1000.0f);
	return (required_accel > configured_accel) ? required_accel : configured_accel;
}

static t_bool shell_debug_turn_pre_run(Motion *motion, t_run_pattern pattern, float turn_velo, float configured_accel)
{
	switch (pattern) {
	case Long_turnR90:
	case Long_turnL90:
		motion->exe_Motion_straight(SECTION * 2.0f,
			shell_debug_accel_in_length(turn_velo, SECTION * 2.0f, configured_accel),
			turn_velo, turn_velo, &shell_debug_straight_param.sp_gain, &shell_debug_straight_param.om_gain,
			&shell_debug_straight_param.ff_gain);
		return True;

	case Long_turnR180:
	case Long_turnL180:
	case Turn_in_R45:
	case Turn_in_L45:
	case Turn_in_R135:
	case Turn_in_L135:
		motion->exe_Motion_straight(SECTION,
			shell_debug_accel_in_length(turn_velo, SECTION, configured_accel),
			turn_velo, turn_velo, &shell_debug_straight_param.sp_gain, &shell_debug_straight_param.om_gain,
			&shell_debug_straight_param.ff_gain);
		return True;

	case Turn_out_R45:
	case Turn_out_L45:
	case Turn_out_R135:
	case Turn_out_L135:
	case Turn_RV90:
	case Turn_LV90:
		motion->exe_Motion_diagonal(DIAG_SECTION * 2.0f,
			shell_debug_accel_in_length(turn_velo, DIAG_SECTION * 2.0f, configured_accel),
			turn_velo, turn_velo, &shell_debug_diagonal_param.sp_gain, &shell_debug_diagonal_param.om_gain,
			&shell_debug_diagonal_param.ff_gain);
		return True;

	default:
		return False;
	}
}

static t_bool shell_debug_turn_post_run(Motion *motion, t_run_pattern pattern, float turn_velo)
{
	switch (pattern) {
	case Long_turnR90:
	case Long_turnL90:
		motion->exe_Motion_straight(SECTION * 2.0f, shell_debug_straight_param.acc,
			turn_velo, 0.0f, &shell_debug_straight_param.sp_gain, &shell_debug_straight_param.om_gain,
			&shell_debug_straight_param.ff_gain);
		return True;

	case Long_turnR180:
	case Long_turnL180:
	case Turn_out_R45:
	case Turn_out_L45:
	case Turn_out_R135:
	case Turn_out_L135:
		motion->exe_Motion_straight(SECTION, shell_debug_straight_param.acc,
			turn_velo, 0.0f, &shell_debug_straight_param.sp_gain, &shell_debug_straight_param.om_gain,
			&shell_debug_straight_param.ff_gain);
		return True;

	case Turn_in_R45:
	case Turn_in_L45:
	case Turn_in_R135:
	case Turn_in_L135:
		motion->exe_Motion_diagonal(DIAG_SECTION * 2.0f, shell_debug_diagonal_param.acc,
			turn_velo, 0.0f, &shell_debug_diagonal_param.sp_gain, &shell_debug_diagonal_param.om_gain,
			&shell_debug_diagonal_param.ff_gain);
		return True;

	case Turn_RV90:
	case Turn_LV90:
		motion->exe_Motion_diagonal(DIAG_SECTION, shell_debug_diagonal_param.acc,
			turn_velo, 0.0f, &shell_debug_diagonal_param.sp_gain, &shell_debug_diagonal_param.om_gain,
			&shell_debug_diagonal_param.ff_gain);
		return True;

	default:
		return False;
	}
}

static int shell_debug_turn_command(int argc, char **argv)
{
	if (argc < 4) {
		printf("debug turn type show|reset|preset|set|exe\r\n");
		return 0;
	}
	t_run_pattern pattern = shell_debug_turn_pattern(argv[2]);
	if (pattern == No_run) {
		printf("DEBUG_TURN_ERROR unknown_type:%s\r\n", argv[2]);
		return -1;
	}
	int preset_speed = 700;
	t_bool reset = (ntlibc_strcmp(argv[3], "reset") == 0) ? True : False;
	if (ntlibc_strcmp(argv[3], "preset") == 0) {
		if (argc != 5) { printf("debug turn %s preset speed\r\n", argv[2]); return -1; }
		preset_speed = ntlibc_atoi(argv[4]);
		reset = True;
	} else if (shell_debug_turn_params[pattern].initialized == True) {
		preset_speed = shell_debug_turn_params[pattern].preset_speed;
	}
	shell_debug_turn_param_t *debug = shell_debug_turn_get(pattern, preset_speed, reset);
	if (debug == NULL) {
		printf("DEBUG_TURN_ERROR no_default\r\n");
		return -1;
	}
	if (ntlibc_strcmp(argv[3], "show") == 0 || ntlibc_strcmp(argv[3], "reset") == 0 ||
		ntlibc_strcmp(argv[3], "preset") == 0) {
		if (ntlibc_strcmp(argv[3], "reset") == 0) printf("DEBUG_TURN_RESET_DONE\r\n");
		if (ntlibc_strcmp(argv[3], "preset") == 0) printf("DEBUG_TURN_PRESET_DONE speed:%d\r\n", preset_speed);
		shell_debug_turn_show(argv[2], debug);
		return 0;
	}
	if (ntlibc_strcmp(argv[3], "set") == 0) {
		if (argc != 20 && argc != 24 && argc != 25 && argc != 26 && argc != 27 && argc != 28 && argc != 30) {
			printf("debug turn %s set velo r_min Lstart Lend degree degree_correction pre_accel sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_sp_bias ff_om_velo ff_om_accel ff_om_decel ff_om_bias ff_om_jerk ff_sp_turn_accel_mag ff_sp_turn_velo_sq] suction_enable suction_duty turn_count\r\n", argv[2]);
			return -1;
		}
		shell_debug_turn_param_t next = *debug;
		float *legacy_values[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart,
			&next.table.Lend, &next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki,
			&next.sp_gain.Kd, &next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.om_velo, &next.ff_gain.om_accel,
			&next.ff_gain.sp_bias, &next.ff_gain.om_bias};
		float *full_values[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart,
			&next.table.Lend, &next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki,
			&next.sp_gain.Kd, &next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.sp_bias, &next.ff_gain.om_velo,
			&next.ff_gain.om_accel, &next.ff_gain.om_decel, &next.ff_gain.om_bias};
		float *full_values_with_jerk[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart,
			&next.table.Lend, &next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki,
			&next.sp_gain.Kd, &next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.sp_bias, &next.ff_gain.om_velo,
			&next.ff_gain.om_accel, &next.ff_gain.om_decel, &next.ff_gain.om_bias, &next.ff_gain.om_jerk};
		float *full_values_with_turn_sp[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart,
			&next.table.Lend, &next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki,
			&next.sp_gain.Kd, &next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.sp_bias, &next.ff_gain.om_velo,
			&next.ff_gain.om_accel, &next.ff_gain.om_decel, &next.ff_gain.om_bias, &next.ff_gain.om_jerk,
			&next.ff_gain.sp_turn_accel_mag, &next.ff_gain.sp_turn_velo_sq};
		float **values = (argc == 30) ? full_values_with_turn_sp : ((argc == 28) ? full_values_with_jerk : ((argc == 27) ? full_values : legacy_values));
		const int value_count = (argc == 30) ? 23 : ((argc == 28) ? 21 : ((argc == 27) ? 20 : ((argc == 26) ? 19 : ((argc == 25) ? 18 : ((argc == 24) ? 17 : 13)))));
		for (int i = 0; i < value_count; i++) {
			if (shell_parse_float(argv[i + 4], values[i]) != True) {
				printf("DEBUG_TURN_ERROR invalid_number:%s\r\n", argv[i + 4]);
				return -1;
			}
		}
		if (argc != 27 && argc != 28 && argc != 30) next.ff_gain.om_decel = next.ff_gain.om_accel;
		if (argc != 28 && argc != 30) next.ff_gain.om_jerk = 0.0f;
		int suction_enable;
		int suction_duty;
		int turn_count;
		const int option_index = (argc == 30) ? 27 : ((argc == 28) ? 25 : ((argc == 27) ? 24 : ((argc == 26) ? 23 : ((argc == 25) ? 22 : ((argc == 24) ? 21 : 17)))));
		if (sscanf(argv[option_index], "%d", &suction_enable) != 1 || sscanf(argv[option_index + 1], "%d", &suction_duty) != 1 ||
			sscanf(argv[option_index + 2], "%d", &turn_count) != 1 || (suction_enable != 0 && suction_enable != 1) ||
			suction_duty < 0 || suction_duty > 990 || turn_count < 1 || turn_count > 100) {
			printf("DEBUG_TURN_ERROR option\r\n");
			return -1;
		}
		next.suction_enable = suction_enable ? True : False;
		next.suction_duty = suction_duty;
		next.turn_count = turn_count;
		if (next.table.velo <= 0.0f || next.pre_accel <= 0.0f || next.table.r_min == 0.0f ||
			next.table.Lstart < 0.0f || next.table.Lend < 0.0f || next.table.degree == 0.0f) {
			printf("DEBUG_TURN_ERROR range\r\n");
			return -1;
		}
		const t_bool right_turn = ((int)pattern % 2 != 0) ? True : False;
		if ((right_turn == True && (next.table.r_min > 0.0f || next.table.degree > 0.0f)) ||
			(right_turn != True && (next.table.r_min < 0.0f || next.table.degree < 0.0f))) {
			printf("DEBUG_TURN_ERROR direction_sign\r\n");
			return -1;
		}
		next.table.turn_dir = (next.table.degree < 0.0f) ? Turn_R : Turn_L;
		*debug = next;
		debug->param.param = &debug->table;
		debug->param.sp_gain = &debug->sp_gain;
		debug->param.om_gain = &debug->om_gain;
		debug->param.ff_gain = &debug->ff_gain;
		shell_debug_turn_show(argv[2], debug);
		printf("DEBUG_TURN_SET_DONE\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[3], "exe") == 0) {
		Motion *motion = &CtrlTask_type8::getInstance();
		IrSensTask *irsens = CtrlTask_type8::getInstance().return_irObj();
		printf("DEBUG_WAIT_SENSOR\r\n");
		shell_debug_wait_sensor(irsens);
		for (int i = 0; i < 21; i++) {
			Indicate_LED((i % 2 == 0) ? 0xff : 0x00);
			HAL_Delay(50);
		}
		// The indicator is reserved for the turn segment below.  Keep it off
		// during the approach run, after the sensor-ready flashing has ended.
		Indicate_LED(0x00);
		motion->Motion_start();
		shell_debug_suction_start(motion, debug->suction_enable, debug->suction_duty);
		LogData::getInstance().data_count = 0;
		LogData::getInstance().log_enable = True;
		if (shell_debug_turn_pre_run(motion, pattern, debug->table.velo, debug->pre_accel) != True) {
			printf("DEBUG_TURN_ERROR unsupported_pre_run:%d\r\n", pattern);
			LogData::getInstance().log_enable = False;
			motion->Motion_end();
			return -1;
		}
		for (int turn_index = 0; turn_index < debug->turn_count; turn_index++) {
			switch (pattern) {
			case Long_turnR90:
			case Long_turnL90:
			case Long_turnR180:
			case Long_turnL180:
				motion->Init_Motion_long_turn(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
				break;

			case Turn_in_R45:
			case Turn_in_L45:
			case Turn_in_R135:
			case Turn_in_L135:
				motion->Init_Motion_turn_in(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
				break;

			case Turn_out_R45:
			case Turn_out_L45:
			case Turn_out_R135:
			case Turn_out_L135:
				motion->Init_Motion_turn_out(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
				break;

			case Turn_RV90:
			case Turn_LV90:
				motion->Init_Motion_turn_v90(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
				break;

			default:
				printf("DEBUG_TURN_ERROR unsupported_pattern:%d\r\n", pattern);
				LogData::getInstance().log_enable = False;
				motion->Motion_end();
				return -1;
			}
			// Init_Motion_* contains Lstart, the angular turn, and Lend.  Light
			// the LEDs only while that complete turn segment is being executed.
			Indicate_LED(0x3f);
			motion->execute_Motion();
			Indicate_LED(0x00);
		}
		if (shell_debug_turn_post_run(motion, pattern, debug->table.velo) != True) {
			printf("DEBUG_TURN_ERROR unsupported_post_run:%d\r\n", pattern);
			LogData::getInstance().log_enable = False;
			motion->Motion_end();
			return -1;
		}
		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		Indicate_LED(0x03<<4);
		HAL_Delay(500);
		return 0;
	}
	printf("Unknown debug turn command found\r\n");
	return -1;
}

typedef struct {
	float degree;
	float rad_acc;
	float rad_velo;
	t_pid_gain sp_gain;
	t_pid_gain om_gain;
	t_ff_gain ff_gain;
	t_bool suction_enable;
	int suction_duty;
} shell_debug_pivot_param_t;

static shell_debug_pivot_param_t shell_debug_pivot_right = {
	-90.0f, -40.0f * PI, -4.0f * PI, sp_gain_pivot_turn, om_gain_pivot_turn,
	ff_gain_pivot_turn_R, False, 650
};
static shell_debug_pivot_param_t shell_debug_pivot_left = {
	90.0f, 40.0f * PI, 4.0f * PI, sp_gain_pivot_turn, om_gain_pivot_turn,
	ff_gain_pivot_turn_L, False, 650
};
static shell_debug_turn_param_t shell_debug_search_right;
static shell_debug_turn_param_t shell_debug_search_left;

static t_bool shell_debug_is_right(const char *direction, t_bool *right)
{
	if (ntlibc_strcmp(direction, "right") == 0) { *right = True; return True; }
	if (ntlibc_strcmp(direction, "left") == 0) { *right = False; return True; }
	return False;
}

static const t_param *shell_debug_search_source(t_bool right, int preset_speed)
{
	switch (preset_speed) {
	case 280: return right ? &param_R90_search_280 : &param_L90_search_280;
	case 300: return right ? &param_R90_search_300 : &param_L90_search_300;
	case 320: return right ? &param_R90_search_320 : &param_L90_search_320;
#if defined(MOUSE_A)
	case 350: return right ? &param_R90_search_350 : &param_L90_search_350;
	case 370: return right ? &param_R90_search_370 : &param_L90_search_370;
	case 400: return right ? &param_R90_search_400 : &param_L90_search_400;
#endif
	default: return NULL;
	}
}

static shell_debug_turn_param_t *shell_debug_search_get(t_bool right, int preset_speed, t_bool reset)
{
	shell_debug_turn_param_t *debug = right ? &shell_debug_search_right : &shell_debug_search_left;
	const t_param *source = shell_debug_search_source(right, preset_speed);
	if (source == NULL) return NULL;
	if (debug->initialized != True || reset == True) {
		debug->table = *source->param;
		if (right == True) {
			debug->table.r_min = -ABS(debug->table.r_min);
			debug->table.degree = -ABS(debug->table.degree);
			debug->table.turn_dir = Turn_R;
		} else {
			debug->table.r_min = ABS(debug->table.r_min);
			debug->table.degree = ABS(debug->table.degree);
			debug->table.turn_dir = Turn_L;
		}
		debug->sp_gain = *source->sp_gain;
		debug->om_gain = *source->om_gain;
		debug->ff_gain = *(source->ff_gain != nullptr ? source->ff_gain : &ff_gain_default);
		debug->param.param = &debug->table;
		debug->param.sp_gain = &debug->sp_gain;
		debug->param.om_gain = &debug->om_gain;
		debug->param.ff_gain = &debug->ff_gain;
		debug->suction_enable = False;
		debug->suction_duty = 650;
		debug->preset_speed = preset_speed;
		debug->turn_count = 1;
		debug->pre_accel = shell_debug_straight_param.acc;
		debug->initialized = True;
	}
	return debug;
}

static int shell_debug_search_command(int argc, char **argv)
{
	if (argc < 4) { printf("debug search_turn right|left show|reset|preset speed|set|exe\r\n"); return 0; }
	t_bool right;
	if (shell_debug_is_right(argv[2], &right) != True) { printf("DEBUG_SEARCH_ERROR direction\r\n"); return -1; }
	shell_debug_turn_param_t *current = right ? &shell_debug_search_right : &shell_debug_search_left;
	int preset_speed = (current->initialized == True) ? current->preset_speed : 320;
	t_bool reset = (ntlibc_strcmp(argv[3], "reset") == 0) ? True : False;
	t_bool preset = (ntlibc_strcmp(argv[3], "preset") == 0) ? True : False;
	if (preset == True) {
		if (argc != 5) { printf("debug search_turn %s preset 280|300|320|350|370|400\r\n", argv[2]); return -1; }
		preset_speed = ntlibc_atoi(argv[4]);
		reset = True;
	}
	shell_debug_turn_param_t *debug = shell_debug_search_get(right, preset_speed, reset);
	if (debug == NULL) { printf("DEBUG_SEARCH_ERROR preset:%d\r\n", preset_speed); return -1; }
	if (ntlibc_strcmp(argv[3], "show") == 0 || reset == True) {
		if (reset == True) {
			debug->turn_count = 1;
			if (preset == True) printf("DEBUG_SEARCH_PRESET_DONE speed:%d\r\n", preset_speed);
			else printf("DEBUG_SEARCH_RESET_DONE\r\n");
		}
		shell_debug_turn_show(argv[2], debug);
		printf("DEBUG_SEARCH_COUNT %d\r\n", debug->turn_count);
		return 0;
	}
	if (ntlibc_strcmp(argv[3], "set") == 0) {
		if (argc != 20 && argc != 24 && argc != 25 && argc != 26 && argc != 27 && argc != 28 && argc != 30) { printf("debug search_turn %s set velo r_min Lstart Lend degree degree_correction pre_accel sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_sp_bias ff_om_velo ff_om_accel ff_om_decel ff_om_bias ff_om_jerk ff_sp_turn_accel_mag ff_sp_turn_velo_sq] suction_enable suction_duty turn_count\r\n", argv[2]); return -1; }
		shell_debug_turn_param_t next = *debug;
		float *legacy_values[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart, &next.table.Lend,
			&next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki, &next.sp_gain.Kd,
			&next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.om_velo, &next.ff_gain.om_accel,
			&next.ff_gain.sp_bias, &next.ff_gain.om_bias};
		float *full_values[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart, &next.table.Lend,
			&next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki, &next.sp_gain.Kd,
			&next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.sp_bias, &next.ff_gain.om_velo,
			&next.ff_gain.om_accel, &next.ff_gain.om_decel, &next.ff_gain.om_bias, &next.ff_gain.om_jerk};
		float *full_values_with_turn_sp[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart, &next.table.Lend,
			&next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki, &next.sp_gain.Kd,
			&next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.sp_bias, &next.ff_gain.om_velo,
			&next.ff_gain.om_accel, &next.ff_gain.om_decel, &next.ff_gain.om_bias, &next.ff_gain.om_jerk,
			&next.ff_gain.sp_turn_accel_mag, &next.ff_gain.sp_turn_velo_sq};
		const t_bool full_format = (argc == 27 || argc == 28 || argc == 30) ? True : False;
		float **values = (argc == 30) ? full_values_with_turn_sp : ((full_format == True) ? full_values : legacy_values);
		const int value_count = (argc == 30) ? 23 : ((argc == 28) ? 21 : ((argc == 27) ? 20 : ((argc == 26) ? 19 : ((argc == 25) ? 18 : ((argc == 24) ? 17 : 13)))));
		for (int i = 0; i < value_count; i++) if (shell_parse_float(argv[i + 4], values[i]) != True) { printf("DEBUG_SEARCH_ERROR number\r\n"); return -1; }
		if (full_format != True) next.ff_gain.om_decel = next.ff_gain.om_accel;
		int enable, duty, count;
		const int option_index = (argc == 30) ? 27 : ((argc == 28) ? 25 : ((argc == 27) ? 24 : ((argc == 26) ? 23 : ((argc == 25) ? 22 : ((argc == 24) ? 21 : 17)))));
		if (sscanf(argv[option_index], "%d", &enable) != 1 || sscanf(argv[option_index + 1], "%d", &duty) != 1 || sscanf(argv[option_index + 2], "%d", &count) != 1 ||
			(enable != 0 && enable != 1) || duty < 0 || duty > 990 || next.table.velo <= 0.0f ||
			next.pre_accel <= 0.0f || next.table.Lstart < 0.0f || next.table.Lend < 0.0f || count < 1 || count > 100) { printf("DEBUG_SEARCH_ERROR range\r\n"); return -1; }
		if ((right == True && (next.table.r_min >= 0.0f || next.table.degree >= 0.0f)) ||
			(right != True && (next.table.r_min <= 0.0f || next.table.degree <= 0.0f))) { printf("DEBUG_SEARCH_ERROR direction_sign\r\n"); return -1; }
		next.table.turn_dir = right ? Turn_R : Turn_L;
		next.suction_enable = enable ? True : False;
		next.suction_duty = duty;
		next.preset_speed = debug->preset_speed;
		*debug = next;
		debug->param.param = &debug->table;
		debug->param.sp_gain = &debug->sp_gain;
		debug->param.om_gain = &debug->om_gain;
		debug->param.ff_gain = &debug->ff_gain;
		debug->turn_count = count;
		shell_debug_turn_show(argv[2], debug);
		printf("DEBUG_SEARCH_COUNT %d\r\n", debug->turn_count);
		printf("DEBUG_SEARCH_SET_DONE\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[3], "exe") == 0) {
		Motion *motion = &CtrlTask_type8::getInstance();
		IrSensTask *irsens = CtrlTask_type8::getInstance().return_irObj();
		printf("DEBUG_WAIT_SENSOR\r\n");
		shell_debug_wait_sensor(irsens);
		motion->Motion_start();
		shell_debug_suction_start(motion, debug->suction_enable, debug->suction_duty);
		LogData::getInstance().data_count = 0;
		LogData::getInstance().log_enable = True;
		motion->exe_Motion_straight(45.0f,
			shell_debug_accel_in_length(debug->table.velo, 45.0f, debug->pre_accel), debug->table.velo,
			debug->table.velo, &shell_debug_straight_param.sp_gain, &shell_debug_straight_param.om_gain,
			&shell_debug_straight_param.ff_gain);
		for (int i = 0; i < debug->turn_count; i++) {
			motion->exe_Motion_search_turn(&debug->param, &debug->sp_gain, &debug->om_gain);
		}
		motion->exe_Motion_straight(45.0f, shell_debug_straight_param.acc, debug->table.velo,
			0.0f, &shell_debug_straight_param.sp_gain, &shell_debug_straight_param.om_gain,
			&shell_debug_straight_param.ff_gain);
		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		Indicate_LED(0x03<<4);
		HAL_Delay(500);
		return 0;
	}
	printf("Unknown debug search_turn command found\r\n"); return -1;
}

static int shell_debug_pivot_command(int argc, char **argv)
{
	if (argc < 4) { printf("debug pivot_turn right|left show|reset|set|exe\r\n"); return 0; }
	t_bool right;
	if (shell_debug_is_right(argv[2], &right) != True) { printf("DEBUG_PIVOT_ERROR direction\r\n"); return -1; }
	shell_debug_pivot_param_t *param = right ? &shell_debug_pivot_right : &shell_debug_pivot_left;
	if (ntlibc_strcmp(argv[3], "reset") == 0) {
		*param = right ? (shell_debug_pivot_param_t){-90.0f, -40.0f * PI, -4.0f * PI, sp_gain_pivot_turn, om_gain_pivot_turn,
			ff_gain_pivot_turn_R, False, 650}
			: (shell_debug_pivot_param_t){90.0f, 40.0f * PI, 4.0f * PI, sp_gain_pivot_turn, om_gain_pivot_turn,
			ff_gain_pivot_turn_L, False, 650};
		printf("DEBUG_PIVOT_RESET_DONE\r\n");
	}
	if (ntlibc_strcmp(argv[3], "show") == 0 || ntlibc_strcmp(argv[3], "reset") == 0) {
		printf("DEBUG_PIVOT_PARAM %s degree:%.4f rad_acc:%.4f rad_velo:%.4f sp:%.4f,%.4f,%.4f om:%.4f,%.4f,%.4f ff:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f suction:%d duty:%d\r\n",
			argv[2], param->degree, param->rad_acc, param->rad_velo, param->sp_gain.Kp, param->sp_gain.Ki,
			param->sp_gain.Kd, param->om_gain.Kp, param->om_gain.Ki, param->om_gain.Kd,
			param->ff_gain.sp_velo, param->ff_gain.sp_accel, param->ff_gain.sp_bias, param->ff_gain.om_velo,
			param->ff_gain.om_accel, param->ff_gain.om_decel, param->ff_gain.om_bias,
			param->suction_enable, param->suction_duty);
		return 0;
	}
	if (ntlibc_strcmp(argv[3], "set") == 0) {
		if (argc != 15 && argc != 19 && argc != 20 && argc != 21 && argc != 22) { printf("debug pivot_turn %s set degree rad_acc rad_velo sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_sp_bias ff_om_velo ff_om_accel ff_om_decel ff_om_bias] suction_enable suction_duty\r\n", argv[2]); return -1; }
		float *legacy_values[] = {&param->degree, &param->rad_acc, &param->rad_velo, &param->sp_gain.Kp, &param->sp_gain.Ki,
			&param->sp_gain.Kd, &param->om_gain.Kp, &param->om_gain.Ki, &param->om_gain.Kd,
			&param->ff_gain.sp_velo, &param->ff_gain.sp_accel, &param->ff_gain.om_velo, &param->ff_gain.om_accel,
			&param->ff_gain.sp_bias, &param->ff_gain.om_bias};
		float *full_values[] = {&param->degree, &param->rad_acc, &param->rad_velo, &param->sp_gain.Kp, &param->sp_gain.Ki,
			&param->sp_gain.Kd, &param->om_gain.Kp, &param->om_gain.Ki, &param->om_gain.Kd,
			&param->ff_gain.sp_velo, &param->ff_gain.sp_accel, &param->ff_gain.sp_bias, &param->ff_gain.om_velo,
			&param->ff_gain.om_accel, &param->ff_gain.om_decel, &param->ff_gain.om_bias};
		float **values = (argc == 22) ? full_values : legacy_values;
		const int value_count = (argc == 22) ? 16 : ((argc == 21) ? 15 : ((argc == 20) ? 14 : ((argc == 19) ? 13 : 9)));
		for (int i = 0; i < value_count; i++) if (shell_parse_float(argv[i + 4], values[i]) != True) { printf("DEBUG_PIVOT_ERROR number\r\n"); return -1; }
		if (argc != 22) param->ff_gain.om_decel = param->ff_gain.om_accel;
		int enable, duty;
		const int option_index = (argc == 22) ? 20 : ((argc == 21) ? 19 : ((argc == 20) ? 18 : ((argc == 19) ? 17 : 13)));
		if (sscanf(argv[option_index], "%d", &enable) != 1 || sscanf(argv[option_index + 1], "%d", &duty) != 1 || (enable != 0 && enable != 1) || duty < 0 || duty > 990 ||
			(right == True && (param->degree >= 0.0f || param->rad_acc >= 0.0f || param->rad_velo >= 0.0f)) ||
			(right != True && (param->degree <= 0.0f || param->rad_acc <= 0.0f || param->rad_velo <= 0.0f))) { printf("DEBUG_PIVOT_ERROR range\r\n"); return -1; }
		param->suction_enable = enable ? True : False; param->suction_duty = duty;
		printf("DEBUG_PIVOT_SET_DONE\r\n"); return 0;
	}
	if (ntlibc_strcmp(argv[3], "exe") == 0) {
		Motion *motion = &CtrlTask_type8::getInstance(); IrSensTask *irsens = CtrlTask_type8::getInstance().return_irObj();
		printf("DEBUG_WAIT_SENSOR\r\n"); shell_debug_wait_sensor(irsens);
		motion->Motion_start(); shell_debug_suction_start(motion, param->suction_enable, param->suction_duty);
		LogData::getInstance().data_count = 0; LogData::getInstance().log_enable = True;
		motion->exe_Motion_pivot_turn(DEG2RAD(param->degree), param->rad_acc, param->rad_velo,
			&param->sp_gain, &param->om_gain, &param->ff_gain);
		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		Indicate_LED(0x03<<4);
		HAL_Delay(500);
		return 0;
	}
	printf("Unknown debug pivot_turn command found\r\n"); return -1;
}

static const t_ff_gain *shell_debug_motion_ff(const shell_debug_param_t *param, t_bool diagonal)
{
	if (param->suction_enable == True && diagonal != True) return &ff_gain_straight_suction;
	return &param->ff_gain;
}

static void shell_debug_show(const char *motion_name, const shell_debug_param_t *param, t_bool diagonal)
{
	const t_ff_gain *ff_gain = shell_debug_motion_ff(param, diagonal);
	printf("DEBUG_PARAM %s distance:%.4f acc:%.4f max_velo:%.4f end_velo:%.4f "
		   "sp:%.4f,%.4f,%.4f om:%.4f,%.4f,%.4f ff:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f suction:%d duty:%d\r\n",
		   motion_name, param->distance, param->acc, param->max_velo, param->end_velo,
		   param->sp_gain.Kp, param->sp_gain.Ki, param->sp_gain.Kd,
		   param->om_gain.Kp, param->om_gain.Ki, param->om_gain.Kd,
		   ff_gain->sp_velo, ff_gain->sp_accel, ff_gain->sp_bias, ff_gain->om_velo,
		   ff_gain->om_accel, ff_gain->om_decel, ff_gain->om_bias,
		   param->suction_enable, param->suction_duty);
}

static int shell_debug_command(int argc, char **argv, const char *motion_name,
							   shell_debug_param_t *param, t_bool diagonal)
{
	if (ntlibc_strcmp(argv[2], "show") == 0) {
		shell_debug_show(motion_name, param, diagonal);
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "set") == 0) {
		if (argc != 15 && argc != 19 && argc != 20 && argc != 21 && argc != 22) {
			printf("debug %s set distance acc max_velo end_velo sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_sp_bias ff_om_velo ff_om_accel ff_om_decel ff_om_bias] suction_enable suction_duty\r\n", motion_name);
			return -1;
		}
		shell_debug_param_t next = *param;
		float *legacy_values[] = {
			&next.distance, &next.acc, &next.max_velo, &next.end_velo,
			&next.sp_gain.Kp, &next.sp_gain.Ki, &next.sp_gain.Kd,
			&next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.om_velo, &next.ff_gain.om_accel,
			&next.ff_gain.sp_bias, &next.ff_gain.om_bias
		};
		float *full_values[] = {
			&next.distance, &next.acc, &next.max_velo, &next.end_velo,
			&next.sp_gain.Kp, &next.sp_gain.Ki, &next.sp_gain.Kd,
			&next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.sp_bias, &next.ff_gain.om_velo,
			&next.ff_gain.om_accel, &next.ff_gain.om_decel, &next.ff_gain.om_bias
		};
		float **values = (argc == 22) ? full_values : legacy_values;
		const int value_count = (argc == 22) ? 17 : ((argc == 21) ? 16 : ((argc == 20) ? 15 : ((argc == 19) ? 14 : 10)));
		for (int i = 0; i < value_count; i++) {
			if (shell_parse_float(argv[i + 3], values[i]) != True) {
				printf("DEBUG_PARAM_ERROR invalid_number:%s\r\n", argv[i + 3]);
				return -1;
			}
		}
		if (argc != 22) next.ff_gain.om_decel = next.ff_gain.om_accel;
		int suction_enable;
		int suction_duty;
		const int option_index = (argc == 22) ? 20 : ((argc == 21) ? 19 : ((argc == 20) ? 18 : ((argc == 19) ? 17 : 13)));
		if (sscanf(argv[option_index], "%d", &suction_enable) != 1 || sscanf(argv[option_index + 1], "%d", &suction_duty) != 1 ||
			(suction_enable != 0 && suction_enable != 1) || suction_duty < 0 || suction_duty > 990) {
			printf("DEBUG_PARAM_ERROR suction\r\n");
			return -1;
		}
		next.suction_enable = suction_enable ? True : False;
		next.suction_duty = suction_duty;
		if (next.distance <= 0.0f || next.acc <= 0.0f || next.max_velo <= 0.0f ||
			next.end_velo < 0.0f || next.end_velo > next.max_velo) {
			printf("DEBUG_PARAM_ERROR range\r\n");
			return -1;
		}
		*param = next;
		printf("DEBUG_PARAM_SET_DONE\r\n");
		shell_debug_show(motion_name, param, diagonal);
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "exe") == 0) {
		Motion *motion = &(CtrlTask_type8::getInstance());
		IrSensTask *irsens = CtrlTask_type8::getInstance().return_irObj();
		printf("DEBUG_WAIT_SENSOR\r\n");
		shell_debug_wait_sensor(irsens);
		for (int i = 0; i < 21; i++) {
			Indicate_LED((i % 2 == 0) ? 0xff : 0x00);
			HAL_Delay(50);
		}
		motion->Motion_start();
		shell_debug_suction_start(motion, param->suction_enable, param->suction_duty);
		const t_ff_gain *ff_gain = shell_debug_motion_ff(param, diagonal);
		LogData::getInstance().data_count = 0;
		LogData::getInstance().log_enable = True;
		if (diagonal == True) {
			motion->Init_Motion_diagonal(param->distance, param->acc, param->max_velo,
									 param->end_velo, &param->sp_gain, &param->om_gain, ff_gain);
		} else {
			motion->Init_Motion_straight(param->distance, param->acc, param->max_velo,
									 param->end_velo, &param->sp_gain, &param->om_gain, ff_gain);
		}
		motion->execute_Motion();
		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		Indicate_LED(0x03<<4);
		HAL_Delay(500);
		return 0;
	}
	printf("Unknown debug sub command found\r\n");
	return -1;
}

static int usrcmd_debug(int argc, char **argv)
{
	if (argc < 3) {
		printf("debug straight show|set|exe\r\n");
		printf("debug diagonal show|set|exe\r\n");
		printf("debug turn type show|reset|set|exe\r\n");
		printf("debug pivot_turn right|left show|reset|set|exe\r\n");
		printf("debug search_turn right|left show|reset|preset speed|set|exe\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "straight") == 0)
	{
		return shell_debug_command(argc, argv, "straight", &shell_debug_straight_param, False);
	}
	if (ntlibc_strcmp(argv[1], "diagonal") == 0)
	{
		return shell_debug_command(argc, argv, "diagonal", &shell_debug_diagonal_param, True);
	}
	if (ntlibc_strcmp(argv[1], "turn") == 0)
	{
		return shell_debug_turn_command(argc, argv);
	}
	if (ntlibc_strcmp(argv[1], "pivot_turn") == 0) return shell_debug_pivot_command(argc, argv);
	if (ntlibc_strcmp(argv[1], "search_turn") == 0) return shell_debug_search_command(argc, argv);
    printf("Unknown sub command found\r\n");
    return -1;
}

static t_bool shell_turnpattern_init_turn(Motion *motion, t_run_pattern pattern, shell_debug_turn_param_t *debug)
{
	switch (pattern) {
	case Long_turnR90: case Long_turnL90: case Long_turnR180: case Long_turnL180:
		motion->Init_Motion_long_turn(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
		return True;
	case Turn_in_R45: case Turn_in_L45: case Turn_in_R135: case Turn_in_L135:
		motion->Init_Motion_turn_in(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
		return True;
	case Turn_out_R45: case Turn_out_L45: case Turn_out_R135: case Turn_out_L135:
		motion->Init_Motion_turn_out(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
		return True;
	case Turn_RV90: case Turn_LV90:
		motion->Init_Motion_turn_v90(&debug->param, pattern, &debug->sp_gain, &debug->om_gain);
		return True;
	default:
		return False;
	}
}

static t_bool shell_turnpattern_post_run(Motion *motion, t_run_pattern pattern, float turn_velo, float configured_accel)
{
	float length;
	if (pattern == Long_turnR90 || pattern == Long_turnL90) {
		length = SECTION * 2.0f;
	} else if (pattern == Turn_in_R45 || pattern == Turn_in_L45 ||
			   pattern == Turn_in_R135 || pattern == Turn_in_L135) {
		length = DIAG_SECTION * 2.0f;
	} else if (pattern == Turn_RV90 || pattern == Turn_LV90) {
		length = DIAG_SECTION;
	} else {
		length = SECTION;
	}
	const float accel = shell_debug_accel_in_length(turn_velo, length, configured_accel);
	if (shell_turnpattern_output_state(pattern) == 1) {
		motion->exe_Motion_diagonal(length, accel, turn_velo, 0.0f,
			&shell_debug_diagonal_param.sp_gain, &shell_debug_diagonal_param.om_gain,
			&shell_debug_diagonal_param.ff_gain);
		return True;
	}
	if (shell_turnpattern_output_state(pattern) == 0) {
		motion->exe_Motion_straight(length, accel, turn_velo, 0.0f,
			&shell_debug_straight_param.sp_gain, &shell_debug_straight_param.om_gain,
			&shell_debug_straight_param.ff_gain);
		return True;
	}
	return False;
}

static void shell_turnpattern_show(void)
{
	printf("TURNPATTERN_CONFIG preset:%d velo_x1000:%ld pre_accel_x1000:%ld post_accel_x1000:%ld suction:%d duty:%d count:%d max:%d\r\n",
		shell_turnpattern.preset_speed, (long)(shell_turnpattern.turn_velo * 1000.0f),
		(long)(shell_turnpattern.pre_accel * 1000.0f), (long)(shell_turnpattern.post_accel * 1000.0f),
		shell_turnpattern.suction_enable, shell_turnpattern.suction_duty,
		shell_turnpattern.count, SHELL_TURNPATTERN_MAX);
	for (uint8_t i = 0; i < shell_turnpattern.count; i++) {
		printf("TURNPATTERN_ITEM %d %s\r\n", i, shell_debug_turn_name(shell_turnpattern.turns[i]));
	}
	printf("TURNPATTERN_SHOW_DONE valid:%d\r\n", shell_turnpattern_validate());
}

static int shell_turnpattern_execute(void)
{
	if (shell_turnpattern_validate() != True) {
		printf("TURNPATTERN_ERROR invalid_sequence\r\n");
		return -1;
	}
	const t_param *const *turn_mode = shell_debug_turn_mode(shell_turnpattern.preset_speed);
	for (uint8_t i = 0; i < shell_turnpattern.count; i++) {
		if (turn_mode == NULL || turn_mode[shell_turnpattern.turns[i]] == NULL) {
			printf("TURNPATTERN_ERROR no_default index:%d\r\n", i);
			return -1;
		}
	}
	Motion *motion = &CtrlTask_type8::getInstance();
	IrSensTask *irsens = CtrlTask_type8::getInstance().return_irObj();
	printf("TURNPATTERN_WAIT_SENSOR\r\n");
	shell_debug_wait_sensor(irsens);
	for (int i = 0; i < 21; i++) {
		Indicate_LED((i % 2 == 0) ? 0xff : 0x00);
		HAL_Delay(50);
	}

	motion->Motion_start();
	shell_debug_suction_start(motion, shell_turnpattern.suction_enable, shell_turnpattern.suction_duty);
	LogData::getInstance().data_count = 0;
	LogData::getInstance().log_enable = True;
	if (shell_debug_turn_pre_run(motion, shell_turnpattern.turns[0],
			shell_turnpattern.turn_velo, shell_turnpattern.pre_accel) != True) {
		printf("TURNPATTERN_ERROR pre_run\r\n");
		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		return -1;
	}

	for (uint8_t i = 0; i < shell_turnpattern.count; i++) {
		t_run_pattern pattern = shell_turnpattern.turns[i];
		shell_debug_turn_param_t *debug = shell_debug_turn_get(pattern, shell_turnpattern.preset_speed, True);
		if (debug == NULL) {
			printf("TURNPATTERN_ERROR no_default index:%d\r\n", i);
			LogData::getInstance().log_enable = False;
			motion->Motion_end();
			return -1;
		}
		debug->table.velo = shell_turnpattern.turn_velo;
		if (shell_turnpattern_init_turn(motion, pattern, debug) != True || motion->execute_Motion() != complete) {
			printf("TURNPATTERN_ERROR motion index:%d\r\n", i);
			LogData::getInstance().log_enable = False;
			motion->Motion_end();
			return -1;
		}
	}

	if (shell_turnpattern_post_run(motion, shell_turnpattern.turns[shell_turnpattern.count - 1],
			shell_turnpattern.turn_velo, shell_turnpattern.post_accel) != True) {
		printf("TURNPATTERN_ERROR post_run\r\n");
		LogData::getInstance().log_enable = False;
		motion->Motion_end();
		return -1;
	}
	LogData::getInstance().log_enable = False;
	motion->Motion_end();
	Indicate_LED(0x03 << 4);
	HAL_Delay(500);
	printf("TURNPATTERN_RUN_DONE\r\n");
	return 0;
}

static int usrcmd_turnpattern(int argc, char **argv)
{
	if (argc < 2) {
		printf("turnpattern clear|config|add|show|exe\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "clear") == 0) {
		shell_turnpattern.count = 0;
		for (int i = 0; i < SHELL_TURNPATTERN_MAX; i++) shell_turnpattern.turns[i] = No_run;
		printf("TURNPATTERN_CLEAR_DONE\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "config") == 0) {
		if (argc != 8) {
			printf("turnpattern config preset_speed turn_velo pre_accel post_accel suction_enable suction_duty\r\n");
			return -1;
		}
		int preset_speed, suction_enable, suction_duty;
		float turn_velo, pre_accel, post_accel;
		if (sscanf(argv[2], "%d", &preset_speed) != 1 || shell_parse_float(argv[3], &turn_velo) != True ||
			shell_parse_float(argv[4], &pre_accel) != True || shell_parse_float(argv[5], &post_accel) != True ||
			sscanf(argv[6], "%d", &suction_enable) != 1 || sscanf(argv[7], "%d", &suction_duty) != 1 ||
			shell_debug_turn_mode(preset_speed) == NULL || turn_velo <= 0.0f || pre_accel <= 0.0f ||
			post_accel <= 0.0f || (suction_enable != 0 && suction_enable != 1) ||
			suction_duty < 0 || suction_duty > 990) {
			printf("TURNPATTERN_ERROR config\r\n");
			return -1;
		}
		shell_turnpattern.preset_speed = preset_speed;
		shell_turnpattern.turn_velo = turn_velo;
		shell_turnpattern.pre_accel = pre_accel;
		shell_turnpattern.post_accel = post_accel;
		shell_turnpattern.suction_enable = suction_enable ? True : False;
		shell_turnpattern.suction_duty = suction_duty;
		printf("TURNPATTERN_CONFIG_DONE\r\n");
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "add") == 0) {
		if (argc != 3) { printf("turnpattern add type\r\n"); return -1; }
		if (shell_turnpattern.count >= SHELL_TURNPATTERN_MAX) {
			printf("TURNPATTERN_ERROR full\r\n");
			return -1;
		}
		t_run_pattern pattern = shell_debug_turn_pattern(argv[2]);
		if (pattern == No_run) {
			printf("TURNPATTERN_ERROR unknown_type:%s\r\n", argv[2]);
			return -1;
		}
		shell_turnpattern.turns[shell_turnpattern.count++] = pattern;
		printf("TURNPATTERN_ADD_DONE count:%d\r\n", shell_turnpattern.count);
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "show") == 0) {
		shell_turnpattern_show();
		return 0;
	}
	if (ntlibc_strcmp(argv[1], "exe") == 0) return shell_turnpattern_execute();
	printf("TURNPATTERN_ERROR unknown_command\r\n");
	return -1;
}

/* ---------------------------------------------------------------
	送受信用ローカル関数
--------------------------------------------------------------- */
static int func_read(char *buf, int cnt, void *extobj)
{
	  for (int16_t i = 0; i < cnt;i ++)
	  {
		  buf[i] = (char) Communicate_RxPopData();
	  }
	  return cnt;
}

static int func_write(const char *buf, int cnt, void *extobj)
{

	  for (int16_t i = 0; i < cnt;i ++)
	  {
		  Communicate_TxPushData((int8_t)(buf[i]));
	  }
	return cnt;
}

/* ---------------------------------------------------------------
	コールバック関数
--------------------------------------------------------------- */
static int user_callback(const char *text, void *extobj)
{

	usrcmd_execute(text);
	return 0;
}

static int usrcmd_execute(const char *text)
{
	return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
{
    if (argc == 0) {
        return 0;
    }
    const cmd_table_t *p = &cmdlist[0];
    for (uint16_t i = 0; i < (sizeof(cmdlist) / sizeof(cmdlist[0])); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    printf("Unknown command found.\r\n");
    return 0;
}

/* ---------------------------------------------------------------
	初期設定関数
--------------------------------------------------------------- */

void Myshell_Initialize( void )
{
	void *extobj = 0;

	ntshell_init(&nts, func_read, func_write, user_callback, extobj);
}

void Myshell_Execute( void )
{

	shell_end_flag = False;
	while(shell_end_flag != True)
	{
		if( (&nts)->initcode != 0x4367 ) {
			return;
		} else;

		unsigned char ch;
		func_read((char *)&ch, sizeof(ch), (&nts)->extobj);
		vtrecv_execute(&((&nts)->vtrecv), &ch, sizeof(ch));

	}
}
