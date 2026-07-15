/*
 * myshell.cpp
 *
 *  Created on: 2024/03/09
 *      Author: sato1
 */

#include "core/ntshell.h"
#include "core/ntlibc.h"
#include "util/ntopt.h"
#include "../Inc/myshell.h"
#include "communicate.h"
#include "../../Module/Inc/log_data.h"
#include "../../Module/Inc/flash.h"
#include "../../Module/Inc/interrupt.h"
#include "../../Subsys/Inc/make_path.h"
#include "typedef.h"
#include "../../Task/Inc/sensing_task.h"
#include "../../Task/Inc/ctrl_task.h"
#include "../../Params/run_param.h"

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
	{ "path" ,"check path generation.", usrcmd_path }
};

static ntshell_t nts;

t_bool	shell_end_flag;
static t_bool shell_wall_data_ready = False;
static uint8_t shell_goal_x = MAZE_GOAL_X;
static uint8_t shell_goal_y = MAZE_GOAL_Y;
static uint8_t shell_goal_size = MAZE_GOAL_SIZE;

static wall_class *shell_wall_data(void)
{
	static wall_class wall_data(&IrSensTask_type8::getInstance());
	return &wall_data;
}

static void shell_read_save_data(wall_class *wall_data)
{
	wall_data->init_maze();
	read_save_data(wall_data);
	wall_data->histry2wall_append();
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
	wall_data->wall_histry.histry_init();
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
    printf("Unknown sub command found\r\n");
    return -1;
}


static int usrcmd_disp(int argc, char **argv)
{
    if (argc != 2) {
    	printf("disp maze\r\n");
    	printf("disp maze_bin\r\n");
    	printf("disp histry\r\n");
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
    if (ntlibc_strcmp(argv[1], "histry") == 0) {
    	wall_class *wall_data = shell_wall_data();
    	shell_read_save_data(wall_data);
    	wall_data->wall_histry.histry_indicate();
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
    	printf("read_save_data done. histry_cnt:%d\r\n", wall_data->wall_histry.get_histry_cnt());
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
	{4.0f, 0.05f, 0.0f}, {0.1f, 0.01f, 0.0f},
	{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_SP_BIAS_COEF}, False, 650
};
static shell_debug_param_t shell_debug_diagonal_param = {
	63.63f * 6.0f, 6.5f, 0.7f, 0.0f,
	{4.0f, 0.05f, 0.0f}, {0.1f, 0.01f, 0.0f},
	{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_SP_BIAS_COEF}, False, 650
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
		   "sp_u:%ld,%ld,%ld om_u:%ld,%ld,%ld ff_u:%ld,%ld,%ld,%ld,%ld suction:%d duty:%d preset:%d count:%d\r\n",
		   name, (long)(debug->table.velo * 1000.0f), (long)(debug->table.r_min * 1000.0f),
		   (long)(debug->table.Lstart * 1000.0f), (long)(debug->table.Lend * 1000.0f),
		   (long)(debug->table.degree * 1000.0f), (long)(debug->table.degree_correction * 1000.0f),
		   (long)(debug->pre_accel * 1000.0f),
		   (long)(debug->sp_gain.Kp * 1000000.0f), (long)(debug->sp_gain.Ki * 1000000.0f),
		   (long)(debug->sp_gain.Kd * 1000000.0f), (long)(debug->om_gain.Kp * 1000000.0f),
		   (long)(debug->om_gain.Ki * 1000000.0f), (long)(debug->om_gain.Kd * 1000000.0f),
		   (long)(debug->ff_gain.sp_velo * 1000000.0f), (long)(debug->ff_gain.sp_accel * 1000000.0f),
		   (long)(debug->ff_gain.om_velo * 1000000.0f), (long)(debug->ff_gain.om_accel * 1000000.0f),
		   (long)(debug->ff_gain.sp_bias * 1000000.0f),
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
		if (argc != 20 && argc != 24 && argc != 25) {
			printf("debug turn %s set velo r_min Lstart Lend degree degree_correction pre_accel sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_om_velo ff_om_accel [ff_sp_bias]] suction_enable suction_duty turn_count\r\n", argv[2]);
			return -1;
		}
		shell_debug_turn_param_t next = *debug;
		float *values[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart,
			&next.table.Lend, &next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki,
			&next.sp_gain.Kd, &next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.om_velo, &next.ff_gain.om_accel,
			&next.ff_gain.sp_bias};
		const int value_count = (argc == 25) ? 18 : ((argc == 24) ? 17 : 13);
		for (int i = 0; i < value_count; i++) {
			if (shell_parse_float(argv[i + 4], values[i]) != True) {
				printf("DEBUG_TURN_ERROR invalid_number:%s\r\n", argv[i + 4]);
				return -1;
			}
		}
		int suction_enable;
		int suction_duty;
		int turn_count;
		const int option_index = (argc == 25) ? 22 : ((argc == 24) ? 21 : 17);
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
			motion->execute_Motion();
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
	-90.0f, -40.0f * PI, -4.0f * PI, {2.0f, 0.016f, 0.0f}, {0.1f, 0.005f, 0.0f},
	{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_SP_BIAS_COEF}, False, 650
};
static shell_debug_pivot_param_t shell_debug_pivot_left = {
	90.0f, 40.0f * PI, 4.0f * PI, {2.0f, 0.016f, 0.0f}, {0.1f, 0.005f, 0.0f},
	{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_SP_BIAS_COEF}, False, 650
};
static shell_debug_turn_param_t shell_debug_search_right;
static shell_debug_turn_param_t shell_debug_search_left;

static t_bool shell_debug_is_right(const char *direction, t_bool *right)
{
	if (ntlibc_strcmp(direction, "right") == 0) { *right = True; return True; }
	if (ntlibc_strcmp(direction, "left") == 0) { *right = False; return True; }
	return False;
}

static shell_debug_turn_param_t *shell_debug_search_get(t_bool right, t_bool reset)
{
	shell_debug_turn_param_t *debug = right ? &shell_debug_search_right : &shell_debug_search_left;
	const t_param *source = right ? &param_R90_search_320 : &param_L90_search_320;
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
		debug->turn_count = 1;
		debug->pre_accel = shell_debug_straight_param.acc;
		debug->initialized = True;
	}
	return debug;
}

static int shell_debug_search_command(int argc, char **argv)
{
	if (argc < 4) { printf("debug search_turn right|left show|reset|set|exe\r\n"); return 0; }
	t_bool right;
	if (shell_debug_is_right(argv[2], &right) != True) { printf("DEBUG_SEARCH_ERROR direction\r\n"); return -1; }
	t_bool reset = (ntlibc_strcmp(argv[3], "reset") == 0) ? True : False;
	shell_debug_turn_param_t *debug = shell_debug_search_get(right, reset);
	if (ntlibc_strcmp(argv[3], "show") == 0 || reset == True) {
		if (reset == True) {
			debug->turn_count = 1;
			printf("DEBUG_SEARCH_RESET_DONE\r\n");
		}
		shell_debug_turn_show(argv[2], debug);
		printf("DEBUG_SEARCH_COUNT %d\r\n", debug->turn_count);
		return 0;
	}
	if (ntlibc_strcmp(argv[3], "set") == 0) {
		if (argc != 20 && argc != 24 && argc != 25) { printf("debug search_turn %s set velo r_min Lstart Lend degree degree_correction pre_accel sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_om_velo ff_om_accel [ff_sp_bias]] suction_enable suction_duty turn_count\r\n", argv[2]); return -1; }
		shell_debug_turn_param_t next = *debug;
		float *values[] = {&next.table.velo, &next.table.r_min, &next.table.Lstart, &next.table.Lend,
			&next.table.degree, &next.table.degree_correction, &next.pre_accel, &next.sp_gain.Kp, &next.sp_gain.Ki, &next.sp_gain.Kd,
			&next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.om_velo, &next.ff_gain.om_accel,
			&next.ff_gain.sp_bias};
		const int value_count = (argc == 25) ? 18 : ((argc == 24) ? 17 : 13);
		for (int i = 0; i < value_count; i++) if (shell_parse_float(argv[i + 4], values[i]) != True) { printf("DEBUG_SEARCH_ERROR number\r\n"); return -1; }
		int enable, duty, count;
		const int option_index = (argc == 25) ? 22 : ((argc == 24) ? 21 : 17);
		if (sscanf(argv[option_index], "%d", &enable) != 1 || sscanf(argv[option_index + 1], "%d", &duty) != 1 || sscanf(argv[option_index + 2], "%d", &count) != 1 ||
			(enable != 0 && enable != 1) || duty < 0 || duty > 990 || next.table.velo <= 0.0f ||
			next.pre_accel <= 0.0f || next.table.Lstart < 0.0f || next.table.Lend < 0.0f || count < 1 || count > 100) { printf("DEBUG_SEARCH_ERROR range\r\n"); return -1; }
		if ((right == True && (next.table.r_min >= 0.0f || next.table.degree >= 0.0f)) ||
			(right != True && (next.table.r_min <= 0.0f || next.table.degree <= 0.0f))) { printf("DEBUG_SEARCH_ERROR direction_sign\r\n"); return -1; }
		next.table.turn_dir = right ? Turn_R : Turn_L;
		next.suction_enable = enable ? True : False;
		next.suction_duty = duty;
		next.preset_speed = 0;
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
		*param = right ? (shell_debug_pivot_param_t){-90.0f, -40.0f * PI, -4.0f * PI, {2.0f, 0.016f, 0.0f}, {0.1f, 0.005f, 0.0f},
			{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_SP_BIAS_COEF}, False, 650}
			: (shell_debug_pivot_param_t){90.0f, 40.0f * PI, 4.0f * PI, {2.0f, 0.016f, 0.0f}, {0.1f, 0.005f, 0.0f},
			{FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_OM_VELO_COEF, FF_OM_ACCEL_COEF, FF_SP_BIAS_COEF}, False, 650};
		printf("DEBUG_PIVOT_RESET_DONE\r\n");
	}
	if (ntlibc_strcmp(argv[3], "show") == 0 || ntlibc_strcmp(argv[3], "reset") == 0) {
		printf("DEBUG_PIVOT_PARAM %s degree:%.4f rad_acc:%.4f rad_velo:%.4f sp:%.4f,%.4f,%.4f om:%.4f,%.4f,%.4f ff:%.6f,%.6f,%.6f,%.6f,%.6f suction:%d duty:%d\r\n",
			argv[2], param->degree, param->rad_acc, param->rad_velo, param->sp_gain.Kp, param->sp_gain.Ki,
			param->sp_gain.Kd, param->om_gain.Kp, param->om_gain.Ki, param->om_gain.Kd,
			param->ff_gain.sp_velo, param->ff_gain.sp_accel, param->ff_gain.om_velo, param->ff_gain.om_accel,
			param->ff_gain.sp_bias,
			param->suction_enable, param->suction_duty);
		return 0;
	}
	if (ntlibc_strcmp(argv[3], "set") == 0) {
		if (argc != 15 && argc != 19 && argc != 20) { printf("debug pivot_turn %s set degree rad_acc rad_velo sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_om_velo ff_om_accel [ff_sp_bias]] suction_enable suction_duty\r\n", argv[2]); return -1; }
		float *values[] = {&param->degree, &param->rad_acc, &param->rad_velo, &param->sp_gain.Kp, &param->sp_gain.Ki,
			&param->sp_gain.Kd, &param->om_gain.Kp, &param->om_gain.Ki, &param->om_gain.Kd,
			&param->ff_gain.sp_velo, &param->ff_gain.sp_accel, &param->ff_gain.om_velo, &param->ff_gain.om_accel,
			&param->ff_gain.sp_bias};
		const int value_count = (argc == 20) ? 14 : ((argc == 19) ? 13 : 9);
		for (int i = 0; i < value_count; i++) if (shell_parse_float(argv[i + 4], values[i]) != True) { printf("DEBUG_PIVOT_ERROR number\r\n"); return -1; }
		int enable, duty;
		const int option_index = (argc == 20) ? 18 : ((argc == 19) ? 17 : 13);
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

static void shell_debug_show(const char *motion_name, const shell_debug_param_t *param)
{
	printf("DEBUG_PARAM %s distance:%.4f acc:%.4f max_velo:%.4f end_velo:%.4f "
		   "sp:%.4f,%.4f,%.4f om:%.4f,%.4f,%.4f ff:%.6f,%.6f,%.6f,%.6f,%.6f suction:%d duty:%d\r\n",
		   motion_name, param->distance, param->acc, param->max_velo, param->end_velo,
		   param->sp_gain.Kp, param->sp_gain.Ki, param->sp_gain.Kd,
		   param->om_gain.Kp, param->om_gain.Ki, param->om_gain.Kd,
		   param->ff_gain.sp_velo, param->ff_gain.sp_accel, param->ff_gain.om_velo, param->ff_gain.om_accel,
		   param->ff_gain.sp_bias,
		   param->suction_enable, param->suction_duty);
}

static int shell_debug_command(int argc, char **argv, const char *motion_name,
							   shell_debug_param_t *param, t_bool diagonal)
{
	if (ntlibc_strcmp(argv[2], "show") == 0) {
		shell_debug_show(motion_name, param);
		return 0;
	}
	if (ntlibc_strcmp(argv[2], "set") == 0) {
		if (argc != 15 && argc != 19 && argc != 20) {
			printf("debug %s set distance acc max_velo end_velo sp_kp sp_ki sp_kd om_kp om_ki om_kd [ff_sp_velo ff_sp_accel ff_om_velo ff_om_accel [ff_sp_bias]] suction_enable suction_duty\r\n", motion_name);
			return -1;
		}
		shell_debug_param_t next = *param;
		float *values[] = {
			&next.distance, &next.acc, &next.max_velo, &next.end_velo,
			&next.sp_gain.Kp, &next.sp_gain.Ki, &next.sp_gain.Kd,
			&next.om_gain.Kp, &next.om_gain.Ki, &next.om_gain.Kd,
			&next.ff_gain.sp_velo, &next.ff_gain.sp_accel, &next.ff_gain.om_velo, &next.ff_gain.om_accel,
			&next.ff_gain.sp_bias
		};
		const int value_count = (argc == 20) ? 15 : ((argc == 19) ? 14 : 10);
		for (int i = 0; i < value_count; i++) {
			if (shell_parse_float(argv[i + 3], values[i]) != True) {
				printf("DEBUG_PARAM_ERROR invalid_number:%s\r\n", argv[i + 3]);
				return -1;
			}
		}
		int suction_enable;
		int suction_duty;
		const int option_index = (argc == 20) ? 18 : ((argc == 19) ? 17 : 13);
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
		shell_debug_show(motion_name, param);
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
		LogData::getInstance().data_count = 0;
		LogData::getInstance().log_enable = True;
		if (diagonal == True) {
			motion->Init_Motion_diagonal(param->distance, param->acc, param->max_velo,
									 param->end_velo, &param->sp_gain, &param->om_gain, &param->ff_gain);
		} else {
			motion->Init_Motion_straight(param->distance, param->acc, param->max_velo,
									 param->end_velo, &param->sp_gain, &param->om_gain, &param->ff_gain);
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
		printf("debug search_turn right|left show|reset|set|exe\r\n");
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
