#ifndef PARAMS_RUN_CONFIG_H_
#define PARAMS_RUN_CONFIG_H_

#include <stdint.h>
#include "Params/run_param.h"

#if defined(MOUSE_A)
#include "Params/Param_A/run_suction_A.h"
#elif defined(MOUSE_B)
#include "Params/Param_B/run_suction_B.h"
#endif

typedef enum {
	RUN_CONFIG_PLAIN = 0,
	RUN_CONFIG_SUCTION,
	RUN_CONFIG_VARIABLE_TURN,
} t_run_config_type;

typedef struct {
	const char *key;
	const char *display_name;
	t_run_config_type type;
	const t_control_profile *straight_control;
	const t_control_profile *diagonal_control;
	const t_param *const *const *turn_mode;
	uint16_t turn_mode_size;
	uint16_t suction;
	// 全モードで指定する。速度候補/加減速度とPID/FFの速度帯は独立に編集する。
	const t_speed_profile *straight_speed = nullptr;
	const t_speed_profile *diagonal_speed = nullptr;
} t_run_config;

#define RUN_CONFIG_ARRAY_COUNT(array) ((uint16_t)(sizeof(array) / sizeof((array)[0])))
#define RUN_CONFIG_SIMPLE_TURN_SET(name) \
	static const t_param *const *const run_config_##name##_turn_set[] = {name}
#define RUN_CONFIG_VALUE(key_, name_, type_, st_, di_, turn_, suction_, st_speed_, di_speed_) \
	{key_, name_, type_, st_, di_, turn_, RUN_CONFIG_ARRAY_COUNT(turn_), suction_, st_speed_, di_speed_}

RUN_CONFIG_SIMPLE_TURN_SET(mode_500);
RUN_CONFIG_SIMPLE_TURN_SET(mode_700);
RUN_CONFIG_SIMPLE_TURN_SET(mode_1000);
RUN_CONFIG_SIMPLE_TURN_SET(mode_1200);
RUN_CONFIG_SIMPLE_TURN_SET(mode_1400);
RUN_CONFIG_SIMPLE_TURN_SET(mode_1600);
RUN_CONFIG_SIMPLE_TURN_SET(mode_1800);
RUN_CONFIG_SIMPLE_TURN_SET(mode_2000);

static const t_run_config run_config_plain500 =
	RUN_CONFIG_VALUE("plain500", "Plain 500", RUN_CONFIG_PLAIN,
		&straight_control_plain500, &diagonal_control_plain500, run_config_mode_500_turn_set, 0,
		&straight_speed_plain500, &diagonal_speed_plain500);
static const t_run_config run_config_plain700 =
	RUN_CONFIG_VALUE("plain700", "Plain 700", RUN_CONFIG_PLAIN,
		&straight_control_700_v0, &diagonal_control_700_v0, run_config_mode_700_turn_set, 0,
		&straight_speed_700_v0, &diagonal_speed_700_v0);
static const t_run_config run_config_uniform1000 =
	RUN_CONFIG_VALUE("uniform1000", "Uniform 1000", RUN_CONFIG_PLAIN,
		&straight_control_1000_v0, &diagonal_control_1000_v0, run_config_mode_1000_turn_set, 0,
		&straight_speed_1000_v0, &diagonal_speed_1000_v0);
static const t_run_config run_config_suction1200_v0 =
	RUN_CONFIG_VALUE("suction1200_v0", "Suction 1200 V0", RUN_CONFIG_SUCTION,
		&straight_control_1200_v0, &diagonal_control_1200_v0, run_config_mode_1200_turn_set, run_suction_1200_v0,
		&straight_speed_1200_v0, &diagonal_speed_1200_v0);
static const t_run_config run_config_suction1200_v1 =
	RUN_CONFIG_VALUE("suction1200_v1", "Suction 1200 V1", RUN_CONFIG_SUCTION,
		&straight_control_1200_v1, &diagonal_control_1200_v1, run_config_mode_1200_turn_set, run_suction_1200_v1,
		&straight_speed_1200_v1, &diagonal_speed_1200_v1);
static const t_run_config run_config_suction1400_v0 =
	RUN_CONFIG_VALUE("suction1400_v0", "Suction 1400 V0", RUN_CONFIG_SUCTION,
		&straight_control_1400_v0, &diagonal_control_1400_v0, run_config_mode_1400_turn_set, run_suction_1400_v0,
		&straight_speed_1400_v0, &diagonal_speed_1400_v0);
static const t_run_config run_config_suction1400_v1_600 =
	RUN_CONFIG_VALUE("suction1400_v1_600", run_suction_1400_v1_600_name, RUN_CONFIG_SUCTION,
		&straight_control_1400_v1, &diagonal_control_1400_v1, run_config_mode_1400_turn_set, run_suction_1400_v1_600,
		&straight_speed_1400_v1, &diagonal_speed_1400_v1);
static const t_run_config run_config_suction1400_v1_650 =
	RUN_CONFIG_VALUE("suction1400_v1_650", run_suction_1400_v1_650_name, RUN_CONFIG_SUCTION,
		&straight_control_1400_v1, &diagonal_control_1400_v1, run_config_mode_1400_turn_set, run_suction_1400_v1_650,
		&straight_speed_1400_v1, &diagonal_speed_1400_v1);
static const t_run_config run_config_suction1400_v2 =
	RUN_CONFIG_VALUE("suction1400_v2", "Suction 1400 V2", RUN_CONFIG_SUCTION,
		&straight_control_1400_v2, &diagonal_control_1400_v1, run_config_mode_1400_turn_set, run_suction_1400_v2,
		&straight_speed_1400_v2, &diagonal_speed_1400_v1);
static const t_run_config run_config_suction1600_v1 =
	RUN_CONFIG_VALUE("suction1600_v1", "Suction 1600 V1", RUN_CONFIG_SUCTION,
		&straight_control_1600_v1, &diagonal_control_1600_v1, run_config_mode_1600_turn_set, run_suction_1600_v1,
		&straight_speed_1600_v1, &diagonal_speed_1600_v1);
static const t_run_config run_config_suction1600_v2 =
	RUN_CONFIG_VALUE("suction1600_v2", "Suction 1600 V2", RUN_CONFIG_SUCTION,
		&straight_control_1600_v2, &diagonal_control_1600_v1, run_config_mode_1600_turn_set, run_suction_1600_v2,
		&straight_speed_1600_v2, &diagonal_speed_1600_v1);
static const t_run_config run_config_suction1800 =
	RUN_CONFIG_VALUE("suction1800", "Suction 1800", RUN_CONFIG_SUCTION,
		&straight_control_1800_v1, &diagonal_control_1800_v1, run_config_mode_1800_turn_set, run_suction_1800,
		&straight_speed_1800_v1, &diagonal_speed_1800_v1);
static const t_run_config run_config_suction2000 =
	RUN_CONFIG_VALUE("suction2000", "Suction 2000", RUN_CONFIG_SUCTION,
		&straight_control_2000_v1, &diagonal_control_2000_v1, run_config_mode_2000_turn_set, run_suction_2000,
		&straight_speed_2000_v1, &diagonal_speed_2000_v1);
static const t_run_config run_config_acc1600_v1 =
	RUN_CONFIG_VALUE("acc1600_v1", "Variable turn 1600 V1", RUN_CONFIG_VARIABLE_TURN,
		&straight_control_1600_v3, &diagonal_control_1600_v2, acc_mode_1600_v1, run_suction_acc1600_v1,
		&straight_speed_1600_v3, &diagonal_speed_1600_v2);
static const t_run_config run_config_acc1600_v2 =
	RUN_CONFIG_VALUE("acc1600_v2", "Variable turn 1600 V2", RUN_CONFIG_VARIABLE_TURN,
		&straight_control_1600_v2, &diagonal_control_1600_v1, acc_mode_1600_v2, run_suction_acc1600_v2,
		&straight_speed_1600_v2, &diagonal_speed_1600_v1);
static const t_run_config run_config_acc1600_v3 =
	RUN_CONFIG_VALUE("acc1600_v3", "Variable turn 1600 V3", RUN_CONFIG_VARIABLE_TURN,
		&straight_control_1600_v2, &diagonal_control_1600_v1, acc_mode_1600_v3, run_suction_acc1600_v3,
		&straight_speed_1600_v2, &diagonal_speed_1600_v1);
static const t_run_config run_config_acc1800_v1 =
	RUN_CONFIG_VALUE("acc1800_v1", "Variable turn 1800 V1", RUN_CONFIG_VARIABLE_TURN,
		&straight_control_1800_v1, &diagonal_control_1800_v1, acc_mode_1800_v1, run_suction_acc1800_v1,
		&straight_speed_1800_v1, &diagonal_speed_1800_v1);

static const t_run_config *const run_config_table[] = {
	&run_config_plain500,
	&run_config_plain700,
	&run_config_uniform1000,
	&run_config_suction1200_v0,
	&run_config_suction1200_v1,
	&run_config_suction1400_v0,
	&run_config_suction1400_v1_600,
	&run_config_suction1400_v1_650,
	&run_config_suction1400_v2,
	&run_config_suction1600_v1,
	&run_config_suction1600_v2,
	&run_config_suction1800,
	&run_config_suction2000,
	&run_config_acc1600_v1,
	&run_config_acc1600_v2,
	&run_config_acc1600_v3,
	&run_config_acc1800_v1,
};

static const uint16_t run_config_count = RUN_CONFIG_ARRAY_COUNT(run_config_table);

static inline t_bool run_config_key_equal(const char *left,const char *right)
{
	while(*left == *right) {
		if(*left == '\0') return True;
		left++;
		right++;
	}
	return False;
}

static inline const t_run_config *find_run_config(const char *key)
{
	for(uint16_t i = 0; i < run_config_count; i++) {
		if(run_config_key_equal(key, run_config_table[i]->key) == True) return run_config_table[i];
	}
	return nullptr;
}

#undef RUN_CONFIG_VALUE
#undef RUN_CONFIG_SIMPLE_TURN_SET
#undef RUN_CONFIG_ARRAY_COUNT

#endif
