#include "Params/run_config.h"
#include "Params/run_param.h"
#include "core/ntlibc.h"

#define ARRAY_COUNT(array) ((uint16_t)(sizeof(array) / sizeof((array)[0])))
#define SIMPLE_TURN_SET(name) static const t_param *const *const name##_set[] = {name}
#define RUN_CONFIG(key_, name_, type_, st_, di_, turn_, suction_) \
	{key_, name_, type_, st_, ARRAY_COUNT(st_), di_, ARRAY_COUNT(di_), \
	 turn_, ARRAY_COUNT(turn_), suction_}

SIMPLE_TURN_SET(mode_500);
SIMPLE_TURN_SET(mode_700);
SIMPLE_TURN_SET(mode_1000);
SIMPLE_TURN_SET(mode_1200);
SIMPLE_TURN_SET(mode_1400);
SIMPLE_TURN_SET(mode_1600);
SIMPLE_TURN_SET(mode_1800);
SIMPLE_TURN_SET(mode_2000);

const t_run_config run_config_table[] = {
	RUN_CONFIG("plain500", "Plain 500", RUN_CONFIG_PLAIN, st_mode_500_v0, di_mode_500_v0, mode_500_set, 0),
	RUN_CONFIG("plain700", "Plain 700", RUN_CONFIG_PLAIN, st_mode_700_v0, di_mode_700_v0, mode_700_set, 0),
	RUN_CONFIG("uniform1000", "Uniform 1000", RUN_CONFIG_PLAIN, st_mode_1000_v0, di_mode_1000_v0, mode_1000_set, 0),
	RUN_CONFIG("suction1200_v0", "Suction 1200 V0", RUN_CONFIG_SUCTION, st_mode_1200_v0, di_mode_1200_v0, mode_1200_set, 600),
	RUN_CONFIG("suction1200_v1", "Suction 1200 V1", RUN_CONFIG_SUCTION, st_mode_1200_v1, di_mode_1200_v1, mode_1200_set, 600),
	RUN_CONFIG("suction1400_v0", "Suction 1400 V0", RUN_CONFIG_SUCTION, st_mode_1400_v0, di_mode_1400_v0, mode_1400_set, 650),
	RUN_CONFIG("suction1400_v1_600", "Suction 1400 V1 / 600", RUN_CONFIG_SUCTION, st_mode_1400_v1, di_mode_1400_v1, mode_1400_set, 600),
	RUN_CONFIG("suction1400_v1_650", "Suction 1400 V1 / 650", RUN_CONFIG_SUCTION, st_mode_1400_v1, di_mode_1400_v1, mode_1400_set, 650),
	RUN_CONFIG("suction1400_v2", "Suction 1400 V2", RUN_CONFIG_SUCTION, st_mode_1400_v2, di_mode_1400_v1, mode_1400_set, 650),
	RUN_CONFIG("suction1600_v1", "Suction 1600 V1", RUN_CONFIG_SUCTION, st_mode_1600_v1, di_mode_1600_v1, mode_1600_set, 650),
	RUN_CONFIG("suction1600_v2", "Suction 1600 V2", RUN_CONFIG_SUCTION, st_mode_1600_v2, di_mode_1600_v1, mode_1600_set, 700),
	RUN_CONFIG("suction1800", "Suction 1800", RUN_CONFIG_SUCTION, st_mode_1800_v1, di_mode_1800_v1, mode_1800_set, 700),
	RUN_CONFIG("suction2000", "Suction 2000", RUN_CONFIG_SUCTION, st_mode_2000_v1, di_mode_2000_v1, mode_2000_set, 800),
	RUN_CONFIG("acc1600_v1", "Variable turn 1600 V1", RUN_CONFIG_VARIABLE_TURN, st_mode_1600_v3, di_mode_1600_v2, acc_mode_1600_v1, 700),
	RUN_CONFIG("acc1600_v2", "Variable turn 1600 V2", RUN_CONFIG_VARIABLE_TURN, st_mode_1600_v2, di_mode_1600_v1, acc_mode_1600_v2, 700),
	RUN_CONFIG("acc1600_v3", "Variable turn 1600 V3", RUN_CONFIG_VARIABLE_TURN, st_mode_1600_v2, di_mode_1600_v1, acc_mode_1600_v3, 700),
	RUN_CONFIG("acc1800_v1", "Variable turn 1800 V1", RUN_CONFIG_VARIABLE_TURN, st_mode_1800_v1, di_mode_1800_v1, acc_mode_1800_v1, 800),
};

const uint16_t run_config_count = ARRAY_COUNT(run_config_table);

const t_run_config *find_run_config(const char *key)
{
	for(uint16_t i = 0; i < run_config_count; i++) {
		if(ntlibc_strcmp(key, run_config_table[i].key) == 0) return &run_config_table[i];
	}
	return nullptr;
}
