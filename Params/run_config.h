#ifndef PARAMS_RUN_CONFIG_H_
#define PARAMS_RUN_CONFIG_H_

#include <stdint.h>
#include "Task/Inc/ctrl_task.h"

typedef enum {
	RUN_CONFIG_PLAIN = 0,
	RUN_CONFIG_SUCTION,
	RUN_CONFIG_VARIABLE_TURN,
} t_run_config_type;

typedef struct {
	const char *key;
	const char *display_name;
	t_run_config_type type;
	const t_straight_param *const *straight_mode;
	uint16_t straight_mode_size;
	const t_straight_param *const *diagonal_mode;
	uint16_t diagonal_mode_size;
	const t_param *const *const *turn_mode;
	uint16_t turn_mode_size;
	uint16_t suction;
} t_run_config;

extern const t_run_config run_config_table[];
extern const uint16_t run_config_count;

const t_run_config *find_run_config(const char *key);

#endif
