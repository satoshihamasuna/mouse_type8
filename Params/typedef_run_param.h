/*
 * typedef_run_param.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TYPEDEF_RUN_PARAM_H_
#define CPP_PARAMS_TYPEDEF_RUN_PARAM_H_

#include "../Component/Inc/controller.h"
#include "../Module/Inc/vehicle.h"
#include "mouse_config.h"

typedef struct{
	float sp_velo;
	float sp_accel;
	float om_velo;
	float om_accel;
	float sp_bias;
}t_ff_gain;

const static t_ff_gain ff_gain_default = {
	FF_SP_VELO_COEF,
	FF_SP_ACCEL_COEF,
	FF_OM_VELO_COEF,
	FF_OM_ACCEL_COEF,
	FF_SP_BIAS_COEF
};

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
	// Logical degree is kept for path/type decisions. This offset is used only
	// when generating the physical turn motion (zero keeps legacy behaviour).
	float degree_correction;
}t_turn_param_table;

typedef struct{
	//float base_velo;
	float max_velo;
	float acc;
}t_velo_param;

typedef struct{
	t_turn_param_table const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
	t_ff_gain const* ff_gain = &ff_gain_default;
}t_param;

typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
	t_ff_gain const* ff_gain = &ff_gain_default;
}t_straight_param;



#endif /* CPP_PARAMS_TYPEDEF_RUN_PARAM_H_ */
