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

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
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
}t_param;

typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_straight_param;



#endif /* CPP_PARAMS_TYPEDEF_RUN_PARAM_H_ */
