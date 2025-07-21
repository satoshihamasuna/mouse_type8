/*
 * init_motion2.cpp
 *
 *  Created on: 2024/10/14
 *      Author: sato1
 */

#include "../Inc/ctrl_task.h"
#include "../Inc/run_typedef.h"
#include "../../Params/turn_table.h"


void Motion::Init_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,
										 float end_velo, float acc,
										 const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	float start_velo = vehicle->ideal.velo.get();
	motion_plan.velo.set				(start_velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(end_velo);

	if(turn_param->param->velo > start_velo)			motion_plan.accel.set			(acc);
	else if(turn_param->param->velo < start_velo)		motion_plan.accel.set			(-acc);
	else												motion_plan.accel.set			(0.0f);

	if(turn_param->param->velo > end_velo)				motion_plan.deccel.set			(-acc);
	else if(turn_param->param->velo < end_velo)			motion_plan.deccel.set			(acc);
	else												motion_plan.deccel.set			(0.0f);

	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set	(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set	(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set	(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set	( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	/*
	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	//vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	//vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();
	*/

	if(ABS(turn_param->param->degree) == 135.0f)
	{
		float diff = vehicle->ego.x_point.get();
		if(ABS(diff) < 10.0f){

			if(turn_param->param->turn_dir == Turn_R)
			{
				motion_plan.fix_prev_run.set(diff);
				motion_plan.fix_post_run.set(diff*SQRT2);
			}
			else if(turn_param->param->turn_dir == Turn_L)
			{
				motion_plan.fix_prev_run.set((-1.0)*diff);
				motion_plan.fix_post_run.set((-1.0)*diff*SQRT2);
			}
		}
	}
	else if(ABS(turn_param->param->degree) == 45.0f)
	{
		float diff = vehicle->ego.x_point.get();
		if(ABS(diff) < 10.0f){
			if(turn_param->param->turn_dir == Turn_R)
			{
				motion_plan.fix_prev_run.set((-1.0)*diff);
				motion_plan.fix_post_run.set(diff*SQRT2);
			}
			else if(turn_param->param->turn_dir == Turn_L)
			{
				motion_plan.fix_prev_run.set(diff);
				motion_plan.fix_post_run.set((-1.0)*diff*SQRT2);
			}
		}
	}

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_turn_out		(	const t_param *turn_param,t_run_pattern run_pt,
											float end_velo, float acc,
											const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	float start_velo = vehicle->ideal.velo.get();
	motion_plan.velo.set				(start_velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(end_velo);

	if(turn_param->param->velo > start_velo)			motion_plan.accel.set			(acc);
	else if(turn_param->param->velo < start_velo)		motion_plan.accel.set			(-acc);
	else												motion_plan.accel.set			(0.0f);

	if(turn_param->param->velo > end_velo)				motion_plan.deccel.set			(-acc);
	else if(turn_param->param->velo < end_velo)			motion_plan.deccel.set			(acc);
	else												motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	/*
	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	//vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	//vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();
	*/

	if(ABS(turn_param->param->degree) == 135.0f)
	{
		float diff = vehicle->ego.x_point.get();
		if(ABS(diff) < 10.0f){
			if(turn_param->param->turn_dir == Turn_R)
			{
				motion_plan.fix_prev_run.set(diff);
				motion_plan.fix_post_run.set(diff*SQRT2);
			}
			else if(turn_param->param->turn_dir == Turn_L)
			{
				motion_plan.fix_prev_run.set((-1.0)*diff);
				motion_plan.fix_post_run.set((-1.0)*diff*SQRT2);
			}
		}
	}
	else if(ABS(turn_param->param->degree) == 45.0f)
	{
		float diff = vehicle->ego.x_point.get();
		if(ABS(diff) < 10.0f){
			if(turn_param->param->turn_dir == Turn_R)
			{
				motion_plan.fix_prev_run.set((-1.0)*diff);
				motion_plan.fix_post_run.set(diff*SQRT2);
			}
			else if(turn_param->param->turn_dir == Turn_L)
			{
				motion_plan.fix_prev_run.set(diff);
				motion_plan.fix_post_run.set((-1.0)*diff*SQRT2);
			}
		}
	}

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(DIAGONAL_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensDiagonal();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_long_turn	(	const t_param *turn_param,t_run_pattern run_pt,
		float end_velo, float acc,
		const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	float start_velo = vehicle->ideal.velo.get();
	motion_plan.velo.set				(start_velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(end_velo);

	if(turn_param->param->velo > start_velo)			motion_plan.accel.set			(acc);
	else if(turn_param->param->velo < start_velo)		motion_plan.accel.set			(-acc);
	else												motion_plan.accel.set			(0.0f);

	if(turn_param->param->velo > end_velo)				motion_plan.deccel.set			(-acc);
	else if(turn_param->param->velo < end_velo)			motion_plan.deccel.set			(acc);
	else												motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	/*
	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	//vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	//vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();
	*/

	if(ABS(turn_param->param->degree) == 90.0f)
	{
		float diff = vehicle->ego.x_point.get();
		if(ABS(diff) < 10.0f){
			if(turn_param->param->turn_dir == Turn_R)
			{
				motion_plan.fix_post_run.set((1.0)*diff);
			}
			else if(turn_param->param->turn_dir == Turn_L)
			{
				motion_plan.fix_post_run.set((-1.0)*diff);
			}
		}
	}

	if(ABS(turn_param->param->degree) == 180.0f)
	{
		float diff = vehicle->ego.x_point.get();
		float r_min_fix = 0.0f;

		if(ABS(diff) < 10.0f){
			if(turn_param->param->turn_dir == Turn_R)
			{
				r_min_fix = (-1.0)*PI/accel_Integral/10.0f*diff;
				if(ABS(r_min_fix) < 3.0)
				{
					float fixed_rad_max = turn_param->param->velo/((turn_param->param->r_min + r_min_fix)/1000.0f);
					motion_plan.rad_max_velo.set(fixed_rad_max);
					motion_plan.turn_r_min.set((turn_param->param->r_min + r_min_fix));
					motion_plan.turn_time_ms.set( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);
					if(r_min_fix > 0.0)
					{
						motion_plan.fix_prev_run.set((1.0f)*ABS(diff));
						motion_plan.fix_post_run.set((1.0f)*ABS(diff));
					}
					else if(r_min_fix < 0.0)
					{
						motion_plan.fix_prev_run.set((-1.0f)*ABS(diff));
						motion_plan.fix_post_run.set((-1.0f)*ABS(diff));
					}
				}
			}
			else if(turn_param->param->turn_dir == Turn_L)
			{
				r_min_fix = (-1.0)*PI/accel_Integral/10.0f*diff;
				if(ABS(r_min_fix) < 3.0)
				{
					float fixed_rad_max = turn_param->param->velo/((turn_param->param->r_min + r_min_fix)/1000.0f);
					motion_plan.rad_max_velo.set(fixed_rad_max);
					motion_plan.turn_r_min.set((turn_param->param->r_min + r_min_fix));
					motion_plan.turn_time_ms.set( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);
					if(r_min_fix > 0.0)
					{
						motion_plan.fix_prev_run.set((-1.0f)*ABS(diff));
						motion_plan.fix_post_run.set((-1.0f)*ABS(diff));
					}
					else if(r_min_fix < 0.0)
					{
						motion_plan.fix_prev_run.set((1.0f)*ABS(diff));
						motion_plan.fix_post_run.set((1.0f)*ABS(diff));
					}
				}
			}
		}




	}

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_turn_v90		(	const t_param *turn_param,t_run_pattern run_pt,
											float end_velo, float acc,
											const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	float start_velo = vehicle->ideal.velo.get();
	motion_plan.velo.set				(start_velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(end_velo);

	if(turn_param->param->velo > start_velo)			motion_plan.accel.set			(acc);
	else if(turn_param->param->velo < start_velo)		motion_plan.accel.set			(-acc);
	else												motion_plan.accel.set			(0.0f);

	if(turn_param->param->velo > end_velo)				motion_plan.deccel.set			(-acc);
	else if(turn_param->param->velo < end_velo)			motion_plan.deccel.set			(acc);
	else												motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	/*
	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	//vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	//vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();
	*/

	float diff = vehicle->ego.x_point.get();
	if(ABS(diff) < 10.0)
	{
		if(turn_param->param->turn_dir == Turn_R)
		{
			motion_plan.fix_prev_run.set(0.0);
			motion_plan.fix_post_run.set(diff*0.0);
		}
		else if(turn_param->param->turn_dir == Turn_L)
		{
			motion_plan.fix_prev_run.set(0.0);
			motion_plan.fix_post_run.set((-1.0)*diff*0.0);
		}
	}

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(DIAGONAL_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	//ir_sens->EnableIrSensStraight();
	ir_sens->EnableIrSensDiagonal();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}


void Motion::Init_Motion_long_turn_v90		(	const t_param *turn_param,t_run_pattern run_pt,
												float end_velo, float acc,
												const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	float start_velo = vehicle->ideal.velo.get();
	motion_plan.velo.set				(start_velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(end_velo);

	if(turn_param->param->velo > start_velo)			motion_plan.accel.set			(acc);
	else if(turn_param->param->velo < start_velo)		motion_plan.accel.set			(-acc);
	else												motion_plan.accel.set			(0.0f);

	if(turn_param->param->velo > end_velo)				motion_plan.deccel.set			(-acc);
	else if(turn_param->param->velo < end_velo)			motion_plan.deccel.set			(acc);
	else												motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_deccel.set		(0.0f);//計算できないわけではない
	motion_plan.rad_max_velo.set		(turn_param->param->velo/(turn_param->param->r_min/1000.0f));
	motion_plan.end_radian.set		(turn_param->param->degree/180*PI);
	motion_plan.radian_accel.set		(0.0f);//計算できないわけではない
	motion_plan.radian_deccel.set	(0.0f);//計算できないわけではない
	motion_plan.turn_r_min.set		(turn_param->param->r_min);
	motion_plan.turn_state.set		(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain & turn_param
	turn_motion_param = *turn_param;
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	/*
	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	//vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	//vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();
	*/

	float diff = vehicle->ego.x_point.get();
	if(ABS(diff) < 10.0)
	{
		if(turn_param->param->turn_dir == Turn_R)
		{
			motion_plan.fix_prev_run.set(0.0);
			motion_plan.fix_post_run.set(diff*0.0);
		}
		else if(turn_param->param->turn_dir == Turn_L)
		{
			motion_plan.fix_prev_run.set(0.0);
			motion_plan.fix_post_run.set((-1.0)*diff*0.0);
		}
	}

	motion_pattern_set(run_pt);
	motion_exeStatus_set(execute);
	motion_state_set(DIAGONAL_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}
