/*
 * set_motion.cpp
 *
 *  Created on: 2024/03/17
 *      Author: sato1
 */


#include "../Inc/ctrl_task.h"
#include "peripheral.h"
#include "../Inc/run_typedef.h"
#include "../../Params/turn_table.h"

void Motion::Motion_start()
{
	Motor_Initialize();
	vehicle->ego_initialize();
	vehicle->ideal_initialize();
	vehicle->suctionStop();
	vehicle->turn_slip_k.set(75.0);
	motion_enable_set();
	motion_pattern_set(No_run);
	motion_exeStatus_set(complete);
	motion_state_set(NOP_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
	vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
}

void Motion::Motion_end()
{
	vehicle->ego_initialize();
	vehicle->ideal_initialize();
	vehicle->suctionStop();
	vehicle->turn_slip_k.set(75.0);
	motion_disable_set();
	motion_pattern_set(No_run);
	//motion_exeStatus_set(complete);
	motion_state_set(NOP_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
	vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
	Motor_Stop();
}

void Motion::Motion_error_handling()
{
	vehicle->ego_initialize();
	vehicle->ideal_initialize();
	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	motion_disable_set();
	motion_exeStatus_set(error);
	motion_state_set(NOP_STATE);
	motion_pattern_set(No_run);
}

void Motion::Init_Motion_free_rotation_set( )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}

	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	motion_pattern_set(motor_free);
	motion_exeStatus_set(execute);
	motion_state_set(NOP_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();
	run_time_limit_ms_set(500.0f);

}

void Motion::Init_Motion_search_straight(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(vehicle->ideal.velo.get());
	motion_plan.max_velo.set			(max_sp);
	motion_plan.end_velo.set			(end_sp);
	motion_plan.accel.set			(acc);
	motion_plan.deccel.set			((-1.0f)*acc);
	motion_plan.end_length.set		(len_target);
	motion_plan.length_accel.set 	(1000*(max_sp*max_sp-motion_plan.velo.get()*motion_plan.velo.get())/(2.0*ABS(motion_plan.accel.get())));
	motion_plan.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_plan.deccel.get())));

	motion_plan.rad_accel.init		();
	motion_plan.rad_deccel.init		();
	motion_plan.rad_max_velo.init	();
	motion_plan.end_radian.init		();
	motion_plan.radian_accel.init	();
	motion_plan.radian_deccel.init	();
	motion_plan.turn_r_min.init		();
	motion_plan.turn_state.init		();
	motion_plan.turn_time_ms.init		();

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;


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

	motion_pattern_set(Search_st_section);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();

}

void Motion::Init_Motion_search_turn	(const t_param *turn_param,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{

	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}

	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
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
	motion_plan.turn_state.set			(Prev_Turn);
	motion_plan.turn_time_ms.set		( ABS(DEG2RAD(turn_param->param->degree)/(accel_Integral*motion_plan.rad_max_velo.get()))*1000.0f);

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param = *turn_param;

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

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	if(ir_sens->sen_fr.is_wall == True && ir_sens->sen_fl.is_wall == True)
	{
		float avg_distance = (ir_sens->sen_fr.distance + ir_sens->sen_fl.distance)/2.0f;

		motion_plan.fix_prev_run.set((-1.0)*(90.0 - avg_distance));

		vehicle->ego.length.init();
		vehicle->ego.radian.init();

		vehicle->ideal.length.init();
		vehicle->ideal.radian.init();
	}

	float diff = vehicle->ego.x_point.get()*1.0;
	if(ABS(diff) > 10.0)
	{
		diff = SIGN(diff) * diff;
	}
	if(turn_param->param->turn_dir == Turn_R)
	{
		motion_plan.fix_post_run.set(diff);
	}
	else if(turn_param->param->turn_dir == Turn_L)
	{
		motion_plan.fix_post_run.set((-1.0)*diff);
	}

	(motion_plan.end_radian.get() > 0) ? motion_pattern_set(Search_slalom_L): motion_pattern_set(Search_slalom_R);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();

}

void Motion::Init_Motion_straight		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(vehicle->ideal.velo.get());
	motion_plan.max_velo.set			(max_sp);
	motion_plan.end_velo.set			(end_sp);
	motion_plan.accel.set			(acc);
	motion_plan.deccel.set			((-1.0f)*acc);
	motion_plan.end_length.set		(len_target);
	motion_plan.length_accel.set 	(1000*(max_sp*max_sp-motion_plan.velo.get()*motion_plan.velo.get())/(2.0*ABS(motion_plan.accel.get())));
	motion_plan.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_plan.deccel.get())));

	motion_plan.rad_accel.init		();
	motion_plan.rad_deccel.init		();
	motion_plan.rad_max_velo.init	();
	motion_plan.end_radian.init		();
	motion_plan.radian_accel.init	();
	motion_plan.radian_deccel.init	();
	motion_plan.turn_r_min.init		();
	motion_plan.turn_state.init		();
	motion_plan.turn_time_ms.init		();

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;


	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	/*
	vehicle->ego.length.init();
	//vehicle->ego.radian.init();
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

	motion_pattern_set(Straight);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();

}

void Motion::Init_Motion_backward		(const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(0.0f);
	motion_plan.max_velo.set			(-0.20f);
	motion_plan.end_velo.set			(0.0f);
	motion_plan.accel.set			(-2.0f);
	motion_plan.deccel.set			((-1.0f)*(-2.0f));
	motion_plan.end_length.set		((-1.0)*15.0);
	motion_plan.length_accel.set 	(1000*(motion_plan.max_velo.get()*motion_plan.max_velo.get())/(2.0*ABS(motion_plan.accel.get())));
	motion_plan.length_deccel.set	(1000*(motion_plan.max_velo.get()*motion_plan.max_velo.get())/(2.0*ABS(motion_plan.deccel.get())));

	motion_plan.rad_accel.init		();
	motion_plan.rad_deccel.init		();
	motion_plan.rad_max_velo.init	();
	motion_plan.end_radian.init		();
	motion_plan.radian_accel.init	();
	motion_plan.radian_deccel.init	();
	motion_plan.turn_r_min.init		();
	motion_plan.turn_state.init		();
	motion_plan.turn_time_ms.init		();

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;


	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();


	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	//vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();


	motion_pattern_set(Backward);
	motion_exeStatus_set(execute);
	motion_state_set(STRAIGHT_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();

}


void Motion::Init_Motion_diagonal		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(vehicle->ideal.velo.get());
	motion_plan.max_velo.set			(max_sp);
	motion_plan.end_velo.set			(end_sp);
	motion_plan.accel.set			(acc);
	motion_plan.deccel.set			((-1.0f)*acc);
	motion_plan.end_length.set		(len_target);
	motion_plan.length_accel.set 	(1000*(max_sp*max_sp-motion_plan.velo.get()*motion_plan.velo.get())/(2.0*ABS(motion_plan.accel.get())));
	motion_plan.length_deccel.set	(1000*(max_sp*max_sp-end_sp*end_sp)/(2.0*ABS(motion_plan.deccel.get())));

	motion_plan.rad_accel.init		();
	motion_plan.rad_deccel.init		();
	motion_plan.rad_max_velo.init	();
	motion_plan.end_radian.init		();
	motion_plan.radian_accel.init	();
	motion_plan.radian_deccel.init	();
	motion_plan.turn_r_min.init		();
	motion_plan.turn_state.init		();
	motion_plan.turn_time_ms.init		();

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	/*
	vehicle->ego.length.init();
	//vehicle->ego.radian.init();
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

	motion_pattern_set(Diagonal);
	motion_exeStatus_set(execute);
	motion_state_set(DIAGONAL_STATE);
	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensDiagonal();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_pivot_turn	(float rad_target,float rad_acc,float rad_velo,const t_pid_gain *sp_gain ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(0.0f);
	motion_plan.max_velo.set			(0.0f);
	motion_plan.end_velo.set			(0.0f);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set 	(0.0f);
	motion_plan.length_deccel.set	(0.0f);

	motion_plan.rad_accel.set		(rad_acc);
	motion_plan.rad_deccel.set		(-rad_acc);
	motion_plan.rad_max_velo.set	(rad_velo);
	motion_plan.end_radian.set		(rad_target);
	motion_plan.radian_accel.set	(rad_velo*rad_velo/ABS(2.0*motion_plan.rad_accel.get()));
	motion_plan.radian_deccel.set	(rad_velo*rad_velo/ABS(2.0*motion_plan.rad_accel.get()));
	motion_plan.turn_r_min.set		(0.0f);
	motion_plan.turn_state.init			();//check
	motion_plan.turn_time_ms.init		();

	motion_plan.fix_prev_run.init();
	motion_plan.fix_post_run.init();

	//Set control gain
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;


	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

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

	(motion_plan.end_radian.get() > 0) ? motion_pattern_set(Pivot_turn_L) : motion_pattern_set(Pivot_turn_R);
	motion_exeStatus_set(execute);
	motion_state_set(PIVTURN_STATE);

	run_time_ms_reset();
	run_time_limit_ms_reset();

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
	motion_plan.end_length.set		(0.0f);
	motion_plan.length_accel.set		(0.0f);
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

void Motion::Init_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
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

	//ir_sens->EnableIrSensStraight();
	ir_sens->EnableIrSensDiagonal();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_long_turn	(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
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

void Motion::Init_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
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


void Motion::Init_Motion_long_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.set				(turn_param->param->velo);
	motion_plan.max_velo.set			(turn_param->param->velo);
	motion_plan.end_velo.set			(turn_param->param->velo);
	motion_plan.accel.set			(0.0f);
	motion_plan.deccel.set			(0.0f);
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

void Motion::Init_Motion_fix_wall		(float set_time,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();

	motion_pattern_set(Fix_wall);
	motion_exeStatus_set(execute);
	motion_state_set(BRAKE_STATE);
	run_time_ms_reset();
	run_time_limit_ms_set(set_time);

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_suction_start	(float suction_voltage,float set_time,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	motion_plan.suction_value.set(suction_voltage);

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();

	vehicle->V_suction.init();
	vehicle->suction_flag_set(True);
	vehicle->turn_slip_k.set(250.0);
	motion_pattern_set(Suction_start);
	motion_exeStatus_set(execute);
	motion_state_set(BRAKE_STATE);
	run_time_ms_reset();
	run_time_limit_ms_set(set_time);

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}

void Motion::Init_Motion_stop_brake	(float set_time,const t_pid_gain *sp_gain  ,const t_pid_gain *om_gain  )
{
	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
		return;
	}
	motion_plan.velo.init();
	motion_plan.max_velo.init();
	motion_plan.end_velo.init();
	motion_plan.accel.init();
	motion_plan.deccel.init();
	motion_plan.end_length.init();
	motion_plan.length_accel.init();
	motion_plan.length_deccel.init();

	motion_plan.rad_accel.init();
	motion_plan.rad_deccel.init();
	motion_plan.rad_max_velo.init();
	motion_plan.end_radian.init();
	motion_plan.radian_accel.init();
	motion_plan.radian_deccel.init();
	motion_plan.turn_r_min.init();
	motion_plan.turn_state.init();
	motion_plan.turn_time_ms.init		();

	//Set control gain & turn_param
	straight_motion_param.sp_gain = sp_gain;
	straight_motion_param.om_gain = om_gain;
	turn_motion_param.sp_gain = sp_gain;
	turn_motion_param.om_gain = om_gain;

	vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
	vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
	vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	vehicle->Vehicle_controller.omega_ctrl.I_param_reset();

	vehicle->ego.length.init();
	vehicle->ego.radian.init();
	vehicle->ego.x_point.init();
	vehicle->ego.turn_x.init();
	vehicle->ego.turn_y.init();
	vehicle->ego.turn_slip_theta.init();

	vehicle->ideal.length.init();
	vehicle->ideal.radian.init();
	vehicle->ideal.x_point.init();
	vehicle->ideal.turn_x.init();
	vehicle->ideal.turn_y.init();
	vehicle->ideal.turn_slip_theta.init();

	motion_pattern_set(run_brake);
	motion_exeStatus_set(execute);
	motion_state_set(BRAKE_STATE);
	run_time_ms_reset();
	run_time_limit_ms_set(set_time);

	ir_sens->EnableIrSensStraight();
	ir_sens->Division_Wall_Correction_Reset();

	error_counter_reset();
}
