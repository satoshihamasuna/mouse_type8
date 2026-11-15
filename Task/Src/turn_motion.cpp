/*
 * turn_motion.cpp
 *
 *  Created on: Feb 13, 2026
 *      Author: sato1
 */

#include "Task/Inc/ctrl_task.h"
#include "Task/Inc/run_typedef.h"
#include "Params/turn_table.h"
#include <math.h>
#include "Peripheral/Inc/peripheral.h"
#include "Task/Inc/wall_diagnostics.h"

void Motion::ApplyWallPositionCorrection(float reference_mm, float delay_mm,
	WallPositionCorrection::Phase phase, uint8_t side, bool side_matches)
{
	auto result = WallPositionCorrection::evaluate(vehicle->ego.length.get(), reference_mm, delay_mm, phase);
	if (!side_matches) {
		result.reason = WallPositionCorrection::Reason::WrongSide;
		result.after_mm = result.before_mm; result.delta_mm = 0.0f;
	}
	if (result.accepted()) vehicle->ego.length.set(result.after_mm);
	auto &diagnostics = WallDiagnostics::instance();
	if (diagnostics.enabled()) {
		const int16_t adc[] = {ir_sens->sen_l.value, ir_sens->sen_r.value, ir_sens->sen_fl.value, ir_sens->sen_fr.value};
		const IrRangeState states[] = {ir_sens->sen_l.range_state, ir_sens->sen_r.range_state,
			ir_sens->sen_fl.range_state, ir_sens->sen_fr.range_state};
		diagnostics.record(HAL_GetTick(), static_cast<uint8_t>(motion_pattern_get()), phase, side, result, adc, states);
	}
}

void Motion::handleTurnPrev_Straight()
{

		motion_control_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				//vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_st)/2.0f);
				//vehicle->ego.length.set(detect_wall_edge_st);
				
				if(ir_sens->r_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_R)
					{
						ApplyWallPositionCorrection(detect_wall_edge_st, ir_sens->r_corner_delay_mm, WallPositionCorrection::Phase::StraightEntry, 2);
					}
					else
					{
						ir_sens->wall_correction = False;
						ApplyWallPositionCorrection(detect_wall_edge_st, ir_sens->r_corner_delay_mm, WallPositionCorrection::Phase::StraightEntry, 2, false);
					}

				}
				if(ir_sens->l_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_L)
					{
						ApplyWallPositionCorrection(detect_wall_edge_st, ir_sens->l_corner_delay_mm, WallPositionCorrection::Phase::StraightEntry, 1);
					}
					else
					{
						ir_sens->wall_correction = False;
						ApplyWallPositionCorrection(detect_wall_edge_st, ir_sens->l_corner_delay_mm, WallPositionCorrection::Phase::StraightEntry, 1, false);
					}
				}


			}

			vehicle->ideal.accel.set(motion_plan.accel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.accel.get() > 0.0f && vehicle->ideal.velo.get() > turn_motion_param.param->velo)
			{
				vehicle->ideal.velo.set( turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
			else if(motion_plan.accel.get() < 0.0f && vehicle->ideal.velo.get() < turn_motion_param.param->velo)
			{
				vehicle->ideal.velo.set( turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms_set(run_time_ms_get());

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);
			active_ff_gain_set(turn_motion_param.ff_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
}

void Motion::handleTurnPrev_Diagonal()
{
		motion_control_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				if(ir_sens->r_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_R)
					{
						ApplyWallPositionCorrection(detect_wall_edge_di, ir_sens->r_corner_delay_mm, WallPositionCorrection::Phase::DiagonalEntry, 2);
					}
					else
					{
						ir_sens->wall_correction = False;
						ApplyWallPositionCorrection(detect_wall_edge_di, ir_sens->r_corner_delay_mm, WallPositionCorrection::Phase::DiagonalEntry, 2, false);
					}

				}
				
				if(ir_sens->l_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_L)
					{
						ApplyWallPositionCorrection(detect_wall_edge_di, ir_sens->l_corner_delay_mm, WallPositionCorrection::Phase::DiagonalEntry, 1);
					}
					else
					{
						ir_sens->wall_correction = False;
						ApplyWallPositionCorrection(detect_wall_edge_di, ir_sens->l_corner_delay_mm, WallPositionCorrection::Phase::DiagonalEntry, 1, false);
					}
				}
			}

			vehicle->ideal.accel.set(motion_plan.accel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.accel.get() > 0.0f && vehicle->ideal.velo.get() > turn_motion_param.param->velo)
			{
				vehicle->ideal.velo.set( turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
			else if(motion_plan.accel.get() < 0.0f && vehicle->ideal.velo.get() < turn_motion_param.param->velo)
			{
				vehicle->ideal.velo.set( turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms_set(run_time_ms_get());

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);
			active_ff_gain_set(turn_motion_param.ff_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}

}

void Motion::handleTurnPost_Straight()
{
		motion_control_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{

			if(ir_sens->Division_Wall_Correction() == True)
			{
				//vehicle->ego.length.set((vehicle->ego.length.get() + (turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_st))/2.0f);
				ApplyWallPositionCorrection(turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_st,
					ir_sens->StraightCornerDelay(), WallPositionCorrection::Phase::StraightExit,
					(ir_sens->l_wall_corner == True ? 1U : 0U) | (ir_sens->r_wall_corner == True ? 2U : 0U));
			}


			vehicle->ideal.accel.set(motion_plan.deccel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.deccel.get() > 0.0f && vehicle->ideal.velo.get() > motion_plan.end_velo.get())
			{
				vehicle->ideal.velo.set( motion_plan.end_velo.get());
				vehicle->ideal.accel.set(0.0f);
			}
			else if(motion_plan.deccel.get() < 0.0f && vehicle->ideal.velo.get() < motion_plan.end_velo.get())
			{
				vehicle->ideal.velo.set( motion_plan.end_velo.get());
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set( motion_plan.end_velo.get());
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);
			vehicle->ideal.turn_slip_dot.set(0.0f);
			vehicle->ideal.horizon_accel.set(0.0f);
			vehicle->ideal.horizon_velo.set(0.0f);

			vehicle->ego.turn_slip_theta.set(0.0f);
			vehicle->ego.turn_slip_dot.set(0.0f);
			vehicle->ego.horizon_accel.set(0.0f);
			vehicle->ego.horizon_velo.set(0.0f);

			vehicle->ego.length.set(-((turn_motion_param.param->Lend + motion_plan.fix_post_run.get()) - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_pattern_set(Run_Pause);
			motion_exeStatus_set(complete);
		}
}


void Motion::handleTurnPost_Diagonal()
{
		motion_control_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{

			if(ir_sens->Division_Wall_Correction() == True)
			{
				if(ir_sens->r_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_L)
					{
						ApplyWallPositionCorrection(turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_di, ir_sens->r_corner_delay_mm, WallPositionCorrection::Phase::DiagonalExit, 2);
					}
					else
					{
						ir_sens->wall_correction = False;
						ApplyWallPositionCorrection(turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_di, ir_sens->r_corner_delay_mm, WallPositionCorrection::Phase::DiagonalExit, 2, false);
					}

				}
				if(ir_sens->l_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_R)
					{
						ApplyWallPositionCorrection(turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_di, ir_sens->l_corner_delay_mm, WallPositionCorrection::Phase::DiagonalExit, 1);
					}
					else
					{
						ir_sens->wall_correction = False;
						ApplyWallPositionCorrection(turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_di, ir_sens->l_corner_delay_mm, WallPositionCorrection::Phase::DiagonalExit, 1, false);
					}
				}
			}

			/*update ideal velo & accel*/
			vehicle->ideal.accel.set(motion_plan.deccel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.deccel.get() > 0.0f && vehicle->ideal.velo.get() > motion_plan.end_velo.get())
			{
				vehicle->ideal.velo.set( motion_plan.end_velo.get());
				vehicle->ideal.accel.set(0.0f);
			}
			else if(motion_plan.deccel.get() < 0.0f && vehicle->ideal.velo.get() < motion_plan.end_velo.get())
			{
				vehicle->ideal.velo.set( motion_plan.end_velo.get());
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			//vehicle->ideal.velo.set(motion_plan.end_velo.get());
			//vehicle->ideal.accel.set(0.0f);

			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);
			vehicle->ideal.turn_slip_dot.set(0.0f);
			vehicle->ideal.horizon_accel.set(0.0f);
			vehicle->ideal.horizon_velo.set(0.0f);

			vehicle->ego.turn_slip_theta.set(0.0f);
			vehicle->ego.turn_slip_dot.set(0.0f);
			vehicle->ego.horizon_accel.set(0.0f);
			vehicle->ego.horizon_velo.set(0.0f);

			vehicle->ego.length.set(-((turn_motion_param.param->Lend + motion_plan.fix_post_run.get()) - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_pattern_set(Run_Pause);
			motion_exeStatus_set(complete);
		}
}
void Motion::handleTurnMain()
{
		motion_control_set(SLATURN_STATE);
		vehicle->Vehicle_controller.speed_ctrl.Disable_Integral();
		float exe_turn_time = (run_time_ms_get() - turn_start_time_ms_get());
		if(exe_turn_time < motion_plan.turn_time_ms.get())
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);
			const float turn_time_ms = motion_plan.turn_time_ms.get();
			const float sample_time_ms = (float)deltaT_ms;
			const float next_time_ms = (exe_turn_time + sample_time_ms < turn_time_ms)
									 ? exe_turn_time + sample_time_ms : turn_time_ms;
			const float next2_time_ms = (exe_turn_time + 2.0f * sample_time_ms < turn_time_ms)
									  ? exe_turn_time + 2.0f * sample_time_ms : turn_time_ms;
			const float rad_max_velo = motion_plan.rad_max_velo.get();
			const float rad_velo = rad_max_velo * get_turn_table_value(turn_time_ms, exe_turn_time);
			const float next_rad_velo = rad_max_velo * get_turn_table_value(turn_time_ms, next_time_ms);
			const float next2_rad_velo = rad_max_velo * get_turn_table_value(turn_time_ms, next2_time_ms);
			const float rad_acc = (next_rad_velo - rad_velo) * 1000.0f / sample_time_ms;
			const float next_rad_acc = (next2_rad_velo - next_rad_velo) * 1000.0f / sample_time_ms;
			const float rad_jerk = (next_rad_acc - rad_acc) * 1000.0f / sample_time_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
			vehicle->ideal.rad_jerk.set(rad_jerk);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_jerk.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			vehicle->ideal.turn_x_dash.set(0.0f);
			vehicle->ideal.turn_y_dash.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);
			vehicle->ideal.x_point.set(0.0f);

			vehicle->ego.turn_x_dash.set(0.0f);
			vehicle->ego.turn_y_dash.set(0.0f);
			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ego.x_point.set(0.0f);

			motion_plan.turn_state.set(Post_Turn);
			ir_sens->PrimeStraightWallControl();
			turn_start_time_ms_reset();

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
			active_ff_gain_set(straight_motion_param.ff_gain);

			ir_sens->Division_Wall_Correction_Reset();
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
			vehicle->Vehicle_controller.speed_ctrl.Enable_Integral();
		}
}

void Motion::updateTurnKinematics()
{
	float set_velo = vehicle->ideal.velo.get();
	if(set_velo == 0.0f) set_velo = 0.001;

	float prev_slip_theta = vehicle->ideal.turn_slip_theta.get();
	float slip_theta = (prev_slip_theta*1000.0f - vehicle->ideal.rad_velo.get())
						/(1000.0f + vehicle->turn_slip_k.get()/(set_velo*(1+prev_slip_theta*prev_slip_theta/2)));
	vehicle->ideal.turn_slip_dot.set(-vehicle->turn_slip_k.get()*(slip_theta)/(set_velo*(1+slip_theta*slip_theta/2))-vehicle->ideal.rad_velo.get());
	vehicle->ideal.turn_slip_theta.set(slip_theta )	;


	float horizon_velo = vehicle->ideal.velo.get()*slip_theta;
	float horizon_acc  = -vehicle->turn_slip_k.get()*(slip_theta) - vehicle->ideal.rad_velo.get()*vehicle->ideal.velo.get();
	vehicle->ideal.horizon_accel.set(horizon_acc);
	vehicle->ideal.horizon_velo.set(horizon_velo);

	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_turn_in		( )
{

	if(motion_plan.turn_state.get() == Prev_Turn) handleTurnPrev_Straight();

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir) handleTurnMain();

	if(motion_plan.turn_state.get() == Post_Turn) handleTurnPost_Diagonal();

	SetIdeal_wall_control();
	run_time_ms_update();
	updateTurnKinematics();

}

void Motion::SetIdeal_turn_out		( )
{
	if(motion_plan.turn_state.get() == Prev_Turn) handleTurnPrev_Diagonal();

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir) handleTurnMain();

	if(motion_plan.turn_state.get() == Post_Turn) handleTurnPost_Straight();

	SetIdeal_wall_control();
	run_time_ms_update();
	updateTurnKinematics();
}

void Motion::SetIdeal_long_turn		( )
{
	if(motion_plan.turn_state.get() == Prev_Turn) handleTurnPrev_Straight();

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir) handleTurnMain();

	if(motion_plan.turn_state.get() == Post_Turn) handleTurnPost_Straight();

	SetIdeal_wall_control();
	run_time_ms_update();
	updateTurnKinematics();
}

void Motion::SetIdeal_turn_v90		( )
{
	if(motion_plan.turn_state.get() == Prev_Turn) handleTurnPrev_Diagonal();

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir) handleTurnMain();

	if(motion_plan.turn_state.get() == Post_Turn) handleTurnPost_Diagonal();

	SetIdeal_wall_control();
	run_time_ms_update();
	updateTurnKinematics();
}

void Motion::SetIdeal_long_turn_v90		( )
{
	if(motion_plan.turn_state.get() == Prev_Turn) handleTurnPrev_Diagonal();

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir) handleTurnMain();

	if(motion_plan.turn_state.get() == Post_Turn) handleTurnPost_Diagonal();

	SetIdeal_wall_control();
	run_time_ms_update();
	updateTurnKinematics();
}
