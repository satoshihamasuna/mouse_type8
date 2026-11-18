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

void Motion::ApplyWallPositionCorrection(float reference_mm,
	WallPositionCorrection::Phase phase, uint8_t expected_side)
{
	if (WallPositionCorrection::diagonal_phase(phase) && (phase==WallPositionCorrection::Phase::DiagonalRun
		|| ir_sens->WallEdgeMode_Get()!=WallPositionCorrection::Mode::Legacy)) {
		ir_sens->RecordDiagonalWallFeatures(vehicle,static_cast<uint8_t>(motion_pattern_get()),phase,expected_side);
		return; // 斜めの新しい枠組みは常にShadow。走行用EKF・位置・指令へ注入しない。
	}
	if (ir_sens->wall_correction == True && phase != WallPositionCorrection::Phase::StraightRun) return;
	const uint8_t mask = (ir_sens->l_wall_corner == True ? 1U : 0U) | (ir_sens->r_wall_corner == True ? 2U : 0U);
	if (!mask) return;
	if ((phase == WallPositionCorrection::Phase::SearchStraight || phase == WallPositionCorrection::Phase::StraightEntry
		|| phase == WallPositionCorrection::Phase::StraightRun)
		&& ir_sens->FeatureEkfEnabled()) {
		ir_sens->QueueWallFeature(vehicle,reference_mm,static_cast<uint8_t>(motion_pattern_get()),phase,expected_side);
		return; // 同じIRQのEKF予測後に観測する。既存の長さ補正を重ねない。
	}
	const auto mode = ir_sens->WallEdgeMode_Get();
	const auto decision = WallPositionCorrection::decide(vehicle->ego.length.get(), reference_mm,
		ir_sens->r_corner_delay_mm, ir_sens->l_corner_delay_mm, mask, expected_side,
		ir_sens->wall_edge_generation, ir_sens->r_corner_generation, ir_sens->l_corner_generation, phase, mode);
	if (decision.selected_side) {
		const auto &result = decision.selected_side == 2U ? decision.right : decision.left;
		if (result.accepted()) {
			vehicle->ego.length.set(result.after_mm);
			vehicle->pose.correct_passage(result.after_mm-result.before_mm,0.0f);
		}
		ir_sens->wall_correction = True;
	}
	auto &diagnostics = WallDiagnostics::instance();
	if (diagnostics.enabled()) {
		const int16_t adc[] = {ir_sens->sen_l.value, ir_sens->sen_r.value, ir_sens->sen_fl.value, ir_sens->sen_fr.value};
		const IrRangeState states[] = {ir_sens->sen_l.range_state, ir_sens->sen_r.range_state,
			ir_sens->sen_fl.range_state, ir_sens->sen_fr.range_state};
		if (mask & 2U) diagnostics.record(HAL_GetTick(), static_cast<uint8_t>(motion_pattern_get()), phase, 2,
			decision.right, adc, states, mode, ir_sens->wall_edge_generation, ir_sens->r_corner_generation);
		if (mask & 1U) diagnostics.record(HAL_GetTick(), static_cast<uint8_t>(motion_pattern_get()), phase, 1,
			decision.left, adc, states, mode, ir_sens->wall_edge_generation, ir_sens->l_corner_generation);
	}
}

void Motion::handleTurnPrev_Straight()
{
		ir_sens->EnableIrSensStraight();

		motion_control_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			ApplyWallPositionCorrection(ir_sens->StraightNodeFeatureReference(detect_wall_edge_st), WallPositionCorrection::Phase::StraightEntry,
				turn_motion_param.param->turn_dir == Turn_R ? 2U : 1U);

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
			// The passage heading precedes this IRQ's measured yaw. Odometry
			// radian may have been reset at a straight boundary or wall edge.
			shortest_turn_heading.clear();
			ir_sens->BeginTurnHeading({});
			if (ir_sens->CanContinueStraightWallPose()) {
				const MotionHeading::Prior prior={ir_sens->StraightHeadingForTurn()
					+ vehicle->ego.rad_velo.get() * static_cast<float>(deltaT_ms) / 1000.0f,
					ir_sens->StraightHeadingVarianceForTurn()
					+ wall_ekf_param.heading_walk_rad_sqrt_s * wall_ekf_param.heading_walk_rad_sqrt_s
						* static_cast<float>(deltaT_ms) / 1000.0f,true};
				shortest_turn_heading.begin(prior.heading_rad,DEG2RAD(turn_motion_param.param->degree),prior.variance_rad2);
				ir_sens->BeginTurnHeading(prior);
			}
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
		ir_sens->EnableIrSensDiagonal();
		motion_control_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			ApplyWallPositionCorrection(detect_wall_edge_di, WallPositionCorrection::Phase::DiagonalEntry,
				turn_motion_param.param->turn_dir == Turn_R ? 2U : 1U);

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
			shortest_turn_heading.clear();
			ir_sens->BeginTurnHeading({});
			float heading_rad=0.0f, variance_rad2=0.0f;
			if (ir_sens->DiagonalHeadingForTurn(heading_rad,variance_rad2)) {
				// Diagonal pose already includes this IRQ's sensing prediction.
				shortest_turn_heading.begin(heading_rad,DEG2RAD(turn_motion_param.param->degree),variance_rad2);
				ir_sens->BeginTurnHeading({heading_rad,variance_rad2,true});
			}
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
		// Switch now: main and post can execute in the same control IRQ.
		ir_sens->EnableIrSensStraight();
		motion_control_set(STRAIGHT_STATE);
		const float wall_reference=turn_motion_param.param->Lend + motion_plan.fix_post_run.get()
			+ ir_sens->StraightNodeFeatureReference(detect_wall_edge_st);
		ir_sens->BeginStraightRunFeatures(wall_reference);
		if (ir_sens->FeatureEkfEnabled())
			ApplyWallPositionCorrection(wall_reference,
				WallPositionCorrection::Phase::StraightRun);
		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{

			if (!ir_sens->FeatureEkfEnabled())
				ApplyWallPositionCorrection(turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_st,
					WallPositionCorrection::Phase::StraightExit);


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

			vehicle->ideal.horizon_accel.set(0.0f);
			vehicle->ideal.horizon_velo.set(0.0f);

			vehicle->ego.turn_slip_theta.set(0.0f);

			vehicle->ego.horizon_accel.set(0.0f);
			vehicle->ego.horizon_velo.set(0.0f);

			ir_sens->RebaseStraightFeatureCandidates(-(turn_motion_param.param->Lend + motion_plan.fix_post_run.get()));
			vehicle->ego.length.set(-((turn_motion_param.param->Lend + motion_plan.fix_post_run.get()) - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->reset_turn_position_for_log();

			motion_pattern_set(Run_Pause);
			motion_exeStatus_set(complete);
		}
}


void Motion::handleTurnPost_Diagonal()
{
		ir_sens->EnableIrSensDiagonal();
		motion_control_set(DIAGONAL_STATE);
		ir_sens->BeginDiagonalExitFeatures(turn_motion_param.param->Lend + motion_plan.fix_post_run.get());
		ir_sens->BeginDiagonalPoseRoute(vehicle,vehicle->ego.length.get()-turn_motion_param.param->Lend-motion_plan.fix_post_run.get(),
			vehicle->ego.radian.get());

		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{

			ApplyWallPositionCorrection(turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_di,
				WallPositionCorrection::Phase::DiagonalExit, turn_motion_param.param->turn_dir == Turn_L ? 2U : 1U);

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

			vehicle->ideal.horizon_accel.set(0.0f);
			vehicle->ideal.horizon_velo.set(0.0f);

			vehicle->ego.turn_slip_theta.set(0.0f);

			vehicle->ego.horizon_accel.set(0.0f);
			vehicle->ego.horizon_velo.set(0.0f);

			ir_sens->RebaseDiagonalFeatureCandidates(-(turn_motion_param.param->Lend + motion_plan.fix_post_run.get()),vehicle->ego.radian.get());
			vehicle->ego.length.set(-((turn_motion_param.param->Lend + motion_plan.fix_post_run.get()) - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->reset_turn_position_for_log();

			motion_pattern_set(Run_Pause);
			motion_exeStatus_set(complete);
		}
}
void Motion::handleTurnMain()
{
		// The entry handler may call main in the same IRQ. Its begin() already
		// includes that sample; each subsequent measured increment belongs once.
		if (run_time_ms_get() > turn_start_time_ms_get())
			shortest_turn_heading.predict(vehicle->ego.rad_velo.get(), static_cast<float>(deltaT_ms) / 1000.0f);
		ir_sens->IrSensMotion_Set(TURN_IRSENS);
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

			const float diagonal_exit_heading=vehicle->ego.radian.get()-DEG2RAD(turn_motion_param.param->degree);
			vehicle->pose.complete_turn(DEG2RAD(turn_motion_param.param->degree));
			MotionHeading::Prior exit_heading={shortest_turn_heading.residual_rad(),shortest_turn_heading.variance_rad2(),shortest_turn_heading.valid()};
			const auto posterior=ir_sens->TurnHeadingPosterior();
			const float posterior_residual=posterior.heading_rad-DEG2RAD(turn_motion_param.param->degree);
			// Both estimates contain the same gyro data. Replace the propagated
			// marginal with the accepted EKF marginal; never fuse them twice.
			if (exit_heading.usable() && posterior.usable()
				&& fabsf(posterior_residual-exit_heading.heading_rad)<=wall_ekf_param.heading_disagreement_limit_rad
				&& fabsf(posterior_residual)<=search_turn_guard_param.exit_residual_limit_rad)
				exit_heading={posterior_residual,posterior.variance_rad2,true};
			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			vehicle->ideal.x_point.set(0.0f);

			vehicle->reset_turn_position_for_log();
			vehicle->ego.x_point.set(0.0f);

			motion_plan.turn_state.set(Post_Turn);
			const auto pattern=motion_pattern_get();
			if (ir_sens->WallEkfMode_Get()==IrSensTask::WallEkfMode::Control
				&& (pattern==Long_turnR90 || pattern==Long_turnL90 || pattern==Long_turnR180 || pattern==Long_turnL180
					|| pattern==Turn_out_R45 || pattern==Turn_out_L45 || pattern==Turn_out_R135 || pattern==Turn_out_L135)) {
				// Enter the new sensor frame BEFORE priming it; a mode change resets
				// pose. Discard rotating ADC averages and retain measured yaw variance.
				ir_sens->EnableIrSensStraight();
				ir_sens->PrimeStraightTurnExit(exit_heading.heading_rad, exit_heading.usable(),exit_heading.variance_rad2, true);
			} else ir_sens->PrimeStraightWallControl();
			shortest_turn_heading.clear();
			if (pattern==Turn_in_R45 || pattern==Turn_in_L45 || pattern==Turn_in_R135 || pattern==Turn_in_L135
				|| pattern==Turn_RV90 || pattern==Turn_LV90 || pattern==Long_turn_RV90 || pattern==Long_turn_LV90) {
				const bool inherited=ir_sens->WallEkfMode_Get()==IrSensTask::WallEkfMode::Control && exit_heading.usable();
				ir_sens->PrimeDiagonalFeatureHeading(inherited?exit_heading.heading_rad:diagonal_exit_heading);
				ir_sens->PrimeDiagonalPoseHeading(inherited?exit_heading:MotionHeading::Prior{});
			}
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
	const float slip_k_m_s2 = vehicle->turn_slip_k_m_s2();
	float set_velo = vehicle->ideal.velo.get();
	if(set_velo == 0.0f) set_velo = 0.001;

	float prev_slip_theta = vehicle->ideal.turn_slip_theta.get();
	float slip_theta = (prev_slip_theta*1000.0f - vehicle->ideal.rad_velo.get())
						/(1000.0f + slip_k_m_s2/(set_velo*(1+prev_slip_theta*prev_slip_theta/2)));

	vehicle->ideal.turn_slip_theta.set(slip_theta )	;


	float horizon_velo = vehicle->ideal.velo.get()*slip_theta;
	float horizon_acc  = -slip_k_m_s2*(slip_theta) - vehicle->ideal.rad_velo.get()*vehicle->ideal.velo.get();
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
