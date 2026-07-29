/*
 * ctrl_task.cpp
 *
 *  Created on: 2024/03/15
 *      Author: sato1
 */

#include <stdio.h>
#include "peripheral.h"
#include "typedef.h"
#include "math_utils.h"
#include "singleton.h"
#include "controller.h"

#include "vehicle.h"

#include "run_param.h"

#include "ctrl_task.h"

#include "run_typedef.h"
#include "sensing_task.h"

	namespace {
		constexpr float OM_JERK_FF_LIMIT_V = 2.0f;
		constexpr float SP_TURN_FF_LIMIT_V = 1.0f;
	}



void CtrlTask::motion_ideal_param_set()
{
	// Motions that do not explicitly generate angular jerk must not inherit the
	// final value from the preceding turn.
	vehicle->ideal.rad_jerk.set(0.0f);

	switch(motion_pattern_get())
	{
		case Search_st_section:
			SetIdeal_search_straight( );
			break;
		case Search_st_half:
		case Straight:
			SetIdeal_straight( );
			break;
		case Diagonal:
			SetIdeal_diagonal( );
			break;
		case Pivot_turn_L:
		case Pivot_turn_R:
			SetIdeal_pivot_turn( );
			break;
		case Search_slalom_L:
		case Search_slalom_R:
			SetIdeal_search_turn( );
			break;
		case Long_turnR180:
		case Long_turnL180:
		case Long_turnR90:
		case Long_turnL90:
			SetIdeal_long_turn( );
			break;
		case Turn_in_R45:
		case Turn_in_L45:
		case Turn_in_R135:
		case Turn_in_L135:
			SetIdeal_turn_in( );
			break;
		case Turn_out_R45:
		case Turn_out_L45:
		case Turn_out_R135:
		case Turn_out_L135:
			SetIdeal_turn_out( );
			break;
		case Turn_LV90:
		case Turn_RV90:
			SetIdeal_turn_v90( );
			break;
		case Long_turn_LV90:
		case Long_turn_RV90:
			SetIdeal_long_turn_v90( );
			break;
		case motor_free:
			SetIdeal_free_rotation_set();
			break;
		case Fix_wall:
			SetIdeal_fix_wall( );
			break;
		case Suction_start:
			SetIdeal_suction_start( );
			break;
		case run_brake:
			SetIdeal_stop_brake( );
			break;
		case Backward:
			SetIdeal_backward();
			break;
		case enkaigei:
			SetIdeal_enkaigei();
			break;
		case No_run:
		case Run_Pause:
			break;
	}
}

void CtrlTask::motion_prev_control()
{
	if(is_control_enable() == True)
	{
		motion_ideal_param_set();
	}
}

void CtrlTask::motion_control()
{
	if(is_control_enable() == True)
	{
		if(motion_pattern_get() != motor_free )
		{
			//
			vehicle->V_r = 0.0f;			vehicle->V_l = 0.0f;
			vehicle->motor_out_r = 0;		vehicle->motor_out_l = 0;

			// Legacy motor-constant feedforward model. Kept for comparison.
#if 0
			//calc motor speed(rpm)
			float motor_r_rpm = (1.0f)*RADPS_2_RPM *GEAR_N*(vehicle->ideal.velo.get()*1000/TIRE_RADIUS
													+ 1.0f*TREAD_WIDTH*vehicle->ideal.rad_velo.get()/(2*TIRE_RADIUS));

			float motor_l_rpm = (1.0f)*RADPS_2_RPM *GEAR_N*(vehicle->ideal.velo.get()*1000/TIRE_RADIUS
														- 1.0f*TREAD_WIDTH*vehicle->ideal.rad_velo.get()/(2*TIRE_RADIUS));

			//calc motor induce ampere
			float motor_r_ampere = 1.0f/(MOTOR_K_TR*GEAR_N)*(WEIGHT*((1.0)*vehicle->ideal.accel.get())/1000.0f*(TIRE_RADIUS)/2.0f)
								 + 1.0f/(MOTOR_K_TR*GEAR_N)*(MOUSE_INERTIA*vehicle->ideal.rad_accel.get()*TIRE_RADIUS/(TREAD_WIDTH/2.0f))
								 + MOTOR_BR*motor_r_rpm /MOTOR_K_TR*1.0;
			float motor_l_ampere = 1.0f/(MOTOR_K_TR*GEAR_N)*(WEIGHT*((1.0)*vehicle->ideal.accel.get())/1000.0f*(TIRE_RADIUS)/2.0f)
								 - 1.0f/(MOTOR_K_TR*GEAR_N)*(MOUSE_INERTIA*vehicle->ideal.rad_accel.get()*TIRE_RADIUS/(TREAD_WIDTH/2.0f))
								 + MOTOR_BR*motor_l_rpm /MOTOR_K_TR*1.0;

			float motor_r_Idot = RADPS_2_RPM*vehicle->ideal.accel.get()/(TIRE_RADIUS_M)*GEAR_N*MOTOR_BR/MOTOR_K_TR;
			float motor_l_Idot = RADPS_2_RPM*vehicle->ideal.accel.get()/(TIRE_RADIUS_M)*GEAR_N*MOTOR_BR/MOTOR_K_TR;

			//calc motor induce voltage
			float sp_FF_control_r =  MOTOR_R*motor_r_ampere + MOTOR_K_ER*motor_r_rpm/1000 + motor_r_Idot*L_BAR_DT;
			float sp_FF_control_l =  MOTOR_R*motor_l_ampere + MOTOR_K_ER*motor_l_rpm/1000 + motor_l_Idot*L_BAR_DT ;
#endif

			// Feedforward model identified from logs.  Suction changes the
			// longitudinal load, so every straight state (including the straight
			// section before/after a shell turn) uses the suction-specific model.
			// Turn and diagonal states keep their active motion-specific gains.
			const t_ff_gain *ff_gain = active_ff_gain_get();
			if (vehicle->suction_flag_get() == True && motion_control_get() == STRAIGHT_STATE) {
				ff_gain = &ff_gain_straight_suction;
			}
			const float ideal_velo = vehicle->ideal.velo.get();
			const float ideal_accel = vehicle->ideal.accel.get();
			float sp_bias_direction = 0.0f;
			if (ABS(ideal_velo) > 0.001f) {
				sp_bias_direction = SIGN(ideal_velo);
			} else if (ABS(ideal_accel) > 0.001f) {
				// At launch the target velocity can still be zero.
				sp_bias_direction = SIGN(ideal_accel);
			}
			float sp_FF_control = ff_gain->sp_bias * sp_bias_direction
								+ ff_gain->sp_velo * ideal_velo
								+ ff_gain->sp_accel * ideal_accel;
			const float ideal_rad_velo = vehicle->ideal.rad_velo.get();
			const float ideal_rad_accel = vehicle->ideal.rad_accel.get();
			const float ideal_rad_jerk = vehicle->ideal.rad_jerk.get();
			float om_bias_direction = 0.0f;
			if (ABS(ideal_rad_velo) > 0.001f) {
				om_bias_direction = SIGN(ideal_rad_velo);
			} else if (ABS(ideal_rad_accel) > 0.001f) {
				// At turn launch the target angular velocity can still be zero.
				om_bias_direction = SIGN(ideal_rad_accel);
			}
			const float ideal_rad_accel_mag = om_bias_direction * ideal_rad_accel;
			const float sp_turn_FF_control = CLAMP(
					ff_gain->sp_turn_accel_mag * ideal_rad_accel_mag
					+ ff_gain->sp_turn_velo_sq * ideal_rad_velo * ideal_rad_velo,
					-SP_TURN_FF_LIMIT_V, SP_TURN_FF_LIMIT_V);
			sp_FF_control += sp_bias_direction * sp_turn_FF_control;
			const bool is_om_decelerating = (ideal_rad_velo * ideal_rad_accel) < 0.0f;
			const float om_accel_gain = is_om_decelerating ? ff_gain->om_decel : ff_gain->om_accel;
			const float om_jerk_FF_control = CLAMP(ff_gain->om_jerk * ideal_rad_jerk,
											-OM_JERK_FF_LIMIT_V, OM_JERK_FF_LIMIT_V);
			float om_FF_control = ff_gain->om_bias * om_bias_direction
								+ ff_gain->om_velo * ideal_rad_velo
								+ om_accel_gain * ideal_rad_accel
								+ om_jerk_FF_control;


			//feedback controll
			vehicle->sp_feedback.set(vehicle->Vehicle_controller.speed_ctrl.Control(vehicle->ideal.velo.get()		,vehicle->ego.velo.get()	, (float)ctr_deltaT_ms));
			vehicle->om_feedback.set(vehicle->Vehicle_controller.omega_ctrl.Control(vehicle->ideal.rad_velo.get() 	,vehicle->ego.rad_velo.get(), (float)ctr_deltaT_ms));

			//feedforward controll
			vehicle->sp_feedforward.set(sp_FF_control);
			vehicle->om_feedforward.set(om_FF_control);

			//feedforward
			float om_feedforward_corr_R = 0.0f;
			float om_feedforward_corr_L = 0.0f;


			if((vehicle->om_feedforward.get()+vehicle->om_feedback.get()) > vehicle->battery.get())
			{
				om_feedforward_corr_L = (vehicle->om_feedforward.get()+vehicle->om_feedback.get()) - vehicle->battery.get();
			}
			else if((vehicle->om_feedforward.get()+vehicle->om_feedback.get()) < -(vehicle->battery.get()))
			{
				om_feedforward_corr_R =  vehicle->battery.get() + (vehicle->om_feedforward.get()+vehicle->om_feedback.get()) ;
			}


			//calc anti windup
			float ctrl_battery = vehicle->battery.get();
			if(ctrl_battery < 3.30f) ctrl_battery = 3.30f;

			float ctrl_limit = MAX( ABS( vehicle->sp_feedforward.get() + vehicle->om_feedforward.get()),
									ABS(-vehicle->sp_feedforward.get() + vehicle->om_feedforward.get()));

			float sp_antiwindup = vehicle->sp_feedback.get();
			float om_antiwindup = vehicle->om_feedback.get();

			if( ctrl_limit < ctrl_battery )
			{
				sp_antiwindup = vehicle->Vehicle_controller.speed_ctrl.Anti_windup_2( vehicle->sp_feedback.get(),ctrl_battery - ctrl_limit);
				om_antiwindup = vehicle->Vehicle_controller.omega_ctrl.Anti_windup_2( vehicle->om_feedback.get(),ctrl_battery - ctrl_limit);
			    //vehicle->Vehicle_controller.omega_ctrl.Anti_windup_2(vehicle->om_feedback.get() + vehicle->om_feedforward.get(),ctrl_battery) - vehicle->om_feedforward.get();
			}
			else
			{
				sp_antiwindup = vehicle->Vehicle_controller.speed_ctrl.Anti_windup_2( vehicle->sp_feedback.get(),0.0);
				om_antiwindup = vehicle->Vehicle_controller.omega_ctrl.Anti_windup_2( vehicle->om_feedback.get(),0.0);
				//vehicle->Vehicle_controller.omega_ctrl.Anti_windup_2(vehicle->om_feedback.get() + vehicle->om_feedforward.get(),ctrl_battery) - vehicle->om_feedforward.get();
			}
			//vehicle->sp_feedback.set( sp_antiwindup );
			//vehicle->om_feedback.set( om_antiwindup );

			//set & supply voltage
			vehicle->V_r =  vehicle->sp_feedforward.get()*FF_GAIN + (vehicle->om_feedforward.get() + om_feedforward_corr_R-om_feedforward_corr_L)*FF_GAIN + vehicle->sp_feedback.get() + vehicle->om_feedback.get();
			vehicle->V_l = -vehicle->sp_feedforward.get()*FF_GAIN + (vehicle->om_feedforward.get() + om_feedforward_corr_L-om_feedforward_corr_R)*FF_GAIN - vehicle->sp_feedback.get() + vehicle->om_feedback.get();

			if(ABS(vehicle->V_r) > 0.0)
			{
				vehicle->V_r = SIGN(vehicle->V_r)*(ABS(vehicle->V_r) + DEAD_VR);
			}
			else
			{
				vehicle->V_r = 0.0f;
			}

			if(ABS(vehicle->V_l) > 0.0)
			{
				vehicle->V_l = SIGN(vehicle->V_l)*(ABS(vehicle->V_l) + DEAD_VL);
			}
			else
			{
				vehicle->V_l = 0.0f;
			}

			float duty_r = vehicle->V_r/ ctrl_battery;
			float duty_l = vehicle->V_l/ ctrl_battery;

			if(ABS(duty_r) > 1.0){
				vehicle->motor_out_r = (int)(SIGN(duty_r) * 1000.0f);
			}else{
				vehicle->motor_out_r  = (int)(duty_r * 1000.0f);
			}
			vehicle->motorSetDuty_r(vehicle->motor_out_r);

			if(ABS(duty_l) > 1.0){
				vehicle->motor_out_l = (int)(SIGN(duty_l) * 1000.0f);
			}else{
				vehicle->motor_out_l = (int)(duty_l * 1000.0f);
			}
			vehicle->motorSetDuty_l(vehicle->motor_out_l);

		}
		else if(motion_pattern_get() == motor_free)
		{
			//vehicle->V_r = 0.0f;			vehicle->V_l = 0.0f;
			//vehicle->motor_out_r = 0;		vehicle->motor_out_l = 0;
			vehicle->motorSetDuty_r(vehicle->motor_out_r);
			vehicle->motorSetDuty_l(vehicle->motor_out_l);
		}
	}

	else
	{
		vehicle->V_r = 0.0f;			vehicle->V_l = 0.0f;
		vehicle->motor_out_r = 0;		vehicle->motor_out_l = 0;
		vehicle->motorSetDuty_r(vehicle->motor_out_r);
		vehicle->motorSetDuty_l(vehicle->motor_out_l);
	}


	if(vehicle->suction_flag_get() == True)
	{
		float suction_ctrl_battery = vehicle->battery.get();
		if(suction_ctrl_battery < 3.30) suction_ctrl_battery = 3.30f;
		vehicle->suction_out = (int)(vehicle->V_suction.get()/suction_ctrl_battery*1000.0f);
		if(vehicle->suction_out >= 990) vehicle->suction_out = 990;
		vehicle->suctionSetDuty(vehicle->suction_out);
	}
	else
	{
		vehicle->suctionStop();
	}

}

void CtrlTask::motion_post_control()
{

	// error process
	if(motion_exeStatus_get() == execute && is_control_enable() == True)
	{
		if(motion_pattern_get() != motor_free)
		{
			if(vehicle->ideal.velo.get() != 0.0f)
			{
				int error_flag = error_counter_get();

				if(ABS((vehicle->ego.velo.get() - vehicle->ideal.velo.get())) >= 2.0)
				{
					error_counter_set(error_counter_get() + 50);
				}

				if(vehicle->ideal.velo.get() >= 0.20f)
				{
					if(ABS((vehicle->ego.velo.get() - vehicle->ideal.velo.get()))/vehicle->ideal.velo.get() >= 0.80f )
					{
						error_counter_set(error_counter_get() + 20);
					}
				}


				if(ABS(vehicle->ego.z_accel.get()) >= 30.0)
				{
					error_counter_set(error_counter_get() + 30);
				}

				if(ABS(vehicle->ego.rad_velo.get() - vehicle->ideal.rad_velo.get()) > 10.0)
				{
					//error_cnt = error_cnt +	10;
				}

				if(ABS(vehicle->V_l) > vehicle->battery.get())
				{
					//error_counter_set(error_counter_get() + 5);
					if(ABS(vehicle->V_l) > vehicle->battery.get()*1.5)
					{
						error_counter_set(error_counter_get() + 3);
					}
					if(ABS(vehicle->V_l) >  12.0)
					{
						error_counter_set(error_counter_get() + 100);
					}
				}

				if(ABS(vehicle->V_r) > vehicle->battery.get())
				{
					//error_counter_set(error_counter_get() + 5);
					if(ABS(vehicle->V_r) > vehicle->battery.get()*1.5)
					{
						error_counter_set(error_counter_get() + 3);
					}
					if(ABS(vehicle->V_r) >  12.0)
					{
						error_counter_set(error_counter_get() + 100);
					}
				}

				if(ABS(vehicle->Vehicle_controller.speed_ctrl.get_I_peration()) > 2.0)
				{
					error_counter_set(error_counter_get() + 100);
				}
				if(ABS(vehicle->Vehicle_controller.omega_ctrl.get_I_peration()) > 2.0)
				{
					//error_counter_set(error_counter_get() + 100);
				}
				if(error_flag == error_counter_get())
				{
					error_counter_set(error_counter_get() - 2);
					if(error_counter_get() < 0) error_counter_reset();
				}
			}

			//error_counter_reset();
		}
		else
		{
			error_counter_reset();
		}
	}

	if(error_counter_get() >= 1000)
	{
		motion_exeStatus_set(error);
	}

	if(motion_exeStatus_get() == error)
	{
		Motion_error_handling();
	}
}



