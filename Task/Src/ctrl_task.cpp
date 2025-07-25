/*
 * ctrl_task.cpp
 *
 *  Created on: 2024/03/15
 *      Author: sato1
 */

#include <stdio.h>
#include "peripheral.h"
#include "typedef.h"

#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controller.h"

#include "../../Module/Inc/vehicle.h"

#include "../../Params/run_param.h"

#include "../Inc/ctrl_task.h"

#include "../Inc/run_typedef.h"
#include "../Inc/run_typedef.h"
#include "../Inc/sensing_task.h"



void CtrlTask::motion_ideal_param_set()
{

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

			//calc motor speed(rpm)
			float motor_r_rpm = (1.0f)*RAD_2_RPM*GEAR_N*(vehicle->ideal.velo.get()*1000/TIRE_RADIUS
														+ 1.0f*TREAD_WIDTH*vehicle->ideal.rad_velo.get()/(2*TIRE_RADIUS));

			float motor_l_rpm = (1.0f)*RAD_2_RPM*GEAR_N*(vehicle->ideal.velo.get()*1000/TIRE_RADIUS
														- 1.0f*TREAD_WIDTH*vehicle->ideal.rad_velo.get()/(2*TIRE_RADIUS));
			//calc friction()
			float friction = 0.0f;
			if(motion_pattern_get() != Fix_wall || motion_pattern_get() != run_brake)
				friction = (ABS(vehicle->ideal.velo.get()) > 0.15) ? (float)(SIGN(vehicle->ideal.velo.get()))*0.05*9.8/(1+0.0*ABS(vehicle->ideal.accel.get())):0.0f;

			//calc motor induce ampere
			float motor_r_ampere = 1.0f/(MOTOR_K_TR*GEAR_N)*(WEIGHT*(vehicle->ideal.accel.get()+friction*0.0)/1000.0f*(TIRE_RADIUS)/2.0f)
								 + 1.0f/(MOTOR_K_TR*GEAR_N)*(MOUSE_INERTIA*vehicle->ideal.rad_accel.get()*TIRE_RADIUS/(TREAD_WIDTH/2.0f))
								 + MOTOR_BR*motor_r_rpm /MOTOR_K_TR*1.0;
			float motor_l_ampere = 1.0f/(MOTOR_K_TR*GEAR_N)*(WEIGHT*(vehicle->ideal.accel.get()+friction*0.0)/1000.0f*(TIRE_RADIUS)/2.0f)
								 - 1.0f/(MOTOR_K_TR*GEAR_N)*(MOUSE_INERTIA*vehicle->ideal.rad_accel.get()*TIRE_RADIUS/(TREAD_WIDTH/2.0f))
								 + MOTOR_BR*motor_l_rpm /MOTOR_K_TR*1.0;

			//calc motor induce voltage
			float sp_FF_control_r =  MOTOR_R*motor_r_ampere + MOTOR_K_ER*motor_r_rpm/1000;
			float sp_FF_control_l =  MOTOR_R*motor_l_ampere + MOTOR_K_ER*motor_l_rpm/1000;


			//feedback controll
			vehicle->sp_feedback.set(vehicle->Vehicle_controller.speed_ctrl.Control(vehicle->ideal.velo.get()		,vehicle->ego.velo.get()	, (float)ctr_deltaT_ms));
			vehicle->om_feedback.set(vehicle->Vehicle_controller.omega_ctrl.Control(vehicle->ideal.rad_velo.get()	,vehicle->ego.rad_velo.get(), (float)ctr_deltaT_ms));

			//feedforward controll
			vehicle->sp_feedforward.set((sp_FF_control_r + sp_FF_control_l)/2.0);
			vehicle->om_feedforward.set((sp_FF_control_r - sp_FF_control_l)/2.0);

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
			vehicle->V_r =  vehicle->sp_feedforward.get()*1.0 + (vehicle->om_feedforward.get() + om_feedforward_corr_R-om_feedforward_corr_L)*1.0 + vehicle->sp_feedback.get() + vehicle->om_feedback.get();
			vehicle->V_l = -vehicle->sp_feedforward.get()*1.0 + (vehicle->om_feedforward.get() + om_feedforward_corr_L-om_feedforward_corr_R)*1.0 - vehicle->sp_feedback.get() + vehicle->om_feedback.get();

			float duty_r = vehicle->V_r/vehicle->battery.get();
			float duty_l = vehicle->V_l/vehicle->battery.get();

			if(ABS(duty_r) > 1.0){
				vehicle->motor_out_r = (int)(SIGN(duty_r) * 4.0f * 250.0f);
			}else{
				vehicle->motor_out_r  = (int)(duty_r * 1000.0f);
			}
			vehicle->motorSetDuty_r(vehicle->motor_out_r);

			if(ABS(duty_l) > 1.0){
				vehicle->motor_out_l = (int)(SIGN(duty_l) * 4.0f * 250.0f);
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

				if(vehicle->ideal.velo.get() >= 0.120f)
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
						error_counter_set(error_counter_get() + 5);
					}
				}
				if(ABS(vehicle->V_r) > vehicle->battery.get())
				{
					//error_counter_set(error_counter_get() + 5);
					if(ABS(vehicle->V_r) > vehicle->battery.get()*1.5)
					{
						error_counter_set(error_counter_get() + 5);
					}
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



