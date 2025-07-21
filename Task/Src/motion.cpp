/*
 * motion.cpp
 *
 *  Created on: 2024/03/17
 *      Author: sato1
 */

#include "../Inc/ctrl_task.h"
#include "../Inc/run_typedef.h"
#include "../../Params/turn_table.h"
#include <math.h>
#include "peripheral.h"


const float detect_wall_edge_st = STRAIGHT_CORRECTION;
const float detect_wall_edge_di = DIAGONAL_CORRECTION;

float get_turn_table_value(float time_period_ms,float time_ms)
{
	float turn_table_value = 0.0f;
	if(time_ms <= time_period_ms)
	{
		//calc array position
		int std_a = (int)((time_ms*1000.0f/time_period_ms));
		int std_b = std_a + 1;
		//
		float m = ((time_ms*1000.0f/time_period_ms)) - (float)(std_a);
		float n = (float)(std_b) -((time_ms*1000.0f/time_period_ms));
		turn_table_value =  (n*accel_table[std_a] + m*accel_table[std_b]);
		return turn_table_value;
	}
	else	;

	return turn_table_value;
}
/*
float get_turn2_table_value(float time_period_ms,float time_ms)
{
	float turn_table_value = 0.0f;
	if(time_ms <= time_period_ms)
	{
		//calc array position
		int std_a = (int)((time_ms*1000.0f/time_period_ms));
		int std_b = std_a + 1;
		//
		float m = ((time_ms*1000.0f/time_period_ms)) - (float)(std_a);
		float n = (float)(std_b) -((time_ms*1000.0f/time_period_ms));
		turn_table_value =  (n*accel2_table[std_a] + m*accel2_table[std_b]);
		return turn_table_value;
	}
	else	;

	return turn_table_value;
}
*/
void Motion::SetIdeal_wall_control()
{
	//ir_sens->set_sidewall_control_cnt(vehicle->ideal.velo.get());
	if(motion_state_get() == STRAIGHT_STATE || motion_state_get() == DIAGONAL_STATE)
	{
		if(motion_state_get() == STRAIGHT_STATE) ir_sens->EnableIrSensStraight();
		if(motion_state_get() == DIAGONAL_STATE) ir_sens->EnableIrSensDiagonal();

		//検討必要
		if(vehicle->ideal.velo.get() <= 0.150 && vehicle->ideal.velo.get() >= 0.0f )
		{
			ir_sens->DisableIrSens();
		}
		else
		{
			ir_sens->EnableIrSens();
		}
		ir_sens->SetWallControl_RadVelo(vehicle, deltaT_ms);
	}
	else
	{
		ir_sens->DisableIrSens();
	}

}

void Motion::Adjust_wall_corner()
{
	if(motion_state_get() == STRAIGHT_STATE)
	{
		if((ir_sens->r_wall_corner == True || ir_sens->l_wall_corner == True) && vehicle->ideal.velo.get() > 0.1 )
		{
			//ir_sens->Division_Wall_Correction();

			if(ir_sens->r_wall_corner == True)	{		Indicate_LED(0x01|Return_LED_Status());		}
			if(ir_sens->l_wall_corner == True)	{		Indicate_LED(0x08|Return_LED_Status());		}

			if(ABS(ir_sens->r_corner_time - ir_sens->l_corner_time) < (int)((8.0)/vehicle->ideal.velo.get())
						&& motion_plan.end_length.get() >= 90.0f)
			{
				int diff_time_ms = (ir_sens->r_corner_time - ir_sens->l_corner_time);
				float diff = ((float)diff_time_ms) * vehicle->ideal.velo.get();
				if(diff != 0.0f)
				{
					vehicle->ego.radian.set(-diff/84.0);
					vehicle->ego.x_point.set(42.0*diff/65.0);
					Indicate_LED((0x0f)|Return_LED_Status());
				}
			}
		}
		if(vehicle->ideal.velo.get() > 0.2)
		{
			if(ir_sens->r_corner_time > (int)((8.0)/vehicle->ideal.velo.get()) )
			{
				Indicate_LED((~0x01)&Return_LED_Status());
			}
			if(ir_sens->l_corner_time > (int)((8.0)/vehicle->ideal.velo.get()) )
			{
				Indicate_LED((~0x08)&Return_LED_Status());
			}
			if(ir_sens->r_corner_time > (int)((8.0)/vehicle->ideal.velo.get())
					&& ir_sens->l_corner_time > (int)((8.0)/vehicle->ideal.velo.get()) )
			{
				Indicate_LED((~0x0f)&Return_LED_Status());
			}
		}
		else
		{
			if(ir_sens->r_corner_time > 40 )
			{
				Indicate_LED((~0x01)&Return_LED_Status());
			}
			if(ir_sens->l_corner_time > 40 )
			{
				Indicate_LED((~0x08)&Return_LED_Status());
			}

			if(ir_sens->l_corner_time > 40 && ir_sens->r_corner_time > 40 )
			{
				Indicate_LED((~0x0f)&Return_LED_Status());
			}
		}
	}
	else if(motion_state_get() == DIAGONAL_STATE)
	{
		if((ir_sens->r_wall_corner == True || ir_sens->l_wall_corner == True) && vehicle->ideal.velo.get() > 0.2 )
		{
			//ir_sens->Division_Wall_Correction();

			if(ir_sens->r_wall_corner == True)	{		Indicate_LED(0x01|Return_LED_Status());		}
			if(ir_sens->l_wall_corner == True)	{		Indicate_LED(0x08|Return_LED_Status());		}


			int time_diff = ABS(ir_sens->r_corner_time - ir_sens->l_corner_time);
			if(ABS((time_diff*vehicle->ideal.velo.get())-DIAG_SECTION) < (int)((10.0)) && motion_plan.end_length.get() > DIAG_SECTION)
			{

				float diff = (time_diff*vehicle->ideal.velo.get())-DIAG_SECTION;
				if(diff != 0.0f)
				{

					vehicle->ego.radian.set(-diff/DIAG_SECTION);
					vehicle->ego.x_point.set(diff/2);
					//ir_sens->Division_Wall_Correction_Reset();
					Indicate_LED((0x0f)|Return_LED_Status());

				}

			}

			if(ir_sens->r_wall_corner == True && vehicle->ideal.velo.get() > 0.2)
			{
				float diff = ir_sens->IrSensorMaxValueFromLog(sensor_sr) - DIAG_SECTION/2.0f;
				vehicle->ego.radian.set(((diff/20.0) + vehicle->ego.radian.get())/2.0f);
				vehicle->ego.x_point.set((diff+vehicle->ego.x_point.get())/2);
				Indicate_LED((0x0f)|Return_LED_Status());

			}
			if(ir_sens->l_wall_corner == True && vehicle->ideal.velo.get() > 0.2)
			{
				float diff = -(ir_sens->IrSensorMaxValueFromLog(sensor_sl) - DIAG_SECTION/2.0f);
				vehicle->ego.radian.set(((diff/20.0) + vehicle->ego.radian.get())/2.0f);
				vehicle->ego.x_point.set((diff+vehicle->ego.x_point.get())/2);
				Indicate_LED((0x0f)|Return_LED_Status());
			}
		}


		if(vehicle->ideal.velo.get() > 0.2)
		{
			if(ir_sens->r_corner_time > (int)((8.0)/vehicle->ideal.velo.get()) )
			{
				Indicate_LED((~0x01)&Return_LED_Status());
			}
			if(ir_sens->l_corner_time > (int)((8.0)/vehicle->ideal.velo.get()) )
			{
				Indicate_LED((~0x08)&Return_LED_Status());
			}
			if(ir_sens->r_corner_time > (int)((8.0)/vehicle->ideal.velo.get())
					&& ir_sens->l_corner_time > (int)((8.0)/vehicle->ideal.velo.get()) )
			{
				Indicate_LED((~0x0f)&Return_LED_Status());
			}
		}
		else
		{
			if(ir_sens->r_corner_time > 40 )
			{
				Indicate_LED((~0x01)&Return_LED_Status());
			}
			if(ir_sens->l_corner_time > 40 )
			{
				Indicate_LED((~0x08)&Return_LED_Status());
			}

			if(ir_sens->l_corner_time > 40 && ir_sens->r_corner_time > 40 )
			{
				Indicate_LED((~0x0f)&Return_LED_Status());
			}
		}
	}
}

void  Motion::SetIdeal_search_straight(){

	motion_state_set(STRAIGHT_STATE);

	if(motion_plan.length_deccel.get() < ( motion_plan.end_length.get() - vehicle->ego.length.get()))
	{
		//accel & constant velo running set up
		vehicle->ideal.accel.set(motion_plan.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_plan.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}

		if(ir_sens->Division_Wall_Correction() == True)
		{
			if(motion_plan.end_length.get()== 90.0f)
			{
				if(ABS(vehicle->ego.length.get() - SEARCH_CORRECTION) < 8.0f)
				{
					vehicle->ego.length.set((vehicle->ego.length.get() + SEARCH_CORRECTION)/2.0f);
				}
			}
		}

		if(ir_sens->r_wall_corner == True || ir_sens->l_wall_corner == True )
		{
			if(ABS(ir_sens->r_corner_time - ir_sens->l_corner_time) < (int)((8.0)/vehicle->ideal.velo.get()))
			{
				int diff_time_ms = (ir_sens->r_corner_time - ir_sens->l_corner_time);
				float diff = ((float)diff_time_ms) * vehicle->ideal.velo.get();
				if(diff != 0.0f)
				{
					vehicle->ego.radian.set(-diff/84.0);
					vehicle->ego.x_point.set(42.0*diff/65.0);
				}
			}
		}

	}
	else if(vehicle->ego.length.get() < motion_plan.end_length.get())
	{
		vehicle->ideal.accel.set(motion_plan.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_plan.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.velo.set( 0.150f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_plan.end_velo.get() == 0.0f)
		{
			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			motion_pattern_set(Run_Pause);
			motion_state_set(STRAIGHT_STATE);
			Init_Motion_stop_brake(400);
			return;
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_pattern_set(Run_Pause);
			motion_state_set(STRAIGHT_STATE);
			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}
void  Motion::SetIdeal_search_turn()
{
	static float turn_start_time_ms;
	vehicle->ideal.velo.set(motion_plan.velo.get());
	vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{

		}
		else
		{
			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
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
			turn_start_time_ms = 0.0f;

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
			vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{

		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);


			vehicle->ego.length.set(-((turn_motion_param.param->Lend + motion_plan.fix_post_run.get()) - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_pattern_set(Run_Pause);
			motion_state_set(STRAIGHT_STATE);
			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);

}

void Motion::SetIdeal_straight()
{
	motion_state_set(STRAIGHT_STATE);
	float offset = 0.0f;
	if(motion_plan.max_velo.get() > motion_plan.end_velo.get())
	{
		offset = 10.0f;
	}

	if((motion_plan.length_deccel.get()+offset) < ( motion_plan.end_length.get() - vehicle->ego.length.get()))
	{
		//accel & constant velo running set up
		vehicle->ideal.accel.set(motion_plan.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_plan.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.max_velo.get());
			vehicle->ideal.accel.set(0.0f);

		}

		//if(ABS(motion_plan.length_deccel.get()+offset) - ( motion_plan.end_length.get() - vehicle->ego.length.get()) < 5.0)
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
	}
	else if(vehicle->ego.length.get() < motion_plan.end_length.get())
	{
		vehicle->ideal.accel.set(motion_plan.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_plan.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.velo.set( 0.150f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_plan.end_velo.get() == 0.0f)
		{
			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);

			vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
			motion_pattern_set(Run_Pause);
			motion_state_set(STRAIGHT_STATE);
			Init_Motion_stop_brake(400);
			return;
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);

			vehicle->ego.length.set(-(motion_plan.end_length.get() - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
			motion_pattern_set(Run_Pause);
			motion_state_set(STRAIGHT_STATE);
			motion_exeStatus_set(complete);
		}
	}

	//if(motion_plan.end_length.get() > 50.0f)
	//{
		vehicle->ideal.radian.set(0.0f);
	//	vehicle->ego.radian.set(0.0f);
	//}

	Adjust_wall_corner();
	if(motion_plan.end_length.get() < 50.0f) ir_sens->DisableIrSens();
	else									 SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
	//vehicle->ideal.radian.set(0.0);
}


void Motion::SetIdeal_backward()
{
	motion_state_set(STRAIGHT_STATE);

	if(ABS(motion_plan.length_deccel.get()) < ( ABS(motion_plan.end_length.get()) - ABS(vehicle->ego.length.get())))
	{
		//accel & constant velo running set up
		vehicle->ideal.accel.set(motion_plan.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.velo.get()) > ABS(motion_plan.max_velo.get()))
		{
			vehicle->ideal.velo.set( motion_plan.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}

	}
	else if(ABS(vehicle->ego.length.get()) < ABS(motion_plan.end_length.get()))
	{
		vehicle->ideal.accel.set(motion_plan.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.velo.get()) < 0.120)
		{
				vehicle->ideal.velo.set( -0.1200f);		vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);	vehicle->ideal.rad_accel.set(0.0f);
		}
	}
	else
	{
			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);
			motion_pattern_set(Run_Pause);
			motion_state_set(STRAIGHT_STATE);
			Init_Motion_stop_brake(200);
			return;
	}
	Adjust_wall_corner();
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_diagonal		( )
{
	motion_state_set(DIAGONAL_STATE);

	float offset = 0.0f;
	if(motion_plan.max_velo.get() > motion_plan.end_velo.get())
	{
		offset = 10.0f;
	}

	if((motion_plan.length_deccel.get()+offset) < ( motion_plan.end_length.get() - vehicle->ego.length.get()))
	{
		vehicle->ideal.accel.set(motion_plan.accel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(vehicle->ideal.velo.get() > motion_plan.max_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.max_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}


	}
	else if(vehicle->ego.length.get() < motion_plan.end_length.get())
	{
		vehicle->ideal.accel.set(motion_plan.deccel.get());
		vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);
		if(motion_plan.end_velo.get() == 0.0f)
		{
			if(vehicle->ideal.velo.get()< 0.15)
			{
				vehicle->ideal.accel.set(0.0f);
				vehicle->ideal.velo.set( 0.150f);
				vehicle->ideal.rad_accel.set(0.0f);
				vehicle->ideal.rad_velo.set( 0.0f);
			}
		}
		else if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
		{
			vehicle->ideal.velo.set( motion_plan.end_velo.get());
			vehicle->ideal.accel.set(0.0f);
		}
	}
	else
	{
		if(motion_plan.end_velo.get() == 0.0f)
		{
			vehicle->ideal.accel.set(0.0f);
			vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			vehicle->ego.turn_slip_theta.set(0.0f);
			motion_pattern_set(Run_Pause);
			motion_state_set(DIAGONAL_STATE);
			Init_Motion_stop_brake(400);
			return;
		}
		else
		{
			vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);

			vehicle->ego.length.set(-(motion_plan.end_length.get() - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_pattern_set(Run_Pause);
			motion_state_set(DIAGONAL_STATE);
			motion_exeStatus_set(complete);
		}
	}

	Adjust_wall_corner();
	SetIdeal_wall_control();
	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_pivot_turn()
{
	vehicle->ideal.velo.set(0.0f);
	vehicle->ideal.accel.set(0.0f);

	motion_state_set(PIVTURN_STATE);
	if(ABS(motion_plan.radian_deccel.get()) < (ABS(motion_plan.end_radian.get()) - ABS(vehicle->ego.radian.get())))
	{
		vehicle->ideal.rad_accel.set(motion_plan.rad_accel.get());
		vehicle->ideal.rad_velo.set(vehicle->ideal.rad_velo.get() + vehicle->ideal.rad_accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.rad_velo.get()) > ABS(motion_plan.rad_max_velo.get()))
		{
			vehicle->ideal.rad_velo.set(motion_plan.rad_max_velo.get());
		}
	}
	else if(ABS(vehicle->ego.radian.get()) < ABS(motion_plan.end_radian.get()))
	{
		vehicle->ideal.rad_accel.set(motion_plan.rad_deccel.get());
		vehicle->ideal.rad_velo.set(vehicle->ideal.rad_velo.get() + vehicle->ideal.rad_accel.get()*(float)deltaT_ms/1000.0f);
		if(ABS(vehicle->ideal.rad_velo.get()) > ABS(motion_plan.rad_max_velo.get()))
		{
			if(vehicle->ideal.rad_velo.get() <= 0.0 && motion_plan.end_radian.get()> 0.0)
			{
				vehicle->ideal.rad_velo.set(0.0) ;
			}
			else if(vehicle->ideal.rad_velo.get() >= 0.0 && motion_plan.end_radian.get() < 0.0)
			{
				vehicle->ideal.rad_velo.set(0.0) ;
			}
		}
	}
	else
	{
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
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
		motion_pattern_set(Run_Pause);
		motion_state_set(PIVTURN_STATE);
		Init_Motion_stop_brake(400);
		return;
	}

	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_turn_in		( )
{
	static float turn_start_time_ms;
	//vehicle->ideal.velo.set(motion_plan.velo.get());
	//vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		;
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_st)/2.0f);
			}

			vehicle->ideal.accel.set(motion_plan.accel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.accel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.accel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);

			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
			float beta = vehicle->ideal.turn_slip_theta.get();
			float alpha_r = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() + TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float alpha_l = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() - TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float acc = -vehicle->ideal.horizon_velo.get()*(vehicle->ideal.rad_velo.get()+vehicle->ideal.turn_slip_dot.get())*0.0
						+vehicle->turn_slip_k.get()*(alpha_r*alpha_r + alpha_l *alpha_l)/2;
			//if( ABS(turn_motion_param.param->degree) == 135.0)
				//vehicle->ideal.accel.set(acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
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
			turn_start_time_ms = 0.0f;

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);

			ir_sens->Division_Wall_Correction_Reset();
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{
			vehicle->ideal.accel.set(motion_plan.deccel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(ir_sens->Division_Wall_Correction() == True)
			{
				if(ir_sens->r_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_L)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() + (turn_motion_param.param->Lend +  motion_plan.fix_post_run.get() +detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}

				}
				if(ir_sens->l_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_R)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() +  (turn_motion_param.param->Lend +  motion_plan.fix_post_run.get() +detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}
				}
			}

			if(motion_plan.deccel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.deccel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(motion_plan.end_velo.get());
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
	SetIdeal_wall_control();
	run_time_ms_update();

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

void Motion::SetIdeal_turn_out		( ){
	static float turn_start_time_ms;
	//vehicle->ideal.velo.set(motion_plan.velo.get());
	//vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				if(ir_sens->r_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_R)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() + (detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}

				}
				if(ir_sens->l_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_L)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() + (detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}
				}
			}

			vehicle->ideal.accel.set(motion_plan.accel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.accel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.accel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
			float beta = vehicle->ideal.turn_slip_theta.get();
			float alpha_r = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() + TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float alpha_l = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() - TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float acc = -vehicle->ideal.horizon_velo.get()*(vehicle->ideal.rad_velo.get()+vehicle->ideal.turn_slip_dot.get())*0.0
						+vehicle->turn_slip_k.get()*(alpha_r*alpha_r + alpha_l *alpha_l)/2;
			//if( ABS(turn_motion_param.param->degree) == 135.0)
				//vehicle->ideal.accel.set(acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
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
			turn_start_time_ms = 0.0f;

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);

			ir_sens->Division_Wall_Correction_Reset();
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{
			vehicle->ideal.accel.set(motion_plan.deccel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + (turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_st))/2.0f);
			}

			if(motion_plan.deccel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.deccel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(motion_plan.end_velo.get());
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
	SetIdeal_wall_control();
	run_time_ms_update();

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

void Motion::SetIdeal_long_turn		( )
{
	static float turn_start_time_ms;
	//vehicle->ideal.velo.set(motion_plan.velo.get());
	//vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		;
		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_st)/2.0f);
				if(ABS(turn_motion_param.param->degree) == 180.0f)
				{
					float length = (detect_wall_edge_st - turn_motion_param.param->Lstart);
					if(length > 0.0f)	motion_plan.fix_post_run.set(length);
				}
			}

			vehicle->ideal.accel.set(motion_plan.accel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.accel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.accel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
			float beta = vehicle->ideal.turn_slip_theta.get();
			float alpha_r = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() + TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float alpha_l = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() - TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float acc = -vehicle->ideal.horizon_velo.get()*(vehicle->ideal.rad_velo.get()+vehicle->ideal.turn_slip_dot.get())
						+vehicle->turn_slip_k.get()*(alpha_r*alpha_r + alpha_l *alpha_l)/2;
			//if( ABS(turn_motion_param.param->degree) == 180.0)
				vehicle->ideal.accel.set(acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
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
			turn_start_time_ms = 0.0f;

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);

			ir_sens->Division_Wall_Correction_Reset();
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(STRAIGHT_STATE);
		;
		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{
			vehicle->ideal.accel.set(motion_plan.deccel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(ir_sens->Division_Wall_Correction() == True)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + (turn_motion_param.param->Lend + motion_plan.fix_post_run.get() + detect_wall_edge_st))/2.0f);
			}

			if(motion_plan.deccel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.deccel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(motion_plan.end_velo.get());
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
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
	SetIdeal_wall_control();
	run_time_ms_update();


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

void Motion::SetIdeal_turn_v90		( )
{
	static float turn_start_time_ms;
	//vehicle->ideal.velo.set(motion_plan.velo.get());
	//vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			if(ir_sens->Division_Wall_Correction() == True)
			{
				if(ir_sens->r_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_R)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() + ( detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}

				}
				if(ir_sens->l_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_L)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() + (detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}
				}
			}

			vehicle->ideal.accel.set(motion_plan.accel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.accel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.accel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}

		}
		else
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);

		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
			float beta = vehicle->ideal.turn_slip_theta.get();
			float alpha_r = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() + TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float alpha_l = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() - TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float acc = -vehicle->ideal.horizon_velo.get()*(vehicle->ideal.rad_velo.get()+vehicle->ideal.turn_slip_dot.get())*0.0
						+vehicle->turn_slip_k.get()*(alpha_r*alpha_r + alpha_l *alpha_l)/2;
			//vehicle->ideal.accel.set(acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			//vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			//vehicle->ideal.turn_slip_theta.set(0.0f);
			//vehicle->ideal.turn_slip_dot.set(0.0f);
			vehicle->ideal.horizon_accel.set(0.0f);
			vehicle->ideal.horizon_velo.set(0.0f);

			vehicle->ego.turn_slip_theta.set(0.0f);
			vehicle->ego.turn_slip_dot.set(0.0f);
			vehicle->ego.horizon_accel.set(0.0f);
			vehicle->ego.horizon_velo.set(0.0f);

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
			turn_start_time_ms = 0.0f;

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
			ir_sens->Division_Wall_Correction_Reset();
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{
			float beta = vehicle->ideal.turn_slip_theta.get();
			float alpha_r = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() + TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float alpha_l = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() - TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float acc = -vehicle->ideal.horizon_velo.get()*(vehicle->ideal.rad_velo.get()+vehicle->ideal.turn_slip_dot.get())*1.0
						+vehicle->turn_slip_k.get()*(alpha_r*alpha_r + alpha_l *alpha_l)/2;
			//vehicle->ideal.accel.set(acc);

			vehicle->ideal.accel.set(motion_plan.deccel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(ir_sens->Division_Wall_Correction() == True)
			{
				if(ir_sens->r_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_L)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() + (turn_motion_param.param->Lend + detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}

				}
				if(ir_sens->l_wall_corner == True)
				{
					if(turn_motion_param.param->turn_dir == Turn_R)
					{
						vehicle->ego.length.set((vehicle->ego.length.get() + (turn_motion_param.param->Lend + detect_wall_edge_di))/2.0f);
					}
					else
					{
						ir_sens->wall_correction = False;
					}
				}
			}

			if(motion_plan.deccel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.deccel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(motion_plan.end_velo.get());
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);
			vehicle->ideal.turn_slip_dot.set(0.0f);
			vehicle->ego.length.set(-((turn_motion_param.param->Lend + motion_plan.fix_post_run.get()) - vehicle->ego.length.get()));
			vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);

			motion_state_set(DIAGONAL_STATE);
			motion_pattern_set(Run_Pause);
			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();

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

void Motion::SetIdeal_long_turn_v90		( )
{
	static float turn_start_time_ms;
	//vehicle->ideal.velo.set(motion_plan.velo.get());
	//vehicle->ideal.accel.set(0.0f);
	if(motion_plan.turn_state.get() == Prev_Turn)
	{
		motion_state_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <= (turn_motion_param.param->Lstart + motion_plan.fix_prev_run.get()))
		{
			if(ir_sens->Division_Wall_Correction() == True && vehicle->ego.length.get() < 10.0)
			{
				vehicle->ego.length.set((vehicle->ego.length.get() + detect_wall_edge_di)/2.0f);
			}

			vehicle->ideal.accel.set(motion_plan.accel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.accel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.accel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < turn_motion_param.param->velo)
				{
					vehicle->ideal.velo.set( turn_motion_param.param->velo);
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(turn_motion_param.param->velo);
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			motion_plan.turn_state.set(turn_motion_param.param->turn_dir);
			turn_start_time_ms = run_time_ms_get();
			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*turn_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*turn_motion_param.om_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == turn_motion_param.param->turn_dir)
	{
		motion_state_set(SLATURN_STATE);
		if((run_time_ms_get() - turn_start_time_ms) < motion_plan.turn_time_ms.get())
		{
			vehicle->ideal.velo.set(turn_motion_param.param->velo);
			vehicle->ideal.accel.set(0.0f);

			float rad_velo 		 	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms));
			float next_rad_velo  	= motion_plan.rad_max_velo.get()*get_turn_table_value(motion_plan.turn_time_ms.get(), (run_time_ms_get() - turn_start_time_ms) + (float)deltaT_ms);
			float rad_acc			= (next_rad_velo - rad_velo)*1000.0f/(float)deltaT_ms;
			vehicle->ideal.rad_velo.set(rad_velo);
			vehicle->ideal.rad_accel.set(rad_acc);
			float beta = vehicle->ideal.turn_slip_theta.get();
			float alpha_r = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() + TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float alpha_l = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() - TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float acc = -vehicle->ideal.horizon_velo.get()*(vehicle->ideal.rad_velo.get()+vehicle->ideal.turn_slip_dot.get())*0.0
						+vehicle->turn_slip_k.get()*(alpha_r*alpha_r + alpha_l *alpha_l)/2;
			//vehicle->ideal.accel.set(acc);
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			//vehicle->ideal.turn_slip_theta.set(0.0f);

			vehicle->ego.length.set(0.0f);
			vehicle->ego.radian.set(0.0f);
			//vehicle->ideal.turn_slip_theta.set(0.0f);
			//vehicle->ideal.turn_slip_dot.set(0.0f);
			vehicle->ideal.horizon_accel.set(0.0f);
			vehicle->ideal.horizon_velo.set(0.0f);

			vehicle->ego.turn_slip_theta.set(0.0f);
			vehicle->ego.turn_slip_dot.set(0.0f);
			vehicle->ego.horizon_accel.set(0.0f);
			vehicle->ego.horizon_velo.set(0.0f);

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
			turn_start_time_ms = 0.0f;

			vehicle->Vehicle_controller.speed_ctrl.Gain_Set(*straight_motion_param.sp_gain);
			vehicle->Vehicle_controller.omega_ctrl.Gain_Set(*straight_motion_param.om_gain);
			//vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
			//vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		}
	}

	if(motion_plan.turn_state.get() == Post_Turn)
	{
		motion_state_set(DIAGONAL_STATE);

		if(vehicle->ego.length.get() <=  (turn_motion_param.param->Lend + motion_plan.fix_post_run.get()))
		{
			float beta = vehicle->ideal.turn_slip_theta.get();
			float alpha_r = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() + TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float alpha_l = vehicle->ideal.velo.get()*beta/(vehicle->ideal.velo.get() - TREAD_WIDTH_M/2*vehicle->ideal.rad_velo.get());
			float acc = -vehicle->ideal.horizon_velo.get()*(vehicle->ideal.rad_velo.get()+vehicle->ideal.turn_slip_dot.get())*1.0
						+vehicle->turn_slip_k.get()*(alpha_r*alpha_r + alpha_l *alpha_l)/2;
			//vehicle->ideal.accel.set(acc);

			vehicle->ideal.accel.set(motion_plan.deccel.get());
			vehicle->ideal.velo.set( vehicle->ideal.velo.get() + vehicle->ideal.accel.get()*(float)deltaT_ms/1000.0f);

			if(motion_plan.deccel.get() > 0.0f)
			{
				if(vehicle->ideal.velo.get() > motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else if(motion_plan.deccel.get() < 0.0f)
			{
				if(vehicle->ideal.velo.get() < motion_plan.end_velo.get())
				{
					vehicle->ideal.velo.set( motion_plan.end_velo.get());
					vehicle->ideal.accel.set(0.0f);
				}
			}
			else
			{
				vehicle->ideal.velo.set(motion_plan.end_velo.get());
				vehicle->ideal.accel.set(0.0f);
			}
		}
		else
		{
			//vehicle->ideal.accel.set(0.0f);
			//vehicle->ideal.velo.set(0.0f);
			vehicle->ideal.length.set(0.0f);

			//vehicle->ideal.rad_accel.set(0.0f);
			//vehicle->ideal.rad_velo.set(0.0f);
			vehicle->ideal.radian.set(0.0f);
			vehicle->ideal.turn_slip_theta.set(0.0f);
			vehicle->ideal.turn_slip_dot.set(0.0f);
			vehicle->ego.length.set(-((turn_motion_param.param->Lend + motion_plan.fix_post_run.get()) - vehicle->ego.length.get()));
			//vehicle->ego.radian.set(0.0f);

			vehicle->ego.turn_x.set(0.0f);
			vehicle->ego.turn_y.set(0.0f);
			vehicle->ideal.turn_x.set(0.0f);
			vehicle->ideal.turn_y.set(0.0f);
			motion_pattern_set(Run_Pause);
			motion_exeStatus_set(complete);
		}
	}
	SetIdeal_wall_control();
	run_time_ms_update();

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

void Motion::SetIdeal_fix_wall		( )
{
	if(run_time_ms_get() <= run_time_limit_ms_get())
	{
		if(ir_sens->sen_fr.distance < 70.0 && ir_sens->sen_fl.distance < 70.0 )
		{
			float sp_err = 0.0f;		float om_err = 0.0f;
			float r_err = 0.0;			float l_err = 0.0;
			//if(ir_sens->sen_fr.distance > 50.0 && ir_sens->sen_fl.distance > 50.0 )
			//{
				//r_err = (ir_sens->sen_fr.distance - 45.0);
				//l_err = (ir_sens->sen_fl.distance - 45.0);
			//}

			float r_ref = 44.0;
			float l_ref = 45.0;

			if((ir_sens->sen_fr.distance < r_ref)  )
			{
				float diff = ir_sens->sen_fr.avg_distance - ir_sens->sen_fr.distance;
				float rwheel = vehicle->ideal.velo.get() - TREAD_WIDTH/1000.0*vehicle->ideal.rad_velo.get();
				if(SIGN(diff) == SIGN(rwheel) && ABS(rwheel) > 0.02)
				{
					r_err = (ir_sens->sen_fr.distance - r_ref);
				}
				else if(SIGN(diff) == SIGN(rwheel))
				{
					r_err = - 1.0;
				}
				else
				{
					float distance = ir_sens->sen_fr.distance-40.0;
					r_err =  -5.0f;//((40.0-distance) -45.0);
				}
			}
			else
			{
				r_err = (ir_sens->sen_fr.distance - r_ref);
			}


			if((ir_sens->sen_fl.distance < l_ref)  )
			{
				float diff = ir_sens->sen_fl.avg_distance - ir_sens->sen_fl.distance;
				float lwheel = vehicle->ideal.velo.get() + TREAD_WIDTH/1000.0*vehicle->ideal.rad_velo.get();
				if(SIGN(diff) == SIGN(lwheel) && ABS(lwheel) > 0.02 )
				{
					l_err = (ir_sens->sen_fl.distance - l_ref);
				}
				else if(SIGN(diff) == SIGN(lwheel))
				{
					l_err = - 1.0;
				}
				else
				{
					float distance = ir_sens->sen_fl.distance-40.0;
					l_err = -5.0;//((40.0-distance*3.0/7.0) -45.0);
				}
			}

			else
			{
				l_err = (ir_sens->sen_fl.distance - l_ref);
			}


			sp_err = (r_err+ l_err)/2.0f;
			om_err = (r_err- l_err)/2.0f;

			float target_acc = (1.0 * sp_err - 100.0*vehicle->ideal.velo.get());
			float target_velo = vehicle->ideal.velo.get() + target_acc/1000.0f;
			float max_set_velo = 0.3;
			if(target_velo >=  max_set_velo)
			{
				target_acc = 0.0;
				target_velo = max_set_velo;
			}
			else if(target_velo <= -max_set_velo){
				target_acc = 0.0;
				target_velo = -max_set_velo;
			}
			vehicle->ideal.accel.set(target_acc);
			vehicle->ideal.velo.set(target_velo);


			float target_rad_accel = (10.0*om_err - 20.0*vehicle->ideal.rad_velo.get());
			float target_rad_velo  = vehicle->ideal.rad_velo.get() + target_rad_accel/1000.0;

			float max_set_rad_velo = 10.0;
			if(target_rad_velo >= max_set_rad_velo)
			{
				target_rad_accel = 0.0;
				target_rad_velo = max_set_rad_velo;
			}
			else if(target_rad_velo <= -max_set_rad_velo)
			{
				target_rad_accel = 0.0;
				target_rad_velo = -max_set_rad_velo;
			}

			vehicle->ideal.rad_accel.set(target_rad_accel);
			vehicle->ideal.rad_velo.set(target_rad_velo);
		}

	}
	else
	{

		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
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
		motion_pattern_set(Run_Pause);
		Init_Motion_stop_brake(100);
		return;

	}

	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);

}


void Motion::SetIdeal_suction_start		( )
{
	if(run_time_ms_get() <= run_time_limit_ms_get())
	{
		ir_sens->EnableIrSens();
		ir_sens->SetWallControl_RadVelo(vehicle, deltaT_ms);
		vehicle->V_suction.set(vehicle->V_suction.get() + 0.05);
		if(vehicle->V_suction.get() >= motion_plan.suction_value.get())
		{
			vehicle->V_suction.set(motion_plan.suction_value.get());
		}
	}
	else
	{
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		vehicle->ideal.radian.set(0.0f);
		vehicle->ideal.turn_slip_theta.set(0.0f);

		vehicle->ego.length.set(0.0f);
		//vehicle->ego.radian.set(0.0f);
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
		//vehicle->ego.x_point.set(0.0f);
		vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
		vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		//Init_Motion_stop_brake(200);
		motion_exeStatus_set(complete);
		motion_state_set(NOP_STATE);
		motion_pattern_set(No_run);

		return;

	}

	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);

}

void Motion::SetIdeal_stop_brake	( )
{
	if(run_time_ms_get() <= run_time_limit_ms_get())
	{
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		//vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		//vehicle->ideal.radian.set(0.0f);
	}
	else
	{
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
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

		vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
		vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		//Init_Motion_stop_brake(200);
		motion_exeStatus_set(complete);
		motion_state_set(NOP_STATE);
		motion_pattern_set(No_run);

		return;
	}

	run_time_ms_update();
	vehicle->ideal.length.set(vehicle->ideal.length.get() + vehicle->ideal.velo.get()*(float)deltaT_ms);
	vehicle->ideal.radian.set(vehicle->ideal.radian.get() + vehicle->ideal.rad_velo.get()*(float)deltaT_ms/1000.0f);
}

void Motion::SetIdeal_free_rotation_set	()
{
	if(run_time_ms_get() <= run_time_limit_ms_get())
	{
		vehicle->motor_out_r = 800;		vehicle->motor_out_l = 800;
	}
	else
	{
		vehicle->motor_out_r = 0;		vehicle->motor_out_l = 0;
		vehicle->ideal.accel.set(0.0f);
		vehicle->ideal.velo.set(0.0f);
		vehicle->ideal.length.set(0.0f);

		vehicle->ideal.rad_accel.set(0.0f);
		vehicle->ideal.rad_velo.set(0.0f);
		vehicle->ideal.radian.set(0.0f);
		vehicle->ideal.turn_slip_theta.set(0.0f);

		vehicle->ego.length.set(0.0f);
		vehicle->ego.radian.set(0.0f);
		vehicle->ego.turn_slip_theta.set(0.0f);

		vehicle->ego.turn_x.set(0.0f);
		vehicle->ego.turn_y.set(0.0f);
		vehicle->ideal.turn_x.set(0.0f);
		vehicle->ideal.turn_y.set(0.0f);
		vehicle->Vehicle_controller.speed_ctrl.I_param_reset();
		vehicle->Vehicle_controller.omega_ctrl.I_param_reset();
		motion_exeStatus_set(complete);
	}

	run_time_ms_update();

}
