/*
 * vhecle.h
 *
 *  Created on: 2024/03/14
 *      Author: sato1
 */

#ifndef CPP_MODULE_INC_VEHICLE_H_
#define CPP_MODULE_INC_VEHICLE_H_

#include "typedef.h"
#include "peripheral.h"

#include "../../Component/Inc/singleton.h"
#include "../../Component/Inc/controller.h"

#include "../../Task/Inc/run_typedef.h"

class param_element
{
	private:
		float param;
	public:
		inline void set(float value) 	{	param = value;	};
		inline void init()				{	param = 0.0f;	};
		inline float get()				{   return param;	};
};

class turn_dir_element
{
	private:
		t_turn_dir param;
	public:
		inline void set(t_turn_dir dir)	{	param = dir;	};
		inline void init()				{	param = Turn_None;	};
		inline t_turn_dir get()			{   return param;	};
};

struct machine_params
{
		param_element velo;
		param_element accel;
		param_element horizon_accel;
		param_element horizon_velo;
		param_element z_accel;
		param_element length;
		param_element rad_accel;
		param_element rad_velo;
		param_element radian;
		param_element x_point;
		param_element turn_x;
		param_element turn_x_dash;
		param_element turn_y;
		param_element turn_y_dash;
		param_element turn_slip_theta;
		param_element turn_slip_dot;
		turn_dir_element 	  turn_dir_state;
};

class Vehicle
{
	protected:
		int r_duty,l_duty;
		int suction_duty;
		t_bool suction_flag = False;
	public:
		machine_params ego;
		void ego_initialize()
		{
			ego.velo.init();
			ego.accel.init();
			ego.horizon_accel.init();
			ego.horizon_velo.init();
			ego.z_accel.init();
			ego.length.init();
			ego.rad_accel.init();
			ego.rad_velo.init();
			ego.radian.init();
			ego.x_point.init();
			ego.turn_x.init();
			ego.turn_x_dash.init();
			ego.turn_y.init();
			ego.turn_y_dash.init();
			ego.turn_slip_theta.init();
			ego.turn_slip_dot.init();
			ego.turn_dir_state.init();
		}

		inline void ego_integral_init()
		{
			ego.length.init();
			ego.radian.init();
			ego.x_point.init();
			ego.turn_x.init();
			ego.turn_y.init();
			ego.turn_slip_theta.init();
		}



		machine_params ideal;
		void ideal_initialize()
		{
			ideal.velo.init();
			ideal.accel.init();
			ideal.horizon_accel.init();
			ideal.horizon_velo.init();
			ideal.z_accel.init();
			ideal.length.init();
			ideal.rad_accel.init();
			ideal.rad_velo.init();
			ideal.radian.init();
			ideal.x_point.init();
			ideal.turn_x.init();
			ideal.turn_x_dash.init();
			ideal.turn_y.init();
			ideal.turn_y_dash.init();
			ideal.turn_slip_theta.init();
			ideal.turn_slip_dot.init();
			ideal.turn_dir_state.init();
		}

		inline void ideal_integral_init()
		{
			ideal.length.init();
			ideal.radian.init();
			ideal.x_point.init();
			ideal.turn_x.init();
			ideal.turn_y.init();
			ideal.turn_slip_theta.init();
		}


		param_element battery;


		mouse_Controller Vehicle_controller;
		float V_r,V_l;
		int motor_out_r,motor_out_l;

		param_element sp_feedforward,sp_feedback;
		param_element om_feedforward,om_feedback;
		virtual void motorSetDuty_l(int out_l)
		{
			l_duty = out_l;
		}
		virtual void motorSetDuty_r(int out_r)
		{
			r_duty = out_r;
		}


		param_element  V_suction;
		param_element  turn_slip_k;
		int suction_out	= 0;
		void suction_flag_set(t_bool flag)
		{
			suction_flag = flag;
		}
		t_bool suction_flag_get()
		{
			return suction_flag;
		}


		virtual void suctionSetDuty(int out_s)
		{
			suction_duty = out_s;
			turn_slip_k.set(250.0f);
		}

		virtual void suctionStop()
		{
			suction_flag = False;
			suction_duty = 0;
			FAN_Motor_SetDuty(suction_duty);
			turn_slip_k.set(75.0f);
		}
};

class Vehicle_type7:public Vehicle,public Singleton<Vehicle_type7>{

	void motorSetDuty_l(int out_l) override
	{
		l_duty = out_l;
		Motor_SetDuty_Left(out_l);
	}

	void motorSetDuty_r(int out_r) override
	{
		r_duty = out_r;
		Motor_SetDuty_Right(out_r);
	}

	void suctionSetDuty(int out_s) override
	{
		suction_duty = out_s;
		FAN_Motor_SetDuty(suction_duty);
		turn_slip_k.set(250.0f);
	}

	void suctionStop() override
	{
		suction_flag = False;
		suction_duty = 0;
		FAN_Motor_SetDuty(suction_duty);
		turn_slip_k.set(75.0f);
	}

};

#endif /* CPP_MODULE_INC_VEHICLE_H_ */
