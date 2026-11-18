/*
 * vhecle.h
 *
 *  Created on: 2024/03/14
 *      Author: sato1
 */

#ifndef CPP_MODULE_INC_VEHICLE_H_
#define CPP_MODULE_INC_VEHICLE_H_

#include "Component/Inc/typedef.h"
#include "Peripheral/Inc/peripheral.h"

#include "Component/Inc/singleton.h"
#include "Component/Inc/controller.h"

#include "Task/Inc/run_typedef.h"
#include "Module/Inc/pose_frames.h"
#include "Params/turn_pose_param.h"

class param_element
{
	private:
		float param = 0.0f;
	public:
		inline void set(float value) 	{	param = value;	};
		inline void init()				{	param = 0.0f;	};
		inline float get() const			{   return param;	};
};

class turn_dir_element
{
	private:
		t_turn_dir param = Turn_None;
	public:
		inline void set(t_turn_dir dir)	{	param = dir;	};
		inline void init()				{	param = Turn_None;	};
		inline t_turn_dir get() const	{   return param;	};
};

struct machine_params
{
		// 前後運動。lengthは区間内の進行位置 [mm]。
		param_element velo;
		param_element accel;
		param_element length;

		// 横運動。x_pointは既存の制御座標における左正の位置 [mm]。
		param_element horizon_accel;
		param_element horizon_velo;
		param_element x_point;

		// 上下加速度。
		param_element z_accel;

		// 姿勢 [rad] と角速度・角加速度・角躍度。
		param_element radian;
		param_element rad_velo;
		param_element rad_accel;
		param_element rad_jerk;

		// 機体の向きと速度方向の差 [rad]。poseの姿勢とは別の状態。
		param_element turn_slip_theta;
		turn_dir_element 	  turn_dir_state;

		void initialize()
		{
			velo.init(); accel.init(); horizon_accel.init(); horizon_velo.init();
			z_accel.init(); length.init(); rad_accel.init(); rad_jerk.init();
			rad_velo.init(); radian.init(); x_point.init();
			turn_slip_theta.init(); turn_dir_state.init();
		}
		void integral_init()
		{
			length.init(); radian.init(); x_point.init();
			turn_slip_theta.init();
		}
};

class Vehicle
{
	protected:
		int r_duty = 0,l_duty = 0;
		int suction_duty = 0;
		t_bool suction_flag = False;
	public:

		machine_params ego;   // 実測・制御用の区間状態。
		machine_params ideal; // 目標状態。egoと同じ項目・単位。
		// 機体が所有するローカル/全体座標と変換原点。pose.snapshot()で同時に参照する。
		// egoの区間積分リセットとは独立し、走行開始/終了時に明示的にstart/stopする。
		PoseFrames::Tracker pose;

		// 電源・吸引と機体モデル。
		param_element battery;
		param_element V_suction;
		int suction_out = 0;

		// 制御器、モータ出力、FF/FBの内訳。
		mouse_Controller Vehicle_controller;
		float V_r = 0.0f,V_l = 0.0f;
		int motor_out_r = 0,motor_out_l = 0;
		param_element sp_feedforward,sp_feedback;
		param_element om_feedforward,om_feedback;

		void ego_initialize()
		{
			ego.initialize();
			reset_turn_position_for_log();
		}

		inline void ego_integral_init()
		{
			ego.integral_init();
			reset_turn_position_for_log();
		}

		// 旧turn_x/y列の参照原点だけを変更する。位置はposeから算出し、積分しない。
		void reset_turn_position_for_log()
		{
			pose.reset_log_origin(ego.radian.get());
		}

		void ideal_initialize()
		{
			ideal.initialize();
		}

		inline void ideal_integral_init()
		{
			ideal.integral_init();
		}
		virtual void motorSetDuty_l(int out_l)
		{
			l_duty = out_l;
		}
		virtual void motorSetDuty_r(int out_r)
		{
			r_duty = out_r;
		}
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
			suction_out = out_s;
		}

		// 係数を状態として重複保持しない。直前のファン出力で機体別の基本値を選ぶ。
		float turn_slip_k_m_s2() const
		{
			return TurnPose::slip_k_m_s2(suction_duty > 0);
		}

		virtual void suctionStop()
		{
			suction_flag = False;
			suction_duty = 0;
			suction_out = 0;
			FAN_Motor_SetDuty(suction_duty);
		}

		virtual ~Vehicle()
		{
		    motorSetDuty_l(0);
		    motorSetDuty_r(0);
		    suctionStop();
		}
};

class Vehicle_type8:public Vehicle,public Singleton<Vehicle_type8>{


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
		Vehicle::suctionSetDuty(out_s);
		FAN_Motor_SetDuty(suction_duty);
	}


};

#endif /* CPP_MODULE_INC_VEHICLE_H_ */
