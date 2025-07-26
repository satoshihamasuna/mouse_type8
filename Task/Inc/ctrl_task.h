/*
 * ctrl.h
 *
 *  Created on: 2024/03/15
 *      Author: sato1
 */

#ifndef CPP_TASK_INC_CTRL_TASK_H_
#define CPP_TASK_INC_CTRL_TASK_H_

#include <stdio.h>
#include "sensing_task.h"

#include "../../Params/run_param.h"

#include "controller.h"

#include "../../Module/Inc/vehicle.h"
#include "run_typedef.h"

#define SUCTION_ACC (0.008)

struct motion_plan_params
{
		param_element velo;
		param_element max_velo;
		param_element end_velo;
		param_element accel;
		param_element deccel;
		param_element end_length;
		param_element length_accel;
		param_element length_deccel;
		param_element rad_accel;
		param_element rad_deccel;
		param_element rad_max_velo;
		param_element end_radian;
		param_element radian_accel;
		param_element radian_deccel;
		param_element turn_r_min;
		param_element fix_post_run;
		param_element fix_prev_run;
		param_element suction_value;
		turn_dir_element turn_state;
		param_element turn_time_ms;
};

typedef enum{
	execute	    = 2,
	complete    = 1,
	error 		= 0,
}t_exeStatus;

typedef enum
{
	NOP_STATE = 0,
	STRAIGHT_STATE  = 1,
	DIAGONAL_STATE  = 2,
	SLATURN_STATE	= 3,
	PIVTURN_STATE	= 4,
	BRAKE_STATE		= 5,
}t_runStatus;

typedef struct{
	t_turn_param_table * param;
	t_pid_gain * sp_gain;
	t_pid_gain * om_gain;
}t_turn_motion_param;

class Motion
{
	private:
		//define motion status
		t_run_pattern		motion_pattern  = No_run;
		t_runStatus			motion_state	= NOP_STATE;
		t_exeStatus		    motion_exeStatus= complete;
		t_bool				is_motion_enable	= False;

		//define motion parameter
	    t_param			turn_motion_param;
		t_straight_param 	straight_motion_param;
		motion_plan_params    motion_plan;

		//define error counter
		int error_cnt = 0;

		//define ir sensor & vehicle instance
		Vehicle *vehicle;
		IrSensTask *ir_sens;


		//float delta_t;
		int deltaT_ms;
		float run_time_ms;
		float run_time_ms_limit;

		void Motion_EndSetting_turn();
		void Motion_EndSetting_straight();

	protected:
		//update Ideal parameters
		void  SetIdeal_search_straight();
		void  SetIdeal_search_turn();

		void SetIdeal_straight		();
		void SetIdeal_diagonal		();
		void SetIdeal_backward		();

		void SetIdeal_pivot_turn	();

		void SetIdeal_turn_in		();
		void SetIdeal_turn_out		();
		void SetIdeal_long_turn		();
		void SetIdeal_turn_v90		();
		void SetIdeal_long_turn_v90	();

		void SetIdeal_fix_wall		();
		void SetIdeal_suction_start ();
		void SetIdeal_stop_brake	();

		void SetIdeal_free_rotation_set	( );

		void SetIdeal_wall_control  ();

		void Adjust_wall_corner();


		//set & get status
		inline void run_time_ms_reset()											{	run_time_ms = 0.0f;					}
		inline void run_time_ms_update()										{	run_time_ms = run_time_ms + (float) deltaT_ms;	}
		inline float run_time_ms_get()											{	return run_time_ms;					}
		inline void run_time_ms_set(float time_ms)								{	run_time_ms = time_ms;				}

		inline void run_time_limit_ms_reset()									{	run_time_ms_limit = 0.0f;					}
		inline void run_time_limit_ms_set(float run_time_ms_limit_)				{	run_time_ms_limit = run_time_ms_limit_;		}
		inline float run_time_limit_ms_get()									{	return run_time_ms_limit;					}

		//inline t_run_pattern motion_pattern_get() 							 	{	return motion_pattern;				}
		inline void			 motion_pattern_set(t_run_pattern _motion_pattern)  {	motion_pattern	= _motion_pattern;	}

		//inline t_runStatus 	 motion_state_get() 							 	{	return motion_state;				}
		inline void			 motion_state_set(t_runStatus _motion_state)   	 	{	motion_state	= _motion_state;	}

		//inline t_exeStatus 	 motion_exeStatus_get() 							 	{	return motion_exeStatus;				}
		inline void			 motion_exeStatus_set(t_exeStatus _motion_exeStatus)   	{	motion_exeStatus	= _motion_exeStatus;	}
		t_exeStatus 		 motion_execute();

		inline	void 		motion_enable_set()										{ 	is_motion_enable		= True;		}
		inline	void 		motion_disable_set()									{ 	is_motion_enable		= False;	}
		//inline  t_bool 		motion_is_enable_get()									{	return	is_motion_enable;			}

		void Motion_error_handling();

		void error_counter_reset()												{	error_cnt = 0;							}
		void error_counter_set	(int cnt)										{	error_cnt = cnt;						}
		int  error_counter_get	()												{	return error_cnt;						}


	public:

		Motion(Vehicle *_vehicle,IrSensTask *_ir_sens,int _deltaT_ms)
		{
			vehicle = _vehicle;
			ir_sens = _ir_sens;
			deltaT_ms = _deltaT_ms;
		}

		inline t_run_pattern motion_pattern_get() 							 		{	return motion_pattern;				}
		inline t_runStatus 	 motion_state_get() 							 		{	return motion_state;				}
		inline t_exeStatus 	 motion_exeStatus_get() 							 	{	return motion_exeStatus;				}
		inline  t_bool 		motion_is_enable_get()									{	return	is_motion_enable;			}

		void Motion_start();
		void Motion_end();


		//Initialize motion parameters
		void Init_Motion_free_rotation_set	();
		void Init_Motion_search_straight(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_search_turn	(const t_param *turn_param,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_straight		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_diagonal		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_backward		(const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_pivot_turn		(float rad_target,float rad_acc,float rad_velo,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_long_turn		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_long_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		void Init_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain );
		void Init_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain );
		void Init_Motion_long_turn		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain );
		void Init_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain );
		void Init_Motion_long_turn_v90	(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain );


		void Init_Motion_fix_wall		(float set_time,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_suction_start	(float suction_voltage,float set_time,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);
		void Init_Motion_stop_brake		(float set_time,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain);

		inline t_exeStatus execute_Motion()
		{
			while(motion_exeStatus_get() == execute);
			return motion_exeStatus_get();
		}

		inline t_exeStatus exe_Motion_search_turn	(const t_param *turn_param,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_search_turn	(turn_param,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_straight		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_straight(len_target,acc,max_sp,end_sp,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_backward		(const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_backward(sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_diagonal		(float len_target,float acc,float max_sp,float end_sp,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_diagonal(len_target,acc,max_sp,end_sp,sp_gain,om_gain);
			return execute_Motion();
		}



		inline t_exeStatus exe_Motion_pivot_turn		(float rad_target,float rad_acc,float rad_velo,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_pivot_turn		(rad_target,rad_acc,rad_velo,sp_gain,om_gain);
			return execute_Motion();
		}


		inline t_exeStatus exe_Motion_fix_wall		(float set_time,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_fix_wall		(set_time,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_suction_start	(float suction_voltage,float set_time,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_suction_start		(suction_voltage,set_time,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_turn_in			(turn_param,run_pt,sp_gain,om_gain);
			return execute_Motion();
		}
		inline t_exeStatus exe_Motion_turn_in		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain )
		{
			Init_Motion_turn_in			(turn_param,run_pt,end_velo,acc,sp_gain,om_gain);
			return execute_Motion();
		}


		inline t_exeStatus exe_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_turn_out		(turn_param,run_pt,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_turn_out		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain )
		{
			Init_Motion_turn_out		(turn_param,run_pt,end_velo,acc,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_long_turn		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_long_turn		(turn_param,run_pt,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_long_turn		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain )
		{
			Init_Motion_long_turn		(turn_param,run_pt,end_velo,acc,sp_gain,om_gain);
			return execute_Motion();
		}


		inline t_exeStatus exe_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain,const t_pid_gain *om_gain = &basic_om_gain)
		{
			Init_Motion_turn_v90		(turn_param,run_pt,sp_gain,om_gain);
			return execute_Motion();
		}

		inline t_exeStatus exe_Motion_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain )
		{
				Init_Motion_turn_v90		(turn_param,run_pt,end_velo,acc,sp_gain,om_gain);
				return execute_Motion();
		}


		inline t_exeStatus exe_Motion_long_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain )
		{
			Init_Motion_long_turn_v90		(turn_param,run_pt,sp_gain,om_gain);
			return execute_Motion();
		}


		inline t_exeStatus exe_Motion_long_turn_v90		(const t_param *turn_param,t_run_pattern run_pt,float end_velo, float acc,const t_pid_gain *sp_gain = &basic_sp_gain ,const t_pid_gain *om_gain = &basic_om_gain )
		{
			Init_Motion_long_turn_v90		(turn_param,run_pt,end_velo,acc,sp_gain,om_gain);
			return execute_Motion();
		}


		Vehicle *return_vehicleObj() 	{return vehicle;};
		IrSensTask *return_irObj() 		{return ir_sens;};

};


class CtrlTask:public Motion
{
	private:
		void motion_ideal_param_set();
		Vehicle *vehicle;
		IrSensTask *ir_sens;
		int ctr_deltaT_ms;
	public:
		CtrlTask(Vehicle *_vehicle,IrSensTask *_ir_sens,int _ctr_deltaT_ms):Motion(_vehicle,_ir_sens,_ctr_deltaT_ms)
		{
			vehicle = _vehicle;
			ir_sens = _ir_sens;
			ctr_deltaT_ms = _ctr_deltaT_ms;
		}

		void motion_prev_control();
		void motion_control();
		void motion_post_control();
		inline t_bool is_control_enable()	{	return	motion_is_enable_get();	}


};


class CtrlTask_type7:public CtrlTask,public Singleton<CtrlTask_type7>{
	public:
		CtrlTask_type7(Vehicle *v = &Vehicle_type7::getInstance(),IrSensTask *ir = &IrSensTask_type7::getInstance()):CtrlTask(v,ir,1){}
};

#endif /* CPP_TASK_INC_CTRL_TASK_H_ */
