/*
 * controll.h
 *
 *  Created on: 2023/06/14
 *      Author: sato1
 */

#ifndef CPP_INC_CONTROLL_H_
#define CPP_INC_CONTROLL_H_

#include "typedef.h"

typedef struct{
	float Kp;
	float Ki;
	float Kd;
}t_pid_gain;


class PID_Controller
{
	private:
		//float Kp,Ki,Kd;
		t_bool enable_integral = True;
	public:
		float Kp = 0.0,Ki = 0.0,Kd = 0.0;
		float target= 0.0,I_target= 0.0,prev_target= 0.0;
		float output= 0.0,I_output= 0.0,prev_output= 0.0;
		float dt;
		void Gain_Set(float _Kp,float _Ki,float _Kd);
		void Gain_Set(t_pid_gain gain);
		void I_param_reset();
		float Control(float _target,float _output,float _dt);
		float Anti_windup_1(float operation,float limit);
		float Anti_windup_2(float operation,float limit);

		void Enable_Integral()	{		enable_integral = True;		}
		void Disable_Integral()	{		enable_integral = False;		}

		float get_P_peration()
		{
			return  Kp*(target - output);
		}

		float get_I_peration()
		{
			return  Ki*(I_target - I_output);
		}

		float get_D_peration()
		{
			return  Kd*(I_target - I_output)/dt;
		}
		//PID_Controller()I
};


class mouse_Controller
{
	public:
		//template <class T> T *runtask;
		PID_Controller omega_ctrl;
		PID_Controller speed_ctrl;
		PID_Controller Ir_ctrl;
};


#endif /* CPP_INC_CONTROLL_H_ */
