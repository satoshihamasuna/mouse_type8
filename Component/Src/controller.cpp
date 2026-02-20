/*
 * controller.cpp
 *
 *  Created on: Jul 20, 2025
 *      Author: sato1
 */




#include "controller.h"
#include "math_utils.h"

#define LOW_PASS_ALPHA 0.045

void PID_Controller::Gain_Set(float _Kp,float _Ki,float _Kd)
{
    if (Ki != 0.0f && _Ki != 0.0f) {
    	I_target  *= Ki / _Ki;
    	I_output  *= Ki / _Ki;
    }
    else {
    	//I_target  = 0.0f;
    	//I_output  = 0.0f;
    }

	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
}


void PID_Controller::Gain_Set(t_pid_gain gain)
{
    if (Ki != 0.0f && gain.Ki != 0.0f) {
    	I_target  *= Ki / gain.Ki;
    	I_output  *= Ki / gain.Ki;
    }
    else {
    	//I_target  = 0.0f;
    	//I_output  = 0.0f;
    }

	Kp = gain.Kp;
	Ki = gain.Ki;
	Kd = gain.Kd;
}


void PID_Controller::I_param_reset()
{
	I_target = I_output = 0.0;
}

/*
float PID_Controller::Control(float _target,float _output,float dt)
{
	float operation = 0.0;
	target = _target;
	output = _output;
	operation = Kp*(target - output) + Ki*(I_target - I_output) + Kd*(prev_target - prev_output)/dt;
	prev_target = 	LOW_PASS_ALPHA*target + (1-LOW_PASS_ALPHA)*prev_target;
	prev_output = 	LOW_PASS_ALPHA*output + (1-LOW_PASS_ALPHA)*prev_output;
    if (Ki != 0.0f ) {
    	I_target 	+= 	target;	I_output		+= 	output;
    }

	return operation;
}
*/

float PID_Controller::Control(float _target,float _output,float _dt)
{
    float operation = 0.0f;

    dt = _dt;
    target = _target;
    output = _output;

    operation = Kp*(target - output)
              + Ki*(I_target - I_output)
              + Kd*(prev_target - prev_output)/dt;

    prev_target = LOW_PASS_ALPHA*target + (1-LOW_PASS_ALPHA)*prev_target;
    prev_output = LOW_PASS_ALPHA*output + (1-LOW_PASS_ALPHA)*prev_output;

    if (Ki != 0.0f) {
        I_target += target;
        I_output += output;
    }

    return operation;
}

float PID_Controller::Anti_windup_1(float operation,float limit)
{
	if(ABS(operation) > limit)
	{
		I_target = I_target - target;
		I_output = I_output - output;
		return SIGN(operation) * limit;
	}
	return operation;
}

float PID_Controller::Anti_windup_2(float operation,float limit)
{
	if(ABS(operation) > limit)
	{
		float diff = operation - SIGN(operation) * limit;
		if(Kp != 0.0f)
		{
			I_target = I_target - 1/Kp*diff/2.0;
			I_output = I_output + 1/Kp*diff/2.0;
		}
		return SIGN(operation) * limit;
	}
	return operation;
}
