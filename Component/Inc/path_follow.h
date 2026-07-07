/*
 * path_follow.h
 *
 *  Created on: Aug 19, 2025
 *      Author: sato1
 */

#ifndef INC_PATH_FOLLOW_H_
#define INC_PATH_FOLLOW_H_


#include <stdio.h>
#include "../../Component/Inc/singleton.h"

class path_follow_class:public Singleton<path_follow_class>
{
	private:
		float yaw_theta_error;
		float position_error;
		float virtual_yaw_rate;
		float virtual_velo;
		float controll_yaw;
		float m_dt = 1.0f;
		float k_yaw = 0.0f;
		float k_pos = 0.0f;
		float return_yaw_theta_error();
		float return_position_error();
	public:
		//path_follow_class(float _m_dt = 1.0f,float _k_yaw = 0.0f,float _k_pos = 0.0f);
		float calc_control_yaw_rate(float ideal_velo,float vehicle_velo,float ideal_theta,float vehicle_theta);
		float return_controll_yaw()		{return controll_yaw;}
		void reset_yaw_theta_error() 	{yaw_theta_error = 0.0f;}
		void reset_position_error() 	{position_error = 0.0f;}
		void set_path_follow_gain(float _k_yaw,float _k_pos)
		{
			k_yaw = _k_yaw;
			k_pos = _k_pos;
		}
};


#endif /* INC_PATH_FOLLOW_H_ */
