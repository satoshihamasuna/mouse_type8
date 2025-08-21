/*
 * path_follow.cpp
 *
 *  Created on: Aug 19, 2025
 *      Author: sato1
 */


#include "path_follow.h"

float path_follow_class::calc_control_yaw_rate(float ideal_velo,float vehicle_velo,float ideal_theta,float vehicle_theta)
{

	//float position_error  = ideal_velo*tmp_sin_yaw_err*m_dt;
	float yaw_theta_error = vehicle_theta - ideal_theta;

	controll_yaw = - k_yaw*yaw_theta_error;//tmp_sin_yaw_err;
	return controll_yaw;
}
