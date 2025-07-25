/*
 * ir_sens_task.h
 *
 *  Created on: 2024/03/21
 *      Author: sato1
 */


/*
 * sensing_task.cpp
 *
 *  Created on: 2023/06/14
 *      Author: sato1
 */


#include "../Inc/sensing_task.h"
#include "../../Params/sens_table.h"
#include "typedef.h"
#include "maze_typedef.h"
#include "peripheral.h"
#include <math.h>


t_wall_state IrSensTask::conv_Sensin2Wall(t_sensor_dir sens_dir)
{
	switch(sens_dir){
		case sensor_fl:
			return ((sen_fl.is_wall)?WALL:NOWALL);
		case sensor_fr:
			return ((sen_fr.is_wall)?WALL:NOWALL);
		case sensor_sl:
			return ((sen_l.is_wall)?WALL:NOWALL);
		case sensor_sr:
			return ((sen_r.is_wall)?WALL:NOWALL);
		default :
			return NOWALL;
	}
}

void IrSensTask::IrSensorSet()
{
	sen_fl.value =  100;//Sensor_GetValue(sensor_fl);
	sen_fr.value =  100;//Sensor_GetValue(sensor_fr);
	sen_l.value  =  100;//Sensor_GetValue(sensor_sl);
	sen_r.value  =  100;//Sensor_GetValue(sensor_sr);
	IrSensorDistanceSet();
	IrSensorWallSet();
}

float IrSensTask::IrSensor_adc2voltage(int16_t value)
{
	return (float)(value)/4096.0*3.3;
}

float IrSensTask::IrSensor_Vce(int16_t value)
{
	return MAX(0.01,(3.30f - IrSensor_adc2voltage(value)));
}

float IrSensTask::IrSensor_SensingCurrent(int16_t value)
{
	return (IrSensor_adc2voltage(value))/1000.0;
}

float IrSensTask::IrSensor_RelativeCurrent(int16_t value)
{
	return (IrSensor_SensingCurrent(value))/((6.4)/5*IrSensor_Vce(value))*1000.0f;
}

float IrSensTask::IrSensor_Irradiance(int16_t value)
{
	float irradiance = IrSensor_RelativeCurrent(value);
	if(irradiance > 4.0)
	{
		irradiance = irradiance * 2 -4.0f;
	}
	return irradiance;
}

float IrSensTask::Sensor_CalcDistance(t_sensor_dir dir,int16_t value)
{
	float distance = 0.0f;
	int array_length = 0;
	int count = 0;
	float m,n;
	switch(dir)
	{
		case sensor_fr:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fr_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fr_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else{

				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fr_table[count] && value > sens_fr_table[count+1]) break;
				}
				m = (float)(sens_fr_table[count] - value);
				n = (float)(value - sens_fr_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);

				/*
				if (value < sens_fr_table[1]){
					float ln_distance = -0.36605*logf((float)(value)) + 6.629735;
					distance = expf(ln_distance);
				}
				else
				{
					m = (float)(sens_fr_table[0] - value);
					n = (float)(value - sens_fr_table[1]);
					distance = (n*(float)sens_front_length_table[0] + m*(float)sens_front_length_table[1])/(m+n);
				}
				*/
				if(distance < 40.0) distance = 40.0;
				else if(distance > 125.0) distance = 125.0;
			}
			break;
		case sensor_fl:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fl_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fl_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else
			{

				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fl_table[count] && value > sens_fl_table[count+1]) break;
				}
				m = (float)(sens_fl_table[count] - value);
				n = (float)(value - sens_fl_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);
				/*
				if (value < sens_fl_table[1]){
					float ln_distance = -0.34692*logf((float)(value)) + 6.39695;
					distance = expf(ln_distance);
				}
				else
				{
					m = (float)(sens_fl_table[0] - value);
					n = (float)(value - sens_fl_table[1]);
					distance = (n*(float)sens_front_length_table[0] + m*(float)sens_front_length_table[1])/(m+n);
				}

				*/
				if(distance < 40.0) distance = 40.0;
				else if(distance > 125.0) distance = 125.0;
			}
			break;
		case sensor_sr:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sr_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sr_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{


				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sr_table[count] && value > sens_sr_table[count+1]) break;
				}
				m = (float)(sens_sr_table[count] - value);
				n = (float)(value - sens_sr_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);

				//float ln_value = logf((float) value);
				//distance = 1.574075*ln_value * ln_value -38.56 * ln_value + 233.3134;
				if(distance < 25.0) distance = 25.0;
				else if(distance > 80.0) distance = 80.0;
			}
			break;
		case sensor_sl:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sl_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sl_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{


				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sl_table[count] && value > sens_sl_table[count+1]) break;
				}
				m = (float)(sens_sl_table[count] - value);
				n = (float)(value - sens_sl_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);


				//float ln_value = logf((float) value);
				//distance = 1.5581*ln_value * ln_value -37.400 * ln_value + 226.7995;
				if(distance < 25.0) distance = 25.0;
				else if(distance > 80.0) distance = 80.0;
			}
			break;
	}
	return distance;
}


void IrSensTask::IrSensorDistanceSet()
{
	static int i = 0;
	i = i + 1;
	if(i == 20) i = 0;
	ir_log_cnt = i;
	sen_fl.value_sum = sen_fl.value_sum - sen_fl.value_log[i%20];
	sen_fr.value_sum = sen_fr.value_sum - sen_fr.value_log[i%20];
	sen_r.value_sum = sen_r.value_sum - sen_r.value_log[i%20];
	sen_l.value_sum = sen_l.value_sum - sen_l.value_log[i%20];

	sen_fl.value_log[i%20] = sen_fl.value;
	sen_fr.value_log[i%20] = sen_fr.value;
	sen_r.value_log[i%20]  = sen_r.value;
	sen_l.value_log[i%20]  = sen_l.value;

	sen_fl.value_sum = sen_fl.value_sum + sen_fl.value_log[i%20];
	sen_fr.value_sum = sen_fr.value_sum + sen_fr.value_log[i%20];
	sen_r.value_sum = sen_r.value_sum + sen_r.value_log[i%20];
	sen_l.value_sum = sen_l.value_sum + sen_l.value_log[i%20];

	sen_fl.distance = Sensor_CalcDistance(sensor_fl,sen_fl.value);
	sen_fr.distance = Sensor_CalcDistance(sensor_fr,sen_fr.value);
	sen_l.distance = Sensor_CalcDistance(sensor_sl,sen_l.value);
	sen_r.distance = Sensor_CalcDistance(sensor_sr,sen_r.value);

	sen_fl.avg_distance = Sensor_CalcDistance(sensor_fl,(int16_t)(sen_fl.value_sum/20));
	sen_fr.avg_distance = Sensor_CalcDistance(sensor_fr,(int16_t)(sen_fr.value_sum/20));
	sen_l.avg_distance = Sensor_CalcDistance(sensor_sl,(int16_t)(sen_l.value_sum/20));
	sen_r.avg_distance = Sensor_CalcDistance(sensor_sr,(int16_t)(sen_r.value_sum/20));

	sen_fl.prev_diff = sen_fl.diff;
	sen_fr.prev_diff = sen_fr.diff;
	sen_l.prev_diff = sen_l.diff;
	sen_r.prev_diff = sen_r.diff;

	sen_fl.diff = sen_fl.distance - sen_fl.avg_distance;
	sen_fr.diff = sen_fr.distance - sen_fr.avg_distance;
	sen_l.diff = sen_l.distance - sen_l.avg_distance;
	sen_r.diff = sen_r.distance - sen_r.avg_distance;

}

void IrSensTask::IrSensorWallSet()
{
	sen_fr.prev_is_wall 	= sen_fr.is_wall ;
	sen_fl.prev_is_wall 	= sen_fl.is_wall ;
	sen_r.prev_is_wall 	= sen_r.is_wall ;
	sen_l.prev_is_wall 	= sen_l.is_wall ;

	sen_fr.is_wall 	= (sen_fr.distance <= FRONT_THRESHOLD)? True:False;
	sen_fl.is_wall 	= (sen_fl.distance <= FRONT_THRESHOLD)? True:False;
	sen_r.is_wall 	= (sen_r.distance <= SIDE_THRESHOLD)? True:False;
	sen_l.is_wall  	= (sen_l.distance <= SIDE_THRESHOLD)? True:False;

	if(sen_r.is_wall == False && sen_r.prev_is_wall == True)
	{
		r_wall_corner = True;
		r_corner_time = 0;
	}
	else if(sen_r.is_wall == False && sen_r.prev_is_wall == True
		&& (sen_r.prev_diff) < 2.0 && (sen_r.diff) > 2.0)
	{
		if(irsens_motion == STRAIGHT_IRSENS)
		{
			r_wall_corner = True;
			r_corner_time = 0;
		}
	}
	else
	{
		r_wall_corner = False;
		r_corner_time++;
	}



	if(sen_l.is_wall == False && sen_l.prev_is_wall == True)
	{
		l_wall_corner = True;
		l_corner_time = 0;
	}
	else if(sen_l.is_wall == False && sen_l.prev_is_wall == True
		&& (sen_l.prev_diff) < 2.0 && (sen_l.diff) > 2.0)
	{
		if(irsens_motion == STRAIGHT_IRSENS)
		{
			l_wall_corner = True;
			l_corner_time = 0;
		}
	}
	else
	{
		l_wall_corner = False;
		l_corner_time++;
	}


	sen_fr.control_cnt = (sen_fr.is_wall == True) ? sen_fr.control_cnt + 1 : 0;
	sen_fl.control_cnt = (sen_fl.is_wall == True) ? sen_fl.control_cnt + 1 : 0;
	sen_r.control_cnt = (sen_r.is_wall == True && ABS(sen_r.distance - sen_r.avg_distance) < 0.5) ? sen_r.control_cnt + 1 : 0;
	sen_l.control_cnt = (sen_l.is_wall == True && ABS(sen_l.distance - sen_l.avg_distance) < 0.5) ? sen_l.control_cnt + 1 : 0;

	//sen_r.control_cnt = (sen_r.is_wall == True ) ? sen_r.control_cnt + 1 : 0;
	//sen_l.control_cnt = (sen_l.is_wall == True ) ? sen_l.control_cnt + 1 : 0;

	sen_fr.control_th = (sen_fr.control_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	sen_fl.control_th = (sen_fl.control_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	//need to update
	if(isEnableIrSens == True)
	{

		if(irsens_motion == STRAIGHT_IRSENS || irsens_motion == DIAGONAL_IRSENS)
		{
			sen_r.control_th = (sen_r.control_cnt > 10) ? SIDE_THRESHOLD: wall_ref;
			sen_l.control_th = (sen_l.control_cnt > 10) ? SIDE_THRESHOLD: wall_ref;
		}
		else
		{
			sen_r.control_th = (sen_r.control_cnt > 10) ? wall_ref: wall_ref;
			sen_l.control_th = (sen_l.control_cnt > 10) ? wall_ref: wall_ref;
		}

		sen_r.is_control 	= (sen_r.is_wall == True && sen_r.distance <= sen_r.control_th)? True:False;
		sen_l.is_control 	= (sen_l.is_wall == True && sen_l.distance <= sen_l.control_th)? True:False;

		if(irsens_motion == STRAIGHT_IRSENS || irsens_motion == DIAGONAL_IRSENS)
		{
			sen_r.is_control 	= (sen_fr.distance > SIDE_THRESHOLD+1.0)? sen_r.is_control:False;
			sen_l.is_control 	= (sen_fl.distance > SIDE_THRESHOLD+1.0)? sen_l.is_control:False;
		}
		else
		{
			sen_r.is_control 	= (sen_fr.distance <= 80.0)? False:sen_r.is_control;
			sen_l.is_control 	= (sen_fl.distance <= 80.0)? False:sen_l.is_control;
		}


		sen_r.error	= (sen_r.is_control == True) ? sen_r.distance - wall_ref : 0.0;
		sen_l.error	= (sen_l.is_control == True) ? sen_l.distance - wall_ref : 0.0;

		if(irsens_motion == STRAIGHT_IRSENS)
		{
			sen_r.error	= sen_r.error;
			sen_l.error	= sen_l.error;

		}
		if(irsens_motion == DIAGONAL_IRSENS)
		{
			sen_r.error	= (sen_r.error	<= 0.0f) ? sen_r.error : 0.0;
			sen_l.error	= (sen_l.error	<= 0.0f) ? sen_l.error : 0.0;
		}
	}
	else
	{

		sen_r.control_th = DIAGONAL_REF;
		sen_l.control_th = DIAGONAL_REF;

		sen_r.is_control 	= False;
		sen_l.is_control 	= False;

		sen_r.error	=  0.0;
		sen_l.error	=  0.0;
	}
}

void IrSensTask::IrSensorReferenceSet(float ref_value)
{
	 wall_ref = ref_value;
}

float IrSensTask::IrSensorMaxValueFromLog(t_sensor_dir dir)
{
	int16_t value;
	switch(dir)
	{
		case sensor_fl:
			value = sen_fl.value_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_fl.value_log[i] >value)
				{
					value = sen_fl.value_log[i];
				}
			}
			break;
		case sensor_fr:
			value = sen_fr.value_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_fr.value_log[i] >value)
				{
					value = sen_fr.value_log[i];
				}
			}
			break;
		case sensor_sr:
			value = sen_r.value_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_r.value_log[i] >value)
				{
					value = sen_r.value_log[i];
				}
			}
			break;
		case sensor_sl:
			value = sen_l.value_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_l.value_log[i] >value)
				{
					value = sen_l.value_log[i];
				}
			}
			break;
	}
	return Sensor_CalcDistance(dir,value);
}

void IrSensTask::SetWallControl_RadVelo(Vehicle *vehicle,float delta_tms)
{
	float ir_rad_acc_control = 0.0;
	const float k1 = 1.0;
	const float k2 = 20.0;
	float s 	= 0.0f;
	float s_dot = 0.0f;

	//sensor_output = k1*ydiff/1000.0 + k2/1000.0*theta;
	if(isEnableIrSens == True)
	{
		if(sen_r.is_control == True && sen_l.is_control == True)
		{
			ir_rad_acc_control = -(sen_l.error - sen_r.error)/2.0;
			{
				vehicle->ego.x_point.set(ir_rad_acc_control);
				//if(ABS(ir_rad_acc_control) < 10.0 )
					//vehicle->ego.radian.set(((-ir_rad_acc_control/20.0) + vehicle->ego.radian.get())/2.0f);
			}
		}
		else
		{
			ir_rad_acc_control = -(sen_l.error - sen_r.error);
			if(sen_r.is_control == True || sen_l.is_control == True)
			{
				vehicle->ego.x_point.set((ir_rad_acc_control+vehicle->ego.x_point.get())/2.0);
				//if(ABS(ir_rad_acc_control) < 10.0)
					//vehicle->ego.radian.set(((-ir_rad_acc_control/20.0) + vehicle->ego.radian.get())/2.0f);
			}
		}
	}

	if(isEnableIrSens == True && (sen_r.is_control == True || sen_l.is_control == True))
	{

		s 		= ir_rad_acc_control;
		s_dot 	= k1*vehicle->ideal.velo.get()*1000.0*(vehicle->ideal.radian.get())*1.0 + k2*vehicle->ideal.rad_velo.get();

	}

	else
	{

		s 		= k1*vehicle->ego.x_point.get()+k2*vehicle->ego.radian.get()*1.0;//k2*machine_->radian;//
		s_dot 	= k1*vehicle->ideal.velo.get()*1000.0*(vehicle->ideal.radian.get())*1.0 + k2*vehicle->ideal.rad_velo.get();


	}


	float target_rad_acc	= 	(-1.0)*300.0*s/k2 - 60.0*1.0/k2*s_dot
							     - k1/k2*(vehicle->ideal.accel.get()*1000.0*vehicle->ideal.radian.get()*0.0
							    		 + vehicle->ideal.velo.get()*vehicle->ideal.rad_velo.get()*1000.0);



	float target_rad_velo	= vehicle->ideal.rad_velo.get() + target_rad_acc*delta_tms/1000.0f;
	vehicle->ideal.rad_accel.set(target_rad_acc);
	vehicle->ideal.rad_velo.set(target_rad_velo);

}



int16_t IrSensTask::IrSensor_Avg()
{
	return (sen_l.value + sen_r.value)/2 ;
}


