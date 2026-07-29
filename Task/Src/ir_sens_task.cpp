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


#include "sensing_task.h"
#include "sens_table.h"
#include "typedef.h"
#include "maze_typedef.h"
#include "peripheral.h"
#include <math.h>
/*
#ifdef MOUSE_A
  #include "sens_table_A.h"
#elif defined(MOUSE_B)
  #include "sens_table_B.h"
#else
  #error "No SENSOR_TYPE defined. Please define SENSOR_TYPE_A or SENSOR_TYPE_B."
#endif
*/
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

void IrSensTask::IrSensorSet(Vehicle *vehicle)
{
	sen_fl.value =  100;//Sensor_GetValue(sensor_fl);
	sen_fr.value =  100;//Sensor_GetValue(sensor_fr);
	sen_l.value  =  100;//Sensor_GetValue(sensor_sl);
	sen_r.value  =  100;//Sensor_GetValue(sensor_sr);
	IrSensorDistanceSet(vehicle);
	IrSensorWallSet(vehicle);
}

float IrSensTask::IrSensor_adc2voltage(int16_t value)
{
	return (float)(value)/4096.0*3.3;
}


float IrSensTask::Sensor_CalcDistance(t_sensor_dir dir, int16_t value)
{
    const int16_t *sens_table = nullptr;
    int array_length = 0;
    float d_start = 0.0f;
    float min_d = 0.0f, max_d = 0.0f;

    constexpr float d_step = 5.0f;

    switch (dir)
    {
        case sensor_fr:
            sens_table   = sens_fr_table;
            array_length = sizeof(sens_fr_table) / sizeof(int16_t);
            d_start = 40.0f;
            min_d   = 40.0f;
            max_d   = 125.0f;
            break;

        case sensor_fl:
            sens_table   = sens_fl_table;
            array_length = sizeof(sens_fl_table) / sizeof(int16_t);
            d_start = 40.0f;
            min_d   = 40.0f;
            max_d   = 125.0f;
            break;

        case sensor_sr:
            sens_table   = sens_sr_table;
            array_length = sizeof(sens_sr_table) / sizeof(int16_t);
            d_start = 25.0f;
            min_d   = 25.0f;
            max_d   = 80.0f;
            break;

        case sensor_sl:
            sens_table   = sens_sl_table;
            array_length = sizeof(sens_sl_table) / sizeof(int16_t);
            d_start = 25.0f;
            min_d   = 25.0f;
            max_d   = 80.0f;
            break;

        default:
            return 0.0f;
    }

    float distance = tableToDistanceStep(
        sens_table,
        array_length,
        d_start,
        d_step,
        value
    );

    return clampf(distance, min_d, max_d);
}

void IrSensTask::IrSensorDistanceSet(Vehicle *vehicle)
{
	static int i = 0;
	i = i + 1;
	if(i == 20) i = 0;
	ir_log_cnt = i;
	sen_fl.distance_sum = sen_fl.distance_sum - sen_fl.distance_log[i%20];
	sen_fr.distance_sum = sen_fr.distance_sum - sen_fr.distance_log[i%20];
	sen_r.distance_sum = sen_r.distance_sum - sen_r.distance_log[i%20];
	sen_l.distance_sum = sen_l.distance_sum - sen_l.distance_log[i%20];

	sen_fl.distance = Sensor_CalcDistance(sensor_fl,sen_fl.value);
	sen_fr.distance = Sensor_CalcDistance(sensor_fr,sen_fr.value);
	sen_l.distance = Sensor_CalcDistance(sensor_sl,sen_l.value);
	sen_r.distance = Sensor_CalcDistance(sensor_sr,sen_r.value);

	sen_fl.distance_log[i%20] = sen_fl.distance;
	sen_fr.distance_log[i%20] = sen_fr.distance;
	sen_r.distance_log[i%20]  = sen_r.distance;
	sen_l.distance_log[i%20]  = sen_l.distance;

	sen_fl.distance_sum = sen_fl.distance_sum + sen_fl.distance_log[i%20];
	sen_fr.distance_sum = sen_fr.distance_sum + sen_fr.distance_log[i%20];
	sen_r.distance_sum = sen_r.distance_sum + sen_r.distance_log[i%20];
	sen_l.distance_sum = sen_l.distance_sum + sen_l.distance_log[i%20];


	sen_fl.avg_distance = (sen_fl.distance_sum/20.0);
	sen_fr.avg_distance = (sen_fr.distance_sum/20.0);
	sen_l.avg_distance = (sen_l.distance_sum/20.0);
	sen_r.avg_distance = (sen_r.distance_sum/20.0);

	sen_fl.prev_diff = sen_fl.diff;
	sen_fr.prev_diff = sen_fr.diff;
	sen_l.prev_diff = sen_l.diff;
	sen_r.prev_diff = sen_r.diff;

	sen_fl.diff = sen_fl.distance - sen_fl.avg_distance;
	sen_fr.diff = sen_fr.distance - sen_fr.avg_distance;
	sen_l.diff = sen_l.distance - sen_l.avg_distance;
	sen_r.diff = sen_r.distance - sen_r.avg_distance;

}

void IrSensTask::IrSensorWallSet(Vehicle *vehicle)
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
		if(irsens_motion == STRAIGHT_IRSENS)
		{
			r_wall_corner = True;
			r_corner_time = 0;
		}
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
	else if( (sen_r.diff > 0.0) && (sen_r.prev_diff < 0.0))
	{

		if(irsens_motion == DIAGONAL_IRSENS && sen_r.distance < CORNER_R_THRESHOLD	)
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
		if(irsens_motion == STRAIGHT_IRSENS)
		{
			l_wall_corner = True;
			l_corner_time = 0;
		}
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
	else if( (sen_l.diff > 0.0) && (sen_l.prev_diff < 0.0))
	{

		if(irsens_motion == DIAGONAL_IRSENS && sen_l.distance < CORNER_L_THRESHOLD	)
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
	sen_fr.control_th = (sen_fr.control_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	sen_fl.control_th = (sen_fl.control_cnt > 10) ? FRONT_THRESHOLD : 90.0;

	sen_r.control_cnt = (sen_r.is_wall == True ) ? sen_r.control_cnt + 1 : 0;
	sen_l.control_cnt = (sen_l.is_wall == True ) ? sen_l.control_cnt + 1 : 0;

	sen_r.control_cnt = (ABS(sen_r.diff) < 0.5) ? sen_r.control_cnt:0 ;
	sen_l.control_cnt = (ABS(sen_l.diff) < 0.5) ? sen_l.control_cnt:0 ;


	sen_r.control_cnt = (r_wall_corner == True ) ? 0 : sen_r.control_cnt;
	sen_l.control_cnt = (l_wall_corner == True ) ? 0 : sen_l.control_cnt;


	/*追記文*/
	sen_r.control_cnt 	= (sen_fr.distance <= SIDECONTROL_FR_THRESHOLD) ? 0 : sen_r.control_cnt;
	sen_l.control_cnt 	= (sen_fl.distance <= SIDECONTROL_FL_THRESHOLD) ? 0 : sen_l.control_cnt;

	sen_r.control_distance =  (sen_r.is_wall == True)? sen_r.control_distance + vehicle->ego.velo.get():0.0f;
	sen_l.control_distance =  (sen_l.is_wall == True)? sen_l.control_distance + vehicle->ego.velo.get():0.0f;

	sen_r.control_distance = (ABS(sen_r.diff) < 0.5) ? sen_r.control_distance:0.0;
	sen_l.control_distance = (ABS(sen_l.diff) < 0.5) ? sen_l.control_distance:0.0;

	sen_r.control_distance = (sen_fr.distance <= SIDECONTROL_FR_THRESHOLD) ? 0.0f : sen_r.control_distance;
	sen_l.control_distance = (sen_fl.distance <= SIDECONTROL_FL_THRESHOLD) ? 0.0f : sen_l.control_distance;

	//need to update
	if(isEnableIrSens == True)
	{

		if(irsens_motion == STRAIGHT_IRSENS || irsens_motion == DIAGONAL_IRSENS)
		{
			sen_r.control_th = (sen_r.control_cnt > 10) ? SIDE_THRESHOLD: wall_ref;
			sen_l.control_th = (sen_l.control_cnt > 10) ? SIDE_THRESHOLD: wall_ref;

			if(sen_r.control_th == SIDE_THRESHOLD)
			sen_r.control_th = (sen_r.control_distance > 10.0) ? SIDE_THRESHOLD: wall_ref;
			if(sen_l.control_th == SIDE_THRESHOLD)
			sen_l.control_th = (sen_l.control_distance > 10.0) ? SIDE_THRESHOLD: wall_ref;

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
			sen_r.is_control 	= (sen_fr.distance <= SIDECONTROL_FR_THRESHOLD)? False:sen_r.is_control;
			sen_l.is_control 	= (sen_fl.distance <= SIDECONTROL_FL_THRESHOLD)? False:sen_l.is_control;

		}

		sen_r.error	= (sen_r.is_control == True) ? sen_r.distance - wall_ref : 0.0;
		sen_l.error	= (sen_l.is_control == True) ? sen_l.distance - wall_ref : 0.0;



		if(irsens_motion == STRAIGHT_IRSENS)
		{
			//sen_r.error	= sen_r.error;
			//sen_l.error	= sen_l.error;

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
	float value;
	switch(dir)
	{
		case sensor_fl:
			value = sen_fl.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_fl.distance_log[i] >value)
				{
					value = sen_fl.distance_log[i];
				}
			}
			break;
		case sensor_fr:
			value = sen_fr.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_fr.distance_log[i] >value)
				{
					value = sen_fr.distance_log[i];
				}
			}
			break;
		case sensor_sr:
			value = sen_r.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_r.distance_log[i] >value)
				{
					value = sen_r.distance_log[i];
				}
			}
			break;
		case sensor_sl:
			value = sen_l.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_l.distance_log[i] >value)
				{
					value = sen_l.distance_log[i];
				}
			}
			break;
	}
	return value;
}

void IrSensTask::SetWallControl_RadVelo(Vehicle *vehicle,float delta_tms)
{
	float ir_rad_acc_control = 0.0;
	const float k1 = 1.0;
	const float k2 = 19.0;
	control_ir.init();
	control_ir_dot.init();
	float ir_xposition = vehicle->ego.x_point.get();
	float deviation_rad = 0.0f;
	float gain = CLAMP(vehicle->ego.velo.get()/0.320f,1.0,20.0);

	//sensor_output = k1*ydiff/1000.0 + k2/1000.0*theta;
	if(isEnableIrSens == True && vehicle->ideal.velo.get() > 0.20)
	{
		if(sen_r.is_control == True && sen_l.is_control == True)
		{
			ir_xposition =  -(sen_l.error - sen_r.error)/2.0;
			if(irsens_motion == STRAIGHT_IRSENS)
			{
				vehicle->ego.x_point.set(ir_xposition);
			}

		}
		else if(sen_r.is_control == True || sen_l.is_control == True)
		{
			ir_xposition =  -(sen_l.error - sen_r.error);
			if(irsens_motion == STRAIGHT_IRSENS)
			{
				//vehicle->ego.x_point.set((ir_xposition+vehicle->ego.x_point.get())/2.0);
			}

		}
	}



	if(irsens_motion == STRAIGHT_IRSENS){
		//deviation_rad = vehicle->ego.radian.get();
	}
	else if(irsens_motion == DIAGONAL_IRSENS)
	{
		deviation_rad =  vehicle->ideal.radian.get();
	}

	ir_rad_acc_control = ir_xposition + k2*deviation_rad*0.0;

	control_ir.set(ir_rad_acc_control );
	//s_dot 	= k1*vehicle->ideal.velo.get()*1000.0*(vehicle->ideal.radian.get())*1.0 + k2*vehicle->ideal.rad_velo.get();
	control_ir_dot.set( k1*vehicle->ideal.velo.get()*1000.0*(deviation_rad)*(1.0) + k2*vehicle->ideal.rad_velo.get());


	float target_rad_acc	= 	(-1.0)*(	300.0*gain*k1/k2*control_ir.get()
										+ 	60.0*gain*1.0/k2*control_ir_dot.get()
										+ 	k1/k2*(vehicle->ideal.accel.get()*1000.0*deviation_rad*0.0 + vehicle->ideal.velo.get()*vehicle->ideal.rad_velo.get()*1000.0));



	float target_rad_velo	= vehicle->ideal.rad_velo.get() + target_rad_acc*delta_tms/1000.0f;
	vehicle->ideal.rad_accel.set(target_rad_acc);
	vehicle->ideal.rad_velo.set(target_rad_velo);

}



int16_t IrSensTask::IrSensor_Avg()
{
	return (sen_l.value + sen_r.value)/2 ;
}


