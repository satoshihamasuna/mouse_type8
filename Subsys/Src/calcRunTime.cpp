/*
 * calcRunTime.cpp
 *
 *  Created on: 2023/07/09
 *      Author: sato1
 */


#include "../Inc/make_path.h"
#include "typedef.h"
#include "wall_class.h"
#include "run_typedef.h"
#include "turn_table.h"
#include "controller.h"
#define OFF_SET_LENGTH 10.0

void calcRunTime::turn_time_set(const t_param *const *mode)
{
	float omega_mx = 0.0f;

	omega_mx = mode[Long_turnL180]->param->velo/(mode[Long_turnL180]->param->r_min/1000.0);
	turn_Long180_time = (mode[Long_turnL180]->param->Lstart/mode[Long_turnL180]->param->velo);
	turn_Long180_time += (DEG2RAD(mode[Long_turnL180]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_Long180_time += (mode[Long_turnL180]->param->Lend/mode[Long_turnL180]->param->velo);

	omega_mx = mode[Long_turnL90]->param->velo/(mode[Long_turnL90]->param->r_min/1000.0);
	turn_Long90_time  = (uint16_t)(mode[Long_turnL90]->param->Lstart/mode[Long_turnL90]->param->velo);
	turn_Long90_time += (uint16_t)(DEG2RAD(mode[Long_turnL90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_Long90_time += (uint16_t)(mode[Long_turnL90]->param->Lend/mode[Long_turnL90]->param->velo);

	omega_mx = mode[Turn_LV90]->param->velo/(mode[Turn_LV90]->param->r_min/1000.0);
	turn_V90_time  = (mode[Turn_LV90]->param->Lstart/mode[Turn_LV90]->param->velo);
	turn_V90_time += (DEG2RAD(mode[Turn_LV90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_V90_time += (mode[Turn_LV90]->param->Lend/mode[Turn_LV90]->param->velo);

	omega_mx = mode[Turn_in_L45]->param->velo/(mode[Turn_in_L45]->param->r_min/1000.0);
	turn_in45_time  = (mode[Turn_in_L45]->param->Lstart/mode[Turn_in_L45]->param->velo);
	turn_in45_time += (DEG2RAD(mode[Turn_in_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_in45_time += (mode[Turn_in_L45]->param->Lend/mode[Turn_in_L45]->param->velo);

	omega_mx = mode[Turn_out_L45]->param->velo/(mode[Turn_out_L45]->param->r_min/1000.0);
	turn_out45_time  = (uint16_t)(mode[Turn_out_L45]->param->Lstart/mode[Turn_out_L45]->param->velo);
	turn_out45_time += (uint16_t)(DEG2RAD(mode[Turn_out_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_out45_time += (uint16_t)(mode[Turn_out_L45]->param->Lend/mode[Turn_out_L45]->param->velo);

	omega_mx = mode[Turn_in_L135]->param->velo/(mode[Turn_in_L135]->param->r_min/1000.0);
	turn_in135_time  = (uint16_t)(mode[Turn_in_L135]->param->Lstart/mode[Turn_in_L135]->param->velo);
	turn_in135_time += (uint16_t)(DEG2RAD(mode[Turn_in_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_in135_time += (uint16_t)(mode[Turn_in_L135]->param->Lend/mode[Turn_in_L135]->param->velo);

	omega_mx = mode[Turn_out_L135]->param->velo/(mode[Turn_out_L135]->param->r_min/1000.0);
	turn_out135_time  = (uint16_t)(mode[Turn_out_L135]->param->Lstart/mode[Turn_out_L135]->param->velo);
	turn_out135_time += (uint16_t)(DEG2RAD(mode[Turn_out_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_out135_time += (uint16_t)(mode[Turn_out_L135]->param->Lend/mode[Turn_out_L135]->param->velo);
}

void calcRunTime::turn_time_set(const t_param *const*const *mode  ,uint16_t mode_size)
{
	float omega_mx = 0.0f;
	for(int i = 0; i < mode_size;i++)
	{
		if(mode[i][Long_turnL180] !=NULL)
		{
			omega_mx = mode[i][Long_turnL180]->param->velo/(mode[i][Long_turnL180]->param->r_min/1000.0);
			turn_Long180_time = (mode[i][Long_turnL180]->param->Lstart/mode[i][Long_turnL180]->param->velo);
			turn_Long180_time += (DEG2RAD(mode[i][Long_turnL180]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_Long180_time += (mode[i][Long_turnL180]->param->Lend/mode[i][Long_turnL180]->param->velo);
		}

		if(mode[i][Long_turnL90] !=NULL)
		{
			omega_mx = mode[i][Long_turnL90]->param->velo/(mode[i][Long_turnL90]->param->r_min/1000.0);
			turn_Long90_time  = (uint16_t)(mode[i][Long_turnL90]->param->Lstart/mode[i][Long_turnL90]->param->velo);
			turn_Long90_time += (uint16_t)(DEG2RAD(mode[i][Long_turnL90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_Long90_time += (uint16_t)(mode[i][Long_turnL90]->param->Lend/mode[i][Long_turnL90]->param->velo);
		}

		if(mode[i][Turn_LV90] !=NULL)
		{
			omega_mx = mode[i][Turn_LV90]->param->velo/(mode[i][Turn_LV90]->param->r_min/1000.0);
			turn_V90_time  = (mode[i][Turn_LV90]->param->Lstart/mode[i][Turn_LV90]->param->velo);
			turn_V90_time += (DEG2RAD(mode[i][Turn_LV90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_V90_time += (mode[i][Turn_LV90]->param->Lend/mode[i][Turn_LV90]->param->velo);
		}

		if(mode[i][Turn_in_L45] !=NULL)
		{
			omega_mx = mode[i][Turn_in_L45]->param->velo/(mode[i][Turn_in_L45]->param->r_min/1000.0);
			turn_in45_time  = (mode[i][Turn_in_L45]->param->Lstart/mode[i][Turn_in_L45]->param->velo);
			turn_in45_time += (DEG2RAD(mode[i][Turn_in_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_in45_time += (mode[i][Turn_in_L45]->param->Lend/mode[i][Turn_in_L45]->param->velo);
		}

		if(mode[i][Turn_out_L45] !=NULL)
		{
			omega_mx = mode[i][Turn_out_L45]->param->velo/(mode[i][Turn_out_L45]->param->r_min/1000.0);
			turn_out45_time  = (uint16_t)(mode[i][Turn_out_L45]->param->Lstart/mode[i][Turn_out_L45]->param->velo);
			turn_out45_time += (uint16_t)(DEG2RAD(mode[i][Turn_out_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_out45_time += (uint16_t)(mode[i][Turn_out_L45]->param->Lend/mode[i][Turn_out_L45]->param->velo);
		}

		if(mode[i][Turn_in_L135] !=NULL)
		{
			omega_mx = mode[i][Turn_in_L135]->param->velo/(mode[i][Turn_in_L135]->param->r_min/1000.0);
			turn_in135_time  = (uint16_t)(mode[i][Turn_in_L135]->param->Lstart/mode[i][Turn_in_L135]->param->velo);
			turn_in135_time += (uint16_t)(DEG2RAD(mode[i][Turn_in_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_in135_time += (uint16_t)(mode[i][Turn_in_L135]->param->Lend/mode[i][Turn_in_L135]->param->velo);
		}

		if(mode[i][Turn_out_L135] !=NULL)
		{
			omega_mx = mode[i][Turn_out_L135]->param->velo/(mode[i][Turn_out_L135]->param->r_min/1000.0);
			turn_out135_time  = (uint16_t)(mode[i][Turn_out_L135]->param->Lstart/mode[i][Turn_out_L135]->param->velo);
			turn_out135_time += (uint16_t)(DEG2RAD(mode[i][Turn_out_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_out135_time += (uint16_t)(mode[i][Turn_out_L135]->param->Lend/mode[i][Turn_out_L135]->param->velo);
		}

	}


}

uint16_t calcRunTime::return_turn_time(t_run_pattern run_pt)
{
	switch(run_pt)
	{
		case Turn_in_L45:
		case Turn_in_R45:
			return (uint16_t)(turn_in45_time);
		case Turn_out_L45:
		case Turn_out_R45:
			return (uint16_t)(turn_out45_time);
		case Turn_in_L135:
		case Turn_in_R135:
			return (uint16_t)(turn_in135_time);
		case Turn_out_L135:
		case Turn_out_R135:
			return (uint16_t)(turn_out135_time);
		case Turn_RV90:
		case Turn_LV90:
			return (uint16_t)(turn_out135_time);
		case Long_turnR90:
		case Long_turnL90:
			return (uint16_t)(turn_Long90_time);
		case Long_turnR180:
		case Long_turnL180:
			return (uint16_t)(turn_Long180_time);
		default:
			return 0;
	}
}

uint16_t calcRunTime::straight_time_set(float length)
{
	uint16_t time = 65535;
	float start_velo 	= st_set_mode[0]->param->max_velo;
	float end_velo 		= st_set_mode[0]->param->max_velo;
	float acc_time = 0.0;	float deacc_time = 0.0;
	float acc_length = 0.0; float deacc_length = 0.0;
	for(int i = st_mode_size-1; i >= 0;i--){
		float max_velo	= st_set_mode[i]->param->max_velo;
		float accel		= st_set_mode[i]->param->acc;
        acc_length		= ((max_velo*1000.0)*(max_velo*1000.0)-(start_velo*1000.0)*(start_velo*1000.0))/(2*accel*1000.0);
        deacc_length    = ((max_velo*1000.0)*(max_velo*1000.0)-(end_velo*1000.0)*(end_velo*1000.0))/(2*accel*1000.0);
        if(length-OFF_SET_LENGTH-(acc_length+deacc_length) >= 0.0)
        {
        	acc_time = (max_velo - start_velo)/accel * 1000.0;
        	deacc_time = (max_velo - end_velo)/accel * 1000.0;
        	time = (uint16_t)OFF_SET_LENGTH/st_set_mode[0]->param->max_velo+(uint16_t)((length-(acc_length+deacc_length))/max_velo) + (uint16_t)acc_time + (uint16_t)deacc_time;
        	break;
        }
	}
	return time;
}

t_straight_param calcRunTime::calc_end_straight_max_velo(float length)
{
	t_straight_param return_param;

	//uint16_t time = 65535;
	float start_velo 	= st_set_mode[0]->param->max_velo;
	float end_velo 		= 0.0;
	return_param.param = st_set_mode[0]->param;
	return_param.sp_gain		  = st_set_mode[0]->sp_gain;
	return_param.om_gain		  = st_set_mode[0]->om_gain;
	//float acc_time = 0.0;	float deacc_time = 0.0;
	float acc_length = 0.0; float deacc_length = 0.0;
	for(int i = st_mode_size-1; i >= 0;i--){
		float max_velo	= st_set_mode[i]->param->max_velo;
		float accel		= st_set_mode[i]->param->acc;
        acc_length		= ((max_velo*1000.0)*(max_velo*1000.0)-(start_velo*1000.0)*(start_velo*1000.0))/(2*accel*1000.0);
        deacc_length    = ((max_velo*1000.0)*(max_velo*1000.0)-(end_velo*1000.0)*(end_velo*1000.0))/(2*accel*1000.0);
        if(length-OFF_SET_LENGTH-(acc_length+deacc_length) >= 0.0)
        {
        	return_param.param 			  =	st_set_mode[i]->param;
        	return_param.sp_gain		  = st_set_mode[i]->sp_gain;
        	return_param.om_gain		  = st_set_mode[i]->om_gain;
        	//acc_time = (max_velo - start_velo)/accel * 1000.0;
        	//deacc_time = (max_velo - end_velo)/accel * 1000.0;
        	//time = (uint16_t)OFF_SET_LENGTH/mode[0]->param->max_velo+(uint16_t)((length-OFF_SET_LENGTH-(acc_length+deacc_length))/max_velo) + (uint16_t)acc_time + (uint16_t)deacc_time;
        	break;
        }
	}
	return return_param;
}

uint16_t calcRunTime::diagonal_time_set(float length)
{
	uint16_t time = 65535;
	float start_velo 	= di_set_mode[0]->param->max_velo;
	float end_velo 		= di_set_mode[0]->param->max_velo;
	float acc_time = 0.0;	float deacc_time = 0.0;
	float acc_length = 0.0; float deacc_length = 0.0;
	for(int i = di_mode_size-1; i >= 0;i--){
		float max_velo	= di_set_mode[i]->param->max_velo;
		float accel		= di_set_mode[i]->param->acc;
        acc_length		= ((max_velo*1000.0)*(max_velo*1000.0)-(start_velo*1000.0)*(start_velo*1000.0))/(2*accel*1000.0);
        deacc_length    = ((max_velo*1000.0)*(max_velo*1000.0)-(end_velo*1000.0)*(end_velo*1000.0))/(2*accel*1000.0);
        if(length-OFF_SET_LENGTH-(acc_length+deacc_length) >= 0.0)
        {
        	acc_time = (max_velo - start_velo)/accel * 1000.0;
        	deacc_time = (max_velo - end_velo)/accel * 1000.0;
        	time = (uint16_t)OFF_SET_LENGTH/di_set_mode[0]->param->max_velo+(uint16_t)((length-(acc_length+deacc_length))/max_velo) + (uint16_t)acc_time + (uint16_t)deacc_time;
        	break;
        }
	}
	return time;
}

t_straight_param calcRunTime::calc_end_diagonal_max_velo(float length)
{
	t_straight_param return_param;

	//uint16_t time = 65535;
	float start_velo 	= di_set_mode[0]->param->max_velo;
	float end_velo 		= 0.0;
	return_param.param = di_set_mode[0]->param;
	return_param.sp_gain		  = di_set_mode[0]->sp_gain;
	return_param.om_gain		  = di_set_mode[0]->om_gain;
	//float acc_time = 0.0;	float deacc_time = 0.0;
	float acc_length = 0.0; float deacc_length = 0.0;
	for(int i = di_mode_size-1; i >= 0;i--){
		float max_velo	= di_set_mode[i]->param->max_velo;
		float accel		= di_set_mode[i]->param->acc;
        acc_length		= ((max_velo*1000.0)*(max_velo*1000.0)-(start_velo*1000.0)*(start_velo*1000.0))/(2*accel*1000.0);
        deacc_length    = ((max_velo*1000.0)*(max_velo*1000.0)-(end_velo*1000.0)*(end_velo*1000.0))/(2*accel*1000.0);
        if(length-OFF_SET_LENGTH-(acc_length+deacc_length) >= 0.0)
        {
        	return_param.param 			  =	di_set_mode[i]->param;
        	return_param.sp_gain		  = di_set_mode[i]->sp_gain;
        	return_param.om_gain		  = di_set_mode[i]->om_gain;
        	//acc_time = (max_velo - start_velo)/accel * 1000.0;
        	//deacc_time = (max_velo - end_velo)/accel * 1000.0;
        	//time = (uint16_t)OFF_SET_LENGTH/mode[0]->param->max_velo+(uint16_t)((length-OFF_SET_LENGTH-(acc_length+deacc_length))/max_velo) + (uint16_t)acc_time + (uint16_t)deacc_time;
        	break;
        }
	}
	return return_param;
}

void calcRunTime::st_param_set(const t_straight_param *const *mode,uint16_t mode_size)
{
	st_set_mode = mode;
	st_mode_size = mode_size;
}
void calcRunTime::di_param_set(const t_straight_param *const *mode,uint16_t mode_size)
{
	di_set_mode = mode;
	di_mode_size = mode_size;
}
