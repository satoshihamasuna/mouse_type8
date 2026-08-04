/*
 * sensing_task.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SENSING_TASK_H_
#define CPP_INC_SENSING_TASK_H_

#include "Module/Inc/vehicle.h"
#include "Component/Inc/singleton.h"
#include "Component/Inc/typedef.h"
#include "Component/Inc/maze_typedef.h"
#include "Task/Inc/run_typedef.h"

#define STRAIGHT_REF		(45.0)
#define DIAGONAL_REF		(32.0)

#define SIDE_R_THRESHOLD		(65.0)
#define SIDE_L_THRESHOLD		(65.0)
#define SIDE_THRESHOLD			(65.0)
#define CORNER_R_THRESHOLD		(60.0)
#define CORNER_L_THRESHOLD		(60.0)

//#define SIDECONTROL_FR_THRESHOLD		(80.0)
//#define SIDECONTROL_FL_THRESHOLD		(80.0)

#if defined(MOUSE_A)
	#define SIDECONTROL_FR_THRESHOLD		(85.0)
	#define SIDECONTROL_FL_THRESHOLD		(85.0)
	#define FRONT_THRESHOLD		(110.0)

#elif defined(MOUSE_B)
	#define SIDECONTROL_FR_THRESHOLD		(80.0)
	#define SIDECONTROL_FL_THRESHOLD		(80.0)
	#define FRONT_THRESHOLD		(105.0)

#else
    #error "MOUSEA または MOUSEB が定義されていません。mouse_select.h を確認してください。"
#endif



#define SIDE_CORNER_THRESHOLD (68.0)

typedef struct{
	int16_t value;
	t_bool prev_is_wall;
	t_bool is_wall;
	t_bool is_control;
	float distance;
	float control_th;
	uint16_t control_cnt;
	float error;
	float distance_log[20];
	float distance_sum;
	float control_distance;
	float avg_distance;
	float diff;
	float prev_diff;
}t_sensor;


typedef enum
{
	STRAIGHT_IRSENS = 0,
	DIAGONAL_IRSENS = 1,
	TURN_IRSENS		= 2,
}t_irsens_motion;

class IrSensTask
{
	private:
		float Sensor_CalcDistance(t_sensor_dir dir,int16_t value);
		float IrSensor_adc2voltage(int16_t value);
		float	 wall_ref = STRAIGHT_REF;
		t_bool	 isEnableIrSens = False;
		t_irsens_motion irsens_motion;
		int ir_log_cnt;

	public:

		param_element control_ir;
		param_element control_ir_dot;
		t_sensor sen_fr,sen_fl,sen_r,sen_l;
		t_bool 	 wall_correction;
		t_bool 	 r_wall_corner,		l_wall_corner;
		uint16_t r_corner_time,		l_corner_time;
		param_element 	 r_corner_length,	l_corner_length;
		t_wall_state conv_Sensin2Wall(t_sensor_dir sens_dir);
		virtual 		void IrSensorSet(Vehicle *vehicle);
		void IrSensMotion_Set(t_irsens_motion _irsens_motion){irsens_motion = _irsens_motion;	}

		void IrSensorReferenceSet(float ref_value);
		void IrSensorDistanceSet(Vehicle *vehicle);
		void IrSensorWallSet(Vehicle *vehicle);
		void SetWallControl_RadVelo(Vehicle *vehicle,float delta_tms);
		inline void EnableIrSens()		{isEnableIrSens = True;}
		inline void DisableIrSens()			{isEnableIrSens = False;}
		void EnableIrSensStraight()		{	EnableIrSens();		IrSensMotion_Set(STRAIGHT_IRSENS);	IrSensorReferenceSet(STRAIGHT_REF);	}
		void EnableIrSensDiagonal()		{	EnableIrSens();		IrSensMotion_Set(DIAGONAL_IRSENS);	IrSensorReferenceSet(DIAGONAL_REF);	}
		float IrSensorMaxValueFromLog(t_sensor_dir dir);
		int16_t IrSensor_Avg();
		t_bool Division_Wall_Correction()
		{
			t_bool flag = False;
			if(r_wall_corner == True)
			{
				if(wall_correction == False)
				{
					wall_correction = True;
					flag = True;
					Indicate_LED(0x10|Return_LED_Status());
				}
				//Indicate_LED(0x10|Return_LED_Status());
			}
			if(l_wall_corner == True)
			{
				if(wall_correction == False)
				{
					wall_correction = True;
					flag = True;
					Indicate_LED(0x20|Return_LED_Status());
				}
				//Indicate_LED(0x20|Return_LED_Status());
			}


			return flag;
		}
		void Division_Wall_Correction_Reset()
		{
			Indicate_LED(0x00);
			//r_check = l_check =
			wall_correction = False;
		}


};

class IrSensTask_type8: public IrSensTask,public Singleton<IrSensTask_type8>
{
	public:
		void IrSensorSet(Vehicle *vehicle) override
		{
			sen_fl.value =  Sensor_GetValue(sensor_fl);
			sen_fr.value =  Sensor_GetValue(sensor_fr);
			sen_l.value  =  Sensor_GetValue(sensor_sl);
			sen_r.value  =  Sensor_GetValue(sensor_sr);
			IrSensorDistanceSet(vehicle);
			IrSensorWallSet(vehicle);
		}

};

#endif /* CPP_INC_SENSING_TASK_H_ */
