/*
 * interrupt.c
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */



#include <math.h>
#include "../Inc/interrupt.h"
#include "../Inc/log_data.h"

#include "peripheral.h"

#include "../../Task/Inc/sensing_task.h"
#include "../../Task/Inc/ctrl_task.h"

#include "../../Component/Inc/controller.h"
#include "../../Component/Inc/half_float.h"


#define GYRO_OFFSET (0.0)

float lambda_slip;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*
    if (htim == &htim5){
    	//Interrupt::getInstance().preprocess();
    	//Interrupt::getInstance().main();
    	//Interrupt::getInstance().postprocess();
    }
    */
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1){
	Interrupt::getInstance().preprocess();
	Interrupt::getInstance().main();
	Interrupt::getInstance().postprocess();
	}
}

void Interrupt_Initialize(){
	//HAL_TIM_Base_Start_IT(&htim5);
}
/*
void Interrupt::preprocess(){

	SensingTask::getInstance().IrSensorSet();

	Encoder_SetSpeed_Left();
	Encoder_SetSpeed_Right();
	t_encoder Renc = Encoder_GetProperty_Right();
	t_encoder Lenc = Encoder_GetProperty_Left();
	acc_sum = acc_sum - acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = read_accel_y_axis();
	acc_sum = acc_sum + acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	velo_sum = velo_sum - velo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	velo_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = (Renc.wheel_speed - Lenc.wheel_speed)/2.0;
	velo_sum = velo_sum + velo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	Rvelo_sum = Rvelo_sum - Rvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	Rvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = Renc.wheel_speed;
	Rvelo_sum = Rvelo_sum + Rvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	Lvelo_sum = Lvelo_sum - Lvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	Lvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = Lenc.wheel_speed;
	Lvelo_sum = Lvelo_sum + Lvelo_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	acc_time_cnt = (acc_time_cnt == (ACC_BUFF_SIZE-1))? 0:acc_time_cnt + 1;


	controll_task::getInstance().mouse.velo 	  = (-1.0/2.0f)*acc_sum/((float)(ACC_BUFF_SIZE))*((float)(ACC_BUFF_SIZE))/1000.0f+((Renc.wheel_speed) - (Lenc.wheel_speed))/2.0;
	//(読み間違い対策)
	controll_task::getInstance().mouse.length  += controll_task::getInstance().mouse.velo;//(1.0)*((((float)(Renc.sp_pulse)) - ((float)(Lenc.sp_pulse))))/2.0*MMPP;
	controll_task::getInstance().mouse.accel    = (-1.0)*acc_sum/((float)(ACC_BUFF_SIZE));
	controll_task::getInstance().mouse.rad_velo = (-1.0)*read_gyro_z_axis()*PI/180;
	controll_task::getInstance().mouse.radian  += controll_task::getInstance().mouse.rad_velo/1000.0;
	controll_task::getInstance().mouse.x_point += (1.0)*(SIGN(Renc.wheel_speed)*ABS(((float)(Renc.sp_pulse)) - SIGN(Lenc.wheel_speed)*ABS((float)(Lenc.sp_pulse))))/2.0*MMPP*controll_task::getInstance().mouse.radian;

	if(controll_task::getInstance().rT.get_run_mode_state() == TURN_MODE )
	{
		controll_task::getInstance().mouse.turn_slip_dot =  -250.0f*controll_task::getInstance().mouse.turn_slip_theta/controll_task::getInstance().mouse.velo-controll_task::getInstance().mouse.rad_velo;
		controll_task::getInstance().mouse.turn_slip_theta += controll_task::getInstance().mouse.turn_slip_dot/1000.0f;
		controll_task::getInstance().mouse.turn_x_dash = controll_task::getInstance().mouse.velo*sin((-1.0)*(controll_task::getInstance().mouse.radian + controll_task::getInstance().mouse.turn_slip_theta)  );
		controll_task::getInstance().mouse.turn_y_dash = controll_task::getInstance().mouse.velo*cos(controll_task::getInstance().mouse.radian + controll_task::getInstance().mouse.turn_slip_theta );
		controll_task::getInstance().mouse.turn_x += controll_task::getInstance().mouse.turn_x_dash;
		controll_task::getInstance().mouse.turn_y += controll_task::getInstance().mouse.turn_y_dash;
	}

}
*/

void Interrupt::preprocess()
{

	//update wall sensor information
	IrSensTask_type7::getInstance().IrSensorSet();

	//update encoder information
	Encoder_SetSpeed_Left();
	Encoder_SetSpeed_Right();
	t_encoder Renc = Encoder_GetProperty_Right();
	t_encoder Lenc = Encoder_GetProperty_Left();

	//update accel information
	acc_sum = acc_sum - acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = (-1.0)*read_accel_y_axis();
	acc_sum = acc_sum + acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	z_acc_sum = z_acc_sum - z_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	z_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = (-1.0)*read_accel_z_axis();
	z_acc_sum = z_acc_sum + z_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	acc_time_cnt = (acc_time_cnt == (ACC_BUFF_SIZE-1))? 0:acc_time_cnt + 1;


	//update vehicle ego information
	float acc_mean   	= acc_sum/(float)(ACC_BUFF_SIZE);
	float z_acc_mean    = z_acc_sum/(float)(ACC_BUFF_SIZE);
	float enc_velo_mean = ((Renc.wheel_speed) - (Lenc.wheel_speed))/2.0;
	float velo			= (1.0/2.0f)*acc_mean*((float)(ACC_BUFF_SIZE))/1000.0f + enc_velo_mean;
	float enc_velo		= ((Renc.sp_pulse ) - (Lenc.sp_pulse))*MMPP/2.0;
	float length		= Vehicle_type7::getInstance().ego.length.get() + enc_velo;

	Vehicle_type7::getInstance().ego.accel.set(acc_mean);
	Vehicle_type7::getInstance().ego.velo.set(velo);
	Vehicle_type7::getInstance().ego.length.set(length);

	Vehicle_type7::getInstance().ego.z_accel.set(z_acc_mean);

	float rad_velo 		= (-1.0)*read_gyro_z_axis()*PI/180 - GYRO_OFFSET;
	float rad			= Vehicle_type7::getInstance().ego.radian.get() + rad_velo/1000.0f;

	Vehicle_type7::getInstance().ego.rad_velo.set(rad_velo);
	Vehicle_type7::getInstance().ego.radian.set(rad);


	float set_velo = velo;
	if(set_velo == 0.0f) set_velo = 0.001;
	float slip_theta = (Vehicle_type7::getInstance().ego.turn_slip_theta.get()*1000.0f - rad_velo)
						/(1000.0f + Vehicle_type7::getInstance().turn_slip_k.get()/(set_velo));

	Vehicle_type7::getInstance().ego.turn_slip_theta.set(slip_theta);
	float slip_theta_dot = -Vehicle_type7::getInstance().turn_slip_k.get()*slip_theta/set_velo - rad_velo;
	Vehicle_type7::getInstance().ego.turn_slip_dot.set(slip_theta_dot);

	float horizon_velo = velo*slip_theta;
	float horizon_acc  =  -Vehicle_type7::getInstance().turn_slip_k.get()*slip_theta - rad_velo*velo;
	Vehicle_type7::getInstance().ego.horizon_accel.set(horizon_acc);
	Vehicle_type7::getInstance().ego.horizon_velo.set(horizon_velo);


	float estimate_theta = Vehicle_type7::getInstance().ego.radian.get();
	float turn_x_dot = velo*sin(estimate_theta);// + horizon_velo*cos(estimate_theta);
	float turn_y_dot = velo*cos(estimate_theta);// - horizon_velo*sin(estimate_theta);

	Vehicle_type7::getInstance().ego.turn_x_dash.set(turn_x_dot);
	Vehicle_type7::getInstance().ego.turn_y_dash.set(turn_y_dot);

	float turn_x = Vehicle_type7::getInstance().ego.turn_x.get() + turn_x_dot;
	float turn_y = Vehicle_type7::getInstance().ego.turn_y.get() + turn_y_dot;

	Vehicle_type7::getInstance().ego.turn_x.set(turn_x);
	Vehicle_type7::getInstance().ego.turn_y.set(turn_y);

	Vehicle_type7::getInstance().ego.x_point.set(Vehicle_type7::getInstance().ego.x_point.get() + turn_x_dot);

	float battery_voltage = 0.95 * Vehicle_type7::getInstance().battery.get() + (0.05)*Battery_GetVoltage();
	Vehicle_type7::getInstance().battery.set(battery_voltage);
}

void Interrupt::main()
{
	/*
	controll_task::getInstance().motion_inInterrupt();
	controll_task::getInstance().motionControll();
	 */

	CtrlTask_type7::getInstance().motion_prev_control();
	CtrlTask_type7::getInstance().motion_control();
	CtrlTask_type7::getInstance().motion_post_control();
}

void Interrupt::postprocess()
{


	LogData::getInstance().logging();
	time_count = time_count + 1;
	IMU_read_DMA_Start();
}

