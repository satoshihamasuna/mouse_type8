/*
 * interrupt.c
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */



#include <math.h>
#include "Module/Inc/interrupt.h"
#include "Module/Inc/log_data.h"
#include "Module/Inc/remote_explore.h"
#include "Module/Inc/interrupt_timing.h"

#include "Peripheral/Inc/peripheral.h"
#include "Peripheral/Inc/ir_sensor_activity.h"
#include "Params/mouse_config.h"

#include "Task/Inc/sensing_task.h"
#include "Task/Inc/ctrl_task.h"

#include "Component/Inc/controller.h"
#include "Component/Inc/half_float.h"


#define GYRO_OFFSET (GYRO_COR_OFF)
#define ACC_OFFSET (0.0f)

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
	const uint32_t entered = InterruptTiming::now();
	Sensor_AdcComplete();
	Interrupt::getInstance().preprocess();
	Interrupt::getInstance().main();
	Interrupt::getInstance().postprocess();
	InterruptTiming::record(entered, InterruptTiming::now());
	}
}

void Interrupt_Initialize(){
	InterruptTiming::initialize();
	//HAL_TIM_Base_Start_IT(&htim5);
}


void Interrupt::preprocess()
{


	//update encoder information
	Encoder_SetSpeed_Left();
	Encoder_SetSpeed_Right();
	t_encoder Renc = Encoder_GetProperty_Right();
	t_encoder Lenc = Encoder_GetProperty_Left();

	//update accel information
	acc_sum = acc_sum - acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = (-1.0)*read_accel_y_axis() - ACC_OFFSET;
	acc_sum = acc_sum + acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	x_acc_sum = x_acc_sum - x_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	x_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = (-1.0)*read_accel_x_axis();
	x_acc_sum = x_acc_sum + x_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	z_acc_sum = z_acc_sum - z_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];
	z_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE] = (-1.0)*read_accel_z_axis();
	z_acc_sum = z_acc_sum + z_acc_buff[(acc_time_cnt)%ACC_BUFF_SIZE];

	acc_time_cnt = (acc_time_cnt == (ACC_BUFF_SIZE-1))? 0:acc_time_cnt + 1;


	//update vehicle ego information
	float acc_mean   	= acc_sum/(float)(ACC_BUFF_SIZE);
	float x_acc_mean  = x_acc_sum/(float)(ACC_BUFF_SIZE);
	float z_acc_mean    = z_acc_sum/(float)(ACC_BUFF_SIZE);
	float enc_velo_mean = ((Renc.wheel_speed) - (Lenc.wheel_speed))/2.0;
	float velo			= (1.0/2.0f)*acc_mean*((float)(ACC_BUFF_SIZE))/1000.0f + enc_velo_mean;
	float enc_velo		= ((Renc.sp_pulse ) - (Lenc.sp_pulse))*MMPP/2.0;
	float length		= Vehicle_type8::getInstance().ego.length.get() + enc_velo;

	Vehicle_type8::getInstance().ego.accel.set(acc_mean);
	Vehicle_type8::getInstance().ego.velo.set(velo);
	Vehicle_type8::getInstance().ego.length.set(length);

	Vehicle_type8::getInstance().ego.z_accel.set(z_acc_mean);

	float rad_velo 		= (GYRO_COR_RATE)*(read_gyro_z_axis()*PI/180.0+GYRO_OFFSET);
	const float heading_before_rad = Vehicle_type8::getInstance().ego.radian.get();
	float rad			= heading_before_rad + rad_velo/1000.0f;

	Vehicle_type8::getInstance().ego.rad_velo.set(rad_velo);
	Vehicle_type8::getInstance().ego.radian.set(rad);


	float set_velo = velo;
	if(set_velo == 0.0f) set_velo = 0.001;
	const float previous_slip_theta = Vehicle_type8::getInstance().ego.turn_slip_theta.get();
	float slip_theta = (previous_slip_theta*1000.0f - rad_velo)
						/(1000.0f + Vehicle_type8::getInstance().turn_slip_k_m_s2()/(set_velo));

	Vehicle_type8::getInstance().ego.turn_slip_theta.set(slip_theta);

	float horizon_velo = enc_velo*slip_theta;
	// x_acc_mean is the ACC_BUFF_SIZE-sample mean of the lateral-axis IMU
	// measurement from read_accel_x_axis().
	float horizon_acc = x_acc_mean;
	Vehicle_type8::getInstance().ego.horizon_accel.set(horizon_acc);
	Vehicle_type8::getInstance().ego.horizon_velo.set(horizon_velo);


	float estimate_theta = Vehicle_type8::getInstance().ego.radian.get();
	const float legacy_left_step_mm = velo*sin(estimate_theta);
	const float legacy_forward_step_mm = velo*cos(estimate_theta);
	// The pose owns position integration. Passage yaw/variance is handed
	// in once at entry and returned only while the turn EKF is trusted.
	const auto timing = InterruptTiming::sample();
	const float dt_s = timing.period_us * 1.0e-6f;
	const bool timing_valid = isfinite(dt_s) && dt_s >= 0.0005f && dt_s <= 0.0025f
		&& isfinite(timing.total_us) && timing.total_us < timing.period_us*0.85f;
	auto &ctrl = CtrlTask_type8::getInstance();
	auto &vehicle = Vehicle_type8::getInstance();
	auto &pose_input = turn_pose_input_;
	pose_input.dt_s = timing_valid ? dt_s : 0.001f;
	pose_input.timing_valid = timing_valid;
	pose_input.speed_m_s = timing_valid ? velo/(dt_s*1000.0f) : velo;
	pose_input.wheel_speed_m_s = timing_valid ? enc_velo_mean/(dt_s*1000.0f) : enc_velo_mean;
	pose_input.target_speed_m_s = vehicle.ideal.velo.get();
	pose_input.gyro_rad_s = rad_velo;
	pose_input.lateral_accel_m_s2 = x_acc_mean;
	pose_input.forward_accel_m_s2 = acc_mean;
	pose_input.heading_before_rad = heading_before_rad;
	pose_input.legacy_left_mm = legacy_left_step_mm;
	pose_input.legacy_forward_mm = legacy_forward_step_mm;
	pose_input.pattern = static_cast<uint8_t>(ctrl.motion_pattern_get());
	pose_input.turning = ctrl.motion_is_enable_get()==True && ctrl.motion_exeStatus_get()==execute
		&& ctrl.motion_control_get()==SLATURN_STATE;
	pose_input.suction = vehicle.suction_flag_get()==True;
	pose_input.suction_duty = static_cast<float>(vehicle.suction_out);
	pose_input.quiet_command = fabsf(vehicle.ideal.velo.get())<0.01f && fabsf(vehicle.ideal.rad_velo.get())<0.05f;
	auto &ir_sens = IrSensTask_type8::getInstance();
	pose_input.heading_prior = pose_input.turning ? ir_sens.TakeTurnHeadingPrior() : MotionHeading::Prior{};
	const auto pose_output = turn_pose_.update(pose_input);
	ir_sens.UpdateTurnHeadingPosterior(turn_pose_.heading_for_handoff());
	auto &frames=vehicle.pose;
	if (pose_output.applied)
		frames.predict_turn(pose_output.forward_mm,pose_output.left_mm,pose_output.source_heading_before_rad,pose_output.yaw_delta_rad);
	else frames.predict_body(enc_velo,0.0f,rad_velo*0.001f);
	if (pose_output.applied) {
		vehicle.ego.turn_slip_theta.set(pose_output.slip_rad);
		vehicle.ego.horizon_velo.set(pose_input.speed_m_s*tanf(pose_output.slip_rad));
	}


	Vehicle_type8::getInstance().ego.x_point.set(Vehicle_type8::getInstance().ego.x_point.get() + legacy_left_step_mm);

	float battery_voltage = 0.98 * Vehicle_type8::getInstance().battery.get() + (0.02)*Battery_GetVoltage();
	Vehicle_type8::getInstance().battery.set(battery_voltage);

	//update wall sensor information
	// enc_velo is this interrupt's signed encoder travel in mm.
	IrSensTask_type8::getInstance().IrSensorSet(&Vehicle_type8::getInstance(), enc_velo, HAL_GetTick());
}

void Interrupt::main()
{
	if(RemoteExplore::should_stop(HAL_GetTick()))
	{
		CtrlTask_type8::getInstance().motion_exeStatus_set(error);
		Sensor_SetHandWaitActive(0U);
		CtrlTask_type8::getInstance().motion_disable_set();
		Vehicle *vehicle = &Vehicle_type8::getInstance();
		vehicle->pose.stop();
		vehicle->suctionStop();
		vehicle->V_r = vehicle->V_l = 0.0f;
		vehicle->motor_out_r = vehicle->motor_out_l = 0;
		Motor_SetDuty_Left(0);
		Motor_SetDuty_Right(0);
		return;
	}
	/*
	controll_task::getInstance().motion_inInterrupt();
	controll_task::getInstance().motionControll();
	 */

	CtrlTask_type8::getInstance().motion_prev_control();
	CtrlTask_type8::getInstance().motion_control();
	CtrlTask_type8::getInstance().motion_post_control();
}

void Interrupt::postprocess()
{
	Update_LED_Pulses(1U);

	const uint32_t log_started = InterruptTiming::now();
	LogData::getInstance().logging();
	InterruptTiming::set_logging_cycles(InterruptTiming::now() - log_started);
	time_count = time_count + 1;
	IMU_read_DMA_Start();
}

