/*
 * mouse_config.h
 *
 *  Created on: Nov 2, 2025
 *      Author: sato1
 */

#ifndef MOUSE_CONFIG_H_
#define MOUSE_CONFIG_H_

#include "Component/Inc/math_utils.h"

// ===== 使用するマウスを選択 =====
#define MOUSE_A
//#define MOUSE_B
// ================================

// ===== 安全チェック =====
#if defined(MOUSE_A) && defined(MOUSE_B)
    #error "MOUSEA と MOUSEB は同時に定義できません。どちらか一方だけ有効にしてください。"
#elif !defined(MOUSE_A) && !defined(MOUSE_B)
    #error "MOUSEA または MOUSEB のどちらかを定義してください。"
#endif

#define SECTION				(90.0)
#define HALF_SECTION		(SECTION/2.0)
#define DIAG_SECTION		(63.6396)
#define DIAG_HALF_SECTION	(DIAG_SECTION/2.0)
#define SEARCH_CORRECTION	(45.0)
#define DIAGONAL_CORRECTION	(0.0)
#define STRAIGHT_CORRECTION (7.0)

#define MAZE_SIZE_X  32
#define MAZE_SIZE_Y  32
#define MAZE_SIZE  1024

#define MAZE_GOAL_X  7
#define MAZE_GOAL_Y  7
#define MAZE_GOAL_SIZE  2
#define MAP_MAX_VALUE 1024


#define ACC_BUFF_SIZE (20)
#define ENC_RESOLUTION	(172)

#define TIRE_DIAMETER	(15.02f)							//mm
#define TIRE_RADIUS		(TIRE_DIAMETER/2.0f)			//mm
#define MMPP			(TIRE_DIAMETER*PI/ENC_RESOLUTION)	//mm
#define TREAD_WIDTH		(28.0)

#if defined(MOUSE_A)
	#define WEIGHT			(20.0)					//g
	#define MOTOR_K_ER		(0.08)					//mV/rpm 0.08
	#define MOTOR_K_TR		(MOTOR_K_ER*(RADPS_2_RPM))	//0.4//0.594				//mNm/A
	#define MOTOR_R			(3.5)//6.0
	#define GEAR_N			(34.0/7.0 )
	#define MOUSE_INERTIA	((1.5/1000.0))//0.001f//0.003,0.0022				//g・m^2
	#define TIRE_RADIUS_M	(TIRE_RADIUS/1000.0)		//m
	#define TREAD_WIDTH_M	(TREAD_WIDTH/1000.0)
	#define MOTOR_BR		(1.0/1000000.0*0.0)			//mNm/rpm
	#define L_BAR_DT		(20.0/1000.0*0.0)

	#define FF_GAIN			(1.0)
	// Feedforward coefficients identified from the 2026-07-15 MOUSE_A logs.
	#define FF_SP_VELO_COEF		(0.4239f)	// V / (m/s)
	#define FF_SP_ACCEL_COEF	(0.09665f)	// V / (m/s^2)
	#define FF_OM_VELO_COEF		(0.00602f)	// V / (rad/s)
	#define FF_OM_ACCEL_COEF	(0.00110f)	// V / (rad/s^2)
	#define FF_SP_BIAS_COEF		(0.3017f)	// V (signed Coulomb-friction compensation)
	#define FF_OM_BIAS_COEF		(0.0f)		// V (signed angular-friction compensation)

	#define DUTY_MIN		(0)
	#define DUTY_MAX		(999)
	#define DEAD_V			(0.05)
	#define DEAD_VR			(DEAD_V)
	#define DEAD_VL			(DEAD_V)

	#define GYRO_COR_RATE	(-1.005f)
	#define GYRO_COR_OFF	(0.0)//(0.005969)


#elif defined(MOUSE_B)
	#define WEIGHT			(20.0)					//g
	#define MOTOR_K_ER		(0.08)					//mV/rpm
	#define MOTOR_K_TR		(0.764)	//0.4//0.594				//mNm/A
	#define MOTOR_R			(3.5)//6.0
	#define GEAR_N			(52.0/8.0)
	#define MOUSE_INERTIA	((1.0/1000.0))//0.001f//0.003,0.0022				//g・m^2
	#define TIRE_RADIUS_M	(TIRE_RADIUS/1000.0)		//m
	#define TREAD_WIDTH_M	(TREAD_WIDTH/1000.0)
	#define MOTOR_BR		(1.0/1000000.0*0.0)			//mNm/rpm
	#define L_BAR_DT		(20.0/1000.0*0.0)

	#define FF_GAIN			(1.0)
	// MOUSE_B has not been identified from logs yet; keep the legacy model values.
	#define FF_SP_VELO_COEF	(MOTOR_K_ER*RADPS_2_RPM*GEAR_N/TIRE_RADIUS)
	#define FF_SP_ACCEL_COEF	(MOTOR_R/(MOTOR_K_TR*GEAR_N)*(WEIGHT/1000.0f*TIRE_RADIUS/2.0f))
	#define FF_OM_VELO_COEF	(MOTOR_K_ER*RADPS_2_RPM*GEAR_N*TREAD_WIDTH/(2.0f*TIRE_RADIUS*1000.0f))
	#define FF_OM_ACCEL_COEF	(MOTOR_R/(MOTOR_K_TR*GEAR_N)*(MOUSE_INERTIA*TIRE_RADIUS/(TREAD_WIDTH/2.0f)))
	#define FF_SP_BIAS_COEF	(0.0f)	// MOUSE_B has not been identified from logs yet.
	#define FF_OM_BIAS_COEF	(0.0f)	// MOUSE_B has not been identified from logs yet.

	#define DUTY_MIN		(0)//(80)
	#define DUTY_MAX		(990)//(1000)
	#define DEAD_V			(0.0)//(0.15)//(0.6)
	#define DEAD_VR			(DEAD_V)
	#define DEAD_VL			(DEAD_V)

	#define GYRO_COR_RATE	(-1.015f)
	#define GYRO_COR_OFF	(0.00)
#else
    #error "MOUSEA または MOUSEB が定義されていません。mouse_select.h を確認してください。"
#endif



#endif /* MOUSE_CONFIG_H_ */
