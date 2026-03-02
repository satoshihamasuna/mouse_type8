/*
 * mouse_config.h
 *
 *  Created on: Nov 2, 2025
 *      Author: sato1
 */

#ifndef MOUSE_CONFIG_H_
#define MOUSE_CONFIG_H_

#include "math_utils.h"

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
#define STRAIGHT_CORRECTION (0.0)

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
	#define MOTOR_K_ER		(0.08)					//mV/rpm
	#define MOTOR_K_TR		(MOTOR_K_ER*(RADPS_2_RPM))	//0.4//0.594				//mNm/A
	#define MOTOR_R			(3.5)//6.0
	#define GEAR_N			(34.0/7.0)
	#define MOUSE_INERTIA	((1.0/1000.0))//0.001f//0.003,0.0022				//g・m^2
	#define TIRE_RADIUS_M	(TIRE_RADIUS/1000.0)		//m
	#define TREAD_WIDTH_M	(TREAD_WIDTH/1000.0)
	#define MOTOR_BR		(1.0/1000000.0*5.0)			//mNm/rpm
	#define L_BAR_DT		(20.0/1000.0*0.0)

	#define FF_GAIN			(2.0)

	#define DUTY_MIN		(50)
	#define DEAD_V			(0.8)
	#define DEAD_VR			(DEAD_V)
	#define DEAD_VL			(DEAD_V)

	#define GYRO_COR_RATE	(-1.00f)


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

	#define DUTY_MIN		(80)
	#define DEAD_V			(0.6)
	#define DEAD_VR			(DEAD_V)
	#define DEAD_VL			(DEAD_V)

	#define GYRO_COR_RATE	(-1.015f)
#else
    #error "MOUSEA または MOUSEB が定義されていません。mouse_select.h を確認してください。"
#endif



#endif /* MOUSE_CONFIG_H_ */
