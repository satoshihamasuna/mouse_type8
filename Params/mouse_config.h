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
//#define MOUSE_A
#define MOUSE_B
// ================================

// ===== 安全チェック =====
#if defined(MOUSE_A) && defined(MOUSE_B)
    #error "MOUSEA と MOUSEB は同時に定義できません。どちらか一方だけ有効にしてください。"
#elif !defined(MOUSE_A) && !defined(MOUSE_B)
    #error "MOUSEA または MOUSEB のどちらかを定義してください。"
#endif

#define ACC_BUFF_SIZE (20)
#define ENC_RESOLUTION	(172)



#define TIRE_DIAMETER	(14.98f)							//mm
#define TIRE_RADIUS		(TIRE_DIAMETER/2.0f)			//mm
#define MMPP			(TIRE_DIAMETER*PI/ENC_RESOLUTION)	//mm
#define TREAD_WIDTH		(28.0)



#if defined(MOUSE_A)
	#define WEIGHT			(20.0)					//g
	#define MOTOR_K_ER		(0.1)					//mV/rpm
	#define MOTOR_K_TR		(0.764)	//0.4//0.594				//mNm/A
	#define MOTOR_R			(3.5)//6.0
	#define GEAR_N			(52.0/8.0)
	#define MOUSE_INERTIA	((2.0/1000.0))//0.001f//0.003,0.0022				//g・m^2
	#define RAD_2_RPM		60.0/(2.0*3.141592)
	#define TIRE_RADIUS_M	(TIRE_RADIUS/1000.0)		//m
	#define TREAD_WIDTH_M	(30.0/1000.0)
	#define MOTOR_BR		(1.0/1000000.0*1.0)			//mNm/rpm
	#define L_BAR_DT		(20.0/1000.0)


#elif defined(MOUSE_B)
	#define WEIGHT			(20.0)					//g
	#define MOTOR_K_ER		(0.08)					//mV/rpm
	#define MOTOR_K_TR		(0.764)	//0.4//0.594				//mNm/A
	#define MOTOR_R			(3.5)//6.0
	#define GEAR_N			(52.0/8.0)
	#define MOUSE_INERTIA	((1.0/1000.0))//0.001f//0.003,0.0022				//g・m^2
	#define RAD_2_RPM		60.0/(2.0*3.141592)
	#define TIRE_RADIUS_M	(TIRE_RADIUS/1000.0)		//m
	#define TREAD_WIDTH_M	(TREAD_WIDTH/1000.0)
	#define MOTOR_BR		(1.0/1000000.0*0.0)			//mNm/rpm
	#define L_BAR_DT		(20.0/1000.0*0.0)
#else
    #error "MOUSEA または MOUSEB が定義されていません。mouse_select.h を確認してください。"
#endif



#endif /* MOUSE_CONFIG_H_ */
