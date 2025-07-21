/*
 * math_utils.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_MATH_UTILS_H_
#define INC_MATH_UTILS_H_



#define MASK_SEARCH	0x01
#define MASK_SECOND	0x03

#define ACC_BUFF_SIZE (30)

#define ENC_RESOLUTION	(172)

#define TIRE_DIAMETER	(14.5f)							//mm
#define TIRE_RADIUS		(TIRE_DIAMETER/2.0f)			//mm
#define MMPP			(TIRE_DIAMETER*PI/ENC_RESOLUTION)	//mm
#define TREAD_WIDTH		((18.0-2.0)*2)

#define WEIGHT			(18.0)					//g
#define MOTOR_K_ER		(0.08)					//mV/rpm
#define MOTOR_K_TR		(0.764)	//0.4//0.594				//mNm/A
#define MOTOR_R			(2.5)//6.0
#define GEAR_N			(52.0/8.0)
#define MOUSE_INERTIA	((0.1/1000.0))//0.001f//0.003,0.0022				//g・m^2
#define RAD_2_RPM		60.0/(2.0*3.141592)
#define TIRE_RADIUS_M	(TIRE_RADIUS/1000.0)		//m
#define TREAD_WIDTH_M	((18.0-2.0)*2.0/1000.0)
#define MOTOR_BR		(0.25/100000.0)			//mNm/rpm
#define L_BAR_DT		(20.0/1000.0)


//#define DEBUG_MODE	0
#define ENABLE_MODE1   0x10
#define ENABLE_MODE2   0x20
#define ENABLE_MODE3   0x30
#define DISENABLE_MODE 0x00

#define G					(9.80665f)					// 重量加速度[m/s^2]
#define PI					(3.1415926f)				// 円周率
#define SQRT2				(1.41421356237f)			// ルート2
#define SQRT3				(1.73205080757f)			// ルート3
#define SQRT5				(2.2360679775f)				// ルート5
#define SQRT7				(2.64575131106f)			// ルート7

#define DEG2RAD(x)			(((x)/180.0f)*PI)			// 度数法からラジアンに変換
#define RAD2DEG(x)			(180.0f*((x)/PI))			// ラジアンから度数法に変換
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))
#define ABS(x) 				((x) < 0 ? -(x) : (x))		// 絶対値
#define SIGN(x)				((x) < 0 ? -1 : 1)			// 符号
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		// 2つのうち大きい方を返します
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		// 2つのうち小さい方を返します
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))

//machine parameter
#define SECTION				(90.0)
#define HALF_SECTION		(SECTION/2.0)
#define DIAG_SECTION		(63.6396)
#define DIAG_HALF_SECTION	(DIAG_SECTION/2.0)
#define SEARCH_CORRECTION	(55.0)
#define DIAGONAL_CORRECTION	(5.0)
#define STRAIGHT_CORRECTION (10.0)

#define MAZE_SIZE_X  32
#define MAZE_SIZE_Y  32
#define MAZE_SIZE  1024

#define MAZE_GOAL_X  7
#define MAZE_GOAL_Y  7
#define MAZE_GOAL_SIZE  2
#define MAP_MAX_VALUE 1024



#endif /* INC_MATH_UTILS_H_ */
