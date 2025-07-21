/*
 * run_task.h
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_TASK_H_
#define CPP_INC_RUN_TASK_H_



typedef enum{
	No_run				= 0,
	Straight 			= 1,
	Diagonal			= 2,
	Long_turnR90		= 3,
	Long_turnL90		= 4,
	Long_turnR180		= 5,
	Long_turnL180		= 6,
	Turn_in_R45			= 7,
	Turn_in_L45			= 8,
	Turn_out_R45		= 9,
	Turn_out_L45		= 10,
	Turn_in_R135		= 11,
	Turn_in_L135		= 12,
	Turn_out_R135		= 13,
	Turn_out_L135		= 14,
	Turn_RV90			= 15,
	Turn_LV90			= 16,
	Long_turn_RV90		= 17,
	Long_turn_LV90		= 18,
	Search_st_section	= 19,
	Search_st_half		= 20,
	Pivot_turn_R		= 21,
	Pivot_turn_L		= 22,
	Search_slalom_R		= 23,
	Search_slalom_L		= 24,
	run_brake			= 25,
	motor_free			= 26,
	Fix_wall			= 27,
	Suction_start		= 28,
	Backward			= 29,
	Run_Pause			= 30,
}t_run_pattern;

typedef enum{
	NOP_MODE 		= 0,
	STRAIGHT_MODE 	= 1,
	DIAGONAL_MODE 	= 2,
	TURN_MODE 		= 3,
	SPIN_TURN_MODE  = 4,
}t_run_mode;

typedef enum
{
	Non_controll = 0,
	Enable_st = 1,
	Enable_di = 2,
}t_wall_controll;

typedef enum{
	Turn_None 	= 0,
	Turn_R 		= 1,
	Turn_L		= 2,
	Prev_Turn	= 3,
	Post_Turn	= 4,
}t_turn_dir;



#endif /* CPP_INC_RUN_TASK_H_ */
