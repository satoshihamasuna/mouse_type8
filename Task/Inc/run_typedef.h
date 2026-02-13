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
	enkaigei			= 31,
}t_run_pattern;

/*
enum class MotionPattern : uint8_t {
	No_run				,
	Straight 			,
	Diagonal			,
	Long_turnR90		,
	Long_turnL90		,
	Long_turnR180		,
	Long_turnL180		,
	Turn_in_R45			,
	Turn_in_L45			,
	Turn_out_R45		,
	Turn_out_L45		,
	Turn_in_R135		,
	Turn_in_L135		,
	Turn_out_R135		,
	Turn_out_L135		,
	Turn_RV90			,
	Turn_LV90			,
	Long_turn_RV90		,
	Long_turn_LV90		,
	Search_st_section	,
	Search_st_half		,
	Pivot_turn_R		,
	Pivot_turn_L		,
	Search_slalom_R		,
	Search_slalom_L		,
	Run_brake			,
	Motor_free			,
	Fix_wall			,
	Suction_start		,
	Backward			,
	Run_Pause			,
	Enkaigei			,
};
*/


typedef enum{
	execute	    = 2,
	complete    = 1,
	error 		= 0,
}t_exeStatus;

typedef enum
{
	NOP_STATE = 0,
	STRAIGHT_STATE  = 1,
	DIAGONAL_STATE  = 2,
	SLATURN_STATE	= 3,
	PIVTURN_STATE	= 4,
	BRAKE_STATE		= 5,
}t_runControl;

/*
enum class MotionControlMode : uint8_t{
    Idle,
    Straight,
    Diagonal,
    SlalomTurn,
    PivotTurn,
    Brake,
};
*/

/*
enum class MotionExecState : uint8_t
{
    Running,       // 実行中
    Completed,     // 正常完了
    Error,         // 異常終了
};
*/
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
