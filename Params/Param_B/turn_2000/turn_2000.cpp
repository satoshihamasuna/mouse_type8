// Params/Param_B/turn_2000/turn_2000.hに対応する機体Bの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_B/run_param_B.h"

// ===== Params/Param_B/turn_2000/turn_2000.h =====
/*
 * turn_2000.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */


#include "Params/typedef_run_param.h"

const t_pid_gain sp_gain_turn90_2000 = {3.0,0.05,0.00};;
const t_pid_gain om_gain_turn90_2000 = {0.20f,0.01f,0.0f};
const t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,12.57,42.89, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,12.57,42.89,-90.0f,Turn_R};
const t_param param_L90_2000 = {&slalom_L90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000};
const t_param param_R90_2000 = {&slalom_R90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000};

const t_pid_gain sp_gain_turn180_2000 = {3.0,0.05,0.00};;
const t_pid_gain om_gain_turn180_2000 = {0.20f,0.01f,0.0f};
const t_turn_param_table slalom_L180_2000_table = {2.00f, 47.0f,7.28,33.08, 180.0f,Turn_L};
const t_turn_param_table slalom_R180_2000_table = {2.00f,-47.0f,7.28,33.08,-180.0f,Turn_R};
const t_param param_L180_2000 = {&slalom_L180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000};
const t_param param_R180_2000 = {&slalom_R180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000};



//k = 300
//not adjust
const t_pid_gain sp_gain_turnV90_2000 = {3.0,0.05,0.00};;
const t_pid_gain om_gain_turnV90_2000 = {0.20f,0.01f,0.0f};
const t_turn_param_table slalom_LV90_2000_table = {2.00f, 38.0f,1.8,35, 90.0f,Turn_L};
const t_turn_param_table slalom_RV90_2000_table = {2.00f,-38.0f,1.8,35,-90.0f,Turn_R};
const t_param param_LV90_2000 = {&slalom_LV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000};
const t_param param_RV90_2000 = {&slalom_RV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000};
//k = 300
const t_pid_gain sp_gain_turnIn45_2000 = {3.0,0.05,0.00};;
const t_pid_gain om_gain_turnIn45_2000 = {0.20f,0.01f,0.0f};
const t_turn_param_table slalom_inL45_2000_table = {2.00f, 53.0f,9.84,41.94, 45.0f,Turn_L};
const t_turn_param_table slalom_inR45_2000_table = {2.00f,-53.0f,9.84,41.94,-45.0f,Turn_R};
const t_param param_inL45_2000 = {&slalom_inL45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000};
const t_param param_inR45_2000 = {&slalom_inR45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000};

//k = 300
const t_pid_gain sp_gain_turnOut45_2000 = {3.0,0.05,0.00};;
const t_pid_gain om_gain_turnOut45_2000 = {0.20f,0.01f,0.0f};
const t_turn_param_table slalom_outL45_2000_table = {2.00f, 55.0f,18.235,20.24, 45.0f,Turn_L};
const t_turn_param_table slalom_outR45_2000_table = {2.00f,-55.0f,18.235,20.24,-45.0f,Turn_R};
const t_param param_outL45_2000 = {&slalom_outL45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000};
const t_param param_outR45_2000 = {&slalom_outR45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000};


const t_pid_gain sp_gain_turnIn135_2000 = {3.0,0.05,0.00};;
const t_pid_gain om_gain_turnIn135_2000 = {0.20f, 0.01, 0.0};//{0.7f, 0.7f, 0.0f};
const t_turn_param_table slalom_inL135_2000_table = {2.00f, 42.0f,8.0,30.0, 135.0f,Turn_L};
const t_turn_param_table slalom_inR135_2000_table = {2.00f,-42.0f,8.0,30.0,-135.0f,Turn_R};
const t_param param_inL135_2000 = {&slalom_inL135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000};
const t_param param_inR135_2000 = {&slalom_inR135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000};

//
const t_pid_gain sp_gain_turnOut135_2000 = {3.0,0.05,0.00};;
const t_pid_gain om_gain_turnOut135_2000 = {0.20f,0.01f,0.0f};
const t_turn_param_table slalom_outL135_2000_table = {2.00f, 42.0f,11.85,40.11, 135.0f,Turn_L};
const t_turn_param_table slalom_outR135_2000_table = {2.00f,-42.0f,11.85,40.1,-135.0f,Turn_R};
const t_param param_outL135_2000 = {&slalom_outL135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000};
const t_param param_outR135_2000 = {&slalom_outR135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000};

const t_param *const mode_2000[] = 	{	NULL,					NULL,			NULL,
												&param_R90_2000,		&param_L90_2000,
												&param_R180_2000,	&param_L180_2000,
												&param_inR45_2000,	&param_inL45_2000,
												&param_outR45_2000,	&param_outL45_2000,
												&param_inR135_2000,	&param_inL135_2000,
												&param_outR135_2000,	&param_outL135_2000,
												&param_RV90_2000,	&param_LV90_2000
											};

const t_param *const mode_2000_acc[] = 	{	NULL,					NULL,			NULL,
												&param_R90_2000,		&param_L90_2000,
												&param_R180_2000,		&param_L180_2000,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL	,
												NULL,					NULL
											};
const t_param *const mode_2000_acc_v2[] = 	{	NULL,					NULL,			NULL,
												&param_R90_2000,		&param_L90_2000,
												&param_R180_2000,	&param_L180_2000,
												&param_inR45_2000,	&param_inL45_2000,
												&param_outR45_2000,	&param_outL45_2000,
												&param_inR135_2000,	&param_inL135_2000,
												&param_outR135_2000,	&param_outL135_2000,
												NULL,					NULL
											};

