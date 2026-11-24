// Params/Param_B/turn_1400/turn_1400.hに対応する機体Bの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_B/run_param_B.h"

// ===== Params/Param_B/turn_1400/turn_1400.h =====
/*
 * turn_1400.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */


#include "Params/typedef_run_param.h"

// 2026-11-22 ユーザー指定: 未設定のdegree_correction[deg]を同じ機体の
// turn_1600.cppの同名ターンから複写。1400での実測は未確認。

//k = 250
const t_pid_gain sp_gain_turn90_1400 = {4.0, 0.05, 0.00};
const t_pid_gain om_gain_turn90_1400 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,22.01,38.31, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,22.01,38.31,-90.0f,Turn_R};
const t_turn_param_table slalom_L90_1400_table = {1.40f, 50.0f,18.88,34.03, 90.0f,Turn_L,-1.0f};
const t_turn_param_table slalom_R90_1400_table = {1.40f,-50.0f,18.88,34.03,-90.0f,Turn_R,1.0f};
const t_param param_L90_1400 = {&slalom_L90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};
const t_param param_R90_1400 = {&slalom_R90_1400_table,&sp_gain_turn90_1400,&om_gain_turn90_1400};

//k = 250
const t_pid_gain sp_gain_turn180_1400 = {4.0, 0.05, 0.00};
const t_pid_gain om_gain_turn180_1400 = {0.20f,0.01f,0.0f};
const t_turn_param_table slalom_L180_1400_table = {1.40f, 43.0f,12.05,28.43, 180.0f,Turn_L,-0.5f};
const t_turn_param_table slalom_R180_1400_table = {1.40f,-43.0f,12.05,28.43,-180.0f,Turn_R,0.5f};
const t_param param_L180_1400 = {&slalom_L180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};
const t_param param_R180_1400 = {&slalom_R180_1400_table,&sp_gain_turn180_1400,&om_gain_turn180_1400};

//not adjust
//k = 250
const t_pid_gain sp_gain_turnV90_1400 = {4.0, 0.05, 0.00};
const t_pid_gain om_gain_turnV90_1400 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,11.55-2.0,26.99, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,11.55-2.0,26.99,-90.0f,Turn_R};
const t_turn_param_table slalom_LV90_1400_table = {1.40f, 38.50f,8.05,22.40, 90.0f,Turn_L,1.20f};
const t_turn_param_table slalom_RV90_1400_table = {1.40f,-38.50f,8.05,22.40,-90.0f,Turn_R,-1.20f};
const t_param param_LV90_1400 = {&slalom_LV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};
const t_param param_RV90_1400 = {&slalom_RV90_1400_table,&sp_gain_turnV90_1400,&om_gain_turnV90_1400};

//k = 250
const t_pid_gain sp_gain_turnIn45_1400 = {4.0, 0.05, 0.00};
const t_pid_gain om_gain_turnIn45_1400 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.5f,8.28,40.77, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1400_table = {1.40f,-55.0f,8.28,40.77,-45.0f,Turn_R};
const t_turn_param_table slalom_inL45_1400_table = {1.40f, 50.0f,12.02,40.72, 45.0f,Turn_L,1.2f};
const t_turn_param_table slalom_inR45_1400_table = {1.40f,-50.0f,12.02,40.72,-45.0f,Turn_R,-1.2f};
const t_param param_inL45_1400 = {&slalom_inL45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};
const t_param param_inR45_1400 = {&slalom_inR45_1400_table,&sp_gain_turnIn45_1400,&om_gain_turnIn45_1400};

//k = 220
//k = 250
const t_pid_gain sp_gain_turnOut45_1400 = {4.0, 0.05, 0.00};
const t_pid_gain om_gain_turnOut45_1400 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_outL45_1400_table = {1.40f, 55.0f,26.22,22.11, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1400_table = {1.40f,-55.0f,26.22,22.11,-45.0f,Turn_R};
const t_turn_param_table slalom_outL45_1400_table = {1.40f, 50.0f,28.90,24.14, 45.0f,Turn_L,0.90f};
const t_turn_param_table slalom_outR45_1400_table = {1.40f,-50.0f,28.90,24.14,-45.0f,Turn_R,-0.90f};
const t_param param_outL45_1400 = {&slalom_outL45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};
const t_param param_outR45_1400 = {&slalom_outR45_1400_table,&sp_gain_turnOut45_1400,&om_gain_turnOut45_1400};

//k= 250
const t_pid_gain sp_gain_turnIn135_1400 = {4.0, 0.05, 0.00};
const t_pid_gain om_gain_turnIn135_1400 = {0.20f,0.01f,0.0f};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1400_table = {1.40f, 42.0f,21.48,31.96, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1400_table = {1.40f,-42.0f,21.48,31.86,-135.0f,Turn_R};
const t_turn_param_table slalom_inL135_1400_table = {1.40f, 42.0f,11.50,19.68, 135.0f,Turn_L,-0.40f};
const t_turn_param_table slalom_inR135_1400_table = {1.40f,-42.0f,11.50,19.68,-135.0f,Turn_R,0.40f};
const t_param param_inL135_1400 = {&slalom_inL135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};
const t_param param_inR135_1400 = {&slalom_inR135_1400_table,&sp_gain_turnIn135_1400,&om_gain_turnIn135_1400};

//220
const t_pid_gain sp_gain_turnOut135_1400 = {4.0, 0.05, 0.00};
const t_pid_gain om_gain_turnOut135_1400 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_outL135_1400_table = {1.40f, 42.0f,13.88,39.77, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1400_table = {1.40f,-42.0f,13.88,39.77,-135.0f,Turn_R};
const t_turn_param_table slalom_outL135_1400_table = {1.40f, 40.0f,10.14,34.19, 135.0f,Turn_L,-1.0f};
const t_turn_param_table slalom_outR135_1400_table = {1.40f,-40.0f,10.14,34.19,-135.0f,Turn_R,1.0f};
const t_param param_outL135_1400 = {&slalom_outL135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};
const t_param param_outR135_1400 = {&slalom_outR135_1400_table,&sp_gain_turnOut135_1400,&om_gain_turnOut135_1400};

const t_param *const mode_1400[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1400,		&param_L90_1400,
												&param_R180_1400,	&param_L180_1400,
												&param_inR45_1400,	&param_inL45_1400,
												&param_outR45_1400,	&param_outL45_1400,
												&param_inR135_1400,	&param_inL135_1400,
												&param_outR135_1400,	&param_outL135_1400,
												&param_RV90_1400,	&param_LV90_1400
											};

const t_param *const mode_1400_acc[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1400,		&param_L90_1400,
												&param_R180_1400,	&param_L180_1400,
												&param_inR45_1400,	&param_inL45_1400,
												&param_outR45_1400,	&param_outL45_1400,
												&param_inR135_1400,	&param_inL135_1400,
												&param_outR135_1400,	&param_outL135_1400,
												&param_RV90_1400,	&param_LV90_1400,
												NULL,				NULL
											};

