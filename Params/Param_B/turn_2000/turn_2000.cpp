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
//const t_pid_gain om_gain_turn90_2000 = {0.20f,0.01f,0.0f};
//const t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,12.57,42.89, 90.0f,Turn_L};
//const t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,12.57,42.89,-90.0f,Turn_R};
// 2026-11-21予測更新：機体B 2000 R90（tools/logs/pixel_camera/turn2000_B_r90_20261120_v1、om_ki=0.005の5試行）から
// 同定した暫定値。Lstart=10.22 mm（出口直線270 mmからの平均前ずれ+2.35 mm補正）、Lend=42.78 mm（Lend右平均90.11 mm補正）、
// 角度補正は右+1.0°／左-1.0°（未補正換算平均90.99°）。左90°は右90°の鏡映しで実機未検証。
// 根拠: tools/turn_analysis/20261121_AB_turn2000_90_180_prediction/README.md
const t_pid_gain om_gain_turn90_2000 = {0.20f,0.005f,0.0f};
const t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,10.22f,42.78f, 90.0f,Turn_L,-1.0f};
const t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,10.22f,42.78f,-90.0f,Turn_R, 1.0f};
const t_param param_L90_2000 = {&slalom_L90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000};
const t_param param_R90_2000 = {&slalom_R90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000};

const t_pid_gain sp_gain_turn180_2000 = {3.0,0.05,0.00};;
//const t_pid_gain om_gain_turn180_2000 = {0.20f,0.01f,0.0f};
//const t_turn_param_table slalom_L180_2000_table = {2.00f, 47.0f,7.28,33.08, 180.0f,Turn_L};
//const t_turn_param_table slalom_R180_2000_table = {2.00f,-47.0f,7.28,33.08,-180.0f,Turn_R};
// 2026-11-21予測更新：機体B 1600 R180実測3試行（r=-47.5, Ls=13.12, Le=27.38, corr=+0.5、幅91.06 mm・角179.78°・頂点186.14 mm）
// と機体B 2000 R90実測（om_ki=0.005、速度低下比・前ずれ増加量）および180°クロソイド運動学から予測した暫定値。左右とも2000実機未検証。
// 根拠: tools/turn_analysis/20261121_AB_turn2000_90_180_prediction/README.md
const t_pid_gain om_gain_turn180_2000 = {0.20f,0.005f,0.0f};
const t_turn_param_table slalom_L180_2000_table = {2.00f, 50.0f, 4.46f,35.12f, 180.0f,Turn_L,-0.5f};
const t_turn_param_table slalom_R180_2000_table = {2.00f,-50.0f, 4.46f,35.12f,-180.0f,Turn_R, 0.5f};
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

