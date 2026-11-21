// Params/Param_B/turn_1600/turn_1600.hに対応する機体Bの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_B/run_param_B.h"

// ===== Params/Param_B/turn_1600/turn_1600.h =====
/*
 * turn_1600.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */


#include "Params/typedef_run_param.h"
// 2026-11-20 ユーザー指定で、機体Aの現行turn_1600の形状テーブル14定義をコピー。
// 参照元: Params/Param_A/turn_1600/turn_1600.cpp（同日の作業時点の値）。
// 速度[m/s]、半径・Lstart・Lend[mm]、角度・角度補正[deg]。Bでの追加調整は各定義の記録を参照。
// 各有効定義の直前に変更前のB値を保持し、PID・FFはBの既存設定を使用する。
// 2026-11-20 B右90度・1600 mm/s・吸引duty650の単体3本を反映。Lstart/Lend = 13.6/26.31 mm。
// 補正は右+1.0度、左はユーザー指定の鏡像-1.0度（左の実走・連続接続は未確認）。
// 根拠: tools/turn_analysis/20261120_B_turn1600_R90_adoption/README.md
//k = 250
const t_pid_gain sp_gain_turn90_1600 = {4.0,0.05,0.0};
const t_pid_gain om_gain_turn90_1600 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 50.50f,19.56,38.43, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-50.50f,19.56,38.43,-90.0f,Turn_R};
// const t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.95,30.5, 90.0f,Turn_L,0.0};
// const t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,12.0f,26.31f, 90.0f,Turn_L,-1.5f};
const t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,13.6f,26.31f, 90.0f,Turn_L,-1.0f};
// const t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.95,30.5,-90.0f,Turn_R,0.0};
// const t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,12.0f,26.31f,-90.0f,Turn_R, 1.5f};
const t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,13.6f,26.31f,-90.0f,Turn_R, 1.0f};
const t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};
const t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600};

// 2026-11-20 B右180度・1600 mm/s・吸引duty650の単体3本を反映。半径47.5 mm・補正+0.5度。
// 左はユーザー指定の鏡像（半径+47.5 mm・補正-0.5度）。左の実走・連続接続は未確認。
// 根拠: tools/turn_analysis/20261120_B_turn1600_R180_adoption/README.md
//k = 250
const t_pid_gain sp_gain_turn180_1600 ={4.0,0.05,0.0};
const t_pid_gain om_gain_turn180_1600 = {0.20, 0.01f, 0.0};
// const t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,8.80,30.10, 180.0f,Turn_L,-1.5f};
// const t_turn_param_table slalom_L180_1600_table = {1.60f, 45.0f,13.12f,27.38f, 180.0f,Turn_L,-2.0f};
const t_turn_param_table slalom_L180_1600_table = {1.60f, 47.5f,13.12f,27.38f, 180.0f,Turn_L,-0.5f};
// const t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,8.80,30.10,-180.0f,Turn_R,1.5f};
// const t_turn_param_table slalom_R180_1600_table = {1.60f,-45.0f,13.12f,27.38f,-180.0f,Turn_R,2.0f};
const t_turn_param_table slalom_R180_1600_table = {1.60f,-47.5f,13.12f,27.38f,-180.0f,Turn_R,0.5f};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f,  45.0f,(15.10),(34.72), 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f, -45.0f,(15.10),(34.72),-180.0f,Turn_R};
const t_param param_L180_1600 = {&slalom_L180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};
const t_param param_R180_1600 = {&slalom_R180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600};

//k = 250
const t_pid_gain sp_gain_turnV90_1600 = {4.0,0.05,0.0};
const t_pid_gain om_gain_turnV90_1600 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 38.0f,9.50,29.66, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-38.0f,9.50,29.66,-90.0f,Turn_R};
// const t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.79,24.95, 90.0f,Turn_L,-1.5f};
const t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,2.20f,20.83f, 90.0f,Turn_L,-1.60f};
// const t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.79,24.95,-90.0f,Turn_R,1.5f};
const t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,2.20f,20.83f,-90.0f,Turn_R, 1.60f};
const t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};
const t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600};

// 2026-11-21 B右45度進入の暫定採用値。1600 mm/s・吸引duty650、同設定2本の相対評価。
// Lstart/Lend = 7.8/34.5 mm、右補正-1.2度。左はユーザー指定の鏡映+1.2度（実走未確認）。
// 角速度Ki = 0.005、角加減速FF = 0.000750をin45専用で反映。
// 根拠・残差・未確認事項: tools/turn_analysis/20261121_B_turn1600_IN45_adoption/README.md
//k = 250
const t_pid_gain sp_gain_turnIn45_1600 = {4.0,0.05,0.0};
// const t_pid_gain om_gain_turnIn45_1600 = {0.20f,0.01f,0.0f};
const t_pid_gain om_gain_turnIn45_1600 = {0.20f,0.005f,0.0f};
const t_ff_gain ff_gain_turnIn45_1600 = {
	0.661202f,0.052929f,0.0f,
	0.009256f,0.000750f,0.000750f,0.0f,
	0.0f,0.0f,0.0f
};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 53.0f,9.84,41.94, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-53.0f,9.84,41.94,-45.0f,Turn_R};
// const t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,7.52,39.62, 45.0f,Turn_L,-1.5f};
// const t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,5.80f,39.86f, 45.0f,Turn_L, 0.90f};
const t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,7.8f,34.5f, 45.0f,Turn_L, 1.2f};
// const t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,7.52,39.62,-45.0f,Turn_R,1.5f};
// const t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,5.80f,39.86f,-45.0f,Turn_R,-0.90f};
const t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,7.8f,34.5f,-45.0f,Turn_R,-1.2f};
const t_param param_inL45_1600 = {&slalom_inL45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_turnIn45_1600};
const t_param param_inR45_1600 = {&slalom_inR45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_turnIn45_1600};

//k = 250
const t_pid_gain sp_gain_turnOut45_1600 = {4.0,0.05,0.0};
const t_pid_gain om_gain_turnOut45_1600 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 55.0f,18.235,20.24, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-55.0f,18.235,20.24,-45.0f,Turn_R};
// const t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.06,20.07, 45.0f,Turn_L,-1.5f};
const t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.80f,16.50f, 45.0f,Turn_L,-0.30f};
// const t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.06,20.07,-45.0f,Turn_R,1.5f};
const t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.80f,16.50f,-45.0f,Turn_R,0.30f};
const t_param param_outL45_1600 = {&slalom_outL45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600};
const t_param param_outR45_1600 = {&slalom_outR45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600};

// 2026-11-20 B右135度入口・1600 mm/s・吸引duty650の単体3本を反映。Lstart/Lend = 10.5/17.2 mm。
// 補正は右+0.40度、左はユーザー指定の鏡像-0.40度（左の実走・連続接続は未確認）。
// 根拠: tools/turn_analysis/20261120_B_turn1600_IN135_adoption/README.md
//k = 250,alpha = 1.0
const t_pid_gain sp_gain_turnIn135_1600 = {4.0,0.05,0.0};
const t_pid_gain om_gain_turnIn135_1600 = {0.20f,0.01f,0.0f};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 44.0f,13.17,26.34, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-44.0f,13.17,26.34,-135.0f,Turn_R};
// const t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,8.23,14.76, 135.0f,Turn_L,-1.5f};
// const t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,5.10f,20.56f, 135.0f,Turn_L,-2.10f};
const t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,10.5f,17.2f, 135.0f,Turn_L,-0.40f};
// const t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,8.23,14.76,-135.0f,Turn_R,1.5f};
// const t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,5.10f,20.56f,-135.0f,Turn_R,2.10f};
const t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,10.5f,17.2f,-135.0f,Turn_R,0.40f};
const t_param param_inL135_1600 = {&slalom_inL135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};
const t_param param_inR135_1600 = {&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600};

//k = 250
const t_pid_gain sp_gain_turnOut135_1600 = {4.0,0.05,0.0};
const t_pid_gain om_gain_turnOut135_1600 = {0.20f,0.01f,0.0f};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 42.0f,11.85,40.11, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-42.0f,11.85,40.1,-135.0f,Turn_R};
// const t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,8.09,37.73, 135.0f,Turn_L,-1.5f};
const t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,1.00f,31.50f, 135.0f,Turn_L,-1.90f};
// const t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,8.09,37.73,-135.0f,Turn_R,1.5f};
const t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,1.00f,31.50f,-135.0f,Turn_R,1.90f};
const t_param param_outL135_1600 = {&slalom_outL135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600};
const t_param param_outR135_1600 = {&slalom_outR135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600};

const t_param *const mode_1600[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1600,		&param_L90_1600,
												&param_R180_1600,	&param_L180_1600,
												&param_inR45_1600,	&param_inL45_1600,
												&param_outR45_1600,	&param_outL45_1600,
												&param_inR135_1600,	&param_inL135_1600,
												&param_outR135_1600,	&param_outL135_1600,
												&param_RV90_1600,	&param_LV90_1600
											};



const t_param *const mode_1600_acc_v1[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1600,		&param_L90_1600,
												&param_R180_1600,	&param_L180_1600,
												NULL,					NULL	,
												NULL,					NULL	,
												&param_inR135_1600,	&param_inL135_1600,
												&param_outR135_1600,	&param_outL135_1600,
												NULL,					NULL	,
												NULL,					NULL
											};

const t_param *const mode_1600_acc_v2[] = 	{	NULL,					NULL,			NULL,
													&param_R90_1600,		&param_L90_1600,
													&param_R180_1600,	&param_L180_1600,
													&param_inR45_1600,	&param_inL45_1600,
													&param_outR45_1600,	&param_outL45_1600,
													&param_inR135_1600,	&param_inL135_1600,
													&param_outR135_1600,	&param_outL135_1600,
													&param_RV90_1600,	&param_LV90_1600,
													NULL,					NULL
											};

