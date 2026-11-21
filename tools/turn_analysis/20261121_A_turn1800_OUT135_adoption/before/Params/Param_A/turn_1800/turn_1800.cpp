// Params/Param_A/turn_1800/turn_1800.hに対応する機体Aの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_A/run_param_A.h"

// ===== Params/Param_A/turn_1800/turn_1800.h =====
/*
 * turn_1800.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */


#include "Params/typedef_run_param.h"

// OM feedforward starts from the corresponding 1600 mm/s turn and is refined
// per shape from the 1800 mm/s voltage/response reports. Short profiles use
// shape-specific jerk gains to limit saturation; accel/decel gains account for
// first/second-half tracking and post-turn residual omega. Turn-specific SP
// feedforward uses the conservative 50% response-model trial value from the
// 20260722 translational analysis.  Only measured shapes receive this value.
// Recheck in45/V90 carefully because their source logs include saturation.
const t_ff_gain ff_gain_long_turn_90_R_1800 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0168f, 0.00092f, 0.00089f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_long_turn_90_L_1800 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0168f, 0.00097f, 0.00088f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_long_turn_180_R_1800 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00088f, 0.00090f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_long_turn_180_L_1800 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00094f, 0.00087f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_in45_R_1800 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00082f, 0.0613f, 0.0000024f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_in45_L_1800 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00097f, 0.00084f, 0.0175f, 0.0000022f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_out45_R_1800 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f
};
const t_ff_gain ff_gain_out45_L_1800 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f
};
const t_ff_gain ff_gain_in135_R_1800 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00091f, 0.00090f, 0.0613f, 0.0000032f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_in135_L_1800 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00096f, 0.00088f, 0.0175f, 0.0000029f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_out135_R_1800 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f
};
const t_ff_gain ff_gain_out135_L_1800 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f
};
const t_ff_gain ff_gain_v90_R_1800 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00093f, 0.00090f, 0.0613f, 0.0000028f, -0.00010034f, 0.00062859f
};
const t_ff_gain ff_gain_v90_L_1800 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00099f, 0.00091f, 0.0175f, 0.0000028f, -0.00010034f, 0.00062859f
};


//k = 250
const t_pid_gain sp_gain_turn90_1800 = {2.0,0.016,0.0};
const t_pid_gain om_gain_turn90_1800 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_L90_1800_table = {1.80f, 52.0f,13.71,38.14, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_1800_table = {1.80f,-52.0f,13.71,38.14,-90.0f,Turn_R};
// 2026-11-20: 機体A・1800 mm/s・右90度の単体相対評価3本の候補を依頼により反映。
// 根拠: tools/logs/pixel_camera/turn1800_A_r90_20261119_v1/right90_selected_candidate.json
// 左90度も依頼により右の候補を鏡映して反映。左単体の実機評価は未実施。
// 論理角±90度は維持し、degree_correctionで物理指令のみを補正する。
// 左右とも旧マーカー長切替から独立。左右連続・迷路での接続は未検証。
const t_turn_param_table slalom_L90_1800_table = {1.80f, 52.0f,7.8f,25.0f, 90.0f,Turn_L,-1.25f};
const t_turn_param_table slalom_R90_1800_table = {1.80f,-52.0f,7.8f,25.0f,-90.0f,Turn_R,1.25f};
const t_param param_L90_1800 = {&slalom_L90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800,&ff_gain_long_turn_90_L_1800};
const t_param param_R90_1800 = {&slalom_R90_1800_table,&sp_gain_turn90_1800,&om_gain_turn90_1800,&ff_gain_long_turn_90_R_1800};
//k = 250
const t_pid_gain sp_gain_turn180_1800 = {2.0,0.2,0.0};
const t_pid_gain om_gain_turn180_1800 = {0.1,0.002,0.0};
//const static t_turn_param_table slalom_L180_1800_table = {1.80f, 48.0f,9.90,39.15, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1800_table = {1.80f,-48.0f,9.90,39.15,-180.0f,Turn_R};
/*20260725_*/
//const static t_turn_param_table slalom_L180_1800_table = {1.80f, 48.0f,22.51,32.38, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_1800_table = {1.80f,-48.0f,22.51,32.38,-180.0f,Turn_R};
// 2026-11-21: 機体A・1800 R180の実測済み暫定値を依頼により左右へ採用。
// 根拠: tools/turn_analysis/20261121_A_turn1800_RL180_adoption/README.md
// 左は右の鏡映で実機未確認。論理角±180度を維持し、補正の符号を反転する。
// 幅約3 mmの残差・速度変動・連続接続は未解決。旧マーカー長切替から独立。
const t_turn_param_table slalom_L180_1800_table = {1.80f, 48.2f,7.52f,31.5f, 180.0f,Turn_L,-2.0f};
const t_turn_param_table slalom_R180_1800_table = {1.80f,-48.2f,7.52f,31.5f,-180.0f,Turn_R, 2.0f};
const t_param param_L180_1800 = {&slalom_L180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800,&ff_gain_long_turn_180_L_1800};
const t_param param_R180_1800 = {&slalom_R180_1800_table,&sp_gain_turn180_1800,&om_gain_turn180_1800,&ff_gain_long_turn_180_R_1800};
//k = 300
//not adjust
const t_pid_gain sp_gain_turnV90_1800 = {2.0,0.05,0.0};
const t_pid_gain om_gain_turnV90_1800 = {0.1,0.002,0.0};
//const static t_turn_param_table slalom_LV90_1800_table = {1.80f, 40.0f,3.10,27.38+00.0, 90.0f,Turn_L};
//const static t_turn_param_table slalom_RV90_1800_table = {1.80f,-40.0f,3.10,27.38+00.0,-90.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 videos, R median copied to L/R; raw Lstart -1.46 mm clamped to zero).
const t_turn_param_table slalom_LV90_1800_table = {1.80f, 40.0f,0.00,29.46, 90.0f,Turn_L};
const t_turn_param_table slalom_RV90_1800_table = {1.80f,-40.0f,0.00,29.46,-90.0f,Turn_R};
#else
const t_turn_param_table slalom_LV90_1800_table = {1.80f, 40.0f,0.00,28.64, 90.0f,Turn_L};
const t_turn_param_table slalom_RV90_1800_table = {1.80f,-40.0f,0.00,28.64,-90.0f,Turn_R};
#endif
const t_param param_LV90_1800 = {&slalom_LV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800,&ff_gain_v90_L_1800};
const t_param param_RV90_1800 = {&slalom_RV90_1800_table,&sp_gain_turnV90_1800,&om_gain_turnV90_1800,&ff_gain_v90_R_1800};
//k = 250
const t_pid_gain sp_gain_turnIn45_1800 = {2.0,0.05,0.0};
const t_pid_gain om_gain_turnIn45_1800 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_inL45_1800_table = {1.80f, 53.0f,9.27,41.09, 45.0f,Turn_L};
//const static t_turn_param_table slalom_inR45_1800_table = {1.80f,-53.0f,9.27,41.09,-45.0f,Turn_R};
// 2026-11-21: 機体A・1800 mm/s・吸引650の右45度進入の単体候補を依頼により反映。
// 根拠: tools/turn_analysis/20261121_A_turn1800_inr45_adoption/README.md
// 入口相対の有効5本を優先。左は追加依頼による鏡映で、左単体の実機評価は未実施。
// 左右とも旧マーカー長切替から独立し、方向別PID・FFは既存値を使う。
const t_turn_param_table slalom_inL45_1800_table = {1.80f, 53.0f,2.0f,40.0f, 45.0f,Turn_L};
const t_turn_param_table slalom_inR45_1800_table = {1.80f,-53.0f,2.0f,40.0f,-45.0f,Turn_R};
const t_param param_inL45_1800 = {&slalom_inL45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800,&ff_gain_in45_L_1800};
const t_param param_inR45_1800 = {&slalom_inR45_1800_table,&sp_gain_turnIn45_1800,&om_gain_turnIn45_1800,&ff_gain_in45_R_1800};

//k = 250
const t_pid_gain sp_gain_turnOut45_1800 = {2.0,0.05,0.0};
const t_pid_gain om_gain_turnOut45_1800 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,23.81,22.01, 45.0f,Turn_L};
//const static t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,23.81,22.01,-45.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
const t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,23.63,23.69, 45.0f,Turn_L};
const t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,23.63,23.69,-45.0f,Turn_R};
#else
const t_turn_param_table slalom_outL45_1800_table = {1.80f, 55.0f,22.66,26.00, 45.0f,Turn_L};
const t_turn_param_table slalom_outR45_1800_table = {1.80f,-55.0f,22.66,26.00,-45.0f,Turn_R};
#endif
const t_param param_outL45_1800 = {&slalom_outL45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800,&ff_gain_out45_L_1800};
const t_param param_outR45_1800 = {&slalom_outR45_1800_table,&sp_gain_turnOut45_1800,&om_gain_turnOut45_1800,&ff_gain_out45_R_1800};

//k = 250
const t_pid_gain sp_gain_turnIn135_1800 = {2.0,0.05,0.0};
const t_pid_gain om_gain_turnIn135_1800 = {0.1,0.002,0.0};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1800_table = {1.80f, 43.0f,10.67,31.00, 135.0f,Turn_L};
//const static t_turn_param_table slalom_inR135_1800_table = {1.80f,-43.0f,10.67,31.00,-135.0f,Turn_R};
// 2026-11-20: 機体A・1800 mm/s・吸引650の右135度進入の単体相対候補を依頼により反映。
// 根拠: tools/logs/pixel_camera/turn1800_A_inr135_20261120_v1/Ls2p5_Le26_c2p4_review_v1/candidate.settings.json
// 右は完全記録2本と動画のみ1本。左は依頼による鏡映で、左単体・連続接続は実機未検証。
// 論理角±135度を維持し、degree_correctionで物理指令角のみを±132.6度へ補正する。
// 旧マーカー長切替から独立した値を使い、方向別PID・FFは既存の参照を維持する。
const t_turn_param_table slalom_inL135_1800_table = {1.80f, 40.0f,2.5f,26.0f, 135.0f,Turn_L,-2.4f};
const t_turn_param_table slalom_inR135_1800_table = {1.80f,-40.0f,2.5f,26.0f,-135.0f,Turn_R, 2.4f};
const t_param param_inL135_1800 = {&slalom_inL135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800,&ff_gain_in135_L_1800};
const t_param param_inR135_1800 = {&slalom_inR135_1800_table,&sp_gain_turnIn135_1800,&om_gain_turnIn135_1800,&ff_gain_in135_R_1800};

//k = 250
const t_pid_gain sp_gain_turnOut135_1800 = {2.0,0.05,0.0};
const t_pid_gain om_gain_turnOut135_1800 = {0.1,0.002,0.0};
//const static t_turn_param_table slalom_outL135_1800_table = {1.80f, 41.0f,9.30,44.12, 135.0f,Turn_L};
//const static t_turn_param_table slalom_outR135_1800_table = {1.80f,-41.0f,9.30,44.12,-135.0f,Turn_R};
#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260725 Lstart=Lend=0 videos, R median copied to L/R).
const t_turn_param_table slalom_outL135_1800_table = {1.80f, 41.0f,8.60,45.97, 135.0f,Turn_L};
const t_turn_param_table slalom_outR135_1800_table = {1.80f,-41.0f,8.60,45.97,-135.0f,Turn_R};
#else
const t_turn_param_table slalom_outL135_1800_table = {1.80f, 41.0f,15.61,46.65, 135.0f,Turn_L};
const t_turn_param_table slalom_outR135_1800_table = {1.80f,-41.0f,15.61,46.65,-135.0f,Turn_R};
#endif
const t_param param_outL135_1800 = {&slalom_outL135_1800_table,&sp_gain_turnOut135_1800,&om_gain_turnOut135_1800,&ff_gain_out135_L_1800};
const t_param param_outR135_1800 = {&slalom_outR135_1800_table,&sp_gain_turnOut135_1800,&om_gain_turnOut135_1800,&ff_gain_out135_R_1800};

const t_param *const mode_1800[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,	&param_L180_1800,
												&param_inR45_1800,	&param_inL45_1800,
												&param_outR45_1800,	&param_outL45_1800,
												&param_inR135_1800,	&param_inL135_1800,
												&param_outR135_1800,	&param_outL135_1800,
												&param_RV90_1800,	&param_LV90_1800
											};

const t_param *const mode_1800_acc_v1[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,		&param_L180_1800,
												NULL,					NULL	,
												NULL,					NULL	,
												&param_inR135_1800,		&param_inL135_1800,
												&param_outR135_1800,	&param_outL135_1800,
												NULL,					NULL	,
												NULL,					NULL
											};

const t_param *const mode_1800_acc_v2[] = 	{	NULL,					NULL,			NULL,
												&param_R90_1800,		&param_L90_1800,
												&param_R180_1800,		&param_L180_1800,
												&param_inR45_1800,		&param_inL45_1800,
												&param_outR45_1800,		&param_outL45_1800,
												&param_inR135_1800,		&param_inL135_1800,
												&param_outR135_1800,	&param_outL135_1800,
												NULL,					NULL	,
												NULL,					NULL
											};

