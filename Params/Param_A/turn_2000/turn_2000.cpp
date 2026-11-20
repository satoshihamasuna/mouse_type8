// Params/Param_A/turn_2000/turn_2000.hに対応する機体Aの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_A/run_param_A.h"

// ===== Params/Param_A/turn_2000/turn_2000.h =====
/*
 * turn_2000.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */


#include "Params/typedef_run_param.h"

// OM feedforward tuned from the 2026-07-21 2000 mm/s turn logs.
// R/L-specific SP turn feedforward was identified independently from the
// 20260722 long-180 voltage response.  Long-180 uses a second 50% iterative
// correction from the repeated FF-on logs.  Other shapes retain the initial
// directional baseline because spreading the stronger correction would drive
// measured long-90 profiles into the common 1.0 V clamp.
const t_ff_gain ff_gain_long_turn_90_R_2000 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0168f, 0.00094f, 0.00092f, 0.0613f, 0.0000032f, -0.00033918f, 0.00018206f
};
const t_ff_gain ff_gain_long_turn_90_L_2000 = {
	0.9030126f, 0.08183161f, 0.1190275f, 0.0168f, 0.00098f, 0.00091f, 0.0175f, 0.0000029f, -0.00028747f, 0.00013429f
};
const t_ff_gain ff_gain_long_turn_180_R_2000 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00088f, 0.00094f, 0.0613f, 0.0000032f, -0.00044945f, 0.00015873f
};
const t_ff_gain ff_gain_long_turn_180_L_2000 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00094f, 0.00086f, 0.0175f, 0.0000029f, -0.00043538f, 0.00013958f
};
const t_ff_gain ff_gain_in45_R_2000 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00085f, 0.0613f, 0.0000032f, -0.00033918f, 0.00018206f
};
const t_ff_gain ff_gain_in45_L_2000 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00097f, 0.00085f, 0.0175f, 0.0000029f, -0.00028747f, 0.00013429f
};
const t_ff_gain ff_gain_out45_R_2000 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00033918f, 0.00018206f
};
const t_ff_gain ff_gain_out45_L_2000 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00028747f, 0.00013429f
};
const t_ff_gain ff_gain_in135_R_2000 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00033918f, 0.00018206f
};
const t_ff_gain ff_gain_in135_L_2000 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00028747f, 0.00013429f
};
const t_ff_gain ff_gain_out135_R_2000 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00033918f, 0.00018206f
};
const t_ff_gain ff_gain_out135_L_2000 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00028747f, 0.00013429f
};
const t_ff_gain ff_gain_v90_R_2000 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00093f, 0.00090f, 0.0613f, 0.0000032f, -0.00033918f, 0.00018206f
};
const t_ff_gain ff_gain_v90_L_2000 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00099f, 0.00091f, 0.0175f, 0.0000029f, -0.00028747f, 0.00013429f
};


const t_pid_gain sp_gain_turn90_2000 = {2.0,0.016,0.0};
const t_pid_gain om_gain_turn90_2000 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,12.57,42.89, 90.0f,Turn_L};
//const static t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,12.57,42.89,-90.0f,Turn_R};
//#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
//const t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,17.66,38.18, 90.0f,Turn_L};
//const t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,18.79,35.66,-90.0f,Turn_R};
//#else
//const t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f,13.67,35.58, 90.0f,Turn_L};
//const t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f,13.55,38.10,-90.0f,Turn_R};
//#endif
// 2026-11-21予測更新：機体A 1600（Ls=12.0, Le=26.31/実測最適22.54, corr=±1.5）→ 1800（Ls=7.8, Le=25.0, corr=±1.25）の
// 吸引ON（duty650）実測スケーリングおよび2000 mm/sログ解析から予測した暫定値。左・実機2000吸引duty650は未検証。
// 根拠: tools/turn_analysis/20261121_AB_turn2000_90_180_prediction/README.md
const t_turn_param_table slalom_L90_2000_table = {2.00f, 52.0f, 3.6f,27.8f, 90.0f,Turn_L,-1.0f};
const t_turn_param_table slalom_R90_2000_table = {2.00f,-52.0f, 3.6f,27.8f,-90.0f,Turn_R, 1.0f};
const t_param param_L90_2000 = {&slalom_L90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000,&ff_gain_long_turn_90_L_2000};
const t_param param_R90_2000 = {&slalom_R90_2000_table,&sp_gain_turn90_2000,&om_gain_turn90_2000,&ff_gain_long_turn_90_R_2000};

const t_pid_gain sp_gain_turn180_2000 = {2.0,0.1,0.00};
const t_pid_gain om_gain_turn180_2000 = {0.10,0.002,0.0};
//const static t_turn_param_table slalom_L180_2000_table = {2.00f, 50.0f,5.28,41.08, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_2000_table = {2.00f,-50.0f,5.28,41.08,-180.0f,Turn_R};
/*20260725*/
//const static t_turn_param_table slalom_L180_2000_table = {2.00f, 52.0f,26.07,35.18, 180.0f,Turn_L};
//const static t_turn_param_table slalom_R180_2000_table = {2.00f,-52.0f,28.25,38.74,-180.0f,Turn_R};
//#if PARAM_A_USE_PINK_LED_TURN_LENGTHS
//const t_turn_param_table slalom_L180_2000_table = {2.00f, 52.0f,9.10,37.03, 180.0f,Turn_L};
//const t_turn_param_table slalom_R180_2000_table = {2.00f,-52.0f,2.41,38.02,-180.0f,Turn_R};
//#else
//const t_turn_param_table slalom_L180_2000_table = {2.00f, 52.0f,2.41,38.02, 180.0f,Turn_L};
//const t_turn_param_table slalom_R180_2000_table = {2.00f,-52.0f,2.41,38.02,-180.0f,Turn_R};
//#endif
// 2026-11-21予測更新：機体A 1600（r=±45.0, Ls=13.12, Le=27.38, corr=±2.0）→ 1800（r=±48.2, Ls=7.52, Le=31.5, corr=±2.0）の
// 吸引ON（duty650）実測スケーリング、および2026-07-26 2000 mm/s（r=-52.0、実測幅89.58 mm、Ls=2.41 mm、旋回角超過+1.89〜+2.21°）から予測した暫定値。
// 左・実機2000吸引duty650は未検証。根拠: tools/turn_analysis/20261121_AB_turn2000_90_180_prediction/README.md
const t_turn_param_table slalom_L180_2000_table = {2.00f, 52.0f, 2.41f,35.62f, 180.0f,Turn_L,-2.0f};
const t_turn_param_table slalom_R180_2000_table = {2.00f,-52.0f, 2.41f,35.62f,-180.0f,Turn_R, 2.0f};
const t_param param_L180_2000 = {&slalom_L180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000,&ff_gain_long_turn_180_L_2000};
const t_param param_R180_2000 = {&slalom_R180_2000_table,&sp_gain_turn180_2000,&om_gain_turn180_2000,&ff_gain_long_turn_180_R_2000};

//k = 300
//not adjust
const t_pid_gain sp_gain_turnV90_2000 = {2.0,0.05,0.0};
const t_pid_gain om_gain_turnV90_2000 = {0.1,0.002,0.0};
const t_turn_param_table slalom_LV90_2000_table = {2.00f, 38.0f,1.8,35, 90.0f,Turn_L};
const t_turn_param_table slalom_RV90_2000_table = {2.00f,-38.0f,1.8,35,-90.0f,Turn_R};
const t_param param_LV90_2000 = {&slalom_LV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000,&ff_gain_v90_L_2000};
const t_param param_RV90_2000 = {&slalom_RV90_2000_table,&sp_gain_turnV90_2000,&om_gain_turnV90_2000,&ff_gain_v90_R_2000};
//k = 300
const t_pid_gain sp_gain_turnIn45_2000 = {2.0,0.051,0.0};
const t_pid_gain om_gain_turnIn45_2000 = {0.1,0.002,0.0};
const t_turn_param_table slalom_inL45_2000_table = {2.00f, 53.0f,9.84,41.94, 45.0f,Turn_L};
const t_turn_param_table slalom_inR45_2000_table = {2.00f,-53.0f,9.84,41.94,-45.0f,Turn_R};
const t_param param_inL45_2000 = {&slalom_inL45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000,&ff_gain_in45_L_2000};
const t_param param_inR45_2000 = {&slalom_inR45_2000_table,&sp_gain_turnIn45_2000,&om_gain_turnIn45_2000,&ff_gain_in45_R_2000};

//k = 300
const t_pid_gain sp_gain_turnOut45_2000 = {2.0,0.051,0.0};
const t_pid_gain om_gain_turnOut45_2000 = {0.1,0.002,0.0};
const t_turn_param_table slalom_outL45_2000_table = {2.00f, 55.0f,18.235,20.24, 45.0f,Turn_L};
const t_turn_param_table slalom_outR45_2000_table = {2.00f,-55.0f,18.235,20.24,-45.0f,Turn_R};
const t_param param_outL45_2000 = {&slalom_outL45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000,&ff_gain_out45_L_2000};
const t_param param_outR45_2000 = {&slalom_outR45_2000_table,&sp_gain_turnOut45_2000,&om_gain_turnOut45_2000,&ff_gain_out45_R_2000};


const t_pid_gain sp_gain_turnIn135_2000 = {2.0,0.051,0.0};
const t_pid_gain om_gain_turnIn135_2000 = {0.1,0.002,0.0};//{0.7f, 0.7f, 0.0f};
const t_turn_param_table slalom_inL135_2000_table = {2.00f, 42.0f,8.0,30.0, 135.0f,Turn_L};
const t_turn_param_table slalom_inR135_2000_table = {2.00f,-42.0f,8.0,30.0,-135.0f,Turn_R};
const t_param param_inL135_2000 = {&slalom_inL135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000,&ff_gain_in135_L_2000};
const t_param param_inR135_2000 = {&slalom_inR135_2000_table,&sp_gain_turnIn135_2000,&om_gain_turnIn135_2000,&ff_gain_in135_R_2000};

//
const t_pid_gain sp_gain_turnOut135_2000 = {2.0,0.051,0.0};
const t_pid_gain om_gain_turnOut135_2000 = {0.1,0.002,0.0};
const t_turn_param_table slalom_outL135_2000_table = {2.00f, 42.0f,11.85,40.11, 135.0f,Turn_L};
const t_turn_param_table slalom_outR135_2000_table = {2.00f,-42.0f,11.85,40.1,-135.0f,Turn_R};
const t_param param_outL135_2000 = {&slalom_outL135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000,&ff_gain_out135_L_2000};
const t_param param_outR135_2000 = {&slalom_outR135_2000_table,&sp_gain_turnOut135_2000,&om_gain_turnOut135_2000,&ff_gain_out135_R_2000};

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

