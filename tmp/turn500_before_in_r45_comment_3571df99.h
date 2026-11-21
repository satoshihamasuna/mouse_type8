/*
 * turn_500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_500_H_
#define CPP_PARAMS_TURN_500_H_

#include "Params/typedef_run_param.h"

// 500 mm/s turn-specific feedforward gains. Angular jerk gains are constrained
// to non-negative values and are fitted independently for each turn shape.
const static t_ff_gain ff_gain_long_turn_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002077371f, 0.000836273f, 0.000930115f, 0.08559123f, 0.000001569006f
};
const static t_ff_gain ff_gain_long_turn_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.003051475f, 0.000741268f, 0.000965438f, 0.08559123f, 0.000001971694f
};
const static t_ff_gain ff_gain_long_turn_180_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002126270f, 0.000531106f, 0.001443002f, 0.08559123f, 0.0f
};
const static t_ff_gain ff_gain_long_turn_180_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002432468f, 0.000538646f, 0.001405904f, 0.08559123f, 0.0f
};
const static t_ff_gain ff_gain_in_out_45_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.000849904f, 0.000794332f, 0.000776772f, 0.08559123f, 0.000003679442f
};
const static t_ff_gain ff_gain_in_out_45_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.001129579f, 0.000934826f, 0.000928198f, 0.08559123f, 0.000003962296f
};
const static t_ff_gain ff_gain_in_out_135_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.003190175f, 0.001071977f, 0.000987594f, 0.08559123f, 0.000000389595f
};
const static t_ff_gain ff_gain_in_out_135_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.003156581f, 0.001066273f, 0.000982570f, 0.08559123f, 0.000000178623f
};
const static t_ff_gain ff_gain_v90_R_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.001678876f, 0.000675940f, 0.001023082f, 0.08559123f, 0.000000758492f
};
const static t_ff_gain ff_gain_v90_L_500 = {
	0.8700466f, 0.1147426f, 0.09459957f, 0.002037948f, 0.000650298f, 0.001040909f, 0.08559123f, 0.000001038299f
};

const static t_pid_gain sp_gain_turn90_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn90_500 = {0.10f,0.01f,0.0f};
// 2026-11-07 Mouse A, legacy/suction OFF: right-90 A/B logs in
// tools/logs/20261107_033834_turn500_tuning/comparison_90/README.md.
const static t_pid_gain om_gain_turn90_R_500 = {0.10f,0.008f,0.0f};
// 2026-11-11 機体A / turn500: R/Lの90・180度は各3本の単体調整候補として保存（未採用）。
// 共通条件: 0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF、カメラ高さ約800 mm、45 mm格子。
// 下記のコメント行は候補値。連続走行・壁あり経路での検証後に採用を判断する。
// tableの順序: velo[m/s], r_min[mm], Lstart[mm], Lend[mm], degree[deg], turn_dir, degree_correction[deg]。
// degreeは経路判定用の論理角度を保持し、角度補正は末尾のdegree_correctionへ分ける。
// RAMで確認したFFはコンパイル済み係数と精度が異なるため、形状値だけで測定条件一式とはしない。
// 以下のFF記録の順序: sp_v, sp_a, sp_bias, om_v, om_a, om_d, om_bias, jerk。turn_sp項は全候補0,0。
// L90候補: SP PID 2/0.016/0、OM PID 0.1/0.01/0、論理角+90度、補正-0.5度。
// RAM FF: {0.870046,0.114741,0.094599,0.003051,0.000741,0.000965,0.085591,0.000001971}。
// 盤面移動後の3本を同じ基準で再解析。終点平均: 左269.620 mm、前270.576 mm、最大位置誤差2.088 mm。
// 根拠: tools/logs/pixel_camera/analysis_L90_visible_20261111_021818_d714/L90_candidate.json
// const static t_turn_param_table slalom_L90_500_table = {0.50f, 37.5f,42.0f,33.7f, 90.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 37.5f,39.05,39.80, 90.0f,Turn_L};
// R90候補: SP PID 2/0.016/0、OM PID 0.1/0.008/0、論理角-90度、補正0度。
// RAM FF: {0.870046,0.114741,0.094599,0.002077,0.000836,0.000930,0.085591,0.000001569}。
// 終点平均: 右270.213 mm、前270.853 mm。各軸の最大誤差: 右0.882 mm、前1.704 mm。
// 根拠: tools/logs/pixel_camera/tuning_gui_20261111_005806_6092e3/R90_candidate.json
// const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,44.0f,35.0f,-90.0f,Turn_R,0.0f};
const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500,&ff_gain_long_turn_L_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_R_500,&ff_gain_long_turn_R_500};

const static t_pid_gain sp_gain_turn180_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turn180_500 = {0.10f,0.01f,0.0f};
// L180候補（2026-11-10最終3本）: SP PID 2/0.016/0、OM PID 0.1/0.01/0、補正0度。
// Lendの指定値16.3 mmに対し、実機の整数読み返しは16.299 mm（R180も同じ）。
// RAM FF: {0.870046,0.114741,0.094599,0.002432,0.000538,0.001405,0.085591,0}。
// 相対中点の終点平均: 左89.803 mm、前-0.513 mm。ばらつき幅: 左1.024 mm、前0.907 mm。
// 根拠: tools/logs/pixel_camera/tuning_gui_20261110_215247_d6a26b/L180_candidate.json
// const static t_turn_param_table slalom_L180_500_table = {0.50f, 44.7f,23.63f,16.3f, 180.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,23.63,24.47, 180.0f,Turn_L};
// 2026-11-10 Mouse A, R180 candidate (retained only; NOT adopted).
// Three single turns, preset 500, suction OFF, pre_accel 6.5 m/s^2, correction 0 deg.
// Requested Lend 16.3 mm; device integer readback 16.299 mm. SP PID: 2/0.016/0; OM: 0.1/0.01/0.
// Tested RAM FF readback (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.002126,0.000531,0.001443,0.085591,0}; turn_sp terms: 0,0.
// Relative midpoint mean: right +90.22 mm, forward -1.05 mm; observed ranges: 0.50/2.72 mm.
// Camera height approx. 800 mm, 45 mm grid scale. Absolute axle/route validation remains open.
// Evidence: tools/logs/pixel_camera/tuning_gui_20261110_083431_c49c98/R180_candidate.json
// Runs: run_083620_bdbc4a, run_083938_e41995, run_084249_bc1737 (CSV + settings + video).
// Geometry candidate only; the tested quantized RAM gains above differ from compiled precision.
// const static t_turn_param_table slalom_R180_500_table = {0.50f,-45.0f,23.63f,16.3f,-180.0f,Turn_R,0.0f};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500,&ff_gain_long_turn_180_L_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500,&ff_gain_long_turn_180_R_500};

const static t_pid_gain sp_gain_turnV90_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnV90_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 41.0f,6.36,12.60, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,6.36,12.60,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500,&ff_gain_v90_L_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500,&ff_gain_v90_R_500};

const static t_pid_gain sp_gain_turnIn45_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 50.0f,13.31,38.00, 45.0f,Turn_L};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,13.31,38.00,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500,&ff_gain_in_out_45_L_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500,&ff_gain_in_out_45_R_500};

const static t_pid_gain sp_gain_turnOut45_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 50.0f,19.33,32.01, 45.0f,Turn_L};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,19.33,32.01,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500,&ff_gain_in_out_45_L_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500,&ff_gain_in_out_45_R_500};

const static t_pid_gain sp_gain_turnIn135_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500,&ff_gain_in_out_135_L_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500,&ff_gain_in_out_135_R_500};

const static t_pid_gain sp_gain_turnOut135_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_500 = {0.10f,0.01f,0.0f};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 30.0f,37.57,46.07, 135.0f,Turn_L};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500,&ff_gain_in_out_135_L_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500,&ff_gain_in_out_135_R_500};

const static t_param *const mode_500[] = 	{	NULL,					NULL,			NULL,
												&param_R90_500,		&param_L90_500,
												&param_R180_500,	&param_L180_500,
												&param_inR45_500,	&param_inL45_500,
												&param_outR45_500,	&param_outL45_500,
												&param_inR135_500,	&param_inL135_500,
												&param_outR135_500,	&param_outL135_500,
												&param_RV90_500,	&param_LV90_500
											};




#endif /* CPP_PARAMS_TURN_500_H_ */
