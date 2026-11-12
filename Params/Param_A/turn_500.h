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
// 2026-11-12 機体A L_V90（l_v90）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径+41.0 mm、Lstart 9.3 mm、Lend 5.1 mm、論理角+90度、角度補正-0.5度。
// table末尾はdegree_correction。論理角は+90度のまま、生成角は+89.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// F/R基準から位置差3 mm以内・向き差1度以内を、静止試写と走行直前の全フレームで確認。
// 前走127.2792 mm（斜め）・後走63.6396 mm（斜め）。絶対車軸位置は未検証。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.002037,0.000650,0.001040,0.085591,0.000001038}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 左128.319 mm、前190.885 mm。最大位置誤差1.141 mm。
// 目標比の平均ずれ: 右-1.040 mm、前-0.034 mm。出口進行方向の最大誤差0.294度。
// 連続走行・壁あり経路・独立240露光/秒は未検証。
// 根拠: tools/logs/pixel_camera/cli_l_v90_lstart9_3_repeat03_ready_20261112_043327/l_v90_candidate.json
// const static t_turn_param_table slalom_LV90_500_table = {0.50f,41.0f,9.3f,5.1f,90.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 41.0f,6.36,12.60, 90.0f,Turn_L};
// 2026-11-11 機体A R_V90（r_v90）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径-41.0 mm、Lstart 10.6 mm、Lend 3.6 mm、論理角-90度、角度補正+0.5度。
// table末尾はdegree_correction。論理角は-90度のまま、生成角は-89.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// 開始位置・向きはF/R画像を目安にそろえ、静止試写と全フレーム解析を確認。
// 前走127.2792 mm（斜め）・後走63.6396 mm（斜め）。絶対車軸位置は未検証。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.001678,0.000675,0.001023,0.085591,0.000000758}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 右126.756 mm、前191.837 mm。最大位置誤差1.420 mm。
// 目標比の平均ずれ: 右-0.523 mm、前+0.918 mm。出口進行方向の最大誤差0.154度。
// 絶対車軸位置・連続走行・壁あり経路は未検証。
// 根拠: tools/logs/pixel_camera/cli_r_v90_10_6_3_6_c05_repeat03_ready_20261111_182538/r_v90_candidate.json
// const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,10.6f,3.6f,-90.0f,Turn_R,0.5f};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,6.36,12.60,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500,&ff_gain_v90_L_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500,&ff_gain_v90_R_500};

const static t_pid_gain sp_gain_turnIn45_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn45_500 = {0.10f,0.01f,0.0f};
// 2026-11-11 機体A in_L45（in_l45）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径+50.0 mm、Lstart 20.0 mm、Lend 25.0 mm、論理角+45度、角度補正0度。
// 撮影: 高さ約800 mm、露出-2.0 EV、AF固定を各録画の撮影結果で確認。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.001129,0.000934,0.000928,0.085591,0.000003962}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、上のコンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 左134.463 mm、前269.669 mm。最大位置誤差1.622 mm。
// 出口進行方向の最大誤差0.468度。絶対位置・連続走行・壁あり経路は未検証。
// 根拠: tools/logs/pixel_camera/cli_lstart20_20261111_130431/in_l45_candidate.json
// const static t_turn_param_table slalom_inL45_500_table = {0.50f,50.0f,20.0f,25.0f,45.0f,Turn_L,0.0f};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 50.0f,13.31,38.00, 45.0f,Turn_L};
// 2026-11-11 機体A in_R45（in_r45）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径-50.0 mm、Lstart 15.3 mm、Lend 27.9 mm、論理角-45度、角度補正+0.5度。
// table末尾はdegree_correction。論理角degreeは-45度のまま保持する。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.000849,0.000794,0.000776,0.085591,0.000003679}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、上のコンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 右134.453 mm、前268.709 mm。最大位置誤差2.037 mm。
// 前方平均1.291 mmの不足が残る。連続走行・壁あり経路は未検証。
// 根拠: tools/logs/pixel_camera/tuning_gui_20261111_030907_85c2f9/in_r45_candidate.json
// const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,15.3f,27.9f,-45.0f,Turn_R,0.5f};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,13.31,38.00,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500,&ff_gain_in_out_45_L_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500,&ff_gain_in_out_45_R_500};

const static t_pid_gain sp_gain_turnOut45_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut45_500 = {0.10f,0.01f,0.0f};
// 2026-11-11 機体A out_L45（out_l45）: 開始姿勢をそろえた単体3本の調整候補（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径+50.0 mm、Lstart 33.9 mm、Lend 11.8 mm、論理角+45度、角度補正-0.5度。
// table末尾はdegree_correction。論理角は+45度のまま、生成角は+44.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// 基準の目印姿勢から位置差3 mm以内・向き差1度以内を通電後の試写と走行直前動画で確認。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.001129,0.000934,0.000928,0.085591,0.000003962}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 左95.934 mm、前286.522 mm。最大位置誤差0.728 mm。
// 目標比の平均ずれ: 右-0.475 mm、前+0.143 mm。出口進行方向の最大誤差0.459度。
// 過去の広い開始姿勢条件の結果・不成立試写も保存。絶対車軸位置・連続走行・壁あり経路は未検証。
// 根拠: tools/logs/pixel_camera/cli_out_l45_cminus05_pose_repeat03_retry_ready_20261111_171826/out_l45_candidate.json
// const static t_turn_param_table slalom_outL45_500_table = {0.50f,50.0f,33.9f,11.8f,45.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 50.0f,19.33,32.01, 45.0f,Turn_L};
// 2026-11-11 機体A out_R45（out_r45）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径-50.0 mm、Lstart 37.2 mm、Lend 9.2 mm、論理角-45度、角度補正+0.5度。
// table末尾はdegree_correction。論理角は-45度のまま、生成角は-44.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.000849,0.000794,0.000776,0.085591,0.000003679}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 右95.779 mm、前288.627 mm。最大位置誤差2.503 mm。
// 目標比の平均ずれ: 右+0.319 mm、前+2.248 mm。出口進行方向の最大誤差0.239度。
// 絶対車軸位置・連続走行・壁あり経路は未検証。
// 根拠: tools/logs/pixel_camera/cli_out_r45_c05_repeat03_ready_20261111_153140/out_r45_candidate.json
// const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,37.2f,9.2f,-45.0f,Turn_R,0.5f};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,19.33,32.01,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500,&ff_gain_in_out_45_L_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500,&ff_gain_in_out_45_R_500};

const static t_pid_gain sp_gain_turnIn135_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnIn135_500 = {0.10f,0.01f,0.0f};
// 2026-11-12 機体A in_L135（in_l135）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径+30.0 mm、Lstart 48.2 mm、Lend 36.5 mm、論理角+135度、角度補正-0.5度。
// table末尾はdegree_correction。論理角は+135度のまま、生成角は+134.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// 開始点を原点化し、旋回前の直進方向へ回転して比較。終点や出口方向への合わせ込みはしない。
// 静止認識と走行直前の姿勢を確認。前走90 mm（直進）・後走127.2792 mm（斜め）。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.003156,0.001066,0.000982,0.085591,0.000000178}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 右-180.055 mm、前45.697 mm。最大位置誤差2.145 mm。
// 目標比の平均ずれ: 右-0.055 mm、前+0.697 mm。出口進行方向の最大誤差0.304度。
// 格子補正: fixed_indices。補正用点と独立検査点の分離・全走行区間の品質を確認。
// 3本の終点間最大差4.126 mm。ばらつき原因は未分離、追加確認が必要。
// 絶対車軸位置・連続走行・壁あり経路・独立240露光/秒は未検証。
// 根拠: tools/logs/pixel_camera/cli_in_l135_cminus05_repeat03_ready_20261112_071201/in_l135_candidate.json
// const static t_turn_param_table slalom_inL135_500_table = {0.50f,30.0f,48.2f,36.5f,135.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
// 2026-11-12 機体A in_R135（in_r135）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径-30.0 mm、Lstart 47.8 mm、Lend 38.35 mm、論理角-135度、角度補正+0.5度。
// table末尾はdegree_correction。論理角は-135度のまま、生成角は-134.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// 開始点を原点化し、旋回前の直進方向へ回転して比較。終点や出口方向への合わせ込みはしない。
// 静止認識と走行直前の姿勢を確認。前走90 mm（直進）・後走127.2792 mm（斜め）。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.003190,0.001071,0.000987,0.085591,0.000000389}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 右180.239 mm、前44.852 mm。最大位置誤差1.078 mm。
// 目標比の平均ずれ: 右+0.239 mm、前-0.148 mm。出口進行方向の最大誤差0.113度。
// 絶対車軸位置・連続走行・壁あり経路・独立240露光/秒は未検証。
// 根拠: tools/logs/pixel_camera/cli_in_r135_lstart47_8_repeat03_ready_20261112_053833/in_r135_candidate.json
// const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,47.8f,38.35f,-135.0f,Turn_R,0.5f};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500,&ff_gain_in_out_135_L_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500,&ff_gain_in_out_135_R_500};

const static t_pid_gain sp_gain_turnOut135_500 = {2.0,0.016,0.00};
const static t_pid_gain om_gain_turnOut135_500 = {0.10f,0.01f,0.0f};
// 2026-11-12 機体A out_L135（out_l135）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径+30.0 mm、Lstart 42.9 mm、Lend 46.07 mm、論理角+135度、角度補正-0.5度。
// table末尾はdegree_correction。論理角は+135度のまま、生成角は+134.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// 開始点を原点化し、旋回前の直進方向へ回転して比較。終点や出口方向への合わせ込みはしない。
// 静止認識と走行直前の姿勢を確認。前走127.2792 mm（斜め）・後走90 mm（直進）。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.003156,0.001066,0.000982,0.085591,0.000000178}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 右-160.593 mm、前95.608 mm。最大位置誤差1.958 mm。
// 目標比の平均ずれ: 右-1.494 mm、前+0.149 mm。出口進行方向の最大誤差0.487度。
// 配置の角度基準は盤面の斜め45度。保存OFF姿勢との差は診断値として保持。
// 格子補正fixed_indices。補正点と独立検査点の分離・全走行区間の品質を確認。
// 3本の終点間最大差1.792 mm。電圧一定の反復検証は未実施。
// 出口角度は許容範囲の端に近い。正式採用前に角度の余裕と連続接続を追加確認する。
// 絶対車軸位置・連続走行・壁あり経路・独立240露光/秒は未検証。
// 根拠: tools/logs/pixel_camera/cli_out_l135_lstart42_9_repeat03_ready_20261112_092618/out_l135_candidate.json
// const static t_turn_param_table slalom_outL135_500_table = {0.50f,30.0f,42.9f,46.07f,135.0f,Turn_L,-0.5f};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 30.0f,37.57,46.07, 135.0f,Turn_L};
// 2026-11-12 機体A out_R135（out_r135）: 単体3本の調整候補として保存（未採用）。
// 条件: 500プリセット、0.50 m/s、pre_accel 6.5 m/s^2、1回ずつ、吸引OFF。
// 半径-30.0 mm、Lstart 40.7 mm、Lend 46.07 mm、論理角-135度、角度補正+0.5度。
// table末尾はdegree_correction。論理角は-135度のまま、生成角は-134.5度。
// 撮影: 高さ約800 mm、露出-1.5 EV、AF固定を各録画の撮影結果で確認。
// 開始点を原点化し、旋回前の直進方向へ回転して比較。終点や出口方向への合わせ込みはしない。
// 静止認識と走行直前の姿勢を確認。前走127.2792 mm（斜め）・後走90 mm（直進）。
// SP PID 2/0.016/0、OM PID 0.1/0.01/0。
// RAM FF (sp_v,sp_a,sp_bias,om_v,om_a,om_d,om_bias,jerk):
// {0.870046,0.114741,0.094599,0.003190,0.001071,0.000987,0.085591,0.000000389}; turn_sp項: 0,0。
// 測定時の量子化済みRAM FFは、コンパイル済み係数と精度が異なる。
// 相対目印中点の終点平均: 右158.567 mm、前94.801 mm。最大位置誤差1.693 mm。
// 目標比の平均ずれ: 右-0.532 mm、前-0.658 mm。出口進行方向の最大誤差0.333度。
// 配置の角度基準は盤面の斜め135度。保存OFF姿勢との差は診断値として保持。
// 格子補正fixed_indices。3本目は充電・交換後。走行前の低電圧中断は本数に含めない。
// 3本の終点間最大差1.515 mm。電圧一定の反復検証は未実施。
// 絶対車軸位置・連続走行・壁あり経路・独立240露光/秒は未検証。
// 根拠: tools/logs/pixel_camera/cli_out_r135_lstart40_7_aftercharge_ready_20261112_082139/out_r135_candidate.json
// const static t_turn_param_table slalom_outR135_500_table = {0.50f,-30.0f,40.7f,46.07f,-135.0f,Turn_R,0.5f};
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
