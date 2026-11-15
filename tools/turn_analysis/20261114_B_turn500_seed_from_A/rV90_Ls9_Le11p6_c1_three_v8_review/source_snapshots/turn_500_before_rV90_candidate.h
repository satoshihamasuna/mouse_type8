/*
 * turn_500.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TURN_500_H_
#define CPP_PARAMS_TURN_500_H_

#include "Params/typedef_run_param.h"

const static t_pid_gain sp_gain_turn90_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turn90_500 = {0.10f,0.002f,0.0f};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 37.5f,39.05,39.80, 90.0f,Turn_L};
// 2026-11-14 機体B R90: 単体3本を確認した候補（実テーブルは既存値を維持）。
// 平均終点は右269.81/前269.77 mm（目標270/270 mm）、出口方向差最大0.43度。
// 根拠: tools/turn_analysis/20261114_B_turn500_seed_from_A/R90_corr1_Lend44_three_review/review.json
// 相対マーカー中点の評価。L90・絶対車軸位置・連続接続は未確認。
// const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,40.00f,44.00f,-90.0f,Turn_R,1.0f};
const static t_turn_param_table slalom_R90_500_table = {0.50f,-37.5f,39.05,39.80,-90.0f,Turn_R};
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};

const static t_pid_gain sp_gain_turn180_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turn180_500 = {0.10f,0.002f,0.0f};
// 2026-11-14 機体B: 右の測定値を左右対称の候補コメントとして保存。実テーブルは既存値を維持。
// R180の3本: 横移動平均90.83 mm（目標90 mm）、最大差0.79 mm、出口方向差最大0.60度。
// 根拠: tools/turn_analysis/20261114_B_turn500_seed_from_A/R180_radius44p5_three_v5/review.json
// L180は右候補の対称値で実走未確認。絶対配置・車軸位置・連続接続の検証は継続する。
// const static t_turn_param_table slalom_L180_500_table = {0.50f, 44.5f,23.63,24.47, 180.0f,Turn_L};
// const static t_turn_param_table slalom_R180_500_table = {0.50f,-44.5f,23.63,24.47,-180.0f,Turn_R};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,23.63,24.47, 180.0f,Turn_L};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,23.63,24.47,-180.0f,Turn_R};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};

const static t_pid_gain sp_gain_turnV90_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnV90_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 41.0f,6.36,12.60, 90.0f,Turn_L};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,6.36,12.60,-90.0f,Turn_R};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};

const static t_pid_gain sp_gain_turnIn45_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnIn45_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 50.0f,13.31,38.00, 45.0f,Turn_L};
// 2026-11-14 機体B in_R45: 単体3本の候補。実テーブル・左右共有ゲインは既存値を維持。
// 半径-50 mm、Lstart14/Lend36 mm、論理-45度/補正+0.5度。SP PID1.50/0.02/0、OM PID0.10/0.002/0。
// 平均終点は右135.39/前269.10 mm（目標135/270 mm）、出口進行方向差最大0.52度。
// P0.30から0.10で前後の角速度振動が減少。旋回中の追従遅れは残る。左45入口・連続接続は未検証。
// 根拠: tools/turn_analysis/20261114_B_turn500_seed_from_A/inR45_kp01_Lstart14_three_review/review.json
// const static t_pid_gain om_gain_inR45_500_candidate = {0.10f,0.002f,0.0f};
// const static t_turn_param_table slalom_inR45_500_candidate_table = {0.50f,-50.0f,14.00f,36.00f,-45.0f,Turn_R,0.5f};
// const static t_param param_inR45_500_candidate = {&slalom_inR45_500_candidate_table,&sp_gain_turnIn45_500,&om_gain_inR45_500_candidate};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-50.0f,13.31,38.00,-45.0f,Turn_R};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};

const static t_pid_gain sp_gain_turnOut45_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnOut45_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 50.0f,19.33,32.01, 45.0f,Turn_L};
// 2026-11-15 機体B out_R45: 同じ2倍撮影の単体3本を確認した候補。実テーブル・共有ゲインは既存値を維持。
// 半径-50 mm、Lstart32.8/Lend18.0 mm、論理-45度/補正+0.5度。SP PID1.50/0.02/0、OM PID0.10/0.002/0。
// 平均終点は右95.71/前285.64 mm（目標95.46/286.38 mm）、出口方向差最大0.24度。
// BのP0.30から0.10で角速度振動が減少。左45出口・絶対車軸位置・連続接続は未検証。BのFFは実機設定JSONを参照。
// 実測入口方向に合わせた相対評価。盤面固定の設置・進入・出口位置の誤差は別途評価する。
// 根拠: tools/turn_analysis/20261114_B_turn500_seed_from_A/outR45_lengths_three_review/review.json
// const static t_pid_gain om_gain_outR45_500_candidate = {0.10f,0.002f,0.0f};
// const static t_turn_param_table slalom_outR45_500_candidate_table = {0.50f,-50.0f,32.80f,18.00f,-45.0f,Turn_R,0.5f};
// const static t_param param_outR45_500_candidate = {&slalom_outR45_500_candidate_table,&sp_gain_turnOut45_500,&om_gain_outR45_500_candidate};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-50.0f,19.33,32.01,-45.0f,Turn_R};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};

const static t_pid_gain sp_gain_turnIn135_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnIn135_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 30.0f,45.29+5,38.35, 135.0f,Turn_L};
// 2026-11-15 機体B in_R135: 単体3本（1倍2本/2倍1本）の候補。実テーブル・左右共有ゲインは既存値を維持。
// 半径-32.3 mm、Lstart44/Lend38.35 mm、論理-135度/補正+0.5度。SP PID1.50/0.02/0、OM PID0.10/0.002/0。
// 平均終点は右179.53/前44.22 mm（目標180/45 mm）、出口方向差最大0.70度。
// 撮影条件差・機体内部画像による設置補助確認は根拠資料を参照。左135入口・絶対車軸位置・連続接続は未検証。
// 根拠: tools/turn_analysis/20261114_B_turn500_seed_from_A/inR135_r32p3_Lstart44_three_review/review.json
// 2026-11-15追記: 2倍の旧検出に基板上の黄色を前マーカーと誤認した例あり。連続検証前に保存映像をv8で再点検する。
// const static t_pid_gain om_gain_inR135_500_candidate = {0.10f,0.002f,0.0f};
// const static t_turn_param_table slalom_inR135_500_candidate_table = {0.50f,-32.3f,44.00f,38.35f,-135.0f,Turn_R,0.5f};
// const static t_param param_inR135_500_candidate = {&slalom_inR135_500_candidate_table,&sp_gain_turnIn135_500,&om_gain_inR135_500_candidate};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-30.0f,45.29+5,38.35,-135.0f,Turn_R};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};

const static t_pid_gain sp_gain_turnOut135_500 = {1.50,0.02,0.0};
const static t_pid_gain om_gain_turnOut135_500 = {0.30f,0.002f,0.0f};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 30.0f,37.57,46.07, 135.0f,Turn_L};
// 2026-11-15 機体B out_R135: 同じ2倍撮影・検出v8で単体3本の候補。実テーブル・共有ゲインは既存値を維持。
// 半径-32.3 mm、Lstart33.7/Lend46.07 mm、論理-135度/補正+1.5度。SP PID1.50/0.02/0、OM PID0.10/0.002/0。
// 平均終点は右159.59/前94.73 mm（目標159.10/95.46 mm）、各軸最大誤差0.63/0.90 mm、出口方向差最大0.08度。
// 実測入口方向に対する相対形状評価。盤面固定の停止点・絶対車軸位置・左135出口・連続接続は未検証。
// 黄緑に映る前黄マーカーを検出するv8を別映像で実写照合し、先行2本も同じ検出で再解析。旧失敗結果を保持。
// 根拠: tools/turn_analysis/20261114_B_turn500_seed_from_A/outR135_corr1p5_Lstart33p7_three_v8_review/review.json
// Bの全FFは B_outR135_single_turn_candidate.settings.json を参照。
// const static t_pid_gain om_gain_outR135_500_candidate = {0.10f,0.002f,0.0f};
// const static t_turn_param_table slalom_outR135_500_candidate_table = {0.50f,-32.3f,33.70f,46.07f,-135.0f,Turn_R,1.5f};
// const static t_param param_outR135_500_candidate = {&slalom_outR135_500_candidate_table,&sp_gain_turnOut135_500,&om_gain_outR135_500_candidate};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-30.0f,37.57,46.07,-135.0f,Turn_R};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};

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
