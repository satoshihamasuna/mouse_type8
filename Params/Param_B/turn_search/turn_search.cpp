// Params/Param_B/turn_search/turn_search.hに対応する機体Bの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_B/run_param_B.h"

// ===== Params/Param_B/turn_search/turn_search.h =====
/* 機体Bの探索ターン設定。速度[m/s]、長さ[mm]、角度[deg]。 */

#include "Params/typedef_run_param.h"

const t_pid_gain sp_gain_search_turn = {1.200,0.02,0.0};//{1.0,0.04,0.05};;//{2.0,0.04};
const t_pid_gain om_gain_search_turn = {0.1f,0.0015f,0.00f};//{0.50f, 0.0005f, 0.001f};

// 2026-11-21 機体B・370 mm/sの単体候補。吸引OFF、前後45 mm、加減速度6.5 m/s^2。
// 右/左各3本の実測入口相対マーカー中点で、終端各軸2 mm・出口方向1.5度以内。
// 速度PIDは370専用。旋回加減速FFはB右の閉ループ推定から50%更新し、左も実走確認。
// 他のFF項はBの既定値を保持。ターン内の入口・出口直進にも試験時のPIDを指定。
// 根拠: tools/turn_analysis/20261121_B_search370_adoption/README.md
// 絶対車軸位置・機体yaw、連続接続・壁あり探索、探索速度のEKF同定は未検証。
const t_pid_gain sp_gain_search_turn_370 = {5.0f,0.02f,0.0f};
const t_ff_gain ff_gain_search_turn_R_370 = {FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, FF_OM_VELO_COEF, 0.000690f, 0.000555f, FF_OM_BIAS_COEF};
const t_ff_gain ff_gain_search_turn_L_370 = {FF_SP_VELO_COEF, FF_SP_ACCEL_COEF, FF_SP_BIAS_COEF, FF_OM_VELO_COEF, 0.000690f, 0.000555f, FF_OM_BIAS_COEF};
const t_turn_param_table slalom_L90_table_370 = {0.37f, 26.00f,7.30,11.50, 90.0f,Turn_L,0.0f};
const t_turn_param_table slalom_R90_table_370 = {0.37f,-26.00f,8.60,10.40,-90.0f,Turn_R,-1.0f};
const t_param param_L90_search_370 = {&slalom_L90_table_370 ,&sp_gain_search_turn_370,&om_gain_search_turn,&ff_gain_search_turn_L_370,&sp_gain_search_turn_370,&om_gain_search_turn};
const t_param param_R90_search_370 = {&slalom_R90_table_370, &sp_gain_search_turn_370,&om_gain_search_turn,&ff_gain_search_turn_R_370,&sp_gain_search_turn_370,&om_gain_search_turn};

// 350 mm/sはBの320から速度のみ変更した未調整の初期値。
const t_turn_param_table slalom_L90_table_350 = {0.35f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_350 = {0.35f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const t_param param_L90_search_350 = {&slalom_L90_table_350 ,&sp_gain_search_turn,&om_gain_search_turn};
const t_param param_R90_search_350 = {&slalom_R90_table_350, &sp_gain_search_turn,&om_gain_search_turn};

const t_turn_param_table slalom_L90_table_320 = {0.32f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_320 = {0.32f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const t_param param_L90_search_320 = {&slalom_L90_table_320 ,&sp_gain_search_turn,&om_gain_search_turn};
const t_param param_R90_search_320 = {&slalom_R90_table_320, &sp_gain_search_turn,&om_gain_search_turn};

const t_turn_param_table slalom_L90_table_300 = {0.30f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_300 = {0.30f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const t_param param_L90_search_300 = {&slalom_L90_table_300 ,&sp_gain_search_turn,&om_gain_search_turn};
const t_param param_R90_search_300 = {&slalom_R90_table_300, &sp_gain_search_turn,&om_gain_search_turn};

const t_turn_param_table slalom_L90_table_280 = {0.28f, 26.00f,9.49,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_280 = {0.28f,-26.00f,9.49,11.16,-90.0f,Turn_R};
const t_param param_L90_search_280 = {&slalom_L90_table_280 ,&sp_gain_search_turn,&om_gain_search_turn};
const t_param param_R90_search_280 = {&slalom_R90_table_280, &sp_gain_search_turn,&om_gain_search_turn};

