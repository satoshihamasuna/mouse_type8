// Params/Param_B/turn_search/turn_search.hに対応する機体Bの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_B/run_param_B.h"

// ===== Params/Param_B/turn_search/turn_search.h =====
/* 機体Bの探索ターン設定。速度[m/s]、長さ[mm]、角度[deg]。 */

#include "Params/typedef_run_param.h"

const t_pid_gain sp_gain_search_turn = {1.200,0.02,0.0};//{1.0,0.04,0.05};;//{2.0,0.04};
const t_pid_gain om_gain_search_turn = {0.1f,0.0015f,0.00f};//{0.50f, 0.0005f, 0.001f};
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

