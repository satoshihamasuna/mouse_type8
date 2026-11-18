/* 機体Aの探索ターン設定。速度[m/s]、長さ[mm]、角度[deg]。 */
#ifndef PARAM_A_TURN_SEARCH_H_
#define PARAM_A_TURN_SEARCH_H_

// 値の定義は同じディレクトリのturn_search.cppにあります。

#include "Params/typedef_run_param.h"

extern const t_pid_gain sp_gain_search_turn;//{10.0,0.05,0.00};////{2.0,0.04};
extern const t_pid_gain om_gain_search_turn;//{0.60f, 0.05f, 0.000f};//{0.50f, 0.0005f, 0.001f};

// Search-turn feedforward is kept per speed and direction so results obtained
// with myshell_debug.py can be applied independently after identification.
// Search-turn OM feedforward uses a conservative 25% learning update from
// measured closed-loop voltage.  The deceleration fit weights the final 35%
// of the angular-speed profile to reduce exit angular-velocity residue.  Jerk
// starts at 6.25% of the identified value because it is newly enabled here.
// Common non-suction SP feedforward identified from the 300, 500 and
// 700 mm/s straight logs and shared by search turns.  The final two values in
// each search-turn gain are the turn-induced common-mode correction:
// K_sp_turn_alpha * sign(target omega) * target alpha
//     + K_sp_turn_omega2 * target omega^2.
// The values below are a conservative 50% learning update from the voltage-
// response model identified with the 2026-07-21/22 baseline and trial logs.
const static float FF_SEARCH_SP_VELO_COEF = 0.4379f;
const static float FF_SEARCH_SP_ACCEL_COEF = 0.1000f;
const static float FF_SEARCH_SP_BIAS_COEF = 0.3329f;
extern const t_ff_gain ff_gain_search_turn_R_400;
extern const t_ff_gain ff_gain_search_turn_L_400;
extern const t_ff_gain ff_gain_search_turn_R_370;
extern const t_ff_gain ff_gain_search_turn_L_370;
extern const t_ff_gain ff_gain_search_turn_R_350;
extern const t_ff_gain ff_gain_search_turn_L_350;
extern const t_ff_gain ff_gain_search_turn_R_320;
extern const t_ff_gain ff_gain_search_turn_L_320;
extern const t_ff_gain ff_gain_search_turn_R_300;
extern const t_ff_gain ff_gain_search_turn_L_300;
extern const t_ff_gain ff_gain_search_turn_R_280;
extern const t_ff_gain ff_gain_search_turn_L_280;

extern const t_turn_param_table slalom_L90_table_400;
extern const t_turn_param_table slalom_R90_table_400;
extern const t_param param_L90_search_400;
extern const t_param param_R90_search_400;

// 2026-11-16 機体A・370 mm/sの単体候補をユーザー依頼で試験用に反映。
// 速度PIDは左右とも5.0/0.04/0、角速度PIDと左右別FFは既存値を維持。
// 吸引OFF・前走加速度6.5 m/s^2、右3本/左5本の相対マーカー中点評価。
// Lstartは9.46 mm、Lendは右8.50/左9.00 mm。出口角度差は右-0.79~-0.11度、
// 左-0.45~+0.60度。左+0.7度の比較は不採用とし、論理角/補正は±90/0度。
// 根拠: tools/logs/pixel_camera/A_search370_candidate_three_20261115/README.md
//       tools/logs/pixel_camera/A_search370_left_candidate_three_20261116/README.md
// 絶対車軸精度・連続接続・壁あり探索・反映後の実機再現性は未検証。
// ターン内の前後直進にも試験時と同じPIDを指定。他速度の共有ゲインは変えない。
extern const t_pid_gain sp_gain_search_turn_370;
extern const t_turn_param_table slalom_L90_table_370;
extern const t_turn_param_table slalom_R90_table_370;
extern const t_param param_L90_search_370;
extern const t_param param_R90_search_370;

extern const t_turn_param_table slalom_L90_table_350;
extern const t_turn_param_table slalom_R90_table_350;
extern const t_param param_L90_search_350;
extern const t_param param_R90_search_350;

extern const t_turn_param_table slalom_L90_table_320;
extern const t_turn_param_table slalom_R90_table_320;
extern const t_param param_L90_search_320;
extern const t_param param_R90_search_320;

extern const t_turn_param_table slalom_L90_table_300;
extern const t_turn_param_table slalom_R90_table_300;
extern const t_param param_L90_search_300;
extern const t_param param_R90_search_300;

extern const t_turn_param_table slalom_L90_table_280;
extern const t_turn_param_table slalom_R90_table_280;
extern const t_param param_L90_search_280;
extern const t_param param_R90_search_280;

#endif /* PARAM_A_TURN_SEARCH_H_ */
