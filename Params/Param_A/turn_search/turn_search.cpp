// Params/Param_A/turn_search/turn_search.hに対応する機体Aの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_A/run_param_A.h"

// ===== Params/Param_A/turn_search/turn_search.h =====
/* 機体Aの探索ターン設定。速度[m/s]、長さ[mm]、角度[deg]。 */

#include "Params/typedef_run_param.h"

const t_pid_gain sp_gain_search_turn =  {2.0,0.04,0.00};//{10.0,0.05,0.00};////{2.0,0.04};
const t_pid_gain om_gain_search_turn =  {0.1f,0.005f,0.00f};//{0.60f, 0.05f, 0.000f};//{0.50f, 0.0005f, 0.001f};

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



const t_ff_gain ff_gain_search_turn_R_400 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0064590f, 0.0015171f, 0.0009990f, 0.0188340f, 0.00000032f, -0.00056554f, 0.00007881f};
const t_ff_gain ff_gain_search_turn_L_400 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0072780f, 0.0014828f, 0.0010880f, 0.0076013f, 0.00000032f, -0.00012914f, 0.00005493f};
const t_ff_gain ff_gain_search_turn_R_370 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0053600f, 0.0015777f, 0.0008080f, 0.0443326f, 0.00000032f, -0.00053817f, 0.00020249f};
const t_ff_gain ff_gain_search_turn_L_370 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0065180f, 0.0014980f, 0.0010350f, 0.0148286f, 0.00000032f, -0.00016902f, 0.00012027f};
const t_ff_gain ff_gain_search_turn_R_350 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0049760f, 0.0015374f, 0.0009230f, 0.0340973f, 0.00000032f, -0.00045500f, 0.00021685f};
const t_ff_gain ff_gain_search_turn_L_350 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0060030f, 0.0014087f, 0.0010080f, 0.0199057f, 0.00000032f, -0.00018576f, 0.00013722f};
const t_ff_gain ff_gain_search_turn_R_320 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0048070f, 0.0014096f, 0.0007200f, 0.0318284f, 0.00000032f, -0.00036311f, 0.00021241f};
const t_ff_gain ff_gain_search_turn_L_320 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0056160f, 0.0012020f, 0.0009570f, 0.0222808f, 0.00000032f, -0.00016557f, 0.00009479f};
const t_ff_gain ff_gain_search_turn_R_300 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0058330f, 0.0016753f, 0.0012683f, 0.0175503f, 0.000000321f, -0.00022961f, 0.00017918f};
const t_ff_gain ff_gain_search_turn_L_300 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0064070f, 0.0015845f, 0.0013100f, 0.0144698f, 0.000000326f, -0.00006533f, 0.00005630f};
const t_ff_gain ff_gain_search_turn_R_280 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0058360f, 0.0015999f, 0.0012282f, 0.0130955f, 0.000000187f, -0.00016090f, 0.00012841f};
const t_ff_gain ff_gain_search_turn_L_280 = {FF_SEARCH_SP_VELO_COEF, FF_SEARCH_SP_ACCEL_COEF, FF_SEARCH_SP_BIAS_COEF, 0.0068350f, 0.0015320f, 0.0012199f, 0.0068564f, 0.000000468f, -0.00004510f, 0.00003266f};

const t_turn_param_table slalom_L90_table_400 = {0.40f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_400 = {0.40f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const t_param param_L90_search_400 = {&slalom_L90_table_400 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_400};
const t_param param_R90_search_400 = {&slalom_R90_table_400, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_400};

// 2026-11-16 機体A・370 mm/sの単体候補をユーザー依頼で試験用に反映。
// 速度PIDは左右とも5.0/0.04/0、角速度PIDと左右別FFは既存値を維持。
// 吸引OFF・前走加速度6.5 m/s^2、右3本/左5本の相対マーカー中点評価。
// Lstartは9.46 mm、Lendは右8.50/左9.00 mm。出口角度差は右-0.79~-0.11度、
// 左-0.45~+0.60度。左+0.7度の比較は不採用とし、論理角/補正は±90/0度。
// 根拠: tools/logs/pixel_camera/A_search370_candidate_three_20261115/README.md
//       tools/logs/pixel_camera/A_search370_left_candidate_three_20261116/README.md
// 絶対車軸精度・連続接続・壁あり探索・反映後の実機再現性は未検証。
// ターン内の前後直進にも試験時と同じPIDを指定。他速度の共有ゲインは変えない。
const t_pid_gain sp_gain_search_turn_370 = {5.0f,0.04f,0.0f};
const t_turn_param_table slalom_L90_table_370 = {0.37f, 26.00f,9.46,9.00, 90.0f,Turn_L,0.0f};
const t_turn_param_table slalom_R90_table_370 = {0.37f,-26.00f,9.46,8.50,-90.0f,Turn_R,0.0f};
const t_param param_L90_search_370 = {&slalom_L90_table_370 ,&sp_gain_search_turn_370,&om_gain_search_turn,&ff_gain_search_turn_L_370,&sp_gain_search_turn_370,&om_gain_search_turn};
const t_param param_R90_search_370 = {&slalom_R90_table_370, &sp_gain_search_turn_370,&om_gain_search_turn,&ff_gain_search_turn_R_370,&sp_gain_search_turn_370,&om_gain_search_turn};

const t_turn_param_table slalom_L90_table_350 = {0.35f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_350 = {0.35f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const t_param param_L90_search_350 = {&slalom_L90_table_350 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_350};
const t_param param_R90_search_350 = {&slalom_R90_table_350, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_350};

const t_turn_param_table slalom_L90_table_320 = {0.32f, 26.00f,9.46,11.16, 90.0f,Turn_L,0.0f};
const t_turn_param_table slalom_R90_table_320 = {0.32f,-26.00f,9.46,11.16,-90.0f,Turn_R,0.0f};
const t_param param_L90_search_320 = {&slalom_L90_table_320 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_320};
const t_param param_R90_search_320 = {&slalom_R90_table_320, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_320};

const t_turn_param_table slalom_L90_table_300 = {0.30f, 26.00f,9.46,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_300 = {0.30f,-26.00f,9.46,11.16,-90.0f,Turn_R};
const t_param param_L90_search_300 = {&slalom_L90_table_300 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_300};
const t_param param_R90_search_300 = {&slalom_R90_table_300, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_300};

const t_turn_param_table slalom_L90_table_280 = {0.28f, 26.00f,9.49,11.16, 90.0f,Turn_L};
const t_turn_param_table slalom_R90_table_280 = {0.28f,-26.00f,9.49,11.16,-90.0f,Turn_R};
const t_param param_L90_search_280 = {&slalom_L90_table_280 ,&sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_L_280};
const t_param param_R90_search_280 = {&slalom_R90_table_280, &sp_gain_search_turn,&om_gain_search_turn,&ff_gain_search_turn_R_280};

