// Params/Param_A/turn_1600/turn_1600.hに対応する機体Aの共有定義。
// 機体別の設定フラグと参照先の宣言を読み、各定数を一度だけリンクする。
#include <cstddef>
#include "Params/Param_A/run_param_A.h"

// ===== Params/Param_A/turn_1600/turn_1600.h =====
/*
 * turn_1600.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */


#include "Params/typedef_run_param.h"

// Long-90 OM feedforward adjusted from the 20260720 voltage-to-response
// analysis. OM velocity and signed bias retain their verified values because a
// single fixed-speed profile cannot identify them independently. Acceleration,
// deceleration, and jerk also account for turn-body tracking and post-turn
// angular-velocity ringing instead of fitting PID compensation alone.
// R/L-specific SP turn feedforward was identified from the 20260722 long-180
// voltage response.  The directional baseline is shared by the turn shapes.
// Long-180 itself uses a second 50% iterative correction from the repeated
// FF-on logs; this avoids projecting the stronger correction into unmeasured
// short shapes and keeps the long-180 voltage below the common 1.0 V clamp.
const t_ff_gain ff_gain_long_turn_R_1600 = {
	0.9030126f, 0.08183161f, 0.2379401f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const t_ff_gain ff_gain_long_turn_L_1600 = {
	0.9030126f, 0.08183161f, 0.2400677f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};

// The long-90 R/L gains provide the 1600 mm/s yaw-plant baseline. Short V90
// and in45 profiles have dedicated acceleration/deceleration corrections from
// their measured response; the remaining shapes retain the baseline first try.
// See tools/turn_analysis/1600/profile_projection.csv for resulting voltages.
const t_ff_gain ff_gain_long_turn_180_R_1600 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00024186f, 0.00008612f
};
const t_ff_gain ff_gain_long_turn_180_L_1600 = {
	0.9030126f, 0.08101898f, 0.2138578f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00023491f, 0.00010562f
};

const t_ff_gain ff_gain_v90_R_1600 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00093f, 0.00090f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const t_ff_gain ff_gain_v90_L_1600 = {
	0.9030126f, 0.07681955f, 0.2104154f, 0.0168f, 0.00099f, 0.00091f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const t_ff_gain ff_gain_in45_R_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00098f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const t_ff_gain ff_gain_in45_L_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00099f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const t_ff_gain ff_gain_out45_R_1600 = {
	0.9030126f, 0.09295480f, 0.04174514f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const t_ff_gain ff_gain_out45_L_1600 = {
	0.9030126f, 0.08297866f, 0.08061591f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const t_ff_gain ff_gain_in135_R_1600 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const t_ff_gain ff_gain_in135_L_1600 = {
	0.9030126f, 0.08105824f, 0.1159204f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};
const t_ff_gain ff_gain_out135_R_1600 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00088f, 0.00085f, 0.0613f, 0.0000032f, -0.00019277f, 0.00003695f
};
const t_ff_gain ff_gain_out135_L_1600 = {
	0.9030126f, 0.08607196f, 0.09289980f, 0.0168f, 0.00094f, 0.00085f, 0.0175f, 0.0000029f, -0.00018248f, 0.00005183f
};

// Lstart/Lend are the median ground-track values from the 20260724
// Lstart=Lend=0 marker videos.  The video trajectory was synchronized to the
// logged angular profile.  Long-180 additionally places its apex at 95 mm.
//k = 250
const t_pid_gain sp_gain_turn90_1600 = {2.0,0.016,0.00};
const t_pid_gain om_gain_turn90_1600 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.95,30.5+15.0, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.95,30.5+15.0,-90.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,14.69,34.06, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,14.69,34.06,-90.0f,Turn_R,0.0f};
// 以前の定義（撮影方式の切替を含めて保持）。
// #if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260724 Lstart=Lend=0 videos, R median copied to L/R).
// const t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,15.97f,27.58f, 90.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,15.97f,27.58f,-90.0f,Turn_R,0.0f};
// #else
// const t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,18.42f,26.31f, 90.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,18.42f,26.31f,-90.0f,Turn_R,0.0f};
// #endif
// 2026-11-18 機体A・右90°の相対マーカー軌道3本から採用。吸引650、前走加速度6.5 m/s^2。
// 根拠: tools/logs/pixel_camera/turn1600_A_r90_20261118_frame18_v1/repeatability_corr1p5_Ls12p0_03/video_comparison.json
// 平均直線角90.12°、出口線の入口方向位置差+0.58 mm。左への同値展開はユーザー指定で未走行確認。
// degree + degree_correctionで物理指令角を生成するため、左の補正は右と逆符号。
// 左右90°は撮影方式フラグによらず採用値を使い、方向別FFは既存の校正値を参照する。
const t_turn_param_table slalom_L90_1600_table = {1.60f, 52.0f,12.0f,26.31f, 90.0f,Turn_L,-1.5f};
const t_turn_param_table slalom_R90_1600_table = {1.60f,-52.0f,12.0f,26.31f,-90.0f,Turn_R, 1.5f};

const t_param param_L90_1600 = {&slalom_L90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600,&ff_gain_long_turn_L_1600};
const t_param param_R90_1600 = {&slalom_R90_1600_table,&sp_gain_turn90_1600,&om_gain_turn90_1600,&ff_gain_long_turn_R_1600};

const t_pid_gain sp_gain_turn180_1600 ={2.0,0.016,0.00};
const t_pid_gain om_gain_turn180_1600 ={0.20,0.012,0.0};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,8.80,30.10, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,8.80,30.10,-180.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,10.79,32.40, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,10.79,32.40,-180.0f,Turn_R,0.0f};
/*20260725_*/
//const static t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,22.68f,26.33f, 180.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,22.68f,26.33f,-180.0f,Turn_R,0.0f};
// 以前の定義（撮影方式の切替を含めて保持）。
// #if PARAM_A_USE_PINK_LED_TURN_LENGTHS
// Pink LED interval (20260726 Lstart=Lend=0 videos, R median copied to L/R).
// const t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,13.12f,27.38f, 180.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,13.12f,27.38f,-180.0f,Turn_R,0.0f};
// #else
// const t_turn_param_table slalom_L180_1600_table = {1.60f, 48.0f,13.12,27.38, 180.0f,Turn_L};
// const t_turn_param_table slalom_R180_1600_table = {1.60f,-48.0f,13.12,27.38,-180.0f,Turn_R};
// #endif
// 左180°はユーザー指定で右の調整値を適用。左の実走は未確認。
// 論理角+180°に-2°を加えて物理指令角+178°を生成する。
const t_turn_param_table slalom_L180_1600_table = {1.60f, 43.0f,13.12f,27.38f, 180.0f,Turn_L,-3.0f};

// 2026-11-19 機体A・右180°の単体走行3本から採用。速度1.6 m/s、吸引duty 650。
// 進入直線基準で旋回角179.73～179.92°、折返し幅89.39～91.54 mm、頂点184.32～185.00 mm。
// 論理角-180°に+2°を加えて物理指令角-178°を生成する。
// 旋回中の横滑り角βの中央値は動画14.3～14.9°、内部ログ11.8～12.0°。
// 横/前速度比はtan(β)で、動画25.5～26.7%、内部ログ21.0～21.3%。
// 横加速度モデル ay=-K*β の検証候補K=191.34 m/s^2。吸引時の採用値は254.90725 m/s^2。
// 旋回開始直後の割り込み超過でEKFが中断されたため、継続適用と位置精度を実機で要確認。
// 測定: tools/logs/pixel_camera/turn1600_A_r180_20261118_v1/corr2p0_radius45_repeatability_03/compare.html
// 再生: tools/logs/pixel_camera/turn1600_A_r180_20261118_v1/corr2p0_radius45_slip_candidates_20261119/compare.html
const t_turn_param_table slalom_R180_1600_table = {1.60f,-45.0f,13.12f,27.38f,-180.0f,Turn_R,2.0f};

const t_param param_L180_1600 = {&slalom_L180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600,&ff_gain_long_turn_180_L_1600};
const t_param param_R180_1600 = {&slalom_R180_1600_table,&sp_gain_turn180_1600,&om_gain_turn180_1600,&ff_gain_long_turn_180_R_1600};

//k = 250
const t_pid_gain sp_gain_turnV90_1600 = {2.0,0.016,0.00};
const t_pid_gain om_gain_turnV90_1600 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.79,24.95-10.0, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.79,24.95-10.0,-90.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,4.51,23.78, 90.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,4.51,23.78,-90.0f,Turn_R,0.0f};
// 旧値。上2行は2026-07-24のピンクLED区間、下2行は従来の既定値。
// const t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,6.87f,21.41f, 90.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,6.87f,21.41f,-90.0f,Turn_R,0.0f};
// const t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,6.49f,20.83f, 90.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,6.49f,20.83f,-90.0f,Turn_R,0.0f};
// 2026-11-19 機体Aの右V90単体試走1本から採用。吸引duty 650、斜め後走の減速20 m/s^2。
// 実測旋回角90.99°、残るLstart補正量+1.04 mm（実測進入方向基準）。
// 測定資料 tools/logs/pixel_camera/turn1600_A_rv90_20261119_v1/next_Ls2p2_c1p6/comparison.html
// 左は右と同じ距離・PIDを使い、半径・論理角・角度補正の符号を反転する。FFは左右それぞれの係数を使う。
// 左の実走は未確認。
const t_turn_param_table slalom_LV90_1600_table = {1.60f, 40.0f,2.20f,20.83f, 90.0f,Turn_L,-1.60f};
const t_turn_param_table slalom_RV90_1600_table = {1.60f,-40.0f,2.20f,20.83f,-90.0f,Turn_R, 1.60f};
const t_param param_LV90_1600 = {&slalom_LV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_v90_L_1600};
const t_param param_RV90_1600 = {&slalom_RV90_1600_table,&sp_gain_turnV90_1600,&om_gain_turnV90_1600,&ff_gain_v90_R_1600};

//k = 250
const t_pid_gain sp_gain_turnIn45_1600 = {2.0,0.016,0.00};
const t_pid_gain om_gain_turnIn45_1600 = {0.120,0.004,0.0};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,7.52,39.62+15.0, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,7.52,39.62+15.0,-45.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,25.41,23.05, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,25.41,23.05,-45.0f,Turn_R,0.0f};
// 旧45°入口（ピンクLED）。
// const t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,8.81f,40.24f, 45.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,8.81f,40.24f,-45.0f,Turn_R,0.0f};
// 旧45°入口（角速度同期）。
// const t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,6.46f,39.86f, 45.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,6.46f,39.86f,-45.0f,Turn_R,0.0f};

// 2026-11-19 機体A・右45°入口の単体走行3本から採用。1.6 m/s、吸引duty 650、前走加速度6.5 m/s^2。
// 進入直線基準で出口直線を目標点に合わせる前後補正は-1.68～+1.34 mm、平均-0.21 mm。
// Lend終端の推定は動画とログの時刻合わせに敏感なため、39.86 mmを使用。
// 測定: tools/logs/pixel_camera/turn1600_A_inr45_20261119_v1/next_Lstart5p8/confirm_repeats/compare.html
// 左は右の測定値を適用し、半径・論理角・角度補正の符号を反転する。
const t_turn_param_table slalom_inL45_1600_table = {1.60f, 55.0f,5.80f,39.86f, 45.0f,Turn_L, 0.90f};
const t_turn_param_table slalom_inR45_1600_table = {1.60f,-55.0f,5.80f,39.86f,-45.0f,Turn_R,-0.90f};

const t_param param_inL45_1600 = {&slalom_inL45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_in45_L_1600};
const t_param param_inR45_1600 = {&slalom_inR45_1600_table,&sp_gain_turnIn45_1600,&om_gain_turnIn45_1600,&ff_gain_in45_R_1600};

//k = 250
const t_pid_gain sp_gain_turnOut45_1600 = {2.0,0.016,0.00};
const t_pid_gain om_gain_turnOut45_1600 = {0.12,0.004,0.0};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.06,20.07, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.06,20.07,-45.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.00,20.61, 45.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.00,20.61,-45.0f,Turn_R,0.0f};
// 旧45°出口（ピンクLED）。
// const t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,24.62f,18.06f, 45.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,24.62f,18.06f,-45.0f,Turn_R,0.0f};
// 旧45°出口（角速度同期）。
// const t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.74f,20.46f, 45.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.74f,20.46f,-45.0f,Turn_R,0.0f};

// 2026-11-19 機体A・右45°出口の単体走行4本から暫定採用。1.6 m/s、吸引duty 650、前走加速度6.5 m/s^2。
// 実測旋回角44.75～45.74°。公称開始姿勢に換算したLend終端は盤面目標(135,225) mmから約2 mm以内。
// LED同期での進行方向の推定幅は約7 mm。前後の直線への接続は連続走行で要確認。
// 測定: tools/logs/pixel_camera/turn1600_A_outr45_20261119_v1/target135_225_candidate_v1/entry135_recheck_v1/compare.html
const t_turn_param_table slalom_outL45_1600_table = {1.60f, 60.0f,21.80f,16.50f, 45.0f,Turn_L,-0.30f};
const t_turn_param_table slalom_outR45_1600_table = {1.60f,-60.0f,21.80f,16.50f,-45.0f,Turn_R,0.30f};

const t_param param_outL45_1600 = {&slalom_outL45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600,&ff_gain_out45_L_1600};
const t_param param_outR45_1600 = {&slalom_outR45_1600_table,&sp_gain_turnOut45_1600,&om_gain_turnOut45_1600,&ff_gain_out45_R_1600};

//k = 250,alpha = 1.0
const t_pid_gain sp_gain_turnIn135_1600 = {2.0,0.016,0.00};
const t_pid_gain om_gain_turnIn135_1600 = {0.2,0.01,0.0};//{0.7f, 0.7f, 0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,8.23+5.0,14.76+10.0, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,8.23+5.0,14.76+10.0,-135.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.0f,11.60,25.47, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.0f,11.60,25.47,-135.0f,Turn_R,0.0f};
// 旧135°入口（ピンクLED）。
// const t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,15.09f,26.35f, 135.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,15.09f,26.35f,-135.0f,Turn_R,0.0f};
// 旧135°入口（角速度同期）。
// const t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,17.43f,20.56f, 135.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,17.43f,20.56f,-135.0f,Turn_R,0.0f};

// 2026-11-19 機体A・右135°入口の単体走行3本から暫定採用。1.60 m/s、吸引duty 650、斜め後走の減速20 m/s^2。
// 実測旋回角134.88～135.23°。実測進入軸に沿った出口直線の補正量は-1.14～+1.95 mm。
// 動画の機首角とジャイロで同期したLend終端は目標より0.31～2.83 mm先。同期による差を考慮し20.56 mmを使用。
// 測定: tools/logs/pixel_camera/turn1600_A_inr135_20261119_v1/next_Ls5p1_c2p1/repeat_validation_v1/compare.html
const t_turn_param_table slalom_inL135_1600_table = {1.60f, 42.50f,5.10f,20.56f, 135.0f,Turn_L,-2.10f};
const t_turn_param_table slalom_inR135_1600_table = {1.60f,-42.50f,5.10f,20.56f,-135.0f,Turn_R,2.10f};

const t_param param_inL135_1600 = {&slalom_inL135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600,&ff_gain_in135_L_1600};
const t_param param_inR135_1600 = {&slalom_inR135_1600_table,&sp_gain_turnIn135_1600,&om_gain_turnIn135_1600,&ff_gain_in135_R_1600};

//k = 250
const t_pid_gain sp_gain_turnOut135_1600 = {2.0,0.016,0.00};
const t_pid_gain om_gain_turnOut135_1600 = {0.20,0.01,0.0};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,8.09,37.73, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,8.09,37.73,-135.0f,Turn_R,0.0f};
//const static t_turn_param_table slalom_outL135_1600_table = {1.60f, 39.3f,12.47,40.99, 135.0f,Turn_L,0.0f};
//const static t_turn_param_table slalom_outR135_1600_table = {1.60f,-39.3f,12.47,40.99,-135.0f,Turn_R,0.0f};
// 旧135°出口（ピンクLED）。
// const t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,12.10f,36.93f, 135.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,12.10f,36.93f,-135.0f,Turn_R,0.0f};
// 旧135°出口（角速度同期）。
// const t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,11.13f,36.50f, 135.0f,Turn_L,0.0f};
// const t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,11.13f,36.50f,-135.0f,Turn_R,0.0f};

// 2026-11-19 機体A・右135°出口の単体走行3本から暫定採用。1.60 m/s、吸引duty 650、直線後走の減速20 m/s^2。
// 実測旋回角134.52～135.54°。盤面X=225 mmで出口直線の目標との差は上0.41～下2.21 mm。
// 実測進入軸基準のLend終端は、動画の機首角とジャイロで同期すると1.47 mm手前～2.24 mm先。
// 測定: tools/logs/pixel_camera/turn1600_A_outr135_20261119_v1/next_Ls1p0_Le31p5_c1p9/repeat_validation_v1/compare.html
const t_turn_param_table slalom_outL135_1600_table = {1.60f, 41.0f,1.00f,31.50f, 135.0f,Turn_L,-1.90f};
const t_turn_param_table slalom_outR135_1600_table = {1.60f,-41.0f,1.00f,31.50f,-135.0f,Turn_R,1.90f};

const t_param param_outL135_1600 = {&slalom_outL135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600,&ff_gain_out135_L_1600};
const t_param param_outR135_1600 = {&slalom_outR135_1600_table,&sp_gain_turnOut135_1600,&om_gain_turnOut135_1600,&ff_gain_out135_R_1600};

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

