#ifndef PARAMS_PARAM_B_RUN_SUCTION_B_H_
#define PARAMS_PARAM_B_RUN_SUCTION_B_H_

#include <stdint.h>

// 機体Bの最短走行用吸引設定。目標電圧[V] = 設定値 / 1000 * 8.40。
// constexprにより、共通の走行プロファイルを静的に初期化する。
// 2026-11-21: 従来のプロファイル別600～800を保持。
constexpr uint16_t run_suction_1200_v0 = 600;
constexpr uint16_t run_suction_1200_v1 = 600;
constexpr uint16_t run_suction_1400_v0 = 650;
constexpr uint16_t run_suction_1400_v1_600 = 600;
constexpr uint16_t run_suction_1400_v1_650 = 650;
constexpr uint16_t run_suction_1400_v2 = 650;
constexpr uint16_t run_suction_1600_v1 = 650;
constexpr uint16_t run_suction_1600_v2 = 700;
constexpr uint16_t run_suction_1800 = 700;
constexpr uint16_t run_suction_2000 = 800;
constexpr uint16_t run_suction_acc1600_v1 = 700;
constexpr uint16_t run_suction_acc1600_v2 = 700;
constexpr uint16_t run_suction_acc1600_v3 = 700;
constexpr uint16_t run_suction_acc1800_v1 = 800;

// 既存のプロファイルキーを保ち、表示名は機体ごとの吸引設定に合わせる。
constexpr char run_suction_1400_v1_600_name[] = "Suction 1400 V1 / 600";
constexpr char run_suction_1400_v1_650_name[] = "Suction 1400 V1 / 650";

#endif
