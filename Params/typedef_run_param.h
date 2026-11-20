/*
 * typedef_run_param.h
 *
 *  Created on: 2024/11/13
 *      Author: sato1
 */

#ifndef CPP_PARAMS_TYPEDEF_RUN_PARAM_H_
#define CPP_PARAMS_TYPEDEF_RUN_PARAM_H_

#include <stdint.h>
#include "Component/Inc/controller.h"
#include "Module/Inc/vehicle.h"
#include "Params/mouse_config.h"
#include "Task/Inc/run_typedef.h"

typedef struct{
	float sp_velo;
	float sp_accel;
	float sp_bias;
	float om_velo;
	float om_accel;  // Used while angular speed magnitude is increasing.
	float om_decel;  // Used while angular speed magnitude is decreasing.
	float om_bias;
	float om_jerk = 0.0f;  // Target angular-acceleration derivative feedforward.
	// Common-mode voltage needed only while turning.  Angular acceleration is
	// expressed along angular-speed magnitude: sign(omega) * alpha.  It is
	// positive on turn entry and negative on exit for either turn direction.
	float sp_turn_accel_mag = 0.0f;
	float sp_turn_velo_sq = 0.0f;
}t_ff_gain;

extern const t_ff_gain ff_gain_default;

// 速度PID・角速度PID・FFをひとまとまりで指定する。
typedef struct{
	t_pid_gain sp_gain;
	t_pid_gain om_gain;
	t_ff_gain const* ff_gain = &ff_gain_default;
}t_control_gain;

// 速度の下限[m/s]と、そこから使うPID・FFの組。先頭より低い速度は先頭の組を使う。
typedef struct{
	float min_speed;
	t_control_gain const* gain;
}t_control_band;

typedef struct{
	t_control_band const* bands;
	uint16_t count;
}t_control_profile;

template<unsigned N>
constexpr t_control_profile make_control_profile(const t_control_band (&bands)[N])
{
	static_assert(N > 0 && N <= 256, "control bands must contain 1..256 entries");
	return {bands,(uint16_t)N};
}

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
	// Logical degree is kept for path/type decisions. This offset is used only
	// when generating the physical turn motion (zero keeps legacy behaviour).
	float degree_correction;
}t_turn_param_table;

typedef struct{
	//float base_velo;
	float max_velo;
	float acc;
	// 0は従来互換（減速度の大きさをaccと同じにする）。単位はm/s^2。
	float decel = 0.0f;
	float deceleration() const { return decel > 0.0f ? decel : acc; }
}t_velo_param;

// 刻み・加減速度が一定の速度範囲。速度はm/s、加減速度はm/s^2。
typedef struct{
	float base;
	float accel;
	float decel;
	float step;
	float max;
}t_speed_range;

// 直進/斜めの速度候補を生成する設定。速度はm/s、加減速度はm/s^2。
// baseは通常の接続基準。実際の入口/出口速度は前後のモーションで決める。
// stepは候補の間隔。端数がある場合もmaxを最後の候補に含める。
typedef struct{
	float base;
	float accel;
	float decel;
	float step;
	float max;
	// 不等間隔・速度別加速度が必要な場合は、昇順で重複のない範囲を組み合わせる。
	// この場合、先頭5項目はmake_speed_profileで範囲から求める。
	const t_speed_range *ranges = nullptr;
	uint16_t range_count = 0;
}t_speed_profile;

template<unsigned N>
constexpr t_speed_profile make_speed_profile(const t_speed_range (&ranges)[N])
{
	static_assert(N > 0 && N <= 256, "speed ranges must contain 1..256 entries");
	return {ranges[0].base, ranges[0].accel, ranges[0].decel, 0.0f,
		ranges[N-1].max, ranges, (uint16_t)N};
}

typedef struct{
	t_turn_param_table const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
	t_ff_gain const* ff_gain = &ff_gain_default;
	// 探索ターン内の前後直進用。未指定なら呼出し元の既定ゲインを使う。
	t_pid_gain const* search_straight_sp_gain = nullptr;
	t_pid_gain const* search_straight_om_gain = nullptr;
}t_param;

typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
	t_ff_gain const* ff_gain = &ff_gain_default;
}t_straight_param;

// 既存のモーションAPIへ共通ゲインの参照を渡す。
constexpr t_straight_param make_straight_param(const t_velo_param *param,const t_control_gain &gain)
{
	return {param,&gain.sp_gain,&gain.om_gain,gain.ff_gain};
}

// 探索・単体走行用の固定速度条件。PID・FFは最短走行と同じ新形式の組を参照する。
typedef struct{
	t_velo_param velocity;
	t_control_gain const* control;
	constexpr t_straight_param parameter() const { return make_straight_param(&velocity,*control); }
}t_motion_profile;



#endif /* CPP_PARAMS_TYPEDEF_RUN_PARAM_H_ */
