/*
 * calcRunTime.cpp
 *
 *  Created on: 2023/07/09
 *      Author: sato1
 */


#include "Subsys/Inc/make_path.h"
#include "Component/Inc/typedef.h"
#include "Subsys/Inc/wall_class.h"
#include "Task/Inc/run_typedef.h"
#include "Params/turn_table.h"
#include "Component/Inc/controller.h"
#include <cmath>
#define OFF_SET_LENGTH 10.0
#define TURN_PENALTY_LENGTH 20.0f

// 候補は都度計算する。速度の刻みを変更しても候補配列の増減は不要。
// 異常設定で経路計算が長時間ループしないよう、候補数を256以下に制限する。
static uint16_t speed_range_count(const t_speed_range &profile)
{
	if(!std::isfinite(profile.base) || !std::isfinite(profile.max) ||
	   !std::isfinite(profile.accel) || !std::isfinite(profile.decel) ||
	   !std::isfinite(profile.step) || profile.base <= 0.0f ||
	   profile.max < profile.base || profile.accel <= 0.0f ||
	   profile.decel <= 0.0f || profile.step < 0.0f) return 0;
	if(profile.max == profile.base) return 1;
	if(profile.step == 0.0f) return 0;
	float intervals = std::ceil((profile.max - profile.base) / profile.step - 0.00001f);
	if(!std::isfinite(intervals) || intervals > 255.0f) return 0;
	if(intervals < 1.0f) intervals = 1.0f;
	return (uint16_t)intervals + 1;
}

static uint16_t speed_candidate_count(const t_speed_profile &profile)
{
	if(profile.ranges == nullptr)
	{
		if(profile.range_count != 0) return 0;
		return speed_range_count({profile.base,profile.accel,profile.decel,profile.step,profile.max});
	}
	if(profile.range_count == 0 || profile.range_count > 256 || profile.step != 0.0f) return 0;
	const auto &first = profile.ranges[0];
	if(profile.base != first.base || profile.accel != first.accel || profile.decel != first.decel ||
	   profile.max != profile.ranges[profile.range_count-1].max) return 0;
	uint16_t count = 0;
	float previous_max = 0.0f;
	for(uint16_t i = 0; i < profile.range_count; ++i)
	{
		const auto &range = profile.ranges[i];
		const uint16_t size = speed_range_count(range);
		if(size == 0 || (i != 0 && range.base <= previous_max) || count + size > 256) return 0;
		count += size;
		previous_max = range.max;
	}
	return count;
}

static t_velo_param speed_range_candidate(const t_speed_range &range,uint16_t index,uint16_t count)
{
	float speed = range.base + index * range.step;
	if(index == count - 1 || speed > range.max) speed = range.max;
	return {speed,range.accel,range.decel};
}

static t_velo_param speed_candidate(const t_speed_profile &profile,uint16_t index,uint16_t count = 0)
{
	if(count == 0) count = speed_candidate_count(profile);
	if(index >= count) return {};
	if(profile.ranges == nullptr)
		return speed_range_candidate({profile.base,profile.accel,profile.decel,profile.step,profile.max},index,count);
	for(uint16_t i = 0; i < profile.range_count; ++i)
	{
		const auto &range = profile.ranges[i];
		const uint16_t size = speed_range_count(range);
		if(index < size) return speed_range_candidate(range,index,size);
		index -= size;
	}
	return {};
}

static bool control_profile_valid(const t_control_profile *profile)
{
	if(profile == nullptr || profile->bands == nullptr || profile->count == 0 || profile->count > 256) return false;
	float previous = 0.0f;
	for(uint16_t i = 0; i < profile->count; ++i)
	{
		const auto &band = profile->bands[i];
		if(!std::isfinite(band.min_speed) || band.min_speed <= previous ||
		   band.gain == nullptr || band.gain->ff_gain == nullptr) return false;
		previous = band.min_speed;
	}
	return true;
}

static const t_control_gain *speed_control_gain(float speed,const t_control_profile *profile)
{
	if(!std::isfinite(speed) || !control_profile_valid(profile)) return nullptr;
	const t_control_gain *selected = profile->bands[0].gain;
	for(uint16_t i = 1; i < profile->count; ++i)
	{
		if(profile->bands[i].min_speed > speed + 0.00001f) break;
		selected = profile->bands[i].gain;
	}
	return selected;
}

struct ProfileSection
{
	bool valid = false;
	t_velo_param velocity{};
	const t_control_gain *control = nullptr;
	float time_ms = 0.0f;
};

// 経路の時間見積りと実走行の両方から使用する。
// 選択には従来の10 mm余裕を残し、時間は実際の全長と入口/出口速度で計算する。
static ProfileSection plan_profile_section(float length_mm,float start,float end,
		const t_speed_profile &profile,const t_control_profile *control_profile)
{
	ProfileSection result;
	const uint16_t count = speed_candidate_count(profile);
	if(count == 0 || !control_profile_valid(control_profile) || !std::isfinite(length_mm) ||
	   !std::isfinite(start) || !std::isfinite(end) || length_mm <= 0.0f ||
	   start < 0.0f || end < 0.0f || start > profile.max || end > profile.max) return result;
	const float length = length_mm / 1000.0f;
	const float endpoints = start > end ? start : end;
	t_velo_param selected{};
	t_velo_param fallback{};
	const float selection_length = length_mm > OFF_SET_LENGTH
			? (length_mm - OFF_SET_LENGTH) / 1000.0f : 0.0f;
	for(int i = count - 1; i >= 0; --i)
	{
		const auto candidate = speed_candidate(profile,(uint16_t)i,count);
		const float speed = candidate.max_velo;
		if(speed + 0.00001f < endpoints) continue;
		// 余裕距離を満たす候補がない場合も、実長で入口/出口をつなげる加減速度を保持する。
		// 低い候補の加速度が小さくても、高い候補の加速度で接続できる場合がある。
		const float minimum = end >= start ? (end * end - start * start) / (2.0f * candidate.acc)
				: (start * start - end * end) / (2.0f * candidate.deceleration());
		if(minimum <= length + 0.000001f) fallback = candidate;
		const float required = (speed * speed - start * start) / (2.0f * candidate.acc)
				+ (speed * speed - end * end) / (2.0f * candidate.deceleration());
		if(required <= selection_length + 0.000001f)
		{
			selected = candidate;
			break;
		}
	}
	if(selected.max_velo == 0.0f) selected = fallback;
	if(selected.max_velo == 0.0f) return result;
	const float accel = selected.acc;
	const float decel = selected.deceleration();
	const float minimum = end >= start ? (end * end - start * start) / (2.0f * accel)
			: (start * start - end * end) / (2.0f * decel);
	if(minimum > length + 0.000001f) return result;
	const float peak = std::sqrt((2.0f * length * accel * decel + decel * start * start
			+ accel * end * end) / (accel + decel));
	float cap = selected.max_velo < peak ? selected.max_velo : peak;
	if(cap < endpoints) cap = endpoints;
	if(!std::isfinite(cap) || cap <= 0.0f || cap > profile.max) return result;
	const float acc_length = (cap * cap - start * start) / (2.0f * accel);
	const float dec_length = (cap * cap - end * end) / (2.0f * decel);
	float cruise_length = length - acc_length - dec_length;
	if(cruise_length < 0.0f) cruise_length = 0.0f;
	result.time_ms = ((cap - start) / accel + cruise_length / cap + (cap - end) / decel) * 1000.0f;
	if(!std::isfinite(result.time_ms)) return result;
	result.velocity = {cap, accel, decel};
	// PID/FFは速度候補と独立した新形式の速度帯から選ぶ。
	result.control = speed_control_gain(cap,control_profile);
	result.valid = result.control != nullptr;
	return result;
}

static t_straight_param store_profile_section(const ProfileSection &section,t_velo_param &storage)
{
	if(!section.valid) return {};
	storage = section.velocity;
	return make_straight_param(&storage,*section.control);
}

static t_straight_param base_speed_parameter(const t_speed_profile *profile,
		const t_control_profile *control_profile,t_velo_param &storage)
{
	if(profile == nullptr || speed_candidate_count(*profile) == 0) return {};
	const auto *control = speed_control_gain(profile->base,control_profile);
	if(control == nullptr) return {};
	storage = {profile->base,profile->accel,profile->decel};
	return make_straight_param(&storage,*control);
}

t_straight_param calcRunTime::straight_base_velo()
{
	return base_speed_parameter(st_speed_profile,st_control_profile,st_base_velocity);
}

t_straight_param calcRunTime::diagonal_base_velo()
{
	return base_speed_parameter(di_speed_profile,di_control_profile,di_base_velocity);
}

bool calcRunTime::speed_profiles_valid() const
{
	return st_speed_profile != nullptr && di_speed_profile != nullptr
		&& speed_candidate_count(*st_speed_profile) != 0 && speed_candidate_count(*di_speed_profile) != 0
		&& control_profile_valid(st_control_profile) && control_profile_valid(di_control_profile);
}

void calcRunTime::turn_time_set(const t_param *const *mode)
{
	float omega_mx = 0.0f;
	turn_Long180_velo = mode[Long_turnL180]->param->velo;
	turn_Long90_velo = mode[Long_turnL90]->param->velo;
	turn_V90_velo = mode[Turn_LV90]->param->velo;
	turn_in45_velo = mode[Turn_in_L45]->param->velo;
	turn_out45_velo = mode[Turn_out_L45]->param->velo;
	turn_in135_velo = mode[Turn_in_L135]->param->velo;
	turn_out135_velo = mode[Turn_out_L135]->param->velo;

	omega_mx = mode[Long_turnL180]->param->velo/(mode[Long_turnL180]->param->r_min/1000.0);
	turn_Long180_time = (mode[Long_turnL180]->param->Lstart/mode[Long_turnL180]->param->velo);
	turn_Long180_time += (DEG2RAD(mode[Long_turnL180]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_Long180_time += (mode[Long_turnL180]->param->Lend/mode[Long_turnL180]->param->velo);
	turn_Long180_time += TURN_PENALTY_LENGTH / mode[Long_turnL180]->param->velo;

	omega_mx = mode[Long_turnL90]->param->velo/(mode[Long_turnL90]->param->r_min/1000.0);
	turn_Long90_time  = mode[Long_turnL90]->param->Lstart/mode[Long_turnL90]->param->velo;
	turn_Long90_time += DEG2RAD(mode[Long_turnL90]->param->degree)/(accel_Integral*omega_mx)*1000.0;
	turn_Long90_time += mode[Long_turnL90]->param->Lend/mode[Long_turnL90]->param->velo;
	turn_Long90_time += TURN_PENALTY_LENGTH / mode[Long_turnL90]->param->velo;

	omega_mx = mode[Turn_LV90]->param->velo/(mode[Turn_LV90]->param->r_min/1000.0);
	turn_V90_time  = (mode[Turn_LV90]->param->Lstart/mode[Turn_LV90]->param->velo);
	turn_V90_time += (DEG2RAD(mode[Turn_LV90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_V90_time += (mode[Turn_LV90]->param->Lend/mode[Turn_LV90]->param->velo);
	turn_V90_time += TURN_PENALTY_LENGTH / mode[Turn_LV90]->param->velo;

	omega_mx = mode[Turn_in_L45]->param->velo/(mode[Turn_in_L45]->param->r_min/1000.0);
	turn_in45_time  = (mode[Turn_in_L45]->param->Lstart/mode[Turn_in_L45]->param->velo);
	turn_in45_time += (DEG2RAD(mode[Turn_in_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	turn_in45_time += (mode[Turn_in_L45]->param->Lend/mode[Turn_in_L45]->param->velo);
	turn_in45_time += TURN_PENALTY_LENGTH / mode[Turn_in_L45]->param->velo;

	omega_mx = mode[Turn_out_L45]->param->velo/(mode[Turn_out_L45]->param->r_min/1000.0);
	turn_out45_time  = mode[Turn_out_L45]->param->Lstart/mode[Turn_out_L45]->param->velo;
	turn_out45_time += DEG2RAD(mode[Turn_out_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0;
	turn_out45_time += mode[Turn_out_L45]->param->Lend/mode[Turn_out_L45]->param->velo;
	turn_out45_time += TURN_PENALTY_LENGTH / mode[Turn_out_L45]->param->velo;

	omega_mx = mode[Turn_in_L135]->param->velo/(mode[Turn_in_L135]->param->r_min/1000.0);
	turn_in135_time  = mode[Turn_in_L135]->param->Lstart/mode[Turn_in_L135]->param->velo;
	turn_in135_time += DEG2RAD(mode[Turn_in_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0;
	turn_in135_time += mode[Turn_in_L135]->param->Lend/mode[Turn_in_L135]->param->velo;
	turn_in135_time += TURN_PENALTY_LENGTH / mode[Turn_in_L135]->param->velo;

	omega_mx = mode[Turn_out_L135]->param->velo/(mode[Turn_out_L135]->param->r_min/1000.0);
	turn_out135_time  = mode[Turn_out_L135]->param->Lstart/mode[Turn_out_L135]->param->velo;
	turn_out135_time += DEG2RAD(mode[Turn_out_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0;
	turn_out135_time += mode[Turn_out_L135]->param->Lend/mode[Turn_out_L135]->param->velo;
	turn_out135_time += TURN_PENALTY_LENGTH / mode[Turn_out_L135]->param->velo;
}

void calcRunTime::turn_time_set(const t_param *const*const *mode  ,uint16_t mode_size)
{
	float omega_mx = 0.0f;
	for(int i = 0; i < mode_size;i++)
	{
		if(mode[i][Long_turnL180] !=NULL)
		{
			turn_Long180_velo = mode[i][Long_turnL180]->param->velo;
			omega_mx = mode[i][Long_turnL180]->param->velo/(mode[i][Long_turnL180]->param->r_min/1000.0);
			turn_Long180_time = (mode[i][Long_turnL180]->param->Lstart/mode[i][Long_turnL180]->param->velo);
			turn_Long180_time += (DEG2RAD(mode[i][Long_turnL180]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_Long180_time += (mode[i][Long_turnL180]->param->Lend/mode[i][Long_turnL180]->param->velo);
			turn_Long180_time += TURN_PENALTY_LENGTH / mode[i][Long_turnL180]->param->velo;
		}

		if(mode[i][Long_turnL90] !=NULL)
		{
			turn_Long90_velo = mode[i][Long_turnL90]->param->velo;
			omega_mx = mode[i][Long_turnL90]->param->velo/(mode[i][Long_turnL90]->param->r_min/1000.0);
			turn_Long90_time  = mode[i][Long_turnL90]->param->Lstart/mode[i][Long_turnL90]->param->velo;
			turn_Long90_time += DEG2RAD(mode[i][Long_turnL90]->param->degree)/(accel_Integral*omega_mx)*1000.0;
			turn_Long90_time += mode[i][Long_turnL90]->param->Lend/mode[i][Long_turnL90]->param->velo;
			turn_Long90_time += TURN_PENALTY_LENGTH / mode[i][Long_turnL90]->param->velo;
		}

		if(mode[i][Turn_LV90] !=NULL)
		{
			turn_V90_velo = mode[i][Turn_LV90]->param->velo;
			omega_mx = mode[i][Turn_LV90]->param->velo/(mode[i][Turn_LV90]->param->r_min/1000.0);
			turn_V90_time  = (mode[i][Turn_LV90]->param->Lstart/mode[i][Turn_LV90]->param->velo);
			turn_V90_time += (DEG2RAD(mode[i][Turn_LV90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_V90_time += (mode[i][Turn_LV90]->param->Lend/mode[i][Turn_LV90]->param->velo);
			turn_V90_time += TURN_PENALTY_LENGTH / mode[i][Turn_LV90]->param->velo;
		}

		if(mode[i][Turn_in_L45] !=NULL)
		{
			turn_in45_velo = mode[i][Turn_in_L45]->param->velo;
			omega_mx = mode[i][Turn_in_L45]->param->velo/(mode[i][Turn_in_L45]->param->r_min/1000.0);
			turn_in45_time  = (mode[i][Turn_in_L45]->param->Lstart/mode[i][Turn_in_L45]->param->velo);
			turn_in45_time += (DEG2RAD(mode[i][Turn_in_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
			turn_in45_time += (mode[i][Turn_in_L45]->param->Lend/mode[i][Turn_in_L45]->param->velo);
			turn_in45_time += TURN_PENALTY_LENGTH / mode[i][Turn_in_L45]->param->velo;
		}

		if(mode[i][Turn_out_L45] !=NULL)
		{
			turn_out45_velo = mode[i][Turn_out_L45]->param->velo;
			omega_mx = mode[i][Turn_out_L45]->param->velo/(mode[i][Turn_out_L45]->param->r_min/1000.0);
			turn_out45_time  = mode[i][Turn_out_L45]->param->Lstart/mode[i][Turn_out_L45]->param->velo;
			turn_out45_time += DEG2RAD(mode[i][Turn_out_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0;
			turn_out45_time += mode[i][Turn_out_L45]->param->Lend/mode[i][Turn_out_L45]->param->velo;
			turn_out45_time += TURN_PENALTY_LENGTH / mode[i][Turn_out_L45]->param->velo;
		}

		if(mode[i][Turn_in_L135] !=NULL)
		{
			turn_in135_velo = mode[i][Turn_in_L135]->param->velo;
			omega_mx = mode[i][Turn_in_L135]->param->velo/(mode[i][Turn_in_L135]->param->r_min/1000.0);
			turn_in135_time  = mode[i][Turn_in_L135]->param->Lstart/mode[i][Turn_in_L135]->param->velo;
			turn_in135_time += DEG2RAD(mode[i][Turn_in_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0;
			turn_in135_time += mode[i][Turn_in_L135]->param->Lend/mode[i][Turn_in_L135]->param->velo;
			turn_in135_time += TURN_PENALTY_LENGTH / mode[i][Turn_in_L135]->param->velo;
		}

		if(mode[i][Turn_out_L135] !=NULL)
		{
			turn_out135_velo = mode[i][Turn_out_L135]->param->velo;
			omega_mx = mode[i][Turn_out_L135]->param->velo/(mode[i][Turn_out_L135]->param->r_min/1000.0);
			turn_out135_time  = mode[i][Turn_out_L135]->param->Lstart/mode[i][Turn_out_L135]->param->velo;
			turn_out135_time += DEG2RAD(mode[i][Turn_out_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0;
			turn_out135_time += mode[i][Turn_out_L135]->param->Lend/mode[i][Turn_out_L135]->param->velo;
			turn_out135_time += TURN_PENALTY_LENGTH / mode[i][Turn_out_L135]->param->velo;
		}

	}


}

uint16_t calcRunTime::return_turn_time(t_run_pattern run_pt)
{
	switch(run_pt)
	{
		case Turn_in_L45:
		case Turn_in_R45:
			return (uint16_t)(turn_in45_time + 0.5f);
		case Turn_out_L45:
		case Turn_out_R45:
			return (uint16_t)(turn_out45_time + 0.5f);
		case Turn_in_L135:
		case Turn_in_R135:
			return (uint16_t)(turn_in135_time + 0.5f);
		case Turn_out_L135:
		case Turn_out_R135:
			return (uint16_t)(turn_out135_time + 0.5f);
		case Turn_RV90:
		case Turn_LV90:
			return (uint16_t)(turn_V90_time + 0.5f);
		case Long_turnR90:
		case Long_turnL90:
			return (uint16_t)(turn_Long90_time + 0.5f);
		case Long_turnR180:
		case Long_turnL180:
			return (uint16_t)(turn_Long180_time + 0.5f);
		default:
			return 0;
	}
}

uint16_t calcRunTime::straight_time_set(float length)
{
	if(!speed_profiles_valid()) return 65535;
	float base_velo = straight_base_velo().param->max_velo;
	return straight_time_set(length,base_velo,base_velo);
}

uint16_t calcRunTime::straight_time_set(float length,float start_velo,float end_velo)
{
	if(!speed_profiles_valid()) return 65535;
	const auto section = plan_profile_section(length,start_velo,end_velo,*st_speed_profile,st_control_profile);
	if(!section.valid || section.time_ms >= 65535.0f) return 65535;
	return (uint16_t)(section.time_ms + 0.5f);
}

t_straight_param calcRunTime::calc_end_straight_max_velo(float length,float start_velo,float end_velo)
{
	if(!speed_profiles_valid()) return {};
	if(start_velo == -1.0f) start_velo = st_speed_profile->base;
	if(end_velo == -1.0f) end_velo = 0.0f;
	return store_profile_section(plan_profile_section(length,start_velo,end_velo,
			*st_speed_profile,st_control_profile),st_section_velocity);
}

uint16_t calcRunTime::diagonal_time_set(float length)
{
	if(!speed_profiles_valid()) return 65535;
	float base_velo = diagonal_base_velo().param->max_velo;
	return diagonal_time_set(length,base_velo,base_velo);
}

uint16_t calcRunTime::diagonal_time_set(float length,float start_velo,float end_velo)
{
	if(!speed_profiles_valid()) return 65535;
	const auto section = plan_profile_section(length,start_velo,end_velo,*di_speed_profile,di_control_profile);
	if(!section.valid || section.time_ms >= 65535.0f) return 65535;
	return (uint16_t)std::ceil(section.time_ms);
}

float calcRunTime::return_turn_entry_velo(t_run_pattern run_pt) const
{
	switch(run_pt)
	{
		case Long_turnR90: case Long_turnL90: return turn_Long90_velo;
		case Long_turnR180: case Long_turnL180: return turn_Long180_velo;
		case Turn_in_R45: case Turn_in_L45: return turn_in45_velo;
		case Turn_out_R45: case Turn_out_L45: return turn_out45_velo;
		case Turn_in_R135: case Turn_in_L135: return turn_in135_velo;
		case Turn_out_R135: case Turn_out_L135: return turn_out135_velo;
		case Turn_RV90: case Turn_LV90: return turn_V90_velo;
		default: return 0.0f;
	}
}

float calcRunTime::return_turn_exit_velo(t_run_pattern run_pt) const
{
	return return_turn_entry_velo(run_pt);
}

t_straight_param calcRunTime::calc_end_diagonal_max_velo(float length,float start_velo,float end_velo)
{
	if(!speed_profiles_valid()) return {};
	if(start_velo == -1.0f) start_velo = di_speed_profile->base;
	if(end_velo == -1.0f) end_velo = 0.0f;
	return store_profile_section(plan_profile_section(length,start_velo,end_velo,
			*di_speed_profile,di_control_profile),di_section_velocity);
}

void calcRunTime::run_config_set(const t_run_config *config)
{
	st_control_profile = config == nullptr ? nullptr : config->straight_control;
	di_control_profile = config == nullptr ? nullptr : config->diagonal_control;
	st_speed_profile = config == nullptr ? nullptr : config->straight_speed;
	di_speed_profile = config == nullptr ? nullptr : config->diagonal_speed;
	if(config != nullptr) turn_time_set(config->turn_mode,config->turn_mode_size);
}
