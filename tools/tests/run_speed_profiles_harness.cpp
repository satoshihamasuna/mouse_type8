#include "Subsys/Inc/make_path.h"
// 実装を同じ翻訳単位へ読み込み、内部の候補生成も保存済みの移行前候補と照合する。
#include "Subsys/Src/calcRunTime.cpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>

static bool close(float a,float b,float tolerance = 0.00001f) { return std::fabs(a-b) < tolerance; }

// 制御器・IR状態遷移を含めず、実ラッパーと初期化の距離計算部分を実行する。
struct Scalar { float value = 0.0f; void set(float v) { value=v; } float get() const { return value; } };
struct TestVehicle { struct { Scalar velo; } ideal; };
struct TestMotionPlan { Scalar velo,max_velo,end_velo,accel,deccel,end_length,length_accel,length_deccel; };
class Motion
{
public:
	TestVehicle data;
	TestVehicle *vehicle = &data;
	TestMotionPlan motion_plan;
	const t_pid_gain *last_sp = nullptr;
	const t_pid_gain *last_om = nullptr;
	const t_ff_gain *last_ff = nullptr;
	bool stopped = false;
	TestVehicle *return_vehicleObj() { return vehicle; }
	void Motion_end() { stopped = true; }
	t_exeStatus execute_Motion() { return complete; }
#include "motion_declarations.inc"
};
#include "motion_initialization.inc"

static void log_disable() {}
#define SECTION 90.0f
#define DIAG_SECTION (90.0f / 1.41421356f)
#include "run_start_constants.inc"
class RunDispatch:public calcRunTime
{
	static constexpr int tail = -1; // 既存ケースは予備区間の後の直線を検証する。
public:
	void straight(Motion *motion,const t_run_config *config,int i)
	{
		const t_param *const *turn_mode = config->turn_mode[0];
		t_run_pattern run_pt_buff[] = {Long_turnR90};
		int section_count = 1;
		float section_end = 0.0f;
		t_straight_param st_parameter{};
#include "run_straight.inc"
	}
	void diagonal(Motion *motion,const t_run_config *config,int i)
	{
		const t_param *const *turn_mode = config->turn_mode[0];
		t_run_pattern run_pt_buff[] = {Turn_out_R45};
		int section_count = 1;
		float section_end = 0.0f;
		t_straight_param st_parameter{};
#include "run_diagonal.inc"
	}
	void suction(Motion *motion,const t_run_config *config,int i,bool diagonal)
	{
		const t_param *const *turn_mode = config->turn_mode[config->turn_mode_size-1];
		t_run_pattern run_pt_buff[] = {diagonal ? Turn_out_R45 : Long_turnR90};
		int section_count = 1;
		float section_end = 0.0f;
		t_straight_param st_parameter{};
		if(diagonal) {
#include "run_suction_diagonal.inc"
		} else {
#include "run_suction_straight.inc"
		}
	}
	void variable(Motion *motion,const t_run_config *config,int i,float end_velo,bool diagonal)
	{
		(void)config;
		int section_count = 1;
		float section_end = 0.0f;
		t_straight_param st_parameter{};
		if(diagonal) {
#include "run_variable_diagonal.inc"
		} else {
#include "run_variable_straight.inc"
		}
	}
};

static t_run_config configured(const t_speed_profile *straight,const t_speed_profile *diagonal)
{
	t_run_config config = run_config_plain500;
	config.straight_speed = straight;
	config.diagonal_speed = diagonal;
	return config;
}

static void current()
{
	const uint16_t expected_suction[] = {
		0,0,0,600,600,650,600,650,650,650,700,700,800,700,700,700,800
	};
	static_assert(sizeof(expected_suction)/sizeof(expected_suction[0]) == run_config_count,
		"all run profiles must have an expected suction value");
	for(uint16_t i = 0; i < run_config_count; ++i)
	{
		assert(run_config_table[i]->suction == expected_suction[i]);
		assert(find_run_config(run_config_table[i]->key) == run_config_table[i]);
	}
	assert(!std::strcmp(run_config_suction1400_v1_600.display_name,"Suction 1400 V1 / 600"));
	assert(!std::strcmp(run_config_suction1400_v1_650.display_name,"Suction 1400 V1 / 650"));
	assert(run_config_plain500.straight_speed == &straight_speed_plain500);
	assert(run_config_plain500.diagonal_speed == &diagonal_speed_plain500);
	assert(&straight_speed_plain500 != &diagonal_speed_plain500);
	calcRunTime time;
	time.run_config_set(&run_config_plain500);
	assert(time.speed_profiles_valid());
	auto p = time.calc_end_straight_max_velo(20.0f,.5f,.5f);
	assert(close(p.param->max_velo,.5f) && p.sp_gain == &debug_straight_500.control->sp_gain);
	assert(close(p.param->acc,6.0f) && close(p.param->deceleration(),6.0f));
	p = time.calc_end_straight_max_velo(30.0f,.5f,.5f);
	assert(close(p.param->max_velo,.6f) && p.om_gain == &straight_gain_low.om_gain);
	p = time.calc_end_straight_max_velo(90.0f,.5f,.5f);
	assert(close(p.param->max_velo,.7f) && p.sp_gain == &debug_straight_700.control->sp_gain);
	assert(p.ff_gain == debug_straight_700.control->ff_gain);
	assert(close(time.straight_base_velo().param->max_velo,.5f));
	assert(close(time.diagonal_base_velo().param->max_velo,.5f));
}

static void generation()
{
	t_speed_profile profile = {.5f,6.0f,6.0f,.15f,.83f};
	auto config = configured(&profile,&profile);
	calcRunTime time; time.run_config_set(&config);
	assert(close(time.calc_end_straight_max_velo(45,.5f,.5f).param->max_velo,.65f));
	assert(close(time.calc_end_straight_max_velo(80,.5f,.5f).param->max_velo,.8f));
	assert(close(time.calc_end_straight_max_velo(90,.5f,.5f).param->max_velo,.83f));
	profile = {.4f,6,6,.15f,.7f};
	assert(close(time.straight_base_velo().param->max_velo,.4f));
	assert(close(time.calc_end_straight_max_velo(20,.4f,.4f).param->max_velo,.4f));
	profile = {.55f,6,6,0,.55f};
	assert(time.speed_profiles_valid());
	assert(close(time.calc_end_straight_max_velo(90,.5f,.5f).param->max_velo,.55f));
}

static void independent()
{
	t_speed_profile straight = {.5f,3,2,.1f,.65f};
	t_speed_profile diagonal = {.5f,4,1,.2f,.9f};
	auto config = configured(&straight,&diagonal);
	calcRunTime time; time.run_config_set(&config);
	auto s = time.calc_end_straight_max_velo(1000,.5f,.5f);
	auto d = time.calc_end_diagonal_max_velo(1000,.5f,.5f);
	assert(close(s.param->max_velo,.65f) && close(s.param->acc,3) && close(s.param->deceleration(),2));
	assert(close(d.param->max_velo,.9f) && close(d.param->acc,4) && close(d.param->deceleration(),1));
}

static void braking()
{
	t_speed_profile profile = {.5f,4,2,.1f,.8f};
	auto config = configured(&profile,&profile);
	calcRunTime time; time.run_config_set(&config);
	auto p = time.calc_end_straight_max_velo(400,.5f,0);
	assert(close(p.param->max_velo,.8f));
	// a=4, d=2: 加速48.75 mm、減速160 mm、定速191.25 mm。
	const float seconds = .3f/4 + .8f/2 + .19125f/.8f;
	assert(time.straight_time_set(400,.5f,0) == (unsigned)(seconds*1000+.5f));
	assert(time.diagonal_time_set(400,.5f,0) == (unsigned)std::ceil(seconds*1000));
	Motion motion; motion.data.ideal.velo.set(.5f);
	motion.exe_Motion_straight(400,p.param->acc,p.param->max_velo,0,p.sp_gain,p.om_gain,p.ff_gain,p.param->deceleration());
	assert(close(motion.motion_plan.deccel.get(),-2));
	assert(close(motion.motion_plan.length_deccel.get(),160,.001f));
	assert(close(motion.motion_plan.length_accel.get(),48.75f,.001f));
	motion.exe_Motion_diagonal(400,4,.8f,0,p.sp_gain,p.om_gain,p.ff_gain,2);
	assert(close(motion.motion_plan.deccel.get(),-2));
	motion.exe_Motion_straight(400,4,.8f,0);
	assert(close(motion.motion_plan.deccel.get(),-4));
	motion.exe_Motion_diagonal(400,4,.8f,0);
	assert(close(motion.motion_plan.deccel.get(),-4));
}

static void short_sections()
{
	t_speed_profile profile = {.5f,4,2,.1f,.8f};
	auto config = configured(&profile,&profile);
	calcRunTime time; time.run_config_set(&config);
	auto p = time.calc_end_straight_max_velo(10,0,0);
	const float peak = std::sqrt(2*.01f/(1.0f/4+1.0f/2));
	assert(p.param && close(p.param->max_velo,peak));
	assert(time.straight_time_set(10,0,0) == (unsigned)((peak/4+peak/2)*1000+.5f));
	assert(time.calc_end_straight_max_velo(10,.5f,0).param == nullptr);
	assert(time.straight_time_set(10,.5f,0) == 65535);
	assert(time.calc_end_diagonal_max_velo(10,.5f,0).param == nullptr);
	assert(time.diagonal_time_set(10,.5f,0) == 65535);
}

static void invalid()
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();
	t_speed_profile bad[] = {{0,6,6,.1f,.7f},{.5f,0,6,.1f,.7f},{.5f,6,0,.1f,.7f},
		{.5f,6,6,.1f,.4f},{.5f,6,6,0,.7f},{.5f,6,6,-.1f,.7f},
		{.5f,6,6,.0001f,.7f},{nan,6,6,.1f,.7f},{.5f,inf,6,.1f,.7f},{.5f,6,6,nan,.7f}};
	calcRunTime time;
	for(const auto &profile : bad)
	{
		auto config = configured(&profile,&profile); time.run_config_set(&config);
		assert(!time.speed_profiles_valid());
		assert(time.straight_time_set(90) == 65535);
		assert(time.diagonal_time_set(90) == 65535);
		assert(time.calc_end_straight_max_velo(90,.5f,.5f).param == nullptr);
	}
	time.run_config_set(&run_config_plain500);
	assert(time.straight_time_set(nan,.5f,.5f) == 65535);
	assert(time.straight_time_set(90,inf,.5f) == 65535);
	assert(time.straight_time_set(90,.5f,-.1f) == 65535);
	assert(time.calc_end_straight_max_velo(90,-.2f,.5f).param == nullptr);
	assert(time.calc_end_straight_max_velo(90,.5f,.8f).param == nullptr);
	auto config = run_config_plain500; config.straight_control=nullptr; time.run_config_set(&config);
	assert(!time.speed_profiles_valid());
}

static void missing_profiles()
{
	calcRunTime time;
	assert(time.speed_profiles_valid()); // 既定値も新形式の300 mm/s。
	for(int missing = 0; missing < 5; ++missing)
	{
		auto config = run_config_plain500;
		if(missing == 0) config.straight_speed = nullptr;
		if(missing == 1) config.diagonal_speed = nullptr;
		if(missing == 2) config.straight_control = nullptr;
		if(missing == 3) config.diagonal_control = nullptr;
		time.run_config_set(missing == 4 ? nullptr : &config);
		assert(!time.speed_profiles_valid());
		assert(time.calc_end_straight_max_velo(90).param == nullptr);
		assert(time.calc_end_diagonal_max_velo(90).param == nullptr);
		assert(time.straight_time_set(90) == 65535 && time.diagonal_time_set(90) == 65535);
	}
	time.run_config_set(&run_config_plain500);
	assert(time.speed_profiles_valid());
	assert(close(time.calc_end_straight_max_velo(45,.5f,.5f).param->max_velo,.6f));
}

static void lifetime()
{
	calcRunTime time; time.run_config_set(&run_config_plain500);
	auto selected = time.calc_end_straight_max_velo(90,.5f,.5f);
	auto base = time.straight_base_velo();
	assert(close(selected.param->max_velo,.7f) && close(base.param->max_velo,.5f));
	time.diagonal_base_velo(); time.calc_end_diagonal_max_velo(20,.5f,.5f);
	assert(close(selected.param->max_velo,.7f));
}

static void dispatch()
{
	t_speed_profile straight = {.6f,4,2,.1f,.7f};
	t_speed_profile diagonal = {.6f,4,3,.1f,.7f};
	auto config = configured(&straight,&diagonal);
	RunDispatch run; run.run_config_set(&config);
	Motion motion; motion.data.ideal.velo.set(.5f);
	run.straight(&motion,&config,1);
	assert(!motion.stopped && close(motion.motion_plan.end_velo.get(),.5f));
	assert(close(motion.motion_plan.accel.get(),4) && close(motion.motion_plan.deccel.get(),-2));
	assert(close(motion.motion_plan.max_velo.get(),.6f));
	assert(motion.last_sp == &straight_gain_low.sp_gain && motion.last_om == &straight_gain_low.om_gain);
	assert(motion.last_ff == straight_gain_low.ff_gain);
	run.diagonal(&motion,&config,1);
	assert(!motion.stopped && close(motion.motion_plan.end_velo.get(),.5f));
	assert(close(motion.motion_plan.deccel.get(),-3));
	assert(motion.last_sp == &straight_gain_low.sp_gain && motion.last_om == &straight_gain_low.om_gain);
	run.straight(&motion,&config,0);
	assert(close(motion.motion_plan.end_velo.get(),0));
	straight.max=.55f; // baseより小さい設定は走行呼出しへ渡さない。
	run.straight(&motion,&config,1);
	assert(motion.stopped);
}

struct ExpectedControlBand
{
	float min_speed;
	t_pid_gain sp_gain;
	t_pid_gain om_gain;
	const t_ff_gain *ff_gain;
};

struct ModeProfileCase
{
	const t_speed_profile *profile;
	const t_velo_param *expected;
	unsigned size;
	const t_control_profile *control;
	const ExpectedControlBand *expected_bands;
	unsigned band_count;
};

static void check_control_gain(const ModeProfileCase &mode,float speed)
{
	const auto *expected = &mode.expected_bands[0];
	for(unsigned i = 1; i < mode.band_count; ++i)
		if(mode.expected_bands[i].min_speed <= speed + .00001f) expected = &mode.expected_bands[i];
	const auto *gain = speed_control_gain(speed,mode.control);
	assert(gain && gain->ff_gain == expected->ff_gain);
	assert(close(gain->sp_gain.Kp,expected->sp_gain.Kp) && close(gain->sp_gain.Ki,expected->sp_gain.Ki));
	assert(close(gain->sp_gain.Kd,expected->sp_gain.Kd));
	assert(close(gain->om_gain.Kp,expected->om_gain.Kp) && close(gain->om_gain.Ki,expected->om_gain.Ki));
	assert(close(gain->om_gain.Kd,expected->om_gain.Kd));
}

static void all_profiles()
{
#include "mode_profile_expected.inc"
	const ModeProfileCase modes[] = {
#include "mode_profile_cases.inc"
	};
	unsigned total = 0;
	for(const auto &mode : modes)
	{
		assert(speed_candidate_count(*mode.profile) == mode.size);
		for(unsigned i = 0; i < mode.size; ++i)
		{
			const auto candidate = speed_candidate(*mode.profile,(uint16_t)i);
			const auto &expected = mode.expected[i];
			assert(close(candidate.max_velo,expected.max_velo));
			assert(close(candidate.acc,expected.acc));
			assert(close(candidate.deceleration(),expected.deceleration()));
			check_control_gain(mode,candidate.max_velo);
			++total;
		}
		assert(speed_candidate(*mode.profile,(uint16_t)mode.size).max_velo == 0.0f);
		for(unsigned i = 0; i < mode.band_count; ++i)
		{
			check_control_gain(mode,mode.expected_bands[i].min_speed-.0001f);
			check_control_gain(mode,mode.expected_bands[i].min_speed);
			check_control_gain(mode,mode.expected_bands[i].min_speed+.0001f);
		}
		check_control_gain(mode,.01f);
		check_control_gain(mode,6.0f);
	}
	assert(total == 354);
	for(const auto *config : run_config_table)
	{
		assert(config->straight_speed && config->diagonal_speed);
		calcRunTime time; time.run_config_set(config);
		assert(time.speed_profiles_valid());
		assert(close(time.straight_base_velo().param->max_velo,config->straight_speed->base));
		assert(close(time.diagonal_base_velo().param->max_velo,config->diagonal_speed->base));
		const float lengths[] = {45,90,180,720};
		for(int axis = 0; axis < 2; ++axis)
		{
			const float start = axis ? config->diagonal_speed->base : config->straight_speed->base;
			for(float length : lengths) for(int stop = 0; stop < 2; ++stop)
			{
				const float end = stop ? 0.0f : start;
				const auto section = axis ? time.calc_end_diagonal_max_velo(length,start,end)
						: time.calc_end_straight_max_velo(length,start,end);
				const unsigned predicted = axis ? time.diagonal_time_set(length,start,end)
						: time.straight_time_set(length,start,end);
				if(section.param == nullptr) { assert(predicted == 65535); continue; }
				const double peak = section.param->max_velo;
				assert(peak+0.00001 >= start && peak+0.00001 >= end);
				// 選んだ台形/三角波形の独立した積分式で、見積りを1 ms以内で照合する。
				const double seconds = length/1000.0/peak
						+ (peak-start)*(peak-start)/(2*section.param->acc*peak)
						+ (peak-end)*(peak-end)/(2*section.param->deceleration()*peak);
				assert(std::fabs(predicted-seconds*1000) <= 1.001);
			}
		}
	}
	assert(close(straight_speed_1800_v1.accel,30) && close(diagonal_speed_1800_v1.accel,20));
}

static void segmented_physics()
{
	calcRunTime time; time.run_config_set(&run_config_suction1200_v0);
	auto p = time.calc_end_straight_max_velo(45,1.2f,1.2f);
	assert(close(p.param->max_velo,1.3f) && close(p.param->acc,10));
	p = time.calc_end_straight_max_velo(63.64f,1.2f,1.2f);
	assert(close(p.param->max_velo,1.4f) && close(p.param->acc,12));
	const float seconds = 2*.2f/12 + (.06364f - 2*(1.4f*1.4f - 1.2f*1.2f)/24)/1.4f;
	assert(time.straight_time_set(63.64f,1.2f,1.2f) == (unsigned)(seconds*1000+.5f));
	const t_speed_range ranges[] = {{.5f,4,2,.1f,.6f},{.8f,3,1,.2f,1.0f}};
	const auto profile = make_speed_profile(ranges);
	auto config = configured(&profile,&profile); time.run_config_set(&config);
	p = time.calc_end_straight_max_velo(400,.5f,0);
	assert(close(p.param->max_velo,.8f) && close(p.param->acc,3) && close(p.param->deceleration(),1));
	// 選んだ800 mm/s候補のa=3,d=1を区間全体へ適用。
	assert(time.straight_time_set(400,.5f,0) == 919);
	p = time.calc_end_straight_max_velo(50,.7f,.7f);
	assert(p.param && p.param->max_velo >= .7f && p.param->max_velo < .8f);
	assert(close(p.param->acc,3) && close(p.param->deceleration(),1));
	const t_speed_range boundary_ranges[] = {{.5f,1,1,.1f,.6f},{.8f,4,4,.2f,1.0f}};
	const auto boundary = make_speed_profile(boundary_ranges);
	config = configured(&boundary,&boundary); time.run_config_set(&config);
	// 低い候補のa=1では55 mm必要だが、高い候補のa=4なら20 mmで接続できる。
	p = time.calc_end_straight_max_velo(20,.6f,.5f);
	assert(p.param && close(p.param->acc,4) && close(p.param->deceleration(),4));
	assert(p.param->max_velo >= .6f && p.param->max_velo < .8f);
}

static void invalid_ranges()
{
	t_speed_range ranges[] = {{.5f,6,6,.1f,.6f},{.8f,6,6,.1f,.9f}};
	auto profile = make_speed_profile(ranges);
	assert(speed_candidate_count(profile) == 4);
	profile.base=.4f; assert(speed_candidate_count(profile) == 0);
	profile = make_speed_profile(ranges); profile.range_count=0;
	assert(speed_candidate_count(profile) == 0);
	profile = make_speed_profile(ranges); profile.ranges=nullptr;
	assert(speed_candidate_count(profile) == 0);
	profile = make_speed_profile(ranges); profile.range_count=257;
	assert(speed_candidate_count(profile) == 0);
	ranges[1] = {.6f,6,6,.1f,.9f}; profile=make_speed_profile(ranges);
	assert(speed_candidate_count(profile) == 0);
	ranges[1] = {.4f,6,6,.1f,.9f}; profile=make_speed_profile(ranges);
	assert(speed_candidate_count(profile) == 0);
	ranges[1] = {.8f,0,6,.1f,.9f}; profile=make_speed_profile(ranges);
	assert(speed_candidate_count(profile) == 0);
	ranges[0] = {.1f,6,6,.001f,.3f}; ranges[1] = {.4f,6,6,.001f,.5f};
	profile=make_speed_profile(ranges);
	assert(speed_candidate_count(profile) == 0);
	calcRunTime time; auto config=configured(&profile,&profile); time.run_config_set(&config);
	assert(!time.speed_profiles_valid() && time.straight_time_set(90) == 65535);
}

static void high_speed_dispatch()
{
	RunDispatch run; run.run_config_set(&run_config_suction1200_v0);
	Motion motion; motion.data.ideal.velo.set(1.2f);
	run.suction(&motion,&run_config_suction1200_v0,1,false);
	assert(close(motion.motion_plan.accel.get(),12) && close(motion.motion_plan.deccel.get(),-12));
	assert(close(motion.motion_plan.end_velo.get(),1.2f));
	assert(motion.last_sp == &speed_control_gain(1.5f,&straight_control_1200_v0)->sp_gain);
	run.suction(&motion,&run_config_suction1200_v0,1,true);
	assert(close(motion.motion_plan.accel.get(),12) && close(motion.motion_plan.deccel.get(),-12));
	assert(motion.last_sp == &speed_control_gain(1.4f,&straight_control_1200_v0)->sp_gain);
	run.run_config_set(&run_config_acc1600_v1);
	motion.data.ideal.velo.set(1.6f);
	run.variable(&motion,&run_config_acc1600_v1,1,1.8f,false);
	assert(!motion.stopped && close(motion.motion_plan.end_velo.get(),1.8f));
	assert(close(motion.motion_plan.accel.get(),50) && close(motion.motion_plan.deccel.get(),-50));
	run.variable(&motion,&run_config_acc1600_v1,1,1.8f,true);
	assert(close(motion.motion_plan.accel.get(),30) && close(motion.motion_plan.deccel.get(),-30));
	run.variable(&motion,&run_config_acc1600_v1,0,1.8f,false);
	assert(close(motion.motion_plan.end_velo.get(),0));
}

static void high_control()
{
	// 旧速度別定義を使わず、機体別に採用した固定値と照合する。
#if defined(MOUSE_A)
	const float expected_sp_kp = 2.0f;
	const float expected_om_kp = .15f, expected_om_ki = .001f;
#else
	const float expected_sp_kp = 4.0f;
	const float expected_om_kp = .2f, expected_om_ki = .01f;
#endif
	const auto &gain = straight_control_high;
	assert(gain.ff_gain == &ff_gain_default);
	assert(close(gain.sp_gain.Kp,expected_sp_kp) && close(gain.sp_gain.Ki,.05f) && close(gain.sp_gain.Kd,0));
	assert(close(gain.om_gain.Kp,expected_om_kp) && close(gain.om_gain.Ki,expected_om_ki) && close(gain.om_gain.Kd,0));
	const auto *diagonal_gain = speed_control_gain(3.0f,&diagonal_control_1600_v1);
#if defined(MOUSE_A)
	assert(close(diagonal_gain->om_gain.Kp,.1f) && close(diagonal_gain->om_gain.Ki,.001f));
	const auto *plain_gain = speed_control_gain(3.0f,&straight_control_plain500);
	assert(close(plain_gain->om_gain.Kp,.1f) && close(plain_gain->om_gain.Ki,.001f));
#else
	assert(diagonal_gain == &gain);
#endif
	const auto *below = speed_control_gain(2.199f,&straight_control_1600_v1);
	assert(below == debug_straight_2000.control);
	// 高速直進の採用値と斜めの保持値を、実モーションへ渡す。
	const float speeds[] = {2.2f,2.99f,3.0f,3.2f,3.4f,4.6f,5.0f};
	for(float speed : speeds)
	{
		t_speed_profile profile = {speed,20,20,0,speed};
		auto config = run_config_suction1600_v1;
		config.straight_speed = &profile;
		config.diagonal_speed = &profile;
		calcRunTime time; time.run_config_set(&config);
		assert(time.speed_profiles_valid());
		assert(time.straight_base_velo().sp_gain == &gain.sp_gain);
		assert(time.diagonal_base_velo().om_gain == &diagonal_gain->om_gain);
		Motion motion;
		for(int diagonal = 0; diagonal < 2; ++diagonal)
		{
			const auto *expected = diagonal ? diagonal_gain : &gain;
			const auto p = diagonal ? time.calc_end_diagonal_max_velo(1000,speed,speed)
					: time.calc_end_straight_max_velo(1000,speed,speed);
			assert(p.param && close(p.param->max_velo,speed));
			assert(p.sp_gain == &expected->sp_gain && p.om_gain == &expected->om_gain);
			assert(p.ff_gain == expected->ff_gain);
			if(diagonal) motion.exe_Motion_diagonal(1000,p.param->acc,speed,speed,p.sp_gain,p.om_gain,p.ff_gain);
			else motion.exe_Motion_straight(1000,p.param->acc,speed,speed,p.sp_gain,p.om_gain,p.ff_gain);
			assert(motion.last_sp == &expected->sp_gain && motion.last_om == &expected->om_gain);
			assert(motion.last_ff == expected->ff_gain);
		}
	}
}

static void invalid_controls()
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();
	const t_control_gain no_ff = {{1,0,0},{1,0,0},nullptr};
	for(int bad = 0; bad < 11; ++bad)
	{
		t_control_band bands[] = {{.5f,&straight_gain_low},{.7f,debug_straight_700.control}};
		auto profile = make_control_profile(bands);
		if(bad == 0) profile.bands = nullptr;
		if(bad == 1) profile.count = 0;
		if(bad == 2) profile.count = 257;
		if(bad == 3) bands[0].min_speed = 0;
		if(bad == 4) bands[0].min_speed = -.1f;
		if(bad == 5) bands[0].min_speed = nan;
		if(bad == 6) bands[0].min_speed = inf;
		if(bad == 7) bands[1].min_speed = .5f;
		if(bad == 8) bands[1].min_speed = .4f;
		if(bad == 9) bands[1].gain = nullptr;
		if(bad == 10) bands[1].gain = &no_ff;
		auto config = run_config_plain500;
		config.straight_control = &profile;
		calcRunTime time; time.run_config_set(&config);
		assert(!time.speed_profiles_valid());
		assert(speed_control_gain(.6f,&profile) == nullptr);
		assert(time.calc_end_straight_max_velo(90).param == nullptr);
		assert(time.straight_base_velo().param == nullptr);
		assert(time.straight_time_set(90) == 65535);
	}
	// 新しい組のPID・FFを一緒に変更し、直進だけへ反映できる。
	const t_ff_gain ff = {1,2,3,4,5,6,7};
	const t_control_gain gain = {{6,7,8},{9,10,11},&ff};
	const t_control_band bands[] = {{.5f,&gain}};
	const auto profile = make_control_profile(bands);
	auto config = run_config_plain500;
	config.straight_control = &profile;
	calcRunTime time; time.run_config_set(&config);
	assert(time.speed_profiles_valid());
	const auto straight = time.calc_end_straight_max_velo(90,.5f,.5f);
	const auto diagonal = time.calc_end_diagonal_max_velo(90,.5f,.5f);
	assert(straight.sp_gain == &gain.sp_gain && straight.om_gain == &gain.om_gain && straight.ff_gain == &ff);
	assert(diagonal.sp_gain == &debug_straight_700.control->sp_gain);
	assert(diagonal.ff_gain == debug_straight_700.control->ff_gain);
}

static void standalone()
{
#if defined(MOUSE_A)
	const float search_speed = .32f, search_kp = 2, high_kp = 2;
	const float high_om_kp = .15f, high_om_ki = .001f;
	const t_pid_gain debug_sp[] = {{2,.02f,0},{2,.02f,0}};
	const t_pid_gain debug_om[] = {{.1f,.005f,0},{.1f,.01f,0}};
	const t_ff_gain *low_ff = &ff_gain_straight_non_suction;
#else
	const float search_speed = .32f, search_kp = 1.2f, high_kp = 4;
	const float high_om_kp = .2f, high_om_ki = .01f;
	const t_pid_gain debug_sp[] = {{1.2f,.02f,0},{1.5f,.05f,0}};
	const t_pid_gain debug_om[] = {{.1f,.01f,0},{.1f,.005f,0}};
	const t_ff_gain *low_ff = &ff_gain_default;
#endif
	const t_motion_profile *search[] = {&search_straight,&search_straight_slow};
	for(int i = 0; i < 2; ++i)
	{
		const auto p = search[i]->parameter();
		assert(p.param == &search[i]->velocity && close(p.param->max_velo,i ? .32f : search_speed));
		assert(close(p.param->acc,4) && close(p.param->deceleration(),4));
		assert(close(p.sp_gain->Kp,search_kp) && close(p.sp_gain->Ki,.02f) && close(p.sp_gain->Kd,0));
		assert(close(p.om_gain->Kp,.1f) && close(p.om_gain->Ki,.005f) && close(p.om_gain->Kd,0));
		assert(p.ff_gain == low_ff);
	}
	const t_motion_profile *debug[] = {&debug_straight_500,&debug_straight_700,
		&debug_straight_1600,&debug_straight_1800,&debug_straight_2000};
	const float speeds[] = {.5f,.7f,1.6f,1.8f,2.0f}, rates[] = {6,6,20,30,30};
	for(int i = 0; i < 5; ++i)
	{
		const auto p = debug[i]->parameter();
		assert(p.param == &debug[i]->velocity && close(p.param->max_velo,speeds[i]));
		assert(close(p.param->acc,rates[i]) && close(p.param->deceleration(),rates[i]));
		assert(close(p.sp_gain->Kp,i < 2 ? debug_sp[i].Kp : high_kp));
		assert(close(p.sp_gain->Ki,i < 2 ? debug_sp[i].Ki : .05f) && close(p.sp_gain->Kd,0));
		assert(close(p.om_gain->Kp,i < 2 ? debug_om[i].Kp : high_om_kp));
		assert(close(p.om_gain->Ki,i < 2 ? debug_om[i].Ki : high_om_ki) && close(p.om_gain->Kd,0));
		assert(p.ff_gain == (i < 2 ? low_ff : &ff_gain_default));
	}
}

int main(int argc,char **argv)
{
	assert(argc == 2);
	if(!std::strcmp(argv[1],"current")) current();
	else if(!std::strcmp(argv[1],"generation")) generation();
	else if(!std::strcmp(argv[1],"independent")) independent();
	else if(!std::strcmp(argv[1],"braking")) braking();
	else if(!std::strcmp(argv[1],"short")) short_sections();
	else if(!std::strcmp(argv[1],"invalid")) invalid();
	else if(!std::strcmp(argv[1],"missing-profiles")) missing_profiles();
	else if(!std::strcmp(argv[1],"lifetime")) lifetime();
	else if(!std::strcmp(argv[1],"dispatch")) dispatch();
	else if(!std::strcmp(argv[1],"all-profiles")) all_profiles();
	else if(!std::strcmp(argv[1],"segmented-physics")) segmented_physics();
	else if(!std::strcmp(argv[1],"invalid-ranges")) invalid_ranges();
	else if(!std::strcmp(argv[1],"high-speed-dispatch")) high_speed_dispatch();
	else if(!std::strcmp(argv[1],"high-control")) high_control();
	else if(!std::strcmp(argv[1],"invalid-controls")) invalid_controls();
	else if(!std::strcmp(argv[1],"standalone")) standalone();
	else return 2;
}
