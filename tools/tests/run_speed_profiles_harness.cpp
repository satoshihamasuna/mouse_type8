#include "Subsys/Inc/make_path.h"
// 実装を同じ翻訳単位へ読み込み、内部の候補生成も実校正表と照合する。
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
	assert(run_config_plain500.straight_speed == &straight_speed_plain500);
	assert(run_config_plain500.diagonal_speed == &diagonal_speed_plain500);
	assert(&straight_speed_plain500 != &diagonal_speed_plain500);
	calcRunTime time;
	time.run_config_set(&run_config_plain500);
	assert(time.speed_profiles_valid());
	auto p = time.calc_end_straight_max_velo(20.0f,.5f,.5f);
	assert(close(p.param->max_velo,.5f) && p.sp_gain == st_param_500.sp_gain);
	assert(close(p.param->acc,6.0f) && close(p.param->deceleration(),6.0f));
	p = time.calc_end_straight_max_velo(30.0f,.5f,.5f);
	assert(close(p.param->max_velo,.6f) && p.om_gain == st_param_600.om_gain);
	p = time.calc_end_straight_max_velo(90.0f,.5f,.5f);
	assert(close(p.param->max_velo,.7f) && p.sp_gain == st_param_700.sp_gain);
	assert(p.ff_gain == st_param_700.ff_gain);
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
	auto config = run_config_plain500; config.straight_mode_size=0; time.run_config_set(&config);
	assert(!time.speed_profiles_valid());
}

static void legacy()
{
	calcRunTime time; time.run_config_set(&run_config_plain500);
	assert(close(time.calc_end_straight_max_velo(45,.5f,.5f).param->max_velo,.6f));
	auto config = configured(nullptr,nullptr); time.run_config_set(&config);
	assert(time.calc_end_straight_max_velo(45,.5f,.5f).param == &param_500);
	assert(time.calc_end_diagonal_max_velo(45,.5f,.5f).param == &param_500);
	assert(time.calc_end_straight_max_velo(90).param == &param_700);
	assert(run_config_plain700.straight_speed != nullptr);
	assert(run_config_suction1200_v1.straight_speed != nullptr);
	time.run_config_set(&run_config_plain500);
	time.st_param_set(st_mode_1000_v1,7);
	assert(time.calc_end_straight_max_velo(10000).param == &param_2000);
	time.di_param_set(di_mode_1000_v1,7);
	assert(time.calc_end_diagonal_max_velo(10000).param == &param_2000);
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
	assert(motion.last_sp == st_param_600.sp_gain && motion.last_om == st_param_600.om_gain);
	assert(motion.last_ff == st_param_600.ff_gain);
	run.diagonal(&motion,&config,1);
	assert(!motion.stopped && close(motion.motion_plan.end_velo.get(),.5f));
	assert(close(motion.motion_plan.deccel.get(),-3));
	assert(motion.last_sp == st_param_600.sp_gain && motion.last_om == st_param_600.om_gain);
	run.straight(&motion,&config,0);
	assert(close(motion.motion_plan.end_velo.get(),0));
	straight.max=.55f; // baseより小さい設定は走行呼出しへ渡さない。
	run.straight(&motion,&config,1);
	assert(motion.stopped);
}

struct ModeProfileCase
{
	const t_speed_profile *profile;
	const t_straight_param *const *original;
	unsigned size;
};

static void all_profiles()
{
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
			const auto *expected = mode.original[i];
			assert(close(candidate.max_velo,expected->param->max_velo));
			assert(close(candidate.acc,expected->param->acc));
			assert(close(candidate.deceleration(),expected->param->deceleration()));
			const auto *band = speed_control_band(candidate.max_velo,mode.original,(uint16_t)mode.size);
			assert(band->sp_gain == expected->sp_gain && band->om_gain == expected->om_gain);
			assert(band->ff_gain == expected->ff_gain);
			++total;
		}
		assert(speed_candidate(*mode.profile,(uint16_t)mode.size).max_velo == 0.0f);
	}
	assert(total == 354);
	for(const auto *config : run_config_table)
	{
		assert(config->straight_speed && config->diagonal_speed);
		calcRunTime time; time.run_config_set(config);
		assert(time.speed_profiles_valid());
		assert(close(time.straight_base_velo().param->max_velo,config->straight_mode[0]->param->max_velo));
		assert(close(time.diagonal_base_velo().param->max_velo,config->diagonal_mode[0]->param->max_velo));
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
	assert(motion.last_sp == st_param_1500.sp_gain);
	run.suction(&motion,&run_config_suction1200_v0,1,true);
	assert(close(motion.motion_plan.accel.get(),12) && close(motion.motion_plan.deccel.get(),-12));
	assert(motion.last_sp == st_param_1400.sp_gain);
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

int main(int argc,char **argv)
{
	assert(argc == 2);
	if(!std::strcmp(argv[1],"current")) current();
	else if(!std::strcmp(argv[1],"generation")) generation();
	else if(!std::strcmp(argv[1],"independent")) independent();
	else if(!std::strcmp(argv[1],"braking")) braking();
	else if(!std::strcmp(argv[1],"short")) short_sections();
	else if(!std::strcmp(argv[1],"invalid")) invalid();
	else if(!std::strcmp(argv[1],"legacy")) legacy();
	else if(!std::strcmp(argv[1],"lifetime")) lifetime();
	else if(!std::strcmp(argv[1],"dispatch")) dispatch();
	else if(!std::strcmp(argv[1],"all-profiles")) all_profiles();
	else if(!std::strcmp(argv[1],"segmented-physics")) segmented_physics();
	else if(!std::strcmp(argv[1],"invalid-ranges")) invalid_ranges();
	else if(!std::strcmp(argv[1],"high-speed-dispatch")) high_speed_dispatch();
	else return 2;
}
