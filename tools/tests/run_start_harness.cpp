#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "Task/Inc/ctrl_task.h"
#include "Params/turn_table.h"
#include "Params/run_config.h"
#include "Subsys/Inc/make_path.h"
#include "Subsys/Src/calcRunTime.cpp"

extern "C" void Sensor_SetMotionActive(uint8_t) {}
static void log_enable() {}
static void log_disable() {}
static const char *test_config = "";
static int test_selected = 0, test_pattern = 0;
static void check(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "%s mode=%d pattern=%d: %s\n", test_config,test_selected,test_pattern,message);
        std::exit(1);
    }
}

void Motion::Motion_start()
{
    return_vehicleObj()->ego_initialize();
    return_vehicleObj()->ideal_initialize();
    motion_exeStatus_set(complete);
    motion_control_set(NOP_STATE);
    motion_enable_set();
}
void Motion::Motion_end() { motion_exeStatus_set(complete); }
void Motion::Motion_error_handling() { motion_exeStatus_set(error); }
void Motion::SetIdeal_wall_control() {}
void Motion::Adjust_wall_corner() {}
void Motion::ApplyWallPositionCorrection(float, WallPositionCorrection::Phase, uint8_t) {}
void Motion::Init_Motion_stop_brake(float, const t_pid_gain *, const t_pid_gain *, const t_ff_gain *)
{ motion_exeStatus_set(complete); }
#include "motion_functions.inc"
#include "run_start_support.inc"

struct Sample { float distance, velocity, acceleration; };
class TestMotion : public Motion {
public:
    using Motion::Motion;
    using Motion::SetIdeal_straight;
    using Motion::handleTurnPrev_Straight;
    using Motion::handleTurnPrev_Diagonal;
    using Motion::active_ff_gain_get;
    using Motion::active_ff_gain_set;
    std::vector<Sample> samples;
    float physical_mm = 0.0f;
    float turn_start_mm = 0.0f;
    int ticks = 0;
    void tick()
    {
        auto *v = return_vehicleObj();
        if (samples.empty() && motion_pattern_get() != Straight)
            check(active_ff_gain_get() == &ff_gain_default,
                "initial turn inherited feedforward from a preceding run");
        physical_mm += v->ideal.velo.get();
        v->ego.length.set(v->ego.length.get() + v->ideal.velo.get());
        if (motion_pattern_get() == Straight) SetIdeal_straight();
        else {
            handleTurnPrev_Straight();
            if (physical_mm >= turn_start_mm) motion_exeStatus_set(complete);
        }
        samples.push_back({physical_mm, v->ideal.velo.get(), v->ideal.accel.get()});
        if (++ticks >= 10000) {
            std::fprintf(stderr,"pattern=%d physical=%f ego=%f velocity=%f acceleration=%f\n",
                motion_pattern_get(),physical_mm,v->ego.length.get(),v->ideal.velo.get(),v->ideal.accel.get());
            check(false, "startup did not terminate");
        }
    }
};
void host_control_tick(Motion *motion) { static_cast<TestMotion *>(motion)->tick(); }

// 実Dijkstraの起動部分と先頭直線/ターン呼出しを実行する。
class RunDispatch : public calcRunTime {
public:
    bool (*abort_requested)() = nullptr;
    void stage_diagonal_pose_route(Motion *,int,int = -1) {}
    void run(TestMotion *motion, const t_run_config *config, t_run_pattern first_pattern, uint8_t selected)
    {
        run_config_set(config);
        const int tail = 1;
        uint8_t run_pt_buff[] = {Long_turnR90, static_cast<uint8_t>(first_pattern)};
        const bool variable_turns = false;
        uint8_t turn_select[] = {selected, selected};
        const t_param *const *const *turn_modes = config->turn_mode;
        const t_param *const *turn_mode = config->turn_mode[selected];
        t_straight_param st_parameter{};
        int section_count = 1;
        float section_end = 0.0f;
        float end_velo = straight_base_velo().param->max_velo;
        motion->Motion_start();
        if (first_pattern != Straight) motion->turn_start_mm = 13.1f + turn_mode[first_pattern]->param->Lstart;
        if (config->type == RUN_CONFIG_PLAIN) {
#include "plain_setup.inc"
#include "plain_first.inc"
        } else if (config->type == RUN_CONFIG_SUCTION) {
#include "suction_setup.inc"
#include "suction_first.inc"
        } else {
            variable(motion, config, first_pattern, selected);
        }
    }
    void variable(TestMotion *motion, const t_run_config *config, t_run_pattern first_pattern, uint8_t selected)
    {
        const int tail = 1;
        uint8_t run_pt_buff[] = {Long_turnR90, static_cast<uint8_t>(first_pattern)};
        uint8_t turn_select[] = {selected, selected};
        const bool variable_turns = true;
        const t_param *const *const *turn_modes = config->turn_mode;
        const t_param *const *turn_mode = turn_modes[selected];
        t_straight_param st_parameter{};
        int section_count = 1;
        float section_end = 0.0f;
        float end_velo = turn_mode[Long_turnR90]->param->velo;
        if (first_pattern != Straight) motion->turn_start_mm = 13.1f + turn_mode[first_pattern]->param->Lstart;
#include "variable_setup.inc"
#include "variable_first.inc"
    }
};

int main()
{
    const t_run_pattern patterns[] = {Straight, Long_turnR90, Long_turnL90, Long_turnR180, Long_turnL180,
        Turn_in_R45, Turn_in_L45, Turn_in_R135, Turn_in_L135};
    for (const auto *config : run_config_table) {
      const uint8_t first_selected = config->type == RUN_CONFIG_VARIABLE_TURN ? 0 : config->turn_mode_size - 1;
      for (uint8_t selected = first_selected; selected < config->turn_mode_size; ++selected) {
        for (const auto pattern : patterns) {
            if (pattern != Straight && config->turn_mode[selected][pattern] == nullptr) continue;
            test_config = config->key; test_selected = selected; test_pattern = pattern;
            Vehicle vehicle{};
            IrSensTask sensors;
            TestMotion motion(&vehicle, &sensors, 1);
            RunDispatch run;
            motion.active_ff_gain_set(&ff_gain_straight_suction);
            run.run(&motion, config, pattern, selected);
            bool crossed = false;
            for (const auto &sample : motion.samples) {
                if (sample.distance > 16.0f) break;
                if (pattern != Straight && sample.distance >= motion.turn_start_mm) break;
                if (sample.distance > 12.0f) crossed = true;
                check(sample.acceleration > 0.0f,
                    "initial acceleration dropped to zero at the preliminary-run boundary");
            }
            check(crossed, "startup trace did not cross the preliminary-run boundary");
            if (pattern != Straight) {
                check(std::fabs(motion.physical_mm - motion.turn_start_mm) < 2.0f,
                    "merging preliminary acceleration changed the physical turn start");
            } else {
                check(std::fabs(motion.physical_mm - (13.1f + SECTION)) < 3.0f,
                    "merging preliminary acceleration changed the straight endpoint");
            }
        }
      }
    }
    std::puts("initial straight/turn acceleration and physical distances passed");
}
