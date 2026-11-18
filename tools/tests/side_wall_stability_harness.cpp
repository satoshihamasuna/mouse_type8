/* Copyright 2026 hmsn
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <initializer_list>
#include <utility>

#include "Task/Inc/wall_detection.h"
#include "Task/Inc/sensing_task.h"
#include "Params/sens_table.h"
#include "Task/Inc/search_turn_heading.h"
#include "Task/Inc/turn_pose_ekf.h"
#include "Subsys/Inc/search_turn_diagnostics.h"
#include "Task/Inc/wall_diagnostics.h"
#include "Task/Inc/run_typedef.h"
#include "Params/typedef_run_param.h"

int16_t side_wall_test_adc[5] = {};

// Compile the real interface LED compositor with GPIO pins captured in memory.
static uint8_t led_gpio;
#define LED1_GPIO_Port 0
#define LED2_GPIO_Port 0
#define LED3_GPIO_Port 0
#define LED4_GPIO_Port 0
#define LED5_GPIO_Port 0
#define LED6_GPIO_Port 0
#define LED1_Pin 5
#define LED2_Pin 3
#define LED3_Pin 2
#define LED4_Pin 1
#define LED5_Pin 0
#define LED6_Pin 4
static void HAL_GPIO_WritePin(int, int pin, int value) {
    led_gpio = (led_gpio & ~(1U << pin)) | ((value & 1U) << pin);
}
#include "led_interface.inc"

class CtrlTask_type8 {
public:
    t_bool enabled = False;
    t_exeStatus status = complete;
    static CtrlTask_type8 &getInstance() { static CtrlTask_type8 value; return value; }
    t_bool is_control_enable() const { return enabled; }
    t_exeStatus motion_exeStatus_get() const { return status; }
};
static uint32_t test_irq_mask;
static uint32_t __get_PRIMASK() { return test_irq_mask; }
static void __disable_irq() { test_irq_mask = 1; }
static void __set_PRIMASK(uint32_t value) { test_irq_mask = value; }
static int ntlibc_strcmp(const char *a, const char *b) { return std::strcmp(a, b); }
#include "wall_ekf_command.inc"
#include "wall_edge_command.inc"

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    std::exit(1); } } while (false)

static bool near(float actual, float expected)
{
    return std::fabs(actual - expected) < 0.0001f;
}

// The production search-turn state machine and wall-control connection run
// against the real IR task. Only the clock, motor gains and turn-table shape
// are stubbed; these checks exercise mode/frame changes within the same IRQ.
class SearchTurnRuntimeHarness {
public:
    Vehicle storage;
    Vehicle *vehicle = &storage;
    IrSensTask_type8 sensors{};
    IrSensTask *ir_sens = &sensors;
    SearchTurnHeading search_turn_heading;
    struct {
        param_element turn_state, velo, fix_prev_run, fix_post_run, turn_time_ms, rad_max_velo;
    } motion_plan;
    t_param turn_motion_param{};
    t_straight_param straight_motion_param{};
    t_runControl control = STRAIGHT_STATE;
    float clock_ms = 0;
    static constexpr int deltaT_ms = 1;
    float run_time_ms_get() const { return clock_ms; }
    void run_time_ms_update() { clock_ms += deltaT_ms; }
    void motion_control_set(t_runControl value) { control = value; }
    t_runControl motion_control_get() const { return control; }
    void motion_pattern_set(t_run_pattern) {}
    void motion_exeStatus_set(t_exeStatus) {}
    void active_ff_gain_set(const t_ff_gain *) {}
    float get_turn_table_value(float, float) { return .5f; }
    void SetIdeal_wall_control() {
#include "search_wall_control.inc"
    }
    void update() {
#include "search_turn_runtime.inc"
    }
};

static void search_turn_runtime()
{
    const float pi = 3.14159265358979323846f;
    const t_pid_gain gain = {1,0,0};
    for (t_turn_dir direction : {Turn_R,Turn_L}) {
        SearchTurnRuntimeHarness motion;
        const float angle = direction == Turn_L ? pi/2 : -pi/2;
        const t_turn_param_table table = {.37f,26,9,9,angle*180/pi,direction,0};
        motion.turn_motion_param = {&table,&gain,&gain};
        motion.straight_motion_param = {nullptr,&gain,&gain};
        motion.motion_plan.velo.set(.37f);
        motion.motion_plan.turn_time_ms.set(100);
        motion.motion_plan.rad_max_velo.set(angle/.1f);
        motion.motion_plan.turn_state.set(Prev_Turn);
        motion.sensors.WallEdgeMode_Set(WallPositionCorrection::Mode::Shadow);
        motion.sensors.EnableIrSensStraight();
        motion.sensors.PrimeSearchTurnExit(.012f,true,.002f);
        motion.sensors.r_wall_corner = True;
        const auto generation = motion.sensors.wall_edge_generation;
        motion.vehicle->ego.length.set(10);
        motion.update();
        CHECK(motion.control == SLATURN_STATE);
        CHECK(motion.sensors.wall_edge_generation != generation);
        CHECK(motion.sensors.r_wall_corner == False);
        const auto rotating_generation = motion.sensors.wall_edge_generation;
        motion.sensors.IrSensMotion_Set(TURN_IRSENS);
        CHECK(motion.sensors.wall_edge_generation == rotating_generation);
        auto &diagnostics = PillarDiagnostics::instance(); diagnostics.begin();
        for (int i=0;i<99;++i) {
            // Repeated ADC peaks while rotating must never become pillar events.
            side_wall_test_adc[sensor_sl] = side_wall_test_adc[sensor_sr] = (i%20)*50;
            side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
            motion.vehicle->ego.rad_velo.set(angle/.1f);
            motion.sensors.IrSensorSet(motion.vehicle,.37f,i);
            motion.sensors.RecordPillarCandidates(motion.vehicle,Search_slalom_L,SLATURN_STATE,direction,false);
            motion.update();
            CHECK(motion.sensors.wall_edge_generation == rotating_generation);
        }
        CHECK(diagnostics.size() == 0);
        motion.vehicle->ego.rad_velo.set(angle/.1f);
        motion.update(); // Main -> Post_Turn and wall control in the same IRQ.
        CHECK(motion.control == STRAIGHT_STATE);
        CHECK(motion.sensors.wall_edge_generation != rotating_generation);
        const auto exit_generation = motion.sensors.wall_edge_generation;
        motion.sensors.IrSensMotion_Set(STRAIGHT_IRSENS);
        CHECK(motion.sensors.wall_edge_generation == exit_generation);
        CHECK(motion.sensors.CanContinueStraightWallPose());
        CHECK(std::fabs(motion.sensors.StraightHeadingForTurn()-.012f) < .0001f);
        CHECK(motion.sensors.r_wall_corner == False && motion.sensors.l_wall_corner == False);
        diagnostics.stop();
    }
}

static void search_turn_heading_handoff_math()
{
    const float pi = 3.14159265358979323846f;
    SearchTurnHeading heading;
    heading.begin(0.01f, -pi / 2.0f);
    for (int i = 0; i < 100; ++i) heading.predict((-pi / 2.0f - 0.03f) / .1f, .001f);
    CHECK(heading.valid() && std::fabs(heading.residual_rad() + .02f) < .0001f);
    heading.begin(-.01f, pi / 2.0f);
    for (int i = 0; i < 100; ++i) heading.predict((pi / 2.0f + .03f) / .1f, .001f);
    CHECK(heading.valid() && std::fabs(heading.residual_rad() - .02f) < .0001f);
    heading.predict(std::numeric_limits<float>::quiet_NaN(), .001f);
    CHECK(!heading.valid());
    heading.begin(0.0f, pi / 2.0f);
    heading.predict(1.0f, .1f);
    CHECK(!heading.valid());
    heading.begin(0.0f, 0.0f);
    heading.predict(20.0f, .019f);
    CHECK(!heading.valid());
}

static void search_turn_exit_reacquisition()
{
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    task.EnableIrSensStraight();
    vehicle.ideal.velo.set(.37f); vehicle.ego.velo.set(.37f);
    side_wall_test_adc[sensor_sl] = sens_sl_table[0];
    side_wall_test_adc[sensor_sr] = 0;
    for (int i = 0; i < 30; ++i) { task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f); }
    vehicle.ego.rad_velo.set(1.0f);
    task.PrimeSearchTurnExit(.04f, true);
    task.SetWallControl_RadVelo(&vehicle, 1.0f); // Same IRQ as the handoff: no second yaw integration.
    CHECK(near(task.control_ir_dot.get(), .04f));
    side_wall_test_adc[sensor_sl] = sens_sl_table[4];
    side_wall_test_adc[sensor_sr] = sens_sr_table[4];
    for (int i = 0; i < 9; ++i) {
        vehicle.ego.radian.set(0.0f);
        task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
        CHECK(near(vehicle.ideal.rad_velo.get(), 0.0f));
        CHECK(!task.straight_wall_ekf_active);
    }
    task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(task.straight_wall_ekf_active);
    CHECK(std::fabs(task.straight_wall_estimate.heading_rad - .05f) < .0001f);
    const float before = task.control_ir_dot.get();
    CHECK(task.CanContinueStraightWallPose());
    task.ResetStraightWallControl(true);
    task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(std::fabs(task.control_ir_dot.get() - before - .001f) < .0001f);
    vehicle.ego.rad_velo.set(0.0f);
    side_wall_test_adc[sensor_sl] = sens_sl_table[2]; // New passage has a real +10 mm offset.
    side_wall_test_adc[sensor_sr] = sens_sr_table[6];
    for (int i = 0; i < 100; ++i) { task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f); }
    CHECK(task.straight_wall_estimate.observed);
    CHECK(vehicle.ego.x_point.get() > 8.0f && vehicle.ego.x_point.get() < 12.0f);
    task.ResetStraightWallControl();
    CHECK(!task.CanContinueStraightWallPose());
}

static void search_turn_heading_covariance_transfer()
{
    const float pi = 3.14159265358979323846f;
    const float prior = .02f * .02f;
    const float walk = wall_ekf_param.heading_walk_rad_sqrt_s;
    const float unknown = wall_ekf_param.initial_heading_std_rad * wall_ekf_param.initial_heading_std_rad;
    const float signs[] = {-1.0f, 1.0f};
    for (float sign : signs) {
        SearchTurnHeading heading;
        heading.begin(.01f*sign, sign*pi/2.0f, prior);
        for (int i = 0; i < 200; ++i) heading.predict(sign*pi/.4f, .001f);
        CHECK(heading.valid() && std::fabs(heading.residual_rad()-.01f*sign) < .0001f);
        CHECK(std::fabs(heading.variance_rad2()-(prior+walk*walk*.2f)) < .00000001f);
        StraightWallEkf filter;
        filter.start(0.0f, heading.residual_rad(), heading.variance_rad2());
        CHECK(near(filter.estimate().heading_rad, heading.residual_rad()));
        CHECK(std::fabs(filter.estimate().variance_heading_rad2-heading.variance_rad2()) < .000000001f);
        CHECK(!filter.estimate().observed && !filter.heading_trusted()); // A prior is not a wall observation.
        CHECK(near(filter.estimate().variance_lateral_m2,
            wall_ekf_param.initial_lateral_std_m * wall_ekf_param.initial_lateral_std_m));
        heading.predict(std::numeric_limits<float>::quiet_NaN(), .001f);
        CHECK(!heading.valid());
    }
    const float invalid_values[] = {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()};
    for (float invalid : invalid_values) {
        SearchTurnHeading heading;
        heading.begin(0.0f, 0.0f, invalid);
        CHECK(near(heading.variance_rad2(), unknown));
        StraightWallEkf filter;
        filter.start(0.0f, 0.0f, invalid);
        CHECK(near(filter.estimate().variance_heading_rad2, unknown));
    }
}

static void search_turn_covariance_real_ir_handoff()
{
    const float prior = .02f * .02f;
    const float walk = wall_ekf_param.heading_walk_rad_sqrt_s;
    const float unknown = wall_ekf_param.initial_heading_std_rad * wall_ekf_param.initial_heading_std_rad;
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    task.EnableIrSensStraight();
    vehicle.ideal.velo.set(.37f); vehicle.ego.velo.set(.37f);
    side_wall_test_adc[sensor_sl] = sens_sl_table[4];
    side_wall_test_adc[sensor_sr] = sens_sr_table[4];
    side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
    task.PrimeSearchTurnExit(.02f, true, prior);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(std::fabs(task.StraightHeadingVarianceForTurn()-prior) < .000000001f);
    for (int i = 0; i < 10; ++i) {
        task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
        if (i < 9) CHECK(!task.straight_wall_ekf_active);
    }
    CHECK(task.straight_wall_ekf_active);
    CHECK(std::fabs(task.straight_wall_estimate.variance_heading_rad2-(prior+walk*walk*.010f)) < .00000001f);
    CHECK(!task.straight_wall_estimate.observed && !task.straight_wall_ekf_used);
    for (int i = 0; i < 120; ++i) { task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f); }
    CHECK(task.straight_wall_ekf_used);
    const float retained = task.StraightHeadingVarianceForTurn();
    task.ResetStraightWallControl(true);
    CHECK(near(task.StraightHeadingVarianceForTurn(), retained));
    task.PrimeSearchTurnExit(.02f, false, prior);
    CHECK(near(task.StraightHeadingVarianceForTurn(), unknown));
    task.PrimeSearchTurnExit(.02f, true); // Unknown incoming covariance must stay conservative.
    CHECK(near(task.StraightHeadingVarianceForTurn(), unknown));
    task.ResetStraightWallControl();
    CHECK(near(task.StraightHeadingVarianceForTurn(), unknown));
}

static SearchTurnGuard::Input reliable_turn_input()
{
    SearchTurnGuard::Input input;
    for (auto &state : input.range) state = IrRangeState::InRange;
    input.left_mm = input.right_mm = 45.0f;
    input.left_wall = input.right_wall = input.front_wall = true;
    input.front_left_mm = 95.0f; input.front_right_mm = 106.0f;
    input.ekf_control = input.pose_trusted = true;
    input.ekf_pose_used = true;
    input.lateral_variance_mm2 = 1.0f;
    input.heading_variance_rad2 = .0001f;
    return input;
}

static void search_turn_front_confirmation_and_fallbacks()
{
    SearchTurnGuard guard;
    auto input = reliable_turn_input();
    for (int i = 0; i < 10; ++i) guard.update(105.0f - i * .5f, 114.0f - i * .5f, .5f, true);
    CHECK(guard.history().same_surface && !guard.history().confirmed);
    guard.update(100.0f, 110.5f, .5f, true);
    auto output = SearchTurnGuard::decide(input, guard.history());
    CHECK(output.reason == SearchTurnGuard::Normal && (output.flags & SearchTurnGuard::Suppressed));
    for (int i = 1; i <= 6; ++i) guard.update(100.0f-i*.5f, 110.5f-i*.5f, .5f, true);
    CHECK(guard.history().confirmed);
    CHECK(SearchTurnGuard::decide(input, guard.history()).reason == SearchTurnGuard::FrontDifference);
    SearchTurnGuard::History short_history{true, false};
    input.ekf_control = false;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::FrontDifference);
    input.ekf_control = true; input.pose_trusted = false;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::FrontDifference);
    input = reliable_turn_input(); input.heading_variance_rad2 = .01f;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::FrontDifference);
    input = reliable_turn_input(); input.lateral_mm = 6.0f;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::FrontDifference);
    input = reliable_turn_input(); input.gyro_radps = 1.0f;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::FrontDifference);
    input = reliable_turn_input(); input.front_right_mm = 115.0f;
    output = SearchTurnGuard::decide(input, short_history);
    CHECK(output.reason == SearchTurnGuard::FrontDifference && (output.flags & SearchTurnGuard::HardDifference));
    input = reliable_turn_input(); input.front_left_mm = input.front_right_mm = 40.0f;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::FrontNear);
    input = reliable_turn_input(); input.right_mm = 30.0f;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::Side);
    input = reliable_turn_input(); input.right_turn = true; input.left_mm = 65.0f;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::Side);
    input = reliable_turn_input(); input.front_wall = false;
    CHECK(SearchTurnGuard::decide(input, short_history).reason == SearchTurnGuard::Normal);
    input = reliable_turn_input();
    CHECK(SearchTurnGuard::decide(input, {}).reason == SearchTurnGuard::FrontDifference);
    guard.update(80.0f, 87.0f, .5f, true);
    CHECK(!guard.history().confirmed);
    guard.update(80.0f, 90.0f, -.5f, true);
    CHECK(!guard.history().same_surface);
    for (int i = 0; i < 20; ++i) guard.update(100.0f, 109.0f, .5f, true);
    CHECK(guard.history().same_surface);
    for (int i = 0; i < 5; ++i) guard.update(100.0f, 109.0f, 0.0f, true);
    CHECK(!guard.history().same_surface);
    guard.update(125.0f, 125.0f, .5f, true);
    CHECK(!guard.history().same_surface);
}

static int16_t front_adc_at(const int16_t *table, float distance_mm)
{
    const float index = (distance_mm - 40.0f) / 5.0f;
    const int i = static_cast<int>(index);
    return static_cast<int16_t>(table[i] + (table[i+1] - table[i]) * (index-i));
}

static void search_turn_guard_real_ir_adapter()
{
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    task.EnableIrSensStraight();
    vehicle.ideal.velo.set(.37f); vehicle.ego.velo.set(.37f);
    side_wall_test_adc[sensor_sl] = sens_sl_table[4]; side_wall_test_adc[sensor_sr] = sens_sr_table[4];
    side_wall_test_adc[sensor_fl] = front_adc_at(sens_fl_table, 95.0f);
    side_wall_test_adc[sensor_fr] = front_adc_at(sens_fr_table, 104.0f);
    for (int i = 0; i < 800; ++i) { task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f); }
    // Approach a consistent front plane; fixed ranges while moving are not a wall model.
    for (int i = 0; i < 15; ++i) {
        side_wall_test_adc[sensor_fl] = front_adc_at(sens_fl_table, 100.0f-i*.37f);
        side_wall_test_adc[sensor_fr] = front_adc_at(sens_fr_table, 109.0f-i*.37f);
        task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
    }
    side_wall_test_adc[sensor_fl] = front_adc_at(sens_fl_table, 100.0f-15*.37f);
    side_wall_test_adc[sensor_fr] = front_adc_at(sens_fr_table, 110.5f-15*.37f);
    task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
    auto output = task.SearchTurnDecision(true, true);
    CHECK(output.reason == SearchTurnGuard::Normal && (output.flags & SearchTurnGuard::Suppressed));
    for (int i = 16; i < 28; ++i) {
        side_wall_test_adc[sensor_fl] = front_adc_at(sens_fl_table, 100.0f-i*.37f);
        side_wall_test_adc[sensor_fr] = front_adc_at(sens_fr_table, 110.5f-i*.37f);
        task.IrSensorSet(&vehicle, .37f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
    }
    CHECK(task.SearchTurnDecision(true, true).reason == SearchTurnGuard::FrontDifference);
    task.WallEkfMode_Set(IrSensTask::WallEkfMode::Shadow);
    CHECK(task.SearchTurnDecision(true, true).reason == SearchTurnGuard::FrontDifference);
    task.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy);
    CHECK(task.SearchTurnDecision(true, true).reason == SearchTurnGuard::FrontDifference);
}

static void search_turn_diagnostics_bounds()
{
    auto &diagnostics = SearchTurnDiagnostics::instance();
    diagnostics.begin();
    auto decision = SearchTurnGuard::decide(reliable_turn_input(), {true, false});
    for (int i = 0; i < 70; ++i) diagnostics.record(i, 1, 2, 0, decision);
    CHECK(diagnostics.total() == 70 && diagnostics.size() == 64 && diagnostics.suppressed() == 70);
    CHECK(diagnostics.count(SearchTurnGuard::Normal) == 70);
    diagnostics.finish_and_print();
    diagnostics.record(100, 1, 2, 0, decision);
    CHECK(diagnostics.total() == 70);
    diagnostics.begin();
    CHECK(diagnostics.total() == 0 && diagnostics.size() == 0);
}

static float search_turn_post_correction(float lateral_mm, t_turn_dir direction)
{
    Vehicle state{};
    Vehicle *vehicle = &state;
    vehicle->ego.x_point.set(lateral_mm);
    struct TurnFields { t_turn_dir turn_dir; } fields{direction};
    struct TurnRef { const TurnFields *param; } turn{&fields};
    const auto *turn_param = &turn;
    struct Plan { param_element fix_post_run; } motion_plan{};
    // Execute the actual correction block from Init_Motion_search_turn.
    #include "search_turn_post_correction.inc"
    return motion_plan.fix_post_run.get();
}

static void search_turn_post_correction_sign_and_limits()
{
    struct Case { float lateral_mm, right_mm, left_mm; };
    const Case cases[] = {
        {-12.0f, -10.0f, 10.0f}, {12.0f, 10.0f, -10.0f},
        {-35.0f, -10.0f, 10.0f}, {35.0f, 10.0f, -10.0f},
        {-10.001f, -10.0f, 10.0f}, {10.001f, 10.0f, -10.0f},
        {-10.0f, -10.0f, 10.0f}, {10.0f, 10.0f, -10.0f},
        {-9.999f, -9.999f, 9.999f}, {9.999f, 9.999f, -9.999f},
        {-4.5f, -4.5f, 4.5f}, {4.5f, 4.5f, -4.5f}, {0.0f, 0.0f, 0.0f},
    };
    for (const auto &test : cases) {
        CHECK(near(search_turn_post_correction(test.lateral_mm, Turn_R), test.right_mm));
        CHECK(near(search_turn_post_correction(test.lateral_mm, Turn_L), test.left_mm));
    }
}

static SideWallStability qualified(float distance = 50.0f)
{
    SideWallStability side;
    side.update(distance, 0.0f, true);
    side.update(distance, 6.0f, true);
    side.update(distance, 5.0f, true);
    side.update(distance, 5.0f, true);
    CHECK(side.ready());
    return side;
}

static void check_reset(const SideWallStability &side)
{
    CHECK(!side.ready());
    CHECK(side.stable_distance_mm() == 0.0f);
    CHECK(side.difference_mm() == 0.0f);
}

static void window()
{
    SideWallStability side;
    side.update(50.0f, 4.0f, true); // First travel only establishes the first sample.
    side.update(50.0f, 5.5f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
    side.update(50.0f, 0.5f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
    side.update(50.0f, 3.0f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 3.0f);
    side.update(50.0f, 6.0f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 9.0f);
    side.update(50.0f, 1.0f, true);
    CHECK(side.ready() && side.stable_distance_mm() == 10.0f);
    for (int i = 0; i < 100; ++i) side.update(50.0f, 6.0f, true);
    CHECK(side.ready() && side.stable_distance_mm() == 10.0f);
}

static void partial_window()
{
    SideWallStability side;
    side.update(50.0f, 0.0f, true);
    side.update(50.0f, 4.0f, true);
    CHECK(side.stable_distance_mm() == 0.0f);
    side.update(50.0f, 4.0f, true);
    CHECK(side.stable_distance_mm() == 2.0f);
    side.update(50.0f, 5.0f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 7.0f);
    side.update(50.0f, 3.0f, true);
    CHECK(side.ready());
}

static void helper_speed()
{
    const float steps[] = {0.125f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f};
    for (float step : steps) {
        SideWallStability side;
        side.update(50.0f, 0.0f, true);
        for (float position = step; position <= 20.0f; position += step) {
            side.update(50.0f, step, true);
            CHECK(side.ready() == (position >= 16.0f));
        }
    }
}

static void linear_wall()
{
    // For d(x)=50+0.1*x, the mean over x,x-1,...,x-6 lags by 0.3 mm.
    const float steps[] = {0.125f, 0.3f, 0.5f, 1.0f, 2.0f, 4.0f};
    for (float step : steps) {
        SideWallStability side;
        side.update(50.0f, 0.0f, true);
        double position = 0.0;
        while (position < 20.0) {
            float delta = static_cast<float>(std::fmin(static_cast<double>(step), 20.0 - position));
            position += delta;
            side.update(50.0f + 0.1f * static_cast<float>(position), delta, true);
        }
        CHECK(near(side.difference_mm(), 0.3f));
        CHECK(side.ready());
    }
}

static void decimal_boundaries()
{
    SideWallStability constant;
    constant.update(50.0f, 0.0f, true);
    for (int i = 1; i <= 80; ++i) {
        constant.update(50.0f, 0.2f, true);
        CHECK(constant.ready() == (i == 80));
    }
    CHECK(constant.stable_distance_mm() == 10.0f);

    SideWallStability split_boundary;
    split_boundary.update(50.0f, 0.0f, true);
    for (int cycle = 0; cycle < 16; ++cycle) {
        split_boundary.update(50.0f + 0.1f * (cycle + 0.9f), 0.9f, true);
        split_boundary.update(50.0f + 0.1f * (cycle + 1.0f), 0.1f, true);
        if (cycle == 5) CHECK(near(split_boundary.difference_mm(), 0.3f));
        CHECK(split_boundary.ready() == (cycle == 15));
    }
}

static void linear_activation()
{
    const float steps[] = {0.125f, 0.5f, 1.0f, 2.0f};
    for (float step : steps) {
        SideWallStability side;
        side.update(50.0f, 0.0f, true);
        for (float position = step; position <= 20.0f; position += step) {
            side.update(50.0f + 0.1f * position, step, true);
            if (position >= 6.0f) CHECK(near(side.difference_mm(), 0.3f));
            CHECK(side.ready() == (position >= 16.0f));
        }
    }
    SideWallStability changing_step;
    changing_step.update(50.0f, 0.0f, true);
    struct Segment { float end, step; };
    const Segment segments[] = {{4.0f, 2.0f}, {8.0f, 0.5f}, {12.0f, 0.125f}, {20.0f, 1.0f}};
    float position = 0.0f;
    for (const auto &segment : segments) {
        while (position < segment.end) {
            position += segment.step;
            changing_step.update(50.0f + 0.1f * position, segment.step, true);
            if (position >= 6.0f) CHECK(near(changing_step.difference_mm(), 0.3f));
            CHECK(changing_step.ready() == (position >= 16.0f));
        }
    }
}

static void difference_limit()
{
    const float limit = SideWallStability::kDifferenceLimitMm;
    const float margin = limit * 0.02f;
    // A linear change over six mm has a seven-point mean at its midpoint.
    auto accepted = qualified(0.0f);
    accepted.update(2.0f * (limit - margin), 6.0f, true);
    CHECK(near(accepted.difference_mm(), limit - margin) && accepted.ready());
    auto boundary = qualified(0.0f);
    boundary.update(2.0f * limit, 6.0f, true);
    CHECK(near(boundary.difference_mm(), limit));
    CHECK(!boundary.ready() && boundary.stable_distance_mm() == 0.0f);
    auto rejected = qualified(0.0f);
    rejected.update(2.0f * (limit + margin), 6.0f, true);
    CHECK(near(rejected.difference_mm(), limit + margin) && !rejected.ready());
}

static void stationary()
{
    SideWallStability side;
    for (int i = 0; i < 10000; ++i) side.update(50.0f, 0.0f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
    side = qualified();
    side.update(50.0f, 0.0f, true);
    CHECK(side.ready());
    side.update(50.0f + 2.0f * SideWallStability::kDifferenceLimitMm, 0.0f, true);
    CHECK(std::fabs(side.difference_mm()) > SideWallStability::kDifferenceLimitMm);
    CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
    for (int i = 0; i < 10000; ++i) side.update(50.0f, 0.0f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
}

static void reset()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float invalid[][2] = {{50.0f, -0.001f}, {nan, 1.0f}, {inf, 1.0f},
                                {50.0f, nan}, {50.0f, inf}};
    for (const auto &sample : invalid) {
        auto side = qualified();
        side.update(sample[0], sample[1], true);
        check_reset(side);
        side.update(50.0f, 6.0f, true);
        CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
    }
    auto side = qualified();
    side.update(50.0f, 1.0f, false);
    check_reset(side);
    side = qualified();
    side.reset();
    check_reset(side);
}

static void gap()
{
    auto side = qualified();
    side.update(50.0f, 6.001f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
    side.update(50.0f, 6.0f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 0.0f);
    side.update(50.0f, 5.0f, true);
    CHECK(!side.ready() && side.stable_distance_mm() == 5.0f);
    side.update(50.0f, 5.0f, true);
    CHECK(side.ready());
}

struct Fixture
{
    IrSensTask task{};
    Vehicle vehicle{};
    Fixture()
    {
        task.sen_l.distance = task.sen_r.distance = 50.0f;
        task.sen_fl.distance = task.sen_fr.distance = 125.0f;
        task.EnableIrSensStraight();
    }
    void sample(float step) { task.IrSensorWallSet(step); }
    void advance(float distance, float step = 1.0f)
    {
        for (float position = 0.0f; position < distance;) {
            const float delta = std::fmin(step, distance - position);
            sample(delta);
            position += delta;
        }
    }
    void qualify()
    {
        sample(0.0f);
        advance(16.0f);
        CHECK(task.sen_l.is_control == True && task.sen_r.is_control == True);
    }
};

static void integration_speed()
{
    const float steps[] = {0.125f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f};
    for (float step : steps) {
        Fixture fixture;
        fixture.sample(0.0f);
        CHECK(fixture.task.sen_l.is_control == False && fixture.task.sen_r.is_control == False);
        for (float position = step; position <= 20.0f; position += step) {
            fixture.sample(step);
            CHECK((fixture.task.sen_l.is_control == True) == (position >= 16.0f));
            CHECK((fixture.task.sen_r.is_control == True) == (position >= 16.0f));
        }
    }
    Fixture few_calls;
    few_calls.sample(0.0f);
    few_calls.sample(6.0f);
    few_calls.sample(6.0f);
    CHECK(few_calls.task.sen_r.is_control == False);
    few_calls.sample(4.0f);
    CHECK(few_calls.task.sen_l.is_control == True && few_calls.task.sen_r.is_control == True);
    CHECK(few_calls.task.sen_l.control_cnt < 10 && few_calls.task.sen_r.control_cnt < 10);
}

static void integration_decimal()
{
    struct Profile { float step; int samples; };
    const Profile profiles[] = {{0.2f, 80}, {0.5f, 32}, {1.6f, 10}};
    for (const auto &profile : profiles) {
        Fixture fixture;
        fixture.sample(0.0f);
        for (int i = 1; i <= profile.samples; ++i) {
            fixture.sample(profile.step);
            CHECK((fixture.task.sen_l.is_control == True) == (i == profile.samples));
            CHECK((fixture.task.sen_r.is_control == True) == (i == profile.samples));
        }
        CHECK(fixture.task.sen_l.control_distance == 10.0f);
        CHECK(fixture.task.sen_r.control_distance == 10.0f);
    }
}

static void integration_near()
{
    Fixture fixture;
    fixture.task.sen_l.distance = 44.0f;
    fixture.sample(0.0f);
    CHECK(fixture.task.sen_l.is_control == True && fixture.task.sen_r.is_control == False);
    CHECK(fixture.task.sen_l.control_th == STRAIGHT_REF);
    CHECK(fixture.task.sen_l.control_distance == 0.0f);
    fixture.task.sen_fl.distance = 66.0f;
    fixture.sample(0.0f);
    CHECK(fixture.task.sen_l.is_control == False);
    fixture.task.sen_fl.distance = 125.0f;
    fixture.sample(-1.0f);
    CHECK(fixture.task.sen_l.is_control == True);
}

static void integration_front()
{
    Fixture fixture;
    fixture.qualify();
    fixture.task.sen_fr.distance = SIDECONTROL_FR_THRESHOLD;
    fixture.sample(0.0f);
    CHECK(fixture.task.sen_r.is_control == False && fixture.task.sen_l.is_control == True);
    CHECK(fixture.task.sen_r.control_distance == 0.0f);
    CHECK(fixture.task.sen_l.control_distance == 10.0f);
    fixture.task.sen_fr.distance = SIDECONTROL_FR_THRESHOLD + 0.01f;
    fixture.sample(0.0f);
    CHECK(fixture.task.sen_r.is_control == False);
    fixture.advance(16.0f);
    CHECK(fixture.task.sen_r.is_control == True);
}

static void integration_loss()
{
    Fixture fixture;
    fixture.qualify();
    fixture.task.sen_r.distance = 80.0f;
    fixture.sample(1.0f);
    CHECK(fixture.task.conv_Sensin2Wall(sensor_sr) == NOWALL);
    CHECK(fixture.task.sen_r.control_distance == 0.0f);
    CHECK(fixture.task.sen_l.is_control == True);
    fixture.task.sen_r.distance = 50.0f;
    fixture.sample(0.0f);
    CHECK(fixture.task.conv_Sensin2Wall(sensor_sr) == WALL);
    CHECK(fixture.task.sen_r.is_control == False && fixture.task.sen_l.is_control == True);
    fixture.advance(15.0f);
    CHECK(fixture.task.sen_r.is_control == False);
    fixture.sample(1.0f);
    CHECK(fixture.task.sen_r.is_control == True);
}

static void integration_corner()
{
    Fixture fixture;
    fixture.task.EnableIrSensDiagonal();
    fixture.qualify();
    bool found = false;
    for (float x = -30.0f; x <= 30.0f; x += .5f) {
        fixture.task.sen_r.distance = std::fmin(80.0f, 40.0f + std::fabs(x));
        fixture.sample(.5f);
        if (fixture.task.r_wall_corner == True) { found = true; break; }
    }
    CHECK(found);
    CHECK(fixture.task.sen_r.is_control == False && fixture.task.sen_l.is_control == True);
    CHECK(fixture.task.sen_r.control_distance == 0.0f);
}

static void diagonal_edge_spatial_position()
{
    float first = 0.0f;
    for (float step : {.125f, .5f, 1.0f, 1.6f, 3.2f}) {
        DiagonalWallEdge detector;
        int events = 0;
        for (float x = 0.0f; x < 100.0f; x += step) {
            const auto event = detector.update(std::fmin(80.0f, 40.0f + std::fabs(x-40.0f)), x == 0.0f ? 0.0f : step);
            if (!event.detected) continue;
            ++events;
            const float reference = x - event.delay_mm;
            if (first == 0.0f) first = reference;
            CHECK(std::fabs(reference - first) < 1.0f);
            CHECK(event.delay_mm >= diagonal_edge_param.confirm_travel_mm);
            CHECK(event.age_ticks > 0);
        }
        CHECK(events == 1);
    }
}

static void diagonal_edge_clipped_plateau()
{
    DiagonalWallEdge detector;
    float old_ranges[20] = {};
    float old_sum = 0.0f, old_diff = 0.0f;
    int old_events = 0, events = 0;
    for (int i = 0; i < 180; ++i) {
        const float x = i*.5f;
        const float range = std::fmin(80.0f, 25.0f + std::fmax(0.0f, std::fmax(20.0f-x, x-40.0f)));
        old_sum += range - old_ranges[i%20]; old_ranges[i%20] = range;
        const float diff = range - old_sum/20.0f;
        if (diff > 0.0f && old_diff < 0.0f && range < 60.0f) ++old_events;
        old_diff = diff;
        const auto event = detector.update(range, i == 0 ? 0.0f : .5f);
        if (event.detected) {
            ++events;
            CHECK(std::fabs(x - event.delay_mm - 40.0f) < .01f);
        }
    }
    CHECK(old_events == 0 && events == 1);
}

static void diagonal_edge_noise_stop_and_reset()
{
    DiagonalWallEdge detector;
    for (int i = 0; i < 100; ++i)
        CHECK(!detector.update(i%2 ? 45.1f : 44.9f, .5f).detected);
    CHECK(!detector.update(80.0f, .5f).detected);
    for (int i = 0; i < 50; ++i) CHECK(!detector.update(45.0f, .5f).detected);
    for (int i = 0; i < 40; ++i) CHECK(!detector.update(60.0f-i*.5f, .5f).detected);
    for (int i = 0; i < 60; ++i) CHECK(!detector.update(25.0f+i*.5f, 0.0f).detected);
    for (int i = 0; i < 40; ++i) CHECK(!detector.update(55.0f+i*.5f, .5f).detected);
    for (int invalid = 0; invalid < 4; ++invalid) {
        detector.reset();
        for (int i = 0; i < 40; ++i) detector.update(60.0f-i*.5f, .5f);
        if (invalid == 0) detector.update(40.0f, -.5f);
        if (invalid == 1) detector.update(40.0f, 7.0f);
        if (invalid == 2) detector.update(std::numeric_limits<float>::quiet_NaN(), .5f);
        if (invalid == 3) detector.update(40.0f, .5f, false);
        for (int i = 0; i < 50; ++i) CHECK(!detector.update(40.0f+i*.5f, .5f).detected);
    }
}

static void diagonal_edge_rearm_and_zero_counts()
{
    for (bool zero_counts : {false, true}) for (float far : {55.0f, 80.0f}) {
        DiagonalWallEdge detector;
        int events = 0;
        for (float x = 0.0f; x < 200.0f; x += .5f) {
            const float phase = std::fmod(x, 100.0f);
            const float range = std::fmin(far, 40.0f+std::fabs(phase-40.0f));
            if (zero_counts) CHECK(!detector.update(range, 0.0f).detected);
            if (detector.update(range, .5f).detected) ++events;
        }
        CHECK(events == 2);
    }
}

static void diagonal_edge_recovers_a_short_rebound()
{
    DiagonalWallEdge detector;
    int events = 0;
    for (float x = 0.0f; x < 100.0f; x += .5f) {
        const float range = near(x, 44.5f) ? 43.7f : std::fmin(80.0f, 40.0f+std::fabs(x-40.0f));
        const auto event = detector.update(range, x == 0.0f ? 0.0f : .5f);
        if (event.detected) { ++events; CHECK(x >= 45.0f); }
    }
    CHECK(events == 1);
}

static void diagonal_corner_pair_delays_and_symmetry()
{
    for (bool right : {false, true}) for (float gap : {DIAG_SECTION+2.0f, 3.0f*DIAG_SECTION+6.0f}) {
        IrSensTask_type8 task{};
        task.r_wall_corner = right ? True : False; task.l_wall_corner = right ? False : True;
        task.r_corner_delay_mm = 2.0f; task.l_corner_delay_mm = 3.0f;
        task.RecordDiagonalCorner(right, 20.0f);
        float error = 99.0f;
        CHECK(!task.DiagonalCornerLateralCorrection(error)); // Opposite event was never seen.
        task.RecordDiagonalCorner(!right, 20.0f + (right ? 3.0f : 2.0f)); // Opposite reference = 20mm.
        task.RecordDiagonalCorner(right, 20.0f + gap + (right ? 2.0f : 3.0f));
        CHECK(task.DiagonalCornerLateralCorrection(error));
        CHECK(near(error, right ? -1.0f : 1.0f));
        task.r_wall_corner = task.l_wall_corner = True;
        CHECK(!task.DiagonalCornerLateralCorrection(error));
        task.Division_Wall_Correction_Reset();
        task.r_wall_corner = right ? True : False; task.l_wall_corner = right ? False : True;
        CHECK(!task.DiagonalCornerLateralCorrection(error));
    }
}

static void diagonal_turn_delay_compensation()
{
    for (int which = 0; which < 2; ++which) for (float delay : {0.0f, .5f, 3.2f}) {
        Vehicle state{}; Vehicle *vehicle = &state;
        struct Fields { float Lend; t_turn_dir turn_dir; } fields{20.0f, Turn_R};
        struct Ref { const Fields *param; } turn_motion_param{&fields};
        struct Plan { param_element fix_post_run; } motion_plan{};
        motion_plan.fix_post_run.set(3.0f);
        const float detect_wall_edge_di = 0.0f;
        vehicle->ego.length.set(10.0f + delay);
        auto ApplyWallPositionCorrection = [&](float reference, WallPositionCorrection::Phase phase, uint8_t) {
            auto result = WallPositionCorrection::evaluate(vehicle->ego.length.get(), reference, delay, phase);
            if (result.accepted()) vehicle->ego.length.set(result.after_mm);
        };
        #include "diagonal_turn_delay.inc"
        CHECK(near(vehicle->ego.length.get() - delay, which < 1 ? 7.0f : 14.0f));
    }
}

static void straight_edge_noise_and_pulse()
{
    Fixture fixture;
    fixture.qualify();
    // A stopped robot must not report an edge just because the IR value changes.
    fixture.task.sen_r.distance = 80.0f;
    fixture.sample(0.0f);
    CHECK(fixture.task.r_wall_corner == False);

    fixture.task.sen_r.distance = 50.0f;
    fixture.advance(4.0f, 0.5f);
    // Repeated 65 mm threshold crossings alone do not establish a wall end.
    const float threshold_noise[] = {65.25f, 64.875f, 65.6875f, 64.5f};
    for (float range : threshold_noise) {
        fixture.task.sen_r.distance = range;
        fixture.sample(0.5f);
        CHECK(fixture.task.r_wall_corner == False);
    }
    fixture.task.sen_r.distance = 80.0f;
    fixture.sample(0.5f);
    CHECK(fixture.task.r_wall_corner == False);
    fixture.sample(0.5f);
    CHECK(fixture.task.r_wall_corner == True);
    // This derivative branch used to retain the previous event in straight mode.
    fixture.task.sen_r.prev_diff = -0.1f;
    fixture.task.sen_r.diff = 0.1f;
    fixture.sample(0.5f);
    CHECK(fixture.task.r_wall_corner == False);
    CHECK(fixture.task.l_wall_corner == False);
}

static void corner_clock_saturates()
{
    Fixture fixture;
    fixture.task.r_corner_time = fixture.task.l_corner_time = 65535U;
    fixture.sample(0.0f);
    CHECK(fixture.task.r_corner_time == 65535U && fixture.task.l_corner_time == 65535U);
}

static void edge_position_across_speeds()
{
    const float steps[] = {0.125f, 0.274f, 0.37f, 0.5f, 1.0f, 1.6f, 3.0f};
    for (const float step : steps) {
        StraightWallEdge detector;
        detector.update(50.0f, 0.0f);
        int events = 0;
        for (float position = step; position < 30.0f; position += step) {
            const float range = std::fmin(80.0f, 50.0f + 5.0f * std::fmax(0.0f, position - 10.0f));
            const auto event = detector.update(range, step);
            if (event.detected) {
                ++events;
                CHECK(std::fabs(position - event.delay_mm - 13.0f) < 0.001f);
                CHECK(event.delay_mm >= 0.5f && event.age_ticks >= 1);
            }
        }
        CHECK(events == 1);
    }
}

static void edge_rejects_spikes_and_invalid_motion()
{
    StraightWallEdge detector;
    auto arm = [&]() {
        detector.reset(); detector.update(50.0f, 0.0f);
        for (int i = 0; i < 8; ++i) CHECK(!detector.update(50.0f, 0.5f).detected);
    };
    arm();
    CHECK(!detector.update(80.0f, 0.5f).detected && detector.pending());
    CHECK(!detector.update(50.0f, 0.5f).detected && !detector.pending());
    CHECK(!detector.update(80.0f, 0.5f).detected);
    CHECK(!detector.update(80.0f, 0.0f).detected && detector.pending());
    CHECK(detector.update(80.0f, 0.5f).detected); // A zero count between pulses is valid.
    for (int i = 0; i < 20; ++i) CHECK(!detector.update(80.0f, 0.5f).detected);
    arm();
    detector.update(80.0f, 0.5f);
    for (int i = 0; i < 40; ++i) CHECK(!detector.update(80.0f, 0.0f).detected);
    CHECK(!detector.pending());
    CHECK(!detector.update(80.0f, 0.5f).detected);
    arm(); detector.update(80.0f, 0.5f);
    CHECK(!detector.update(80.0f, -0.5f).detected && !detector.pending());
    arm(); detector.update(80.0f, 0.5f);
    CHECK(!detector.update(80.0f, 0.5f, false).detected && !detector.pending());
    arm(); detector.update(80.0f, 0.5f);
    CHECK(!detector.update(std::numeric_limits<float>::quiet_NaN(), 0.5f).detected);
    arm(); detector.update(80.0f, 0.5f);
    CHECK(!detector.update(80.0f, 6.1f).detected && !detector.pending());
}

static void edge_adapter_symmetry_offsets_and_pair_validity()
{
    const bool sides[] = {false, true};
    for (const bool right : sides) {
        Fixture fixture;
        fixture.qualify();
        if (right) fixture.task.sen_r.distance = 80.0f;
        else fixture.task.sen_l.distance = 80.0f;
        fixture.sample(0.5f);
        fixture.sample(0.5f);
        CHECK((right ? fixture.task.r_wall_corner : fixture.task.l_wall_corner) == True);
        const float delay = right ? fixture.task.r_corner_delay_mm : fixture.task.l_corner_delay_mm;
        CHECK(delay > 0.5f && near(delay, fixture.task.StraightCornerDelay()));
        fixture.task.RecordStraightCorner(right, 20.0f);
        CHECK(near((right ? fixture.task.r_corner_length : fixture.task.l_corner_length).get(), 20.0f - delay));
        CHECK(!fixture.task.StraightCornerPairValid());
        fixture.task.RecordStraightCorner(!right, 21.0f);
        CHECK(fixture.task.StraightCornerPairValid());
        fixture.task.Division_Wall_Correction_Reset();
        CHECK(!fixture.task.StraightCornerPairValid());
        fixture.sample(0.5f);
        CHECK(fixture.task.r_wall_corner == False && fixture.task.l_wall_corner == False);
        CHECK((right ? fixture.task.sen_l.is_control : fixture.task.sen_r.is_control) == True);
        fixture.task.ResetWallCornerDetection();
        CHECK(fixture.task.r_corner_time == 65535U && fixture.task.l_corner_time == 65535U);
    }
}

static void edge_adc_rotation_and_mode_reentry()
{
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    task.EnableIrSensStraight();
    side_wall_test_adc[sensor_sl] = sens_sl_table[4];
    side_wall_test_adc[sensor_sr] = sens_sr_table[4];
    side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
    for (int i = 0; i < 20; ++i) task.IrSensorSet(&vehicle, 0.5f);
    vehicle.ego.rad_velo.set(3.0f);
    side_wall_test_adc[sensor_sr] = 0;
    for (int i = 0; i < 5; ++i) {
        task.IrSensorSet(&vehicle, 0.5f);
        CHECK(task.r_wall_corner == False);
    }
    vehicle.ego.rad_velo.set(0.0f);
    for (int i = 0; i < 5; ++i) {
        task.IrSensorSet(&vehicle, 0.5f);
        CHECK(task.r_wall_corner == False); // Rotation discarded the old wall.
    }
    side_wall_test_adc[sensor_sr] = sens_sr_table[4];
    for (int i = 0; i < 10; ++i) task.IrSensorSet(&vehicle, 0.5f);
    side_wall_test_adc[sensor_sr] = 0;
    task.IrSensorSet(&vehicle, 0.5f);
    CHECK(task.r_wall_corner == False);
    task.IrSensorSet(&vehicle, 0.5f);
    CHECK(task.r_wall_corner == True && task.r_corner_delay_mm > 0.5f);
    task.EnableIrSensDiagonal();
    task.EnableIrSensStraight();
    for (int i = 0; i < 5; ++i) {
        task.IrSensorSet(&vehicle, 0.5f);
        CHECK(task.r_wall_corner == False);
    }
}

static void edge_recalls_distant_and_short_walls()
{
    const float wall_ranges[] = {50.0f, 62.0f, 64.5f};
    for (const float wall_range : wall_ranges) {
        StraightWallEdge detector;
        detector.update(wall_range, 0.0f);
        detector.update(wall_range, 0.5f); // Only a short visible wall.
        CHECK(!detector.update(80.0f, 0.5f).detected);
        const auto event = detector.update(80.0f, 0.5f);
        CHECK(event.detected && event.delay_mm >= 0.5f);
        for (int i = 0; i < 10; ++i) CHECK(!detector.update(80.0f, 0.5f).detected);
    }
}

static void edge_recalls_weak_distance_rises()
{
    StraightWallEdge detector;
    detector.update(50.0f, 0.0f);
    for (int i = 0; i < 10; ++i) detector.update(50.0f, 0.5f);
    detector.update(62.0f, 0.5f);
    CHECK(!detector.update(65.4f, 0.274f).detected);
    CHECK(!detector.update(66.0f, 0.274f).detected);
    const auto event = detector.update(66.5f, 0.274f);
    CHECK(event.detected); // Do not wait for 68 mm and lose a weak return.
    CHECK(event.delay_mm >= 0.5f);
}

static void edge_recalls_crossing_between_encoder_pulses()
{
    StraightWallEdge detector;
    detector.update(50.0f, 0.0f);
    for (int i = 0; i < 10; ++i) detector.update(50.0f, 0.5f);
    detector.update(64.5f, 0.274f);
    CHECK(!detector.update(66.0f, 0.0f).detected);
    CHECK(detector.pending()); // IR can cross between quantized encoder pulses.
    CHECK(!detector.update(80.0f, 0.274f).detected);
    CHECK(detector.update(80.0f, 0.274f).detected);
    detector.reset(); detector.update(50.0f, 0.0f);
    for (int i = 0; i < 10; ++i) detector.update(50.0f, 0.5f);
    for (int i = 0; i < 10; ++i) detector.update(50.0f, 0.0f);
    CHECK(!detector.update(80.0f, 0.0f).detected && !detector.pending());
    for (int i = 0; i < 10; ++i) CHECK(!detector.update(80.0f, 0.5f).detected);
}

static void edge_recall_real_adc_at_sixty_five_mm()
{
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    task.EnableIrSensStraight();
    side_wall_test_adc[sensor_sl] = sens_sl_table[4];
    side_wall_test_adc[sensor_sr] = sens_sr_table[8]; // 25 + 8 * 5 = 65 mm.
    side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
    task.IrSensorSet(&vehicle, 0.0f);
    task.IrSensorSet(&vehicle, 0.5f);
    CHECK(std::fabs(task.sen_r.distance - 65.0f) < 0.01f);
    side_wall_test_adc[sensor_sr] = 0;
    task.IrSensorSet(&vehicle, 0.0f); // IR transition in a zero-count control tick.
    CHECK(task.r_wall_corner == False);
    task.IrSensorSet(&vehicle, 0.5f);
    CHECK(task.r_wall_corner == True && near(task.r_corner_delay_mm, 0.5f));
    task.IrSensorSet(&vehicle, 0.5f);
    CHECK(task.r_wall_corner == False);
}

static void integration_disabled()
{
    Fixture fixture;
    fixture.task.DisableIrSens();
    fixture.sample(0.0f);
    fixture.advance(16.0f);
    CHECK(fixture.task.sen_l.control_distance == 10.0f);
    CHECK(fixture.task.sen_r.control_distance == 10.0f);
    CHECK(fixture.task.sen_l.is_control == False && fixture.task.sen_r.is_control == False);
    CHECK(fixture.task.sen_l.error == 0.0f && fixture.task.sen_r.error == 0.0f);
    fixture.task.EnableIrSensStraight();
    fixture.sample(0.0f);
    CHECK(fixture.task.sen_l.is_control == True && fixture.task.sen_r.is_control == True);
}

static void integration_speed_gate()
{
    Fixture fixture;
    fixture.task.EnableIrSensDiagonal();
    fixture.task.sen_l.distance = 30.0f;
    fixture.task.sen_r.distance = 31.0f;
    fixture.qualify();
    fixture.vehicle.ego.x_point.set(7.0f);
    fixture.vehicle.ideal.velo.set(0.19f);
    fixture.task.SetWallControl_RadVelo(&fixture.vehicle, 1.0f);
    CHECK(fixture.task.control_ir.get() == 7.0f);
    CHECK(fixture.vehicle.ego.x_point.get() == 7.0f);
    fixture.vehicle.ideal.velo.set(0.21f);
    fixture.vehicle.ideal.rad_velo.set(0.0f);
    fixture.vehicle.ego.velo.set(0.32f);
    fixture.task.SetWallControl_RadVelo(&fixture.vehicle, 1.0f);
    CHECK(fixture.task.control_ir.get() == 0.5f);
    CHECK(fixture.vehicle.ego.x_point.get() == 7.0f);
    const float nominal_accel = fixture.vehicle.ideal.rad_accel.get();
    fixture.vehicle.ideal.rad_velo.set(0.0f);
    fixture.vehicle.ego.velo.set(0.64f);
    fixture.task.SetWallControl_RadVelo(&fixture.vehicle, 1.0f);
    CHECK(near(fixture.vehicle.ideal.rad_accel.get(), 2.0f * nominal_accel));
}

static void integration_adc()
{
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    side_wall_test_adc[sensor_sl] = sens_sl_table[5]; // Exactly 50 mm.
    side_wall_test_adc[sensor_sr] = sens_sr_table[5];
    side_wall_test_adc[sensor_fl] = sens_fl_table[17]; // Exactly 125 mm.
    side_wall_test_adc[sensor_fr] = sens_fr_table[17];
    task.EnableIrSensStraight();
    task.IrSensorSet(&vehicle, 0.0f);
    CHECK(near(task.sen_l.distance, 50.0f) && near(task.sen_r.distance, 50.0f));
    for (int i = 1; i <= 8; ++i) {
        task.IrSensorSet(&vehicle, 2.0f);
        CHECK((task.sen_l.is_control == True) == (i == 8));
        CHECK((task.sen_r.is_control == True) == (i == 8));
    }
    // At 16 mm, only nine temporal readings exist and their old zero-filled
    // average is still far away. Neither temporal condition may veto control.
    CHECK(std::fabs(task.sen_l.diff) > 0.5f && std::fabs(task.sen_r.diff) > 0.5f);
    CHECK(task.sen_l.control_cnt < 10 && task.sen_r.control_cnt < 10);
}

static ZirconiaWallControl::Input wall_input()
{
    ZirconiaWallControl::Input in;
    in.enabled = in.sample_updated = true;
    in.left_m = in.right_m = 0.045f;
    in.left_signal = in.right_signal = 1000.0f;
    in.travel_m = 0.001f;
    in.velocity_mps = 1.0f;
    return in;
}

static void zirconia_symmetry()
{
    ZirconiaWallControl a, b;
    auto right_offset = wall_input();
    right_offset.left_m = 0.050f;
    right_offset.right_m = 0.040f;
    auto left_offset = right_offset;
    left_offset.left_m = 0.040f;
    left_offset.right_m = 0.050f;
    ZirconiaWallControl::Output ra, rb;
    for (int i = 0; i < 150; ++i) {
        ra = a.update(right_offset);
        rb = b.update(left_offset);
        CHECK(near(ra.omega_radps, -rb.omega_radps));
        CHECK(std::fabs(ra.alpha_radps2) <= 20.001f);
        CHECK(std::fabs(ra.omega_radps) <= 1.20001f);
    }
    CHECK(ra.phase == ZirconiaWallControl::Tracking && ra.trusted_mask == 3);
    CHECK(ra.omega_radps > 0.0f && rb.omega_radps < 0.0f);
    // At the same right offset, already pointing left must reduce steering.
    right_offset.heading_rad = 0.15f;
    const float before = ra.omega_radps;
    for (int i = 0; i < 20; ++i) ra = a.update(right_offset);
    CHECK(ra.omega_radps < before);
}

static void zirconia_independent()
{
    ZirconiaWallControl controller;
    auto in = wall_input();
    in.left_m = 0.040f;
    ZirconiaWallControl::Output out;
    for (int i = 0; i < 80; ++i) out = controller.update(in);
    CHECK(out.trusted_mask == 3 && out.transition == 1.0f);
    float previous = out.omega_radps;
    // A close pillar on the right must not pollute the continuous left wall.
    in.right_m = 0.030f;
    in.right_signal = 3000.0f;
    for (int i = 0; i < 3; ++i) out = controller.update(in);
    CHECK(out.trusted_mask == 1 && out.phase == ZirconiaWallControl::Tracking);
    CHECK(near(out.raw_error_m, -0.005f));
    CHECK(out.pose_control && out.transition == 1.0f);
    CHECK(std::fabs(out.omega_radps - previous) <= 0.06001f);
    previous = out.omega_radps;
    in.right_m = 0.080f;
    out = controller.update(in);
    CHECK(out.trusted_mask == 1 && out.observed_mask == 1);
    CHECK(out.phase == ZirconiaWallControl::Tracking && out.transition == 1.0f);
    CHECK(std::fabs(out.omega_radps - previous) <= 0.02001f);
    // Losing the last reliable wall resets yaw, then ramps estimated control.
    in.left_m = 0.080f;
    in.estimated_error_m = 0.004f;
    out = controller.update(in);
    CHECK(out.phase == ZirconiaWallControl::Estimated && out.trusted_mask == 0);
    CHECK(out.omega_radps == 0.0f);
    for (int i = 0; i < 50; ++i) {
        out = controller.update(in);
        CHECK(std::fabs(out.alpha_radps2) <= 4.001f);
        CHECK(std::fabs(out.omega_radps) <= 0.20001f);
    }
    CHECK(out.omega_radps > 0.0f);
}

static void zirconia_spatial_confidence()
{
    const float steps[] = {0.0002f, 0.0005f, 0.001f, 0.002f};
    for (float step : steps) {
        ZirconiaWallControl controller;
        auto in = wall_input();
        in.travel_m = step;
        in.right_m = 0.080f;
        in.left_m = 0.055f; // Far single wall: approaching it is eventually allowed.
        ZirconiaWallControl::Output out;
        float travel = 0.0f;
        for (int i = 0; i < 200 && !out.pose_control; ++i) {
            out = controller.update(in);
            travel += step;
        }
        CHECK(out.pose_control && out.trusted_mask == 1);
        CHECK(travel >= 0.006f && travel <= 0.0141f);
        for (int i = 0; i < 150; ++i) out = controller.update(in);
        CHECK(out.omega_radps > 0.0f);
    }
    ZirconiaWallControl controller;
    auto in = wall_input();
    in.travel_m = 0.0f;
    ZirconiaWallControl::Output out;
    for (int i = 0; i < 200; ++i) out = controller.update(in);
    CHECK(out.phase == ZirconiaWallControl::Confirming && out.trusted_mask == 0);
    CHECK(out.omega_radps == 0.0f);
    in.travel_m = 0.001f;
    in.heading_rad = 0.1f;
    for (int i = 0; i < 80; ++i) {
        const float lateral = in.travel_m * std::sin(in.heading_rad) * 1.7f;
        in.left_m -= lateral;
        in.right_m += lateral;
        out = controller.update(in);
        if (i > 20) CHECK(out.trusted_mask == 3);
    }
}

static void zirconia_reset_and_sync()
{
    ZirconiaWallControl controller;
    auto in = wall_input();
    in.left_m = 0.046f;
    in.right_m = 0.044f;
    ZirconiaWallControl::Output out;
    for (int i = 0; i < 60; ++i) out = controller.update(in);
    CHECK(out.sync_estimator && near(out.raw_error_m, 0.001f));
    in.enabled = false;
    out = controller.update(in);
    CHECK(out.phase == ZirconiaWallControl::Disabled && out.omega_radps == 0.0f);
    in.enabled = true;
    out = controller.update(in);
    CHECK(out.phase == ZirconiaWallControl::Confirming && out.trusted_mask == 0);
    in.heading_rad = std::numeric_limits<float>::quiet_NaN();
    out = controller.update(in);
    CHECK(out.phase == ZirconiaWallControl::Disabled && out.omega_radps == 0.0f);
}

static void zirconia_adc_adapter()
{
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    task.EnableIrSensStraight();
    vehicle.ideal.velo.set(0.5f);
    vehicle.ego.velo.set(0.5f);
    // Exact table points: right-of-center requires positive/left yaw.
    side_wall_test_adc[sensor_sl] = sens_sl_table[5]; // 50 mm
    side_wall_test_adc[sensor_sr] = sens_sr_table[3]; // 40 mm
    side_wall_test_adc[sensor_fl] = sens_fl_table[0]; // Front wall must not suppress side control.
    side_wall_test_adc[sensor_fr] = sens_fr_table[0];
    for (int i = 0; i < 150; ++i) {
        task.IrSensorSet(&vehicle, 0.5f);
        task.SetWallControl_RadVelo(&vehicle, 1.0f);
        if (i < 9) CHECK(vehicle.ideal.rad_velo.get() == 0.0f);
        CHECK(std::fabs(vehicle.ideal.rad_velo.get()) <= 1.20001f);
    }
    CHECK(task.straight_wall_output.trusted_mask == 3);
    CHECK(vehicle.ideal.rad_velo.get() > 0.0f);
    CHECK(task.sen_l.is_control == True && task.sen_r.is_control == True);
    // The output remains bounded even though last cycle's ideal yaw is nonzero.
    CHECK(near(task.control_ir.get(), -5.0f));
    vehicle.ideal.velo.set(0.19f);
    task.IrSensorSet(&vehicle, 0.5f);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(vehicle.ideal.rad_velo.get() == 0.0f && vehicle.ideal.rad_accel.get() == 0.0f);
    vehicle.ideal.velo.set(0.5f);
    for (int i = 0; i < 100; ++i) {
        task.IrSensorSet(&vehicle, 0.5f);
        task.SetWallControl_RadVelo(&vehicle, 1.0f);
    }
    task.DisableIrSens();
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(vehicle.ideal.rad_velo.get() == 0.0f);
    task.EnableIrSensDiagonal();
    CHECK(task.straight_wall_output.phase == ZirconiaWallControl::Disabled);
    task.EnableIrSensStraight();
    task.IrSensorSet(&vehicle, 0.5f);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(task.straight_wall_output.phase == ZirconiaWallControl::Confirming);
    // Slalom exit from a diagonal approach primes the known wall layout.
    task.EnableIrSensDiagonal();
    task.PrimeStraightWallControl();
    task.EnableIrSensStraight();
    task.IrSensorSet(&vehicle, 0.5f);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(task.straight_wall_output.phase == ZirconiaWallControl::Tracking);
    CHECK(task.straight_wall_output.trusted_mask == 0);
    CHECK(task.straight_wall_output.pose_control);
    // The heading term must survive legacy resets of ego.radian at wall edges.
    vehicle.ego.rad_velo.set(1.0f);
    for (int i = 0; i < 20; ++i) {
        vehicle.ego.radian.set(0.0f);
        task.IrSensorSet(&vehicle, 0.5f);
        task.SetWallControl_RadVelo(&vehicle, 1.0f);
    }
    CHECK(near(task.control_ir_dot.get(), 0.020f));
    task.ResetStraightWallControl();
    CHECK(task.straight_wall_output.phase == ZirconiaWallControl::Disabled);
}

static void ekf_math_and_dropout()
{
    // Before an EKF observation is accepted, retain only the existing
    // conservative response away from a newly detected close wall.
    ZirconiaWallControl confirming;
    auto close = wall_input();
    close.use_estimated_pose = true;
    close.measurement_mask = 0;
    close.left_m = 0.030f; close.right_m = 0.080f;
    ZirconiaWallControl::Output response;
    for (int i = 0; i < 30; ++i) response = confirming.update(close);
    CHECK(!response.pose_control && response.trusted_mask == 0);
    CHECK(response.omega_radps < 0.0f && response.omega_radps >= -0.20001f);
    StraightWallEkf filter;
    filter.start(0.004f, 0.04f);
    const float before = filter.estimate().variance_lateral_m2;
    for (int i = 0; i < 100; ++i) {
        CHECK(filter.predict(0.001f, 0.0f, 0.001f));
        CHECK(filter.observe_walls(0.080f, 0.080f, 0, true) == 0);
    }
    CHECK(std::fabs(filter.estimate().lateral_m - (0.004f + 0.1f * std::sin(0.04f))) < 1e-6f);
    CHECK(near(filter.estimate().heading_rad, 0.04f));
    CHECK(filter.estimate().variance_lateral_m2 > before);
    CHECK(!filter.lateral_trusted() && filter.estimate().wall_updates == 0);
    filter.start(0.0f, 0.0f);
    for (int i = 0; i < 500; ++i) {
        const float travel = 0.0005f * i;
        const float e = travel * std::sin(0.034906585f);
        CHECK(filter.predict(0.0005f, 0.0f, 0.001f));
        filter.observe_walls(0.045f - e, 0.045f + e, 3, true);
        const auto p = filter.estimate();
        CHECK(p.variance_lateral_m2 >= 0.0f && p.variance_heading_rad2 >= 0.0f);
        CHECK(p.variance_lateral_m2 * p.variance_heading_rad2 - p.covariance_m_rad * p.covariance_m_rad >= -1e-12f);
    }
    CHECK(filter.heading_trusted());
    CHECK(std::fabs(filter.estimate().heading_rad - 0.034906585f) < 0.006f);
    CHECK(filter.estimate().wall_updates >= 45 && filter.estimate().wall_updates <= 50);
    const auto updates = filter.estimate().wall_updates;
    const float e = filter.estimate().lateral_m;
    // A false right reading is rejected while the continuous left wall remains.
    CHECK(filter.observe_walls(0.045f - e, 0.015f, 3, true) == 1);
    CHECK(filter.estimate().wall_updates == updates); // Same averaged sample.
    for (int i = 0; i < 100; ++i) {
        filter.predict(0.001f, 0.0f, 0.001f);
        filter.observe_walls(0.080f, 0.080f, 0, false);
    }
    CHECK(!filter.heading_trusted());
    CHECK(filter.estimate().wall_updates == updates);
    CHECK(!filter.predict(0.0f, 0.0f, 0.1f) && !filter.estimate().initialized);
}

static void ekf_adapter_switching()
{
    IrSensTask_type8 task{};
    Vehicle vehicle{};
    CHECK(task.WallEkfMode_Get() == IrSensTask::WallEkfMode::Shadow);
    task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    task.EnableIrSensStraight();
    vehicle.ideal.velo.set(0.5f);
    vehicle.ego.velo.set(0.5f);
    side_wall_test_adc[sensor_sl] = sens_sl_table[5];
    side_wall_test_adc[sensor_sr] = sens_sr_table[3];
    side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
    for (int i = 0; i < 220; ++i) {
        task.IrSensorSet(&vehicle, 0.5f);
        task.SetWallControl_RadVelo(&vehicle, 1.0f);
    }
    CHECK(task.straight_wall_ekf_active && task.straight_wall_estimate.observed);
    CHECK(task.straight_wall_ekf_used);
    CHECK(task.straight_wall_estimate.accepted_mask == 3);
    CHECK(task.straight_wall_output.trusted_mask == 3);
    CHECK(std::fabs(vehicle.ego.x_point.get() + 5.0f) < 0.3f);
    const uint32_t updates = task.straight_wall_estimate.wall_updates;
    const float variance = task.straight_wall_estimate.variance_lateral_m2;
    side_wall_test_adc[sensor_sl] = side_wall_test_adc[sensor_sr] = 0;
    // An edge must immediately remove that side, before the ten-point average empties.
    task.IrSensorSet(&vehicle, 0.5f);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(task.straight_wall_estimate.accepted_mask == 0);
    for (int i = 0; i < 250; ++i) {
        vehicle.ego.length.set(i == 80 ? 0.0f : 5.0f); // Existing progress corrections must not enter ds.
        vehicle.ego.radian.set(0.0f);
        task.IrSensorSet(&vehicle, 0.5f);
        task.SetWallControl_RadVelo(&vehicle, 1.0f);
        CHECK(std::fabs(vehicle.ideal.rad_velo.get()) <= 0.20001f);
    }
    CHECK(task.straight_wall_estimate.wall_updates == updates);
    CHECK(task.straight_wall_estimate.variance_lateral_m2 > variance);
    CHECK(task.straight_wall_ekf_active);
    CHECK(task.control_ir.get() == 0.0f); // No centring toward an unobserved wall.
    side_wall_test_adc[sensor_sl] = sens_sl_table[5]; // Reacquire only the left wall.
    for (int i = 0; i < 100; ++i) {
        task.IrSensorSet(&vehicle, 0.5f);
        task.SetWallControl_RadVelo(&vehicle, 1.0f);
    }
    CHECK(task.straight_wall_estimate.wall_updates > updates);
    CHECK(task.straight_wall_estimate.accepted_mask == 1);
    CHECK(task.straight_wall_output.trusted_mask == 1);
    // A continuous straight section boundary preserves the two-state estimate.
    const auto retained = task.straight_wall_estimate;
    task.ResetStraightWallControl(true);
    task.IrSensorSet(&vehicle, 0.5f);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(task.straight_wall_estimate.wall_updates >= retained.wall_updates);
    CHECK(std::fabs(task.straight_wall_estimate.heading_rad - retained.heading_rad) < 0.01f);
    task.EnableIrSensDiagonal();
    CHECK(!task.straight_wall_ekf_active);
    task.EnableIrSensStraight();
    task.IrSensorSet(&vehicle, 0.5f);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(task.straight_wall_estimate.wall_updates == 0);
}

static int16_t side_adc_at(const int16_t *table, float distance_mm)
{
    if(distance_mm<=25.0f)return table[0];
    // A/B side calibrations end at 80 mm. Do not read the following entry
    // when the interpolation weight is zero at that endpoint.
    if(distance_mm>=80.0f)return table[11];
    const float index = (distance_mm - 25.0f) / 5.0f;
    const int i = static_cast<int>(index);
    return static_cast<int16_t>(table[i] + (table[i+1] - table[i]) * (index-i));
}

static void diagonal_edge_real_adc_and_modes()
{
    for (bool right : {false, true}) for (int scenario = 0; scenario < 4; ++scenario) {
        IrSensTask_type8 task{}; Vehicle v{};
        task.EnableIrSensDiagonal();
        v.ideal.velo.set(.5f); v.ego.velo.set(.5f);
        int events = 0;
        for (float x = -30.0f; x <= 30.0f; x += .5f) {
            const float range = 40.0f + std::fabs(x);
            side_wall_test_adc[sensor_sr] = side_adc_at(sens_sr_table, right ? range : 45.0f);
            side_wall_test_adc[sensor_sl] = side_adc_at(sens_sl_table, right ? 45.0f : range);
            side_wall_test_adc[sensor_fr] = side_wall_test_adc[sensor_fl] = 0;
            v.ego.rad_velo.set(scenario == 1 && x >= -2.0f && x <= 10.0f ? (right ? 2.1f : -2.1f) : 0.0f);
            v.ideal.velo.set(scenario == 3 ? 0.0f : .5f);
            if (scenario == 2 && near(x, -.5f)) {
                task.EnableIrSensStraight(); task.EnableIrSensDiagonal();
            }
            task.IrSensorSet(&v, .5f);
            if ((right ? task.r_wall_corner : task.l_wall_corner) == True) {
                ++events;
                CHECK((right ? task.r_corner_delay_mm : task.l_corner_delay_mm) > 0.0f);
                CHECK((right ? task.r_corner_time : task.l_corner_time) > 0);
            }
        }
        CHECK(events == (scenario == 0 ? 1 : 0));
        const auto stats = task.DiagonalWallDiagnostics();
        CHECK((right ? stats.edges_right : stats.edges_left) == static_cast<unsigned>(events));
        task.ResetDiagonalWallDiagnostics();
        CHECK(task.DiagonalWallDiagnostics().edges_right == 0 && task.DiagonalWallDiagnostics().edge_delay_max_mm == 0.0f);
    }
}

static void diagonal_ekf_symmetry_and_covariance()
{
    DiagonalWallControl left{}, right{};
    DiagonalWallControl::Input a, b;
    a.enabled = a.fresh = b.enabled = b.fresh = true;
    a.travel_m = b.travel_m = .0005f;
    a.left_m = b.right_m = .030f;
    a.eligible_mask = 1; b.eligible_mask = 2;
    a.gyro_radps = .05f; b.gyro_radps = -.05f;
    DiagonalWallControl::Output x, y;
    for (int i = 0; i < 120; ++i) {
        x = left.update(a); y = right.update(b);
        CHECK(near(x.error_m, -y.error_m));
        CHECK(x.estimate.variance_lateral_m2 * x.estimate.variance_heading_rad2
            - x.estimate.covariance_m_rad * x.estimate.covariance_m_rad >= -1e-12f);
        CHECK(std::fabs(x.estimate.heading_rad - i*.001f*.05f) < .000001f);
        if (i < 6) CHECK(x.candidate_mask == 0 && !x.used_ekf);
    }
    CHECK(x.used_ekf && y.used_ekf && x.estimate.lateral_m > 0.0f && y.estimate.lateral_m < 0.0f);
    CHECK(x.error_m > 0.0f && y.error_m < 0.0f);
    CHECK(x.estimate.wall_updates > 0 && x.estimate.wall_updates <= 12);
    const float prior = wall_ekf_param.initial_heading_std_rad * wall_ekf_param.initial_heading_std_rad;
    CHECK(x.estimate.variance_heading_rad2 >= prior); // Pillars must not manufacture yaw certainty.
}

static void diagonal_ekf_sparse_pillars()
{
    DiagonalWallControl controller;
    DiagonalWallControl::Input in;
    in.enabled = in.fresh = true; in.travel_m = .0005f; in.left_m = .030f; in.eligible_mask = 1;
    DiagonalWallControl::Output out;
    for (int i = 0; i < 100; ++i) out = controller.update(in);
    CHECK(out.used_ekf);
    const auto before = out.estimate;
    in.left_m = .080f; in.eligible_mask = 0;
    for (int i = 0; i < 200; ++i) {
        out = controller.update(in);
        CHECK(!out.used_ekf && !out.updated && out.error_m == 0.0f && out.accepted_mask == 0);
        CHECK(out.estimate.wall_updates == before.wall_updates);
    }
    CHECK(out.error_m == 0.0f && out.estimate.lateral_m > 0.0f);
    in.right_m = .030f; in.eligible_mask = 2;
    for (int i = 0; i < 100; ++i) out = controller.update(in);
    CHECK(out.error_m < 0.0f); // A new opposite pillar changes the lateral observation sign.
    controller.reset();
    in.left_m = .030f; in.right_m = .080f; in.eligible_mask = 1;
    for (int i = 0; i < 80; ++i) out = controller.update(in);
    in.right_m = .030f; in.eligible_mask = 3;
    for (int i = 0; i < 5; ++i) {
        out = controller.update(in);
        CHECK(!out.used_ekf && out.accepted_mask == 0);
    }
    for (int i = 0; i < 150; ++i) out = controller.update(in);
    CHECK(out.error_m == 0.0f); // Symmetric close pillars must not inherit a one-sided observation.
    in.enabled = false;
    out = controller.update(in);
    CHECK(!out.active && out.omega_radps == 0.0f);
}

static void diagonal_ekf_observation_gates()
{
    DiagonalWallControl controller;
    DiagonalWallControl::Input in;
    in.enabled = in.fresh = true; in.left_m = .030f; in.eligible_mask = 1;
    for (int i = 0; i < 50; ++i) CHECK(controller.update(in).candidate_mask == 0); // Stationary.
    in.travel_m = .0005f; in.fresh = false;
    for (int i = 0; i < 50; ++i) CHECK(controller.update(in).candidate_mask == 0);
    in.fresh = true; in.gyro_radps = 3.0f;
    for (int i = 0; i < 50; ++i) CHECK(controller.update(in).accepted_mask == 0);
    in.gyro_radps = 0.0f; in.left_m = .040f;
    for (int i = 0; i < 50; ++i) CHECK(controller.update(in).accepted_mask == 0); // A far pillar is not centre zero.
    in.left_m = .030f; in.travel_m = -.001f;
    CHECK(!controller.update(in).active);
    in.travel_m = .0005f; in.dt_s = .1f;
    CHECK(!controller.update(in).active);
    in.dt_s = .001f; in.gyro_radps = std::numeric_limits<float>::quiet_NaN();
    CHECK(!controller.update(in).active);
}

static void diagonal_ekf_real_ir_modes()
{
    IrSensTask_type8 legacy{}, shadow{}, control{};
    Vehicle a{}, b{}, c{};
    legacy.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy);
    control.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    legacy.EnableIrSensDiagonal(); shadow.EnableIrSensDiagonal(); control.EnableIrSensDiagonal();
    a.ideal.velo.set(.5f); b.ideal.velo.set(.5f); c.ideal.velo.set(.5f);
    a.ego.velo.set(.5f); b.ego.velo.set(.5f); c.ego.velo.set(.5f);
    side_wall_test_adc[sensor_sl] = sens_sl_table[1]; // 30mm, calibration must remain per-machine.
    side_wall_test_adc[sensor_sr] = 0;
    side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
    for (int i = 0; i < 120; ++i) {
        legacy.IrSensorSet(&a, .5f); legacy.SetWallControl_RadVelo(&a, 1.0f);
        shadow.IrSensorSet(&b, .5f); shadow.SetWallControl_RadVelo(&b, 1.0f);
        control.IrSensorSet(&c, .5f); control.SetWallControl_RadVelo(&c, 1.0f);
        CHECK(near(a.ideal.rad_velo.get(), b.ideal.rad_velo.get()));
        CHECK(near(a.ideal.rad_accel.get(), b.ideal.rad_accel.get()));
        CHECK(std::isfinite(c.ideal.rad_velo.get()) && std::isfinite(c.ideal.rad_accel.get()));
        CHECK(near(c.ideal.rad_velo.get(), control.diagonal_wall_output.omega_radps));
    }
    CHECK(control.diagonal_wall_output.used_ekf && c.ideal.rad_velo.get() < 0.0f);
    CHECK(shadow.DiagonalWallDiagnostics().updates > 0 && shadow.DiagonalWallDiagnostics().used == 0);
    CHECK(legacy.DiagonalWallDiagnostics().updates == 0);
    auto stats = control.DiagonalWallDiagnostics();
    CHECK(stats.cycles == 120 && stats.used > 0 && stats.updates > 0 && stats.segments == 1);
    CHECK(stats.used + stats.fallback == stats.cycles);
    control.DisableIrSens();
    CHECK(!control.diagonal_wall_output.active && control.DiagonalWallDiagnostics().used == stats.used);
    control.ResetDiagonalWallDiagnostics();
    CHECK(control.DiagonalWallDiagnostics().cycles == 0);
}

static void diagonal_ekf_reset_and_no_phantom_wall()
{
    IrSensTask_type8 task{}; Vehicle vehicle{};
    task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    task.EnableIrSensDiagonal();
    vehicle.ideal.velo.set(.5f); vehicle.ego.velo.set(.5f); vehicle.ego.x_point.set(20.0f);
    side_wall_test_adc[sensor_sl] = side_wall_test_adc[sensor_sr] = 0;
    side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
    for (int i = 0; i < 30; ++i) {
        vehicle.ego.radian.set(0.0f); vehicle.ego.rad_velo.set(.1f);
        task.IrSensorSet(&vehicle, .5f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
    }
    CHECK(vehicle.ideal.rad_velo.get() < 0.0f && task.diagonal_wall_output.estimate.wall_updates == 0);
    CHECK(!task.diagonal_wall_output.used_ekf); // Odometry control is not a new wall observation.
    CHECK(task.diagonal_wall_output.estimate.heading_rad > .002f); // Survives ego.radian reset.
    task.EnableIrSensStraight();
    CHECK(!task.diagonal_wall_output.active);
    task.EnableIrSensDiagonal();
    task.IrSensorSet(&vehicle, .5f); task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(near(task.diagonal_wall_output.estimate.heading_rad, 0.0f));
    vehicle.ideal.velo.set(.2f);
    task.SetWallControl_RadVelo(&vehicle, 1.0f);
    CHECK(!task.diagonal_wall_output.active && vehicle.ideal.rad_velo.get() == 0.0f);
}

static void diagonal_fallback_logged_samples()
{
    // A / run_013012_054417, 0.361 / 0.381 / 0.421s: sampled states only.
    // No reconstruction of missing 1ms ADC values or closed-loop trajectory.
    const float samples[][6] = {
        {1.959961f, 1.814453f, 20.0f, .00002f, .012512f, 71.125f},
        {2.359375f, 2.371094f, 20.0f, .00002f, -.287109f, 53.375f},
        {3.0f, 3.066406f, 0.0f, .00130f, 8.195312f, 66.125f},
        {1.6f, 1.7f, 0.0f, .003f, 4.0f, 30.0f}, // Close, but not yet confirmed over 3mm.
    };
    for (const auto &s : samples) for (float sign : {-1.0f, 1.0f}) {
        IrSensTask_type8 legacy{}, control{};
        legacy.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy);
        control.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
        legacy.EnableIrSensDiagonal(); control.EnableIrSensDiagonal();
        Vehicle a{}, b{};
        for (auto *v : {&a, &b}) {
            v->ideal.velo.set(s[0]); v->ego.velo.set(s[1]); v->ideal.accel.set(s[2]);
            v->ideal.radian.set(sign*s[3]); v->ego.x_point.set(sign*s[4]);
            v->ideal.rad_velo.set(0.0f);
        }
        side_wall_test_adc[sensor_sl] = sign > 0 ? 0 : side_adc_at(sens_sl_table, s[5]);
        side_wall_test_adc[sensor_sr] = sign > 0 ? side_adc_at(sens_sr_table, s[5]) : 0;
        side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
        legacy.IrSensorSet(&a, 1.0f); control.IrSensorSet(&b, 1.0f);
        legacy.SetWallControl_RadVelo(&a, 1.0f); control.SetWallControl_RadVelo(&b, 1.0f);
        CHECK(!control.diagonal_wall_output.used_ekf);
        CHECK(near(a.ideal.rad_velo.get(), b.ideal.rad_velo.get()));
        CHECK(near(a.ideal.rad_accel.get(), b.ideal.rad_accel.get()));
        CHECK(near(legacy.control_ir.get(), control.control_ir.get()));
        CHECK(std::fabs(b.ideal.rad_velo.get()) > .00001f);
    }
}

static void diagonal_front_pillar_observations()
{
    for (bool left : {false, true}) {
        IrSensTask_type8 task{}; Vehicle v{};
        task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
        task.EnableIrSensDiagonal();
        v.ideal.velo.set(.5f); v.ego.velo.set(.5f);
        side_wall_test_adc[sensor_sl] = left ? sens_sl_table[1] : 0;
        side_wall_test_adc[sensor_sr] = left ? 0 : sens_sr_table[1];
        side_wall_test_adc[sensor_fl] = left ? sens_fl_table[0] : 0;
        side_wall_test_adc[sensor_fr] = left ? 0 : sens_fr_table[0];
        for (int i = 0; i < 80; ++i) {
            task.IrSensorSet(&v, .5f); task.SetWallControl_RadVelo(&v, 1.0f);
        }
        CHECK((left ? task.sen_fl.distance : task.sen_fr.distance) <= 66.0f);
        CHECK(task.diagonal_wall_output.used_ekf && task.DiagonalWallDiagnostics().updates > 0);
        CHECK(left ? task.control_ir.get() > 0.0f : task.control_ir.get() < 0.0f);
        CHECK(left ? v.ideal.rad_velo.get() < 0.0f : v.ideal.rad_velo.get() > 0.0f);
        // A close front pair must revoke the observation and its confirmation.
        side_wall_test_adc[sensor_fl] = sens_fl_table[0];
        side_wall_test_adc[sensor_fr] = sens_fr_table[0];
        const auto updates = task.DiagonalWallDiagnostics().updates;
        for (int i = 0; i < 30; ++i) {
            task.IrSensorSet(&v, .5f); task.SetWallControl_RadVelo(&v, 1.0f);
            CHECK(!task.diagonal_wall_output.used_ekf);
        }
        CHECK(task.DiagonalWallDiagnostics().updates == updates);
        CHECK(task.DiagonalWallDiagnostics().front_blocked > 0);
        side_wall_test_adc[left ? sensor_fr : sensor_fl] = 0;
        task.IrSensorSet(&v, .5f); task.SetWallControl_RadVelo(&v, 1.0f);
        CHECK(task.diagonal_wall_output.candidate_mask == 0);
        CHECK(task.DiagonalWallDiagnostics().used + task.DiagonalWallDiagnostics().fallback
            == task.DiagonalWallDiagnostics().cycles);
    }
}

static void diagonal_ekf_keeps_trajectory_damping()
{
    for (float velocity : {.5f, 1.6f, 3.0f}) {
        IrSensTask_type8 task{}; Vehicle v{};
        task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
        task.EnableIrSensDiagonal();
        v.ideal.velo.set(velocity); v.ego.velo.set(velocity);
        side_wall_test_adc[sensor_sl] = sens_sl_table[1];
        side_wall_test_adc[sensor_sr] = 0;
        side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
        for (int i = 0; i < 80; ++i) {
            task.IrSensorSet(&v, velocity); task.SetWallControl_RadVelo(&v, 1.0f);
        }
        v.ideal.radian.set(.01f); v.ideal.rad_velo.set(.3f); v.ideal.accel.set(2.0f);
        task.IrSensorSet(&v, velocity); task.SetWallControl_RadVelo(&v, 1.0f);
        CHECK(task.diagonal_wall_output.used_ekf);
        // Ask the real Legacy controller to process the same trajectory state
        // with the accepted EKF lateral input. No copied controller equation.
        IrSensTask_type8 reference{}; Vehicle expected{};
        reference.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy);
        reference.EnableIrSensDiagonal();
        expected.ideal.velo.set(velocity); expected.ego.velo.set(velocity);
        expected.ideal.radian.set(.01f); expected.ideal.rad_velo.set(.3f); expected.ideal.accel.set(2.0f);
        expected.ego.x_point.set(task.diagonal_wall_output.error_m * 1000.0f);
        reference.SetWallControl_RadVelo(&expected, 1.0f);
        CHECK(near(expected.ideal.rad_velo.get(), v.ideal.rad_velo.get()));
        CHECK(near(expected.ideal.rad_accel.get(), v.ideal.rad_accel.get()));
    }
}

static void ekf_turn_exit_heading_disagreement()
{
    // Shape of A's 20261106 run_224247 left-turn exit (8,0). Only the
    // sampled range/gyro trend is replayed, not the missing 1ms ADC history.
    // Mirroring exercises the same safety condition through each A/B calibration.
    const float samples[][3] = {
        {0,37.9375f,-.70947f},{21,38.0f,.06384f},{41,37.78125f,.29468f},
        {61,37.96875f,.58936f},{81,38.59375f,1.05371f},{101,39.875f,1.38965f},
        {121,41.5f,.78076f},{142,42.53125f,.32910f},{162,43.09375f,-.19397f},
        {182,43.0f,-.50586f},{202,42.65625f,-.77344f},{222,41.625f,-1.29199f},
        {242,40.15625f,-1.20801f},{263,39.0f,-1.17871f},{283,37.71875f,-1.08301f},
    };
    for (int mirror = 0; mirror < 2; ++mirror) {
        IrSensTask_type8 control{}, legacy{}, continued{};
        Vehicle a{}, b{}, c{};
        control.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
        legacy.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy);
        continued.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
        control.EnableIrSensStraight(); legacy.EnableIrSensStraight(); continued.EnableIrSensStraight();
        const float sign = mirror ? -1.0f : 1.0f;
        control.PrimeSearchTurnExit(-.02573f * sign, true);
        legacy.PrimeSearchTurnExit(-.02573f * sign, true);
        continued.PrimeSearchTurnExit(-.02573f * sign, true, .006f*.006f);
        a.ideal.velo.set(.37f); b.ideal.velo.set(.37f);
        a.ego.velo.set(.37f); b.ego.velo.set(.37f);
        c.ideal.velo.set(.37f); c.ego.velo.set(.37f);
        unsigned segment = 0, disagreements = 0;
        float unknown_peak = 0.0f, continued_peak = 0.0f;
        for (int ms = 0; ms <= 283; ++ms) {
            while (segment + 2 < sizeof(samples)/sizeof(samples[0]) && ms > samples[segment+1][0]) ++segment;
            const float u = (ms-samples[segment][0]) / (samples[segment+1][0]-samples[segment][0]);
            const float range = samples[segment][1] + u*(samples[segment+1][1]-samples[segment][1]);
            const float gyro = sign*(samples[segment][2] + u*(samples[segment+1][2]-samples[segment][2]));
            side_wall_test_adc[sensor_sl] = mirror ? side_adc_at(sens_sl_table, range) : 0;
            side_wall_test_adc[sensor_sr] = mirror ? 0 : side_adc_at(sens_sr_table, range);
            side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
            a.ego.rad_velo.set(gyro); b.ego.rad_velo.set(gyro);
            c.ego.rad_velo.set(gyro);
            control.IrSensorSet(&a, .37f); control.SetWallControl_RadVelo(&a, 1.0f);
            legacy.IrSensorSet(&b, .37f); legacy.SetWallControl_RadVelo(&b, 1.0f);
            continued.IrSensorSet(&c, .37f); continued.SetWallControl_RadVelo(&c, 1.0f);
            const auto &pose = control.straight_wall_estimate;
            if (ms <= 200 && pose.observed && continued.straight_wall_estimate.observed) {
                unknown_peak = fmaxf(unknown_peak, std::fabs(pose.heading_rad-legacy.control_ir_dot.get()));
                continued_peak = fmaxf(continued_peak,
                    std::fabs(continued.straight_wall_estimate.heading_rad-legacy.control_ir_dot.get()));
            }
            const float limit = wall_ekf_param.heading_disagreement_limit_rad;
            if (pose.observed && pose.variance_heading_rad2 <= limit*limit
                && std::fabs(pose.heading_rad-legacy.control_ir_dot.get()) > limit) {
                ++disagreements;
                CHECK(!control.straight_wall_ekf_used);
                CHECK(near(control.control_ir_dot.get(), legacy.control_ir_dot.get()));
                CHECK(near(control.StraightHeadingForTurn(), legacy.StraightHeadingForTurn()));
                CHECK(near(a.ideal.rad_velo.get(), b.ideal.rad_velo.get()));
                CHECK(!(control.SearchTurnDecision(true, true).flags & SearchTurnGuard::ReliablePose));
                CHECK(control.SearchTurnDecision(true, true).flags & SearchTurnGuard::EkfNotUsed);
            }
        }
        CHECK(disagreements > 5);
        CHECK(continued_peak < unknown_peak * .5f);
    }
}

static void ekf_corner_and_mode()
{
    IrSensTask_type8 shadow{}, legacy{}, control{};
    Vehicle a{}, b{}, c{};
    legacy.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy);
    control.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    side_wall_test_adc[sensor_sl] = sens_sl_table[5];
    side_wall_test_adc[sensor_sr] = sens_sr_table[3];
    side_wall_test_adc[sensor_fl] = side_wall_test_adc[sensor_fr] = 0;
    IrSensTask_type8 *tasks[] = {&shadow, &legacy, &control};
    Vehicle *vehicles[] = {&a, &b, &c};
    for (auto task : tasks) task->EnableIrSensStraight();
    for (auto vehicle : vehicles) { vehicle->ideal.velo.set(0.5f); vehicle->ego.velo.set(0.5f); }
    for (int i = 0; i < 180; ++i) {
        shadow.IrSensorSet(&a, 0.5f); shadow.SetWallControl_RadVelo(&a, 1.0f);
        legacy.IrSensorSet(&b, 0.5f); legacy.SetWallControl_RadVelo(&b, 1.0f);
        control.IrSensorSet(&c, 0.5f); control.SetWallControl_RadVelo(&c, 1.0f);
        CHECK(a.ideal.rad_velo.get() == b.ideal.rad_velo.get());
        CHECK(a.ego.x_point.get() == b.ego.x_point.get());
    }
    CHECK(shadow.straight_wall_estimate.wall_updates > 0);
    CHECK(!legacy.straight_wall_ekf_active);
    const auto before = control.straight_wall_estimate;
    const float old_position = c.ego.x_point.get();
    CHECK(control.CorrectStraightWallCorner(&c, -4.0f));
    CHECK(control.straight_wall_estimate.corner_updates == before.corner_updates);
    CHECK(c.ego.x_point.get() == old_position); // Queued until this tick's prediction.
    control.IrSensorSet(&c, 0.5f); control.SetWallControl_RadVelo(&c, 1.0f);
    CHECK(control.straight_wall_estimate.corner_updates == before.corner_updates + 1);
    CHECK(c.ego.x_point.get() < -4.0f && c.ego.x_point.get() > -5.2f); // No hard jump to event value.
    const float corrected = c.ego.x_point.get();
    CHECK(!control.CorrectStraightWallCorner(&c, 100.0f));
    CHECK(c.ego.x_point.get() == corrected);
    control.IrSensorSet(&c, 0.5f); control.SetWallControl_RadVelo(&c, 1.0f);
    CHECK(control.straight_wall_estimate.corner_updates == before.corner_updates + 1);
    CHECK(shadow.CorrectStraightWallCorner(&a, -4.0f) && a.ego.x_point.get() == -4.0f);
    // Existing wall-edge radian resets cannot erase the independently predicted heading.
    c.ego.rad_velo.set(0.5f);
    side_wall_test_adc[sensor_sl] = side_wall_test_adc[sensor_sr] = 0;
    for (int i = 0; i < 30; ++i) {
        c.ego.radian.set(0.0f);
        control.IrSensorSet(&c, 0.5f); control.SetWallControl_RadVelo(&c, 1.0f);
    }
    CHECK(control.straight_wall_estimate.heading_rad > 0.01f);
    control.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy);
    CHECK(!control.straight_wall_ekf_active);
}

static void ekf_shell()
{
    auto &ir = IrSensTask_type8::getInstance();
    auto &motion = CtrlTask_type8::getInstance();
    char *args[] = {const_cast<char *>("wall_ekf"), const_cast<char *>("control"), const_cast<char *>("extra")};
    CHECK(usrcmd_wall_ekf(2, args) == 0);
    CHECK(ir.WallEkfMode_Get() == IrSensTask::WallEkfMode::Control && test_irq_mask == 0);
    motion.status = execute;
    args[1] = const_cast<char *>("legacy");
    CHECK(usrcmd_wall_ekf(2, args) == -1 && ir.WallEkfMode_Get() == IrSensTask::WallEkfMode::Control);
    motion.status = complete; motion.enabled = True;
    CHECK(usrcmd_wall_ekf(2, args) == -1 && test_irq_mask == 0);
    motion.enabled = False;
    args[1] = const_cast<char *>("show");
    test_irq_mask = 1;
    CHECK(usrcmd_wall_ekf(2, args) == 0 && test_irq_mask == 1);
    test_irq_mask = 0;
    args[1] = const_cast<char *>("bad");
    CHECK(usrcmd_wall_ekf(2, args) == -1);
    CHECK(usrcmd_wall_ekf(3, args) == -1);
    args[1] = const_cast<char *>("legacy");
    CHECK(usrcmd_wall_ekf(2, args) == 0 && ir.WallEkfMode_Get() == IrSensTask::WallEkfMode::Legacy);
    args[1] = const_cast<char *>("shadow");
    CHECK(usrcmd_wall_ekf(2, args) == 0 && ir.WallEkfMode_Get() == IrSensTask::WallEkfMode::Shadow);
}

static void wall_observation_diagnostics()
{
    using namespace WallPositionCorrection;
    CHECK(ir_range_state(sens_sl_table[0], sens_sl_table, 12) == IrRangeState::InRange);
    CHECK(ir_range_state(sens_sl_table[0]+1, sens_sl_table, 12) == IrRangeState::NearLimit);
    CHECK(ir_range_state(sens_sl_table[11]-1, sens_sl_table, 12) == IrRangeState::FarLimit);
    CHECK(ir_range_state(-1, sens_sl_table, 12) == IrRangeState::Invalid);
    for (auto phase : {Phase::StraightEntry, Phase::DiagonalEntry, Phase::StraightExit, Phase::DiagonalExit, Phase::SearchStraight}) {
        auto small = evaluate(10.0f, 11.0f, .5f, phase);
        CHECK(small.accepted() && near(small.innovation_mm, 1.5f));
        auto reverse = evaluate(11.5f, 10.0f, 0.0f, phase);
        CHECK(reverse.accepted() && near(reverse.delta_mm, -small.delta_mm));
        const auto &policy = wall_position_param[static_cast<unsigned>(phase)];
        auto limited = evaluate(0.0f, policy.window_mm, 0.0f, phase);
        CHECK(limited.reason == (policy.window_mm * policy.blend > policy.maximum_step_mm ? Reason::Limited : Reason::Accepted));
        CHECK(near(limited.delta_mm, policy.maximum_step_mm));
        auto outlier = evaluate(0.0f, policy.window_mm+.1f, 0.0f, phase);
        CHECK(outlier.reason == Reason::OutsideWindow && outlier.after_mm == 0.0f);
        CHECK(evaluate(1, 1, -1, phase).reason == Reason::Invalid);
        CHECK(evaluate(1, 1, 1000, phase).reason == Reason::Invalid);
        CHECK(evaluate(1, std::numeric_limits<float>::quiet_NaN(), 0, phase).reason == Reason::Invalid);
    }
    auto input = reliable_turn_input();
    auto decision = SearchTurnGuard::decide(input, {true, false, 1.2f});
    CHECK(decision.checks == 1023 && near(decision.candidate_mm, 1.2f));
    input.range[2] = IrRangeState::FarLimit;
    decision = SearchTurnGuard::decide(input, {true, false});
    CHECK(!(decision.checks & SearchTurnGuard::MetricPair) && !(decision.flags & SearchTurnGuard::Suppressed));
    ZirconiaWallControl control;
    ZirconiaWallControl::Input in;
    in.left_m = .025f; in.right_m = .045f; in.range_valid_mask = 2;
    in.travel_m = .0005f; in.dt_s = .001f; in.sample_updated = true; in.enabled = true; in.velocity_mps = .37f;
    auto output = control.update(in);
    CHECK(!output.pose_control && !(output.trusted_mask & 1));
    for (int i=0; i<400; ++i) output = control.update(in);
    CHECK(!output.pose_control && !output.sync_estimator && output.omega_radps < 0);
    CHECK(!(output.trusted_mask & 1)); // 片側が近距離限界でも反対壁は観測可能
    IrSensTask_type8 task{}; Vehicle vehicle{};
    side_wall_test_adc[sensor_sl] = sens_sl_table[0]+1;
    side_wall_test_adc[sensor_sr] = sens_sr_table[11]-1;
    side_wall_test_adc[sensor_fl] = -1;
    side_wall_test_adc[sensor_fr] = sens_fr_table[0];
    task.IrSensorSet(&vehicle, .37f);
    CHECK(task.sen_l.range_state == IrRangeState::NearLimit && near(task.sen_l.distance,25));
    CHECK(task.sen_r.range_state == IrRangeState::FarLimit && near(task.sen_r.distance,80));
    CHECK(task.sen_fl.range_state == IrRangeState::Invalid);
    CHECK(task.sen_fr.range_state == IrRangeState::InRange);
    auto &diag = WallDiagnostics::instance(); diag.begin();
    diag.sample(IrRangeState::InRange, IrRangeState::NearLimit, IrRangeState::FarLimit, IrRangeState::Invalid, true);
    diag.sample(IrRangeState::Unknown, IrRangeState::Unknown, IrRangeState::Unknown, IrRangeState::Unknown, false);
    int16_t adc[4]{}; IrRangeState states[4]{};
    for (unsigned i=0; i<70; ++i) diag.record(i,1,Phase::StraightEntry,1,evaluate(10,11,0,Phase::StraightEntry),adc,states);
    CHECK(diag.total()==70 && diag.size()==64 && diag.event(63).time_ms==63);
    diag.stop();
    diag.record(70,1,Phase::StraightEntry,1,{},adc,states);
    CHECK(diag.total()==70);
    diag.finish_and_print();
    diag.record(71,1,Phase::StraightEntry,1,{},adc,states);
    CHECK(diag.total()==70); diag.begin(); CHECK(diag.total()==0 && diag.size()==0);
}

static void wall_edge_trial_detector()
{
    using namespace WallPositionCorrection;
    for (float depth : {50.0f, 61.0f, 63.4f, 64.3f}) {
        StraightWallEdge detector;
        detector.update(80, 0);
        for (int i=0; i<10; ++i) CHECK(!detector.update(depth,.5f,true,wall_edge_trial_param).detected);
        for (int i=0; i<12; ++i) CHECK(!detector.update(80,.5f,true,wall_edge_trial_param).detected);
    }
    IrSensTask_type8 task{}; Vehicle vehicle{};
    task.WallEdgeMode_Set(Mode::Shadow); task.EnableIrSensStraight();
    vehicle.ideal.velo.set(.37f);
    side_wall_test_adc[sensor_sl]=0; side_wall_test_adc[sensor_sr]=sens_sr_table[4];
    side_wall_test_adc[sensor_fl]=side_wall_test_adc[sensor_fr]=0;
    for (int i=0;i<29;++i) task.IrSensorSet(&vehicle,.5f);
    const auto old_generation=task.wall_edge_generation;
    task.Division_Wall_Correction_Reset(); // New consumer frame preserves pre-edge arm history.
    CHECK(task.wall_edge_generation!=old_generation);
    const auto generation=task.wall_edge_generation;
    task.EnableIrSensStraight(); // Straight -> Prev_Turn keeps the same IR mode and arm history.
    CHECK(task.wall_edge_generation==generation);
    side_wall_test_adc[sensor_sr]=0;
    task.IrSensorSet(&vehicle,.5f); CHECK(task.r_wall_corner==False);
    task.IrSensorSet(&vehicle,.5f); CHECK(task.r_wall_corner==True && task.r_corner_generation==generation);
    side_wall_test_adc[sensor_sr]=sens_sr_table[4];
    for (int i=0;i<29;++i) task.IrSensorSet(&vehicle,.5f);
    side_wall_test_adc[sensor_sr]=0;task.IrSensorSet(&vehicle,.5f);
    task.Division_Wall_Correction_Reset(); // Crossing in the old frame cannot confirm in the new one.
    task.IrSensorSet(&vehicle,.5f);CHECK(task.r_wall_corner==False);
    task.IrSensMotion_Set(TURN_IRSENS); CHECK(task.r_wall_corner==False && task.wall_edge_generation!=generation);
    task.EnableIrSensStraight();
    for (int i=0;i<5;++i) { task.IrSensorSet(&vehicle,.5f); CHECK(task.r_wall_corner==False); }
#if defined(MOUSE_A)
    CHECK(near(search_reference(Mode::Control),49.5f));
#else
    CHECK(near(search_reference(Mode::Control),SEARCH_CORRECTION));
#endif
    CHECK(near(edge_param(Mode::Legacy).arm_travel_mm,.5f));
}

static void wall_edge_trial_decision()
{
    using namespace WallPositionCorrection;
    auto wrong=decide(8,10,1,1,1,2,4,4,4,Phase::StraightEntry,Mode::Control);
    CHECK(!wrong.selected_side && wrong.left.reason==Reason::WrongSide);
    auto both=decide(8,10,1,1,3,2,4,4,4,Phase::StraightEntry,Mode::Control);
    CHECK(both.selected_side==2 && both.right.accepted() && near(both.right.after_mm,11));
    CHECK(both.left.reason==Reason::WrongSide && near(both.left.after_mm,8));
    auto residual=decide(8,10,1,7,3,0,4,4,4,Phase::StraightEntry,Mode::Control);
    CHECK(residual.selected_side==2 && residual.left.reason==Reason::OutsideWindow);
    auto closest=decide(8,10,2,1,3,0,4,4,4,Phase::StraightEntry,Mode::Control);
    CHECK(closest.selected_side==1 && closest.right.reason==Reason::NotSelected);
    CHECK(near(closest.right.after_mm,8) && near(closest.left.after_mm,11));
    auto stale=decide(8,10,1,1,2,0,4,3,4,Phase::StraightEntry,Mode::Control);
    CHECK(!stale.selected_side && stale.right.reason==Reason::StaleGeneration);
    auto invalid=decide(std::numeric_limits<float>::quiet_NaN(),10,1,1,3,2,4,3,4,Phase::StraightEntry,Mode::Control);
    CHECK(!invalid.selected_side && invalid.right.reason==Reason::Invalid && invalid.left.reason==Reason::Invalid);
    for (auto phase : {Phase::StraightEntry,Phase::DiagonalEntry,Phase::StraightExit,Phase::DiagonalExit,Phase::SearchStraight}) {
        auto shadow=decide(8,10,1,1,2,0,4,4,4,phase,Mode::Shadow);
        CHECK(shadow.selected_side==2 && shadow.right.reason==Reason::Shadow);
        CHECK(near(shadow.right.after_mm,8) && shadow.right.delta_mm==0 && shadow.right.proposed_delta_mm>0);
        auto control=decide(8,10,1,1,2,0,4,4,4,phase,Mode::Control);
        const bool allowed=phase==Phase::StraightEntry || phase==Phase::SearchStraight;
        CHECK(control.right.accepted()==allowed);
        CHECK(allowed || (control.right.reason==Reason::Shadow && near(control.right.after_mm,8)));
        auto legacy=decide(8,10,1,1,2,0,4,4,4,phase,Mode::Legacy);
        CHECK(legacy.right.accepted());
    }
}

static void wall_edge_trial_motion_application()
{
    using namespace WallPositionCorrection;
    Vehicle state{}; Vehicle *vehicle=&state;
    IrSensTask_type8 sensors{}; IrSensTask *ir_sens=&sensors;
    auto motion_pattern_get=[](){return Straight;};
    auto HAL_GetTick=[](){return uint32_t(100);};
    auto apply=[&](float reference_mm,Phase phase,uint8_t expected_side) {
        #include "wall_position_apply.inc"
    };
    auto &diag=WallDiagnostics::instance();diag.begin(Mode::Control);
    sensors.WallEkfMode_Set(IrSensTask::WallEkfMode::Legacy); // This case tests the direct fallback path.
    sensors.WallEdgeMode_Set(Mode::Control); vehicle->ego.length.set(8);
    sensors.l_wall_corner=True;sensors.l_corner_generation=sensors.wall_edge_generation;
    sensors.l_corner_delay_mm=1;
    apply(10,Phase::StraightEntry,2);
    CHECK(sensors.wall_correction==False && near(vehicle->ego.length.get(),8));
    CHECK(diag.size()==1 && diag.event(0).correction.reason==Reason::WrongSide);
    sensors.r_wall_corner=True;sensors.r_corner_generation=sensors.wall_edge_generation;
    sensors.r_corner_delay_mm=1;
    apply(10,Phase::StraightEntry,2);
    CHECK(sensors.wall_correction==True && near(vehicle->ego.length.get(),11));
    CHECK(diag.event(1).side==2 && diag.event(1).correction.accepted());
    CHECK(diag.event(2).side==1 && diag.event(2).correction.reason==Reason::WrongSide);
    apply(10,Phase::StraightEntry,2); CHECK(diag.size()==3); // One consumption per boundary.
    sensors.WallEdgeMode_Set(Mode::Shadow);vehicle->ego.length.set(8);
    sensors.r_wall_corner=True;sensors.r_corner_generation=sensors.wall_edge_generation;
    sensors.r_corner_delay_mm=1;
    apply(10,Phase::StraightEntry,2);
    CHECK(sensors.wall_correction==True && near(vehicle->ego.length.get(),8));
    CHECK(diag.event(3).correction.reason==Reason::Shadow && near(diag.event(3).correction.proposed_after_mm,11));
    sensors.Division_Wall_Correction_Reset();sensors.r_wall_corner=True;
    sensors.r_corner_generation=sensors.wall_edge_generation-1;
    apply(10,Phase::StraightEntry,2);
    CHECK(sensors.wall_correction==False && diag.event(4).correction.reason==Reason::StaleGeneration);
    // A single IRQ can pass main->post. The mode switch erases the old flags.
    sensors.IrSensMotion_Set(TURN_IRSENS);sensors.EnableIrSensStraight();
    apply(10,Phase::StraightExit,0); CHECK(diag.size()==5 && near(vehicle->ego.length.get(),8));
    diag.finish_and_print();
}

static void wall_edge_trial_shell()
{
    using namespace WallPositionCorrection;
    auto &ir=IrSensTask_type8::getInstance();auto &motion=CtrlTask_type8::getInstance();
    char *args[]={const_cast<char *>("wall_edge"),const_cast<char *>("shadow"),const_cast<char *>("extra")};
    CHECK(ir.WallEdgeMode_Get()==Mode::Legacy);
    ir.r_wall_corner=True;
    CHECK(usrcmd_wall_edge(2,args)==0 && ir.WallEdgeMode_Get()==Mode::Shadow && ir.r_wall_corner==False);
    const auto generation=ir.wall_edge_generation;
    args[1]=const_cast<char *>("control");motion.status=execute;
    CHECK(usrcmd_wall_edge(2,args)==-1 && ir.WallEdgeMode_Get()==Mode::Shadow && ir.wall_edge_generation==generation);
    motion.status=complete;motion.enabled=True;
    CHECK(usrcmd_wall_edge(2,args)==-1 && test_irq_mask==0);
    motion.enabled=False;CHECK(usrcmd_wall_edge(2,args)==0 && ir.WallEdgeMode_Get()==Mode::Control);
    args[1]=const_cast<char *>("show");test_irq_mask=1;
    CHECK(usrcmd_wall_edge(2,args)==0 && test_irq_mask==1);test_irq_mask=0;
    args[1]=const_cast<char *>("bad");CHECK(usrcmd_wall_edge(2,args)==-1);
    CHECK(usrcmd_wall_edge(3,args)==-1);
}

static void straight_pillar_peak()
{
    for (float step : {.125f,.37f,1.0f,1.6f}) {
        StraightPillarEdge detector;int count=0;
        for (float s=0;s<100;s+=step) {
            const float height=s<20?0:s<40?(s-20)*12:s<60?(60-s)*12:0;
            auto e=detector.update(80,static_cast<int16_t>(180+height),s==0?0:step);
            if(e.detected){++count;CHECK(std::fabs(s-e.delay_mm-40)<step+1);CHECK(e.motion_clean && e.prominence>200);}
        }
        CHECK(count==1); // ADC below the calibration endpoint still produces a feature.
    }
    StraightPillarEdge wall;
    for(int i=0;i<160;++i)CHECK(!wall.update(i<80?45:80,i<80?1400:180,.5f).detected);
    StraightPillarEdge entering;
    for(int i=0;i<240;++i)CHECK(!entering.update(i<40?80:45,i<40?180:1400,.5f).detected);
    // Recorded A search section 14: an earlier Open interval must not remain
    // qualified after a long wall, whose gentle ADC hill ends in Wall->Open.
    StraightPillarEdge stale_open;
    for(int i=0;i<60;++i)CHECK(!stale_open.update(80,0,.5f).detected);
    for(int i=0;i<200;++i)CHECK(!stale_open.update(55,side_adc_at(sens_sr_table,55),.5f).detected);
    for(int i=0;i<50;++i) {
        const float range=55-i*.16f;
        CHECK(!stale_open.update(range,side_adc_at(sens_sr_table,range),.5f).detected);
    }
    for(int i=0;i<100;++i) {
        const float range=fminf(80,47+i*.5f);
        CHECK(!stale_open.update(range,side_adc_at(sens_sr_table,range),.5f).detected);
    }
    // Requalify Open and still detect the next weak, genuinely separate pillar.
    unsigned next_pillar=0;
    for(int i=0;i<160;++i) {
        const float x=i*.5f;
        const float height=fmaxf(0,1-fabsf(x-40)/20);
        next_pillar+=stale_open.update(80,static_cast<int16_t>(240*height),.5f).detected;
    }
    CHECK(next_pillar==1);
    StraightPillarEdge spike;
    for(int i=0;i<60;++i)CHECK(!spike.update(80,180,.5f).detected);
    CHECK(!spike.update(80,1400,.5f).detected);
    for(int i=0;i<30;++i)CHECK(!spike.update(80,180,.5f).detected);
    CHECK(!spike.update(80,180,-.5f).detected);
    CHECK(!spike.update(80,-1,.5f).detected);
    CHECK(!spike.update(80,180,6.1f).detected);
}

static void straight_close_pillar_history()
{
    // A search 2026-11-17 section 12: Open before/after a close pillar,
    // including >12mm below 62mm. Neither side may become Wall->Open.
    using Mode=WallPositionCorrection::Mode;
    for (Mode mode : {Mode::Shadow,Mode::Control,Mode::Legacy})
    for (bool rotating : {false,true})
    for (bool right : {false,true}) for (float step : {.125f,.37f,1.0f,1.6f}) {
        IrSensTask_type8 task{};Vehicle vehicle{};
        task.WallEdgeMode_Set(mode);task.EnableIrSensStraight();
        vehicle.ideal.velo.set(.37f);
        auto &diag=PillarDiagnostics::instance();diag.begin();
        const auto *table=right?sens_sr_table:sens_sl_table;
        unsigned wall_count=0;bool entered_prev=false;
        for(float x=0;x<100;x+=step) {
            if(x>=25 && !entered_prev) {task.EnableIrSensStraight();entered_prev=true;}
            const float range=x<28?80:x<35?80-(x-28)*18/7:x<42?62-(x-35)
                :x<49?55+(x-42):x<56?62+(x-49)*18/7:80;
            side_wall_test_adc[right?sensor_sr:sensor_sl]=side_adc_at(table,range);
            side_wall_test_adc[right?sensor_sl:sensor_sr]=0;
            side_wall_test_adc[sensor_fl]=side_wall_test_adc[sensor_fr]=0;
            vehicle.ego.length.set(x);
            vehicle.ego.rad_velo.set(rotating && x>=40 && x<44?3.f:0.f);
            task.IrSensorSet(&vehicle,x==0?0:step,1000+static_cast<uint32_t>(x/step));
            wall_count+=task.r_wall_corner==True || task.l_wall_corner==True;
            Update_LED_Pulses(65535);Indicate_LED(0);
            task.RecordPillarCandidates(&vehicle,Search_st_section,STRAIGHT_STATE,0,true);
            if(rotating)CHECK(led_gpio==0); // Record a tainted candidate; never indicate it.
        }
        CHECK(wall_count==(mode==Mode::Legacy?1U:0U));
        CHECK(diag.size()==(mode==Mode::Legacy?0U:1U));
        if(mode!=Mode::Legacy) {
            CHECK(diag.event(0).side==(right?2:1));
            CHECK(diag.event(0).flags==(rotating?PillarDiagnostics::Rotating:0));
            CHECK(std::fabs(diag.event(0).feature_mm-42)<step+2);
        }
        diag.stop();
    }
}

static void straight_history_recovery()
{
    // A long wall after Open must lose its old Open identity, even if there
    // are brief distant readings or a later ADC hill. Wall->Open remains valid.
    StraightWallEdge edge;StraightPillarEdge pillar;
    unsigned wall_count=0,pillar_count=0;
    auto sample=[&](float range,float step=.5f,bool valid=true) {
        wall_count+=edge.update(range,step,valid,wall_edge_trial_param,true).detected;
        pillar_count+=pillar.update(range,side_adc_at(sens_sr_table,range),step,valid).detected;
    };
    for(int i=0;i<60;++i)sample(80);
    for(int i=0;i<80;++i)sample(55);
    sample(80); // One distant sample cannot requalify Open.
    for(int i=0;i<80;++i)sample(55);
    for(int i=0;i<50;++i)sample(55-i*.16f);
    for(int i=0;i<100;++i)sample(fminf(80,47+i*.5f));
    CHECK(wall_count==1 && pillar_count==0);

    // Startup on a real wall, with no qualified Open, still arms after 12mm.
    edge.reset();pillar.reset();wall_count=pillar_count=0;
    for(int i=0;i<32;++i)sample(50);
    for(int i=0;i<20;++i)sample(80);
    CHECK(wall_count==1 && pillar_count==0);

    // A stopped close pulse cannot later resume as either feature. A fresh
    // continuous wall must rearm normally afterwards.
    edge.reset();pillar.reset();wall_count=pillar_count=0;
    for(int i=0;i<60;++i)sample(80);
    for(int i=0;i<30;++i)sample(55);
    for(int i=0;i<6;++i)sample(55,0);
    for(int i=0;i<20;++i)sample(80);
    CHECK(wall_count==0 && pillar_count==0);
    for(int i=0;i<100;++i)sample(50);
    for(int i=0;i<30;++i)sample(80);
    CHECK(wall_count==1 && pillar_count==0);

    // Invalid/reverse samples must erase the qualified Open as well.
    for(bool reverse : {false,true}) {
        edge.reset();pillar.reset();wall_count=pillar_count=0;
        for(int i=0;i<60;++i)sample(80);
        sample(80,reverse?-.5f:.5f,reverse);
        for(int i=0;i<32;++i)sample(55);
        for(int i=0;i<20;++i)sample(80);
        CHECK(wall_count==1 && pillar_count==0);
    }
}

static void open_origin_wall_control()
{
#ifndef OPEN_ORIGIN_BEFORE
    using Mode=WallPositionCorrection::Mode;
    for(auto mode:{IrSensTask::WallEkfMode::Shadow,IrSensTask::WallEkfMode::Control})
    for(bool right:{false,true})for(bool opposite_wall:{false,true})for(float step:{.125f,.7f,1.6f}) {
        IrSensTask_type8 task{};Vehicle vehicle{};
        task.WallEkfMode_Set(mode);task.WallEdgeMode_Set(mode==IrSensTask::WallEkfMode::Control?Mode::Control:Mode::Shadow);
        task.EnableIrSensStraight();task.PrimeSearchTurnExit(.02f,true);
        vehicle.ideal.velo.set(.7f);vehicle.ego.velo.set(.7f);vehicle.ego.x_point.set(1);
        for(auto &a:side_wall_test_adc)a=0;
        auto &diag=PillarDiagnostics::instance();diag.begin();unsigned walls=0;uint32_t tick=0;
        for(float s=0;s<135;s+=step) {
            const float range=s<30?80:s<60?80-(s-30)*50/30:s<67?30:s<78?30+(s-67)*50/11:80;
            side_wall_test_adc[right?sensor_sr:sensor_sl]=range<=25?4095:side_adc_at(right?sens_sr_table:sens_sl_table,range);
            side_wall_test_adc[right?sensor_sl:sensor_sr]=opposite_wall?side_adc_at(right?sens_sl_table:sens_sr_table,45):0;
            vehicle.ego.length.set(s);task.IrSensorSet(&vehicle,s==0?0:step,++tick);
            walls+=right?task.r_wall_corner==True:task.l_wall_corner==True;
            task.RecordPillarCandidates(&vehicle,Straight,STRAIGHT_STATE,0,false);
            task.SetWallControl_RadVelo(&vehicle,1);
            const unsigned side=right?2U:1U,other=right?1U:2U;
            if(s>32 && s<78) {
                CHECK(task.straight_pillar_guard_mask&side);
                CHECK(!(task.straight_wall_output.observed_mask&side));
                CHECK(!(task.straight_wall_output.trusted_mask&side));
                CHECK(!(task.straight_wall_candidate_mask&side));
                if(opposite_wall) {
                    CHECK(task.straight_wall_output.observed_mask&other);
                    CHECK(task.straight_wall_output.trusted_mask&other);
                    CHECK(task.straight_wall_candidate_mask&other);
                }
                else {
                    CHECK(task.straight_wall_output.phase==ZirconiaWallControl::Estimated);
                    CHECK(near(task.control_ir.get(),0));
                    CHECK(vehicle.ideal.rad_velo.get()<0); // Hold the inherited +yaw back toward the axis.
                }
            }
        }
        CHECK(walls==0 && diag.size()==1 && diag.event(0).flags==0);diag.stop();
        task.IrSensMotion_Set(TURN_IRSENS);CHECK(task.straight_pillar_guard_mask==0);
    }
    // Open evidence does not blind the detector forever: a stable real wall
    // reacquires after the existing width bound; even a near-limit wall has a one-cell bound.
    for(bool saturated:{false,true}) {
        StraightPillarEdge peak;
        for(int i=0;i<40;++i)peak.update(80,0,.5f);
        CHECK(peak.open_origin());
        for(int i=0;i<200;++i)peak.update(saturated?25:50,saturated?4095:1200,.5f,true,true,0,!saturated);
        CHECK(!peak.open_origin());
    }
    // Startup on a close actual wall has no qualified Open; retain repulsion.
    for(bool right:{false,true}) {
        IrSensTask_type8 task{};Vehicle vehicle{};task.WallEkfMode_Set(IrSensTask::WallEkfMode::Shadow);
        task.EnableIrSensStraight();vehicle.ideal.velo.set(.7f);vehicle.ego.velo.set(.7f);
        for(auto &a:side_wall_test_adc)a=0;
        side_wall_test_adc[right?sensor_sr:sensor_sl]=4095;
        for(int i=0;i<130;++i){task.IrSensorSet(&vehicle,.5f,i);task.SetWallControl_RadVelo(&vehicle,1);}
        CHECK(!(task.straight_pillar_guard_mask&(right?2U:1U)));
        CHECK(std::fabs(task.control_ir.get())>10 && !task.straight_wall_output.pose_control);
    }
#endif
}

static void recorded_open_pillar(const char *path)
{
    struct Sample {unsigned ms;float length,gyro;int l,r,fl,fr;float speed;};
    Sample samples[80];unsigned n=0;char line[256];FILE *f=std::fopen(path,"r");CHECK(f);
    CHECK(std::fgets(line,sizeof(line),f));
    while(std::fgets(line,sizeof(line),f)) {
        CHECK(n<80);auto &s=samples[n++];
        CHECK(std::sscanf(line,"%u,%f,%f,%d,%d,%d,%d,%f",&s.ms,&s.length,&s.gyro,&s.l,&s.r,&s.fl,&s.fr,&s.speed)==8);
    }
    std::fclose(f);CHECK(n==55);
    for(auto mode:{IrSensTask::WallEkfMode::Shadow,IrSensTask::WallEkfMode::Control})for(bool quiet:{false,true}) {
        IrSensTask_type8 task{};Vehicle v{};task.WallEkfMode_Set(mode);task.EnableIrSensStraight();
        // Deliberately place the synthetic references near the recorded peaks:
        // rejection must come from floor/plateau quality, not an absent frame.
        task.BeginStraightRunFeatures(59.5f);const auto frame=task.StraightRunFeatureContext();
        v.ideal.velo.set(.7f);v.ego.velo.set(.7f);v.ego.x_point.set(.8f);
        for(auto &a:side_wall_test_adc)a=0;
        for(int i=0;i<40;++i){task.IrSensorSet(&v,.7f,i);task.SetWallControl_RadVelo(&v,1);}
        auto &diag=PillarDiagnostics::instance();diag.begin();unsigned wall_count=0,protected_samples=0,wrong_observation=0;float max_error=0;
        float previous=samples[0].length;
        for(unsigned i=1;i<n;++i) {
            const auto &a=samples[i-1],&b=samples[i];const unsigned dt=b.ms-a.ms;CHECK(dt<=6);
            for(unsigned k=1;k<=dt;++k) {
                const float t=static_cast<float>(k)/dt;
                side_wall_test_adc[sensor_sl]=static_cast<int16_t>(a.l+t*(b.l-a.l));
                side_wall_test_adc[sensor_sr]=static_cast<int16_t>(a.r+t*(b.r-a.r));
                side_wall_test_adc[sensor_fl]=static_cast<int16_t>(a.fl+t*(b.fl-a.fl));
                side_wall_test_adc[sensor_fr]=static_cast<int16_t>(a.fr+t*(b.fr-a.fr));
                const float length=a.length+t*(b.length-a.length),step=length-previous;previous=length;
                v.ego.length.set(length);v.ego.velo.set(a.speed+t*(b.speed-a.speed));v.ego.rad_velo.set(quiet?0:a.gyro+t*(b.gyro-a.gyro));
                task.IrSensorSet(&v,step,a.ms+k);wall_count+=task.r_wall_corner==True;
                task.RecordPillarCandidates(&v,Straight,STRAIGHT_STATE,0,false,frame);task.SetWallControl_RadVelo(&v,1);
                max_error=std::fmax(max_error,std::fabs(task.control_ir.get()));
#ifndef OPEN_ORIGIN_BEFORE
                if(task.straight_pillar_guard_mask&2U) {
                    ++protected_samples;
                    wrong_observation+=(task.straight_wall_output.observed_mask&2U)!=0;
                    CHECK(!(task.straight_wall_candidate_mask&2U));
                }
#endif
            }
        }
        unsigned right_count=0,right_clean=0,right_clipped=0;
        for(unsigned j=0;j<diag.size();++j)if(diag.event(j).side==2){++right_count;right_clean+=diag.event(j).flags==0;right_clipped+=(diag.event(j).flags&PillarDiagnostics::SideNearLimit)!=0;}
        std::printf("{\"mode\":\"%s\",\"quiet\":%s,\"wall_events\":%u,\"pillar_events\":%u,\"right_pillar_events\":%u,\"right_clean\":%u,\"right_clipped\":%u,\"protected_samples\":%u,\"wrong_observation\":%u,\"max_control_ir_mm\":%.4f}\n",
            mode==IrSensTask::WallEkfMode::Control?"control":"shadow",quiet?"true":"false",wall_count,diag.size(),right_count,right_clean,right_clipped,protected_samples,wrong_observation,max_error);
#ifndef OPEN_ORIGIN_BEFORE
        CHECK(wall_count==0 && right_count==2 && protected_samples>150 && wrong_observation==0);
        CHECK(max_error<1.f);
        CHECK(right_clipped==2 && right_clean==0);
        CHECK(task.straight_wall_estimate.longitudinal_updates==0);
#endif
        diag.stop();
    }
}

static void diagonal_pillar_minimum()
{
    for(float step : {.125f,.37f,1.0f,1.6f}) {
        DiagonalPillarEdge early;int count=0;
        for(float s=0;s<20;s+=step) {
            auto e=early.update(30+2*std::fabs(s-5),2000,s==0?0:step);
            if(e.detected){++count;CHECK(s<11 && std::fabs(s-e.delay_mm-5)<step+1);}
        }
        CHECK(count==1);
        DiagonalPillarEdge flat;count=0;
        for(float s=0;s<65;s+=step) {
            const float range=s<10?40-1.5f*s:s<28?25:std::fmin(80.0f,25+1.5f*(s-28));
            auto e=flat.update(range,range==25?4060:1500,s==0?0:step);
            if(e.detected){++count;CHECK(e.width_mm>16 && e.width_mm<21);CHECK(std::fabs(s-e.delay_mm-19)<step+1);}
        }
        CHECK(count==1);
    }
    DiagonalPillarEdge wide;int count=0;
    for(float s=0;s<155;s+=.5f) {
        const float range=s<90?70-.5f*s:s<108?25:std::fmin(80.0f,25+1.5f*(s-108));
        auto e=wide.update(range,2000,s==0?0:.5f);
        if(e.detected){++count;CHECK(e.width_mm>=18);CHECK(std::fabs(s-e.delay_mm-99)<1);}
    }
    CHECK(count==1); // Long approach before the minimum must not time out the plateau.
}

static void pillar_quality_rearm()
{
    DiagonalPillarEdge rotating;int count=0;
    for(float s=0;s<60;s+=.5f) {
        const float range=s<10?40-1.5f*s:s<28?25:std::fmin(80.0f,25+1.5f*(s-28));
        auto e=rotating.update(range,2000,s==0?0:.5f,true,!(s>=20 && s<24),s==22?PillarDiagnostics::FrontPairClose:0);
        if(e.detected){++count;CHECK(!e.motion_clean && e.width_mm>=18 && (e.context_flags & PillarDiagnostics::FrontPairClose));}
    }
    CHECK(count==1);
    DiagonalPillarEdge ripples;float last=-100;count=0;
    for(float s=0;s<180;s+=.5f){
        const float phase=std::fmod(s,12.0f);const float range=50+std::fabs(phase-6);
        auto e=ripples.update(range,1500,s==0?0:.5f);
        if(e.detected){CHECK(s-last>=20);last=s;++count;}
    }
    CHECK(count>=2 && count<12);
    DiagonalPillarEdge stopped;
    stopped.update(40,2000,0);stopped.update(35,2100,1);stopped.update(30,2200,1);
    for(int i=0;i<65540;++i)CHECK(!stopped.update(25,2300,0).detected);
    CHECK(!stopped.update(40,2100,.5f).detected);
    CHECK(!stopped.update(25,2100,-.1f).detected);
    CHECK(!stopped.update(25,2100,.1f,false).detected);
}

static void pillar_ir_wire()
{
    IrSensTask_type8 task{};Vehicle vehicle{};
    task.WallEdgeMode_Set(WallPositionCorrection::Mode::Shadow);task.EnableIrSensDiagonal();
    PillarDiagnostics::instance().begin();vehicle.ideal.velo.set(.5f);
    vehicle.ego.x_point.set(4);vehicle.ego.radian.set(.03f);vehicle.ego.rad_velo.set(.1f);
    side_wall_test_adc[sensor_sl]=side_wall_test_adc[sensor_fl]=0;
    side_wall_test_adc[sensor_fr]=sens_fr_table[8]; // A single front return must not erase the pillar.
    int i=0;
    auto sample=[&](int16_t adc){
        side_wall_test_adc[sensor_sr]=adc;vehicle.ego.length.set(i*.5f);
        task.IrSensorSet(&vehicle,.5f,1000+i);task.RecordPillarCandidates(&vehicle,Diagonal,DIAGONAL_STATE,Post_Turn,false);
        CHECK(near(vehicle.ego.length.get(),i*.5f) && near(vehicle.ego.x_point.get(),4));++i;
    };
    for(int index=5;index>=0;--index)for(int k=0;k<3;++k)sample(sens_sr_table[index]);
    for(int k=0;k<36;++k)sample(sens_sr_table[0]);
    for(int index=1;index<7;++index)for(int k=0;k<3;++k)sample(sens_sr_table[index]);
    auto &diag=PillarDiagnostics::instance();CHECK(diag.size()==1);
    const auto &e=diag.event(0);CHECK(e.axis==1 && e.side==2 && e.flags==0 && e.peak.width_mm>=18);
    CHECK(e.minimum.adc[3]==sens_fr_table[8] && e.minimum.range[2]==IrRangeState::FarLimit);
    CHECK(e.minimum.ms<e.confirmation.ms && !e.reference_valid);
    CHECK(task.wall_correction==False); // New features never consume Wall->Open's correction latch.
    diag.finish_and_print();
    // Close front pairs and frame/position changes are recorded, not silently erased.
    diag.begin();auto m=e.minimum;auto c=e.confirmation;auto peak=e.peak;
    m.distance[2]=m.distance[3]=60;m.range[2]=m.range[3]=IrRangeState::InRange;
    c.generation=m.generation+1;c.length_mm+=5;
    diag.record(peak,m,c,1,2,Diagonal,DIAGONAL_STATE,Post_Turn,WallPositionCorrection::Mode::Shadow,false);
    CHECK((diag.event(0).flags & PillarDiagnostics::FrontPairClose) && (diag.event(0).flags & PillarDiagnostics::FrameChanged));
    CHECK(diag.event(0).flags & PillarDiagnostics::PositionJump);
    diag.stop();
}

static void wall_pillar_leds()
{
    // Pulses survive base clears/read-modify-write, expire independently, and
    // restore exactly the requested base pattern rather than latching forever.
    Update_LED_Pulses(65535); Indicate_LED(0x03);
    Indicate_LED_Pulse(0x10,50);
    CHECK(led_gpio==0x13 && Return_LED_Status()==0x03);
    Indicate_LED(Return_LED_Status() | 0x04); CHECK(led_gpio==0x17);
    Update_LED_Pulses(25); Indicate_LED_Pulse(0x20,50);
    Update_LED_Pulses(24); CHECK(led_gpio==0x37);
    Update_LED_Pulses(1); CHECK(led_gpio==0x27);
    Update_LED_Pulses(25); CHECK(led_gpio==0x07);
    Indicate_LED_Pulse(0x30,0); CHECK(led_gpio==0x07);
    Indicate_LED_Pulse(0x10,50); Update_LED_Pulses(40);
    Indicate_LED_Pulse(0x10,50); Indicate_LED(0);
    Update_LED_Pulses(49); CHECK(led_gpio==0x10);
    Update_LED_Pulses(1); CHECK(led_gpio==0);

    using Mode=WallPositionCorrection::Mode;
    for (bool diagonal : {false,true}) for (bool right : {false,true}) {
        const auto *table=right?sens_sr_table:sens_sl_table;
        const uint8_t mask=right?0x10:0x20;
        // Both legacy feature types light at detection, including Shadow.
        for (Mode mode : {Mode::Legacy,Mode::Shadow,Mode::Control}) {
            IrSensTask_type8 task{};Vehicle vehicle{};
            task.WallEdgeMode_Set(mode);
            if(diagonal) task.EnableIrSensDiagonal();else task.EnableIrSensStraight();
            vehicle.ideal.velo.set(.37f);
            side_wall_test_adc[sensor_sl]=side_wall_test_adc[sensor_sr]=0;
            side_wall_test_adc[sensor_fl]=side_wall_test_adc[sensor_fr]=0;
            unsigned detections=0;
            for(int i=0;i<180;++i) {
                const float x=i*.5f;
                const float range=diagonal ? fminf(80,30+fabsf(x-25)*2) : fminf(80,45+fmaxf(0,x-40)*4);
                Update_LED_Pulses(65535); Indicate_LED(0x03);
                side_wall_test_adc[right?sensor_sr:sensor_sl]=side_adc_at(table,range);
                task.IrSensorSet(&vehicle,.5f,1000+i);
                const bool found=right?task.r_wall_corner==True:task.l_wall_corner==True;
                CHECK(led_gpio==(found?(0x03|mask):0x03));
                detections+=found;
            }
            CHECK(detections==1);
        }
        // New peak detectors work without logging, and when its buffer is full.
        for (bool full_log : {false,true}) for(bool bad_quality : {false,true}) {
            if(!diagonal && bad_quality)continue; // Rotation prevents straight Open arming.
            IrSensTask_type8 task{};Vehicle vehicle{};
            task.WallEdgeMode_Set(Mode::Shadow);
            if(diagonal)task.EnableIrSensDiagonal();else task.EnableIrSensStraight();
            vehicle.ideal.velo.set(.37f);vehicle.ego.rad_velo.set(bad_quality?3.f:0.f);
            auto &diag=PillarDiagnostics::instance();diag.begin();
            if(full_log) {
                SpatialPillarPeak::Event peak{};peak.detected=peak.motion_clean=true;
                PillarDiagnostics::Sample sample{};
                for(unsigned i=0;i<diag.capacity;++i)diag.record(peak,sample,sample,0,1,19,1,0,Mode::Shadow,false);
            } else diag.stop();
            side_wall_test_adc[sensor_sl]=side_wall_test_adc[sensor_sr]=0;
            side_wall_test_adc[sensor_fl]=side_wall_test_adc[sensor_fr]=0;
            unsigned indications=0;
            for(int i=0;i<180;++i) {
                const float x=i*.5f;
                const float height=fmaxf(0,1-fabsf(x-40)/20);
                const int peak_adc=static_cast<int>(fminf(240,table[11]-10)); CHECK(peak_adc>100);
                const int16_t adc=diagonal ? side_adc_at(table,fminf(80,30+fabsf(x-25)*2)) : static_cast<int16_t>(peak_adc*height);
                side_wall_test_adc[right?sensor_sr:sensor_sl]=adc;
                vehicle.ego.length.set(x);
                task.IrSensorSet(&vehicle,.5f,2000+i);
                // Remove the old difference detector's pulse before checking
                // that the new minimum/ADC detector itself drives the LED.
                Update_LED_Pulses(65535); Indicate_LED(0x03);
                task.RecordPillarCandidates(&vehicle,diagonal?Diagonal:Search_st_section,diagonal?DIAGONAL_STATE:STRAIGHT_STATE,0,!diagonal);
                if(led_gpio & mask) {++indications;CHECK(!bad_quality && led_gpio==(0x03|mask));}
                CHECK(Return_LED_Status()==0x03);
            }
            CHECK(indications==(bad_quality?0U:1U));
            if(full_log)CHECK(diag.size()==diag.capacity);
            diag.stop();
        }
    }
    Update_LED_Pulses(65535);Indicate_LED(0);
}

class ShortestEntryHarness {
public:
    Vehicle storage{}; Vehicle *vehicle=&storage;
    IrSensTask_type8 sensors{}; IrSensTask *ir_sens=&sensors;
    struct {param_element turn_state,end_length,fix_prev_run,fix_post_run,accel,deccel,end_velo,turn_time_ms,rad_max_velo;} motion_plan;
    t_param turn_motion_param{};t_straight_param straight_motion_param{};
    t_run_pattern pattern=Straight;t_runControl control=STRAIGHT_STATE;
    uint32_t tick=10000;float clock=0,start_clock=0;
    SearchTurnHeading shortest_turn_heading;
    static constexpr int deltaT_ms=1;
    static constexpr float detect_wall_edge_st=STRAIGHT_CORRECTION;
    static constexpr float detect_wall_edge_di=0;
    t_run_pattern motion_pattern_get() const{return pattern;}
    t_runControl motion_control_get() const{return control;}
    t_bool motion_is_enable_get()const{return True;}
    void motion_pattern_set(t_run_pattern p){pattern=p;}
    void motion_exeStatus_set(t_exeStatus){}
    void motion_control_set(t_runControl c){control=c;}
    float run_time_ms_get() const{return clock;}
    void run_time_ms_update(){clock+=1;}
    void turn_start_time_ms_set(float t){start_clock=t;}
    float turn_start_time_ms_get() const{return start_clock;}
    void turn_start_time_ms_reset(){start_clock=0;}
    uint32_t HAL_GetTick()const{return tick;}
    float get_turn_table_value(float,float){return .5f;}
    void active_ff_gain_set(const t_ff_gain*){}
    void updateTurnKinematics(){}
    void handleTurnPost_Diagonal(){
#include "diagonal_post.inc"
    }
    void SetIdeal_wall_control(){
#include "search_wall_control.inc"
    }
    void record(){
#include "shortest_record.inc"
    }
    void rebase(){
#include "shortest_rebase.inc"
    }
    void ApplyWallPositionCorrection(float reference_mm,WallPositionCorrection::Phase phase,uint8_t expected_side=0){
#include "wall_position_apply.inc"
    }
    void handleTurnPrev_Straight(){
#include "shortest_prev.inc"
    }
    void handleTurnPrev_Diagonal(){
#include "shortest_diagonal_prev.inc"
    }
    void handleTurnMain(){
#include "shortest_main.inc"
    }
    void update(){
#include "shortest_update.inc"
    }
    void post(){
#include "shortest_post.inc"
    }
    void run_features(){
#include "shortest_run.inc"
    }
    void resume(){
#include "shortest_resume.inc"
        pattern=Straight;
    }
    void pause(){
#include "shortest_pause.inc"
    }
    void resume_diagonal(){
        const bool continue_diagonal_pose=motion_control_get()==DIAGONAL_STATE;
#include "diagonal_resume.inc"
        pattern=Diagonal;control=DIAGONAL_STATE;
    }
    void init_long_v90(bool variable){
        if(variable) {
#include "variable_long_v90_init.inc"
        } else {
#include "long_v90_init.inc"
        }
    }
};

static void diagonal_feature_model()
{
    using namespace WallPositionCorrection;
    using S=DiagonalFeatureShadow;
    CHECK(!S::apply_enabled());
    for(auto mode:{Mode::Legacy,Mode::Shadow,Mode::Control})
        CHECK(decide(10,11,0,0,1,0,0,0,0,Phase::DiagonalRun,mode).left.reason==Reason::Shadow);
    for(auto phase:{Phase::DiagonalEntry,Phase::DiagonalRun,Phase::DiagonalExit})
    for(auto kind:{S::DifferenceEdge,S::PillarMinimum})for(unsigned side:{1U,2U}) {
        auto p=diagonal_feature_shadow_param;
        for(auto &stage:p.reference)for(auto &type:stage)for(auto &r:type) {
            CHECK(!r.calibrated);r={true,40,127.27922f,side==1?1.0f:-1.0f,side==1?20.0f:-20.0f};
        }
        S s;for(int i=0;i<88;++i)s.advance(.5f,0,p);
        S::Input input;input.phase=phase;input.kind=kind;input.side=side;input.before_mm=44;input.delay_mm=2;
        input.left_mm=side==1?2:-2;input.heading_rad=side==1?.02f:-.02f;
        auto d=s.observe(input,p);
        CHECK(d.model.status==S::Ready && d.model.model_ready && d.model.reference_valid);
        CHECK(near(d.model.pose_shift_mm,2.4f) && near(d.position.target_mm,44.4f));
        CHECK(d.position.reason==Reason::Shadow && d.position.after_mm==44 && d.position.delta_mm==0);
        CHECK(d.position.ekf && d.position.ekf_gain>0 && d.position.ekf_gain<=.5f && d.position.proposed_delta_mm>0);
        CHECK(s.observe(input,p).model.status==S::Duplicate);
        for(int i=0;i<256;++i)s.advance(.5f,0,p);
        input.before_mm+=127.27922f;CHECK(s.observe(input,p).model.status==S::Ready);
        s.rebase(-90,.02f,3);input.before_mm-=90;
        CHECK(near(s.heading(0),.02f) && near(s.lateral(0),3));
        CHECK(s.observe(input,p).model.status==S::Duplicate);
        input.generation=1;CHECK(s.observe(input,p).model.status==S::Stale);
        input.event_generation=1;input.expected_side=side==1?2:1;CHECK(s.observe(input,p).model.status==S::WrongSide);
        input.expected_side=0;input.quality_flags=PillarDiagnostics::FrontPairClose;CHECK(s.observe(input,p).model.status==S::QualityRejected);
        input.quality_flags=0;input.left_mm=9;CHECK(s.observe(input,p).model.status==S::PoseOutside);
        input.left_mm=0;input.heading_rad=.1f;CHECK(s.observe(input,p).model.status==S::PoseOutside);
        input.heading_rad=0;input.gyro_radps=2.1f;CHECK(s.observe(input,p).model.status==S::PoseOutside);
        input.gyro_radps=0;input.before_mm+=15;CHECK(s.observe(input,p).model.status==S::OutsideWindow);
        S uncalibrated;uncalibrated.advance(.5f,0,diagonal_feature_shadow_param);
        CHECK(uncalibrated.observe(input,diagonal_feature_shadow_param).model.status==S::Uncalibrated);
        input.delay_mm=100;CHECK(uncalibrated.observe(input,p).model.status==S::Invalid);
        S outlier;outlier.advance(.5f,0,p);p.initial_std_mm=.01f;p.wall_std_mm=p.pillar_std_mm=.01f;
        outlier.reset();outlier.advance(.5f,0,p);input.delay_mm=2;input.before_mm=45;
        CHECK(outlier.observe(input,p).model.status==S::NisRejected);
    }
    // Post_Turn and the following run have separate calibrated origins.
    auto p=diagonal_feature_shadow_param;p.reference[2][1][0]={true,40,0,0,0};p.reference[1][1][0]={true,20,0,0,0};
    S s;s.begin_exit(20);s.advance(.5f,0,p);s.rebase(-20);
    S::Input in;in.before_mm=22;in.delay_mm=2;
    auto d=s.observe(in,p);CHECK(d.model.status==S::Ready && near(d.position.target_mm,22) && d.position.delta_mm==0);
}

static void diagonal_pose_route()
{
    using namespace DiagonalPose;
    const Parameters geometry{true,0.7f,{{{20,2},{20,2},{20,2},{20,2}},{{20,2},{20,2},{20,2},{20,2}}}};
    for (unsigned count:{1U,4U,9U}) for (unsigned first_side:{0U,1U}) {
        Filter filter;
        Route route;
        route.count=count;
        for (unsigned i=0;i<count;++i) route.pillars[i]=(i+first_side)%2;
        if (count==9) route.pillars[3]=Route::unknown;
        float travel=-35.0f;
        filter.start(travel,0,route);
        Result result;
        const float actual_lateral=4.0f;
        for (unsigned i=0;i<count;++i) {
            if (count==9 && (i==2 || i==3)) continue; // missing return and unknown map corner
            const unsigned side=(i+first_side)%2;
            const float lateral=(side==0?actual_lateral:-actual_lateral);
            const float pass=(i+0.5f)*DIAG_SECTION-20-0.7f*(0.5f*DIAG_SECTION-lateral);
            while (travel<pass-0.001f) {
                const float ds=fminf(1.0f,pass-travel);
                filter.advance(ds,0); travel+=ds;
            }
            const auto peak=filter.odometry();
            for (unsigned j=0;j<5;++j) { filter.advance(1,0); travel+=1; }
            result=filter.observe(side,peak,0,0,false,0,geometry);
            CHECK(result.status==Updated && result.index==i+1);
        }
        CHECK(result.used==count-(count==9?2U:0U));
        CHECK(result.sigma_s>8);
        if (count==1) CHECK(!result.lateral_heading_ready);
        else {
            CHECK(result.lateral_heading_ready && std::fabs(result.pose.e-actual_lateral)<0.6f);
            CHECK(std::fabs(result.pose.heading)<.02f);
        }
        const auto duplicate=filter.observe((count-1+first_side)%2,filter.odometry(),0,0,false,0,geometry);
        CHECK(duplicate.status==NoMatch);
        filter.stop();
        CHECK(filter.observe(first_side,{},0,0,false,0,geometry).status==NoRoute);
    }
    // A point halfway between mapped columns is outside the geometric gate.
    // A wide saturated interval that overlaps both still needs more evidence.
    Filter filter; Route ambiguous;
    ambiguous.count=3; ambiguous.pillars[1]=1;
    filter.start(1.5f*DIAG_SECTION-20-0.7f*(.5f*DIAG_SECTION),0,ambiguous);
    CHECK(filter.observe(0,filter.odometry(),0,0,false,0,geometry).status==NoMatch);
    CHECK(filter.observe(0,filter.odometry(),0,40,true,0,geometry).status==Ambiguous);
    Route clipped; clipped.count=1;
    filter.start(.5f*DIAG_SECTION-20-0.7f*(.5f*DIAG_SECTION),0,clipped);
    const auto interval=filter.observe(0,filter.odometry(),0,14,true,0,geometry);
    CHECK(interval.status==Interval && interval.used==0 && near(interval.sigma_s,20.0f));

    // The calibrated entry prior still permits the first mapped pillar to be
    // absent. Associate a later same-side peak without forcing pillar 1.
    const float next_left_peak=2.5f*DIAG_SECTION-20-0.7f*(.5f*DIAG_SECTION);
    filter.start(next_left_peak,0,ambiguous);
    const auto later=filter.observe(0,filter.odometry(),0,0,false,0,geometry);
    CHECK(later.status==Updated && later.index==3 && later.used==1);
}

static void diagonal_pose_steering_runtime()
{
    using namespace DiagonalPose;
    const Steering::Command fallback={0,0};
    for (float v:{0.5f,1.6f}) for (float sign:{-1.0f,1.0f}) {
        Steering control;
        const Pose pose={0,5*sign,0.045f*sign};
        CHECK(control.update(&pose,v,0,.001f,0,fallback).omega_radps==0);
        float omega=0;
        for (unsigned i=0;i<100;++i) {
            const auto cmd=control.update(&pose,v,v,.001f,omega,fallback);
            CHECK(std::fabs(cmd.omega_radps-omega)<=.01801f && std::fabs(cmd.alpha_radps2)<=18.01f);
            omega=cmd.omega_radps;
        }
        CHECK(sign*omega<-.1f && std::fabs(omega)<=.901f && v*std::fabs(omega)<=.551f);
        for (unsigned i=0;i<60;++i) {
            // A motion boundary may reset ideal.rad_velo before the slew ends.
            const auto cmd=control.update(nullptr,v,v,.001f,0,fallback);
            CHECK(std::fabs(cmd.omega_radps-omega)<=.01801f);
            omega=cmd.omega_radps;
        }
        CHECK(!control.engaged() && near(omega,0));
    }

    // Real IR conversion and candidate pipeline. During this synthetic pass
    // the robot stays 4 mm left of the diagonal and hence should steer right.
    for (IrSensTask::WallEkfMode mode:{IrSensTask::WallEkfMode::Control,IrSensTask::WallEkfMode::Shadow}) {
        IrSensTask_type8 task{};
        Vehicle vehicle{};
        auto &world=vehicle.pose;CHECK(world.set_start({}));world.start();world.set_phase(DIAGONAL_STATE);
        task.WallEkfMode_Set(mode);
        task.WallEdgeMode_Set(mode==IrSensTask::WallEkfMode::Control ? WallPositionCorrection::Mode::Control
            : WallPositionCorrection::Mode::Shadow);
        task.EnableIrSensDiagonal();
        vehicle.ideal.velo.set(.5f);vehicle.ego.velo.set(.5f);
        Route route;route.count=5;
        route.origin_x=2;route.origin_y=3;route.dx=1;route.dy=1;
        for (unsigned i=0;i<route.count;++i) route.pillars[i]=2U+(i%2); // forward wall, measured on both sides
        task.StageDiagonalPoseRoute(route);
        task.BeginDiagonalPoseRoute(&vehicle,-33,0);
        WallPositionCorrection::FeatureContext frame;
        frame.valid=true;frame.phase=WallPositionCorrection::Phase::DiagonalRun;
        for(auto &adc:side_wall_test_adc) adc=0;
        auto &diagnostics=PillarDiagnostics::instance();
        diagnostics.begin();
        bool correcting=false;
        for (unsigned step=1;step<=460;++step) {
            const float s=-33+step*.5f;
            float range[2]={80,80};
            for (unsigned i=0;i<route.count;++i) {
                const unsigned side=i%2;
                const auto &model=diagonal_pose_param.pillar[side][route.pillars[i]>>1];
                const float center=(i+.5f)*DIAG_SECTION-(diagonal_pose_param.calibrated ? model.offset_mm : 20)
                    -(0.5f*DIAG_SECTION+(side?4:-4))*.700208f;
                range[side]=fminf(range[side],40+2*fabsf(s-center));
            }
            side_wall_test_adc[sensor_sl]=side_adc_at(sens_sl_table,range[0]);
            side_wall_test_adc[sensor_sr]=side_adc_at(sens_sr_table,range[1]);
            vehicle.ego.length.set(s);
            world.predict_body(.5f,0,0);
            task.IrSensorSet(&vehicle,.5f,step);
            const auto before_world=world.snapshot();
            task.RecordPillarCandidates(&vehicle,Diagonal,DIAGONAL_STATE,0,false,frame);
            if(world.snapshot().corrections!=before_world.corrections) {
                Pose current;CHECK(task.DiagonalPoseControlReady(current));
                const auto adopted=world.snapshot();
                CHECK(std::fabs(adopted.world.x_mm-current.s-33)<.001f);
                CHECK(std::fabs(adopted.world.y_mm-current.e)<.001f);
                CHECK(std::fabs(adopted.world.yaw_rad-current.heading)<.00001f);
                const auto roundtrip=PoseFrames::compose(adopted.origin,adopted.local);
                CHECK(std::fabs(roundtrip.x_mm-adopted.world.x_mm)<.001f && std::fabs(roundtrip.y_mm-adopted.world.y_mm)<.001f);
            }
            task.SetWallControl_RadVelo(&vehicle,1);
            correcting=correcting || vehicle.ideal.rad_velo.get()<-.05f;
            CHECK(near(vehicle.ego.length.get(),s) && near(vehicle.ego.radian.get(),0));
        }
        const bool expected=diagonal_pose_param.calibrated && mode==IrSensTask::WallEkfMode::Control;
        if (correcting!=expected || (task.DiagonalWallDiagnostics().pose_control_cycles>0)!=expected) {
            std::fprintf(stderr,"mode=%d corrected=%d cycles=%lu\n",static_cast<int>(mode),correcting,
                         static_cast<unsigned long>(task.DiagonalWallDiagnostics().pose_control_cycles));
            for (unsigned i=0;i<diagnostics.size();++i) {
                const auto &p=diagnostics.event(i).diagonal_pose;
                std::fprintf(stderr,"event %u: status=%d index=%u used=%u ready=%d e=%f heading=%f\n",i,p.status,p.index,p.used,p.lateral_heading_ready,p.pose.e,p.pose.heading);
            }
        }
        diagnostics.stop();
        CHECK(correcting==expected && (task.DiagonalWallDiagnostics().pose_control_cycles>0)==expected);
        CHECK((world.snapshot().corrections>0)==expected);
        if(!expected) CHECK(std::fabs(world.snapshot().world.x_mm-230)<.001f && world.snapshot().world.y_mm==0);
        world.stop();
        // Diagonal -> out turn -> straight exit/entry -> in turn -> diagonal.
        // Advance yaw only from the real sensing update, including TURN_IRSENS.
        Route next=route;
        next.origin_x+=3;next.origin_y+=3;next.dx=-1;next.dy=-1;
        task.StageDiagonalPoseRoute(next,true);
        task.IrSensMotion_Set(TURN_IRSENS);
        CHECK(!task.DiagonalPoseControlsCorner());
        vehicle.ego.rad_velo.set(PI/100.0f*1000.0f);
        for(unsigned i=0;i<50;++i) task.IrSensorSet(&vehicle,0,461+i);
        task.EnableIrSensStraight();
        if (mode==IrSensTask::WallEkfMode::Control) task.PrimeStraightTurnExit(0,true,.001f,true);
        else task.PrimeStraightWallControl();
        task.ResetStraightWallControl(false,true);
        task.StageDiagonalPoseRoute(next);
        task.IrSensMotion_Set(TURN_IRSENS);
        for(unsigned i=0;i<50;++i) task.IrSensorSet(&vehicle,0,511+i);
        task.EnableIrSensDiagonal();
        task.BeginDiagonalPoseRoute(&vehicle,-6,0);
        CHECK(task.DiagonalWallDiagnostics().pose_handoffs==(diagonal_pose_param.calibrated ? 1U:0U));
        // A coordinate handoff alone never resumes steering.
        CHECK(!task.DiagonalPoseControlsCorner());
    }
}

static void diagonal_pose_handoff()
{
    using namespace DiagonalPose;
    const Parameters model{true,0.7f,{{{20,2},{20,2},{20,2},{20,2}},{{20,2},{20,2},{20,2},{20,2}}}};
    Route route;
    route.count=5;route.origin_x=2;route.origin_y=3;route.dx=1;route.dy=1;
    for(unsigned i=0;i<route.count;++i) route.pillars[i]=i%2;
    Filter initial;
    float travel=-30.0f;
    initial.start(travel,0,route);
    Result before;
    for (unsigned i=0;i<route.count;++i) {
        const unsigned side=i%2;
        const float peak=(i+.5f)*DIAG_SECTION-20-0.7f*(.5f*DIAG_SECTION+(side?4:-4));
        while (travel<peak-.001f) {
            const float step=fminf(1,peak-travel);
            initial.advance(step,0);travel+=step;
        }
        before=initial.observe(side,initial.odometry(),0,0,false,0,model);
        CHECK(before.status==Updated && before.index==i+1);
    }
    Pose pose;
    CHECK(initial.control_pose(pose) && !initial.begin_bridge(route));
    for(int turn:{-1,1,2}) {
        Filter filter=initial;
        Route next=route;
        next.origin_x+=4;next.origin_y+=4;
        if (turn==2) { next.dx=-1;next.dy=-1; }
        else { next.dx=-turn;next.dy=turn; }
        CHECK(filter.begin_bridge(next));
        filter.advance(0,turn*PI*.5f+.03f);
        CHECK(filter.finish_bridge(0,next));
        CHECK(!filter.bridging() && !filter.control_pose(pose));
        Pose nonexistent=filter.odometry();nonexistent.s+=1000;
        const Result rotated=filter.observe(0,nonexistent,0,0,false,0,model);
        CHECK(rotated.status==NoMatch && rotated.used==0);
        const float progress_from_next=before.pose.s-4*DIAG_SECTION;
        CHECK(near(rotated.pose.heading,.03f+before.pose.heading));
        if(turn==2) {
            CHECK(near(rotated.pose.s,-progress_from_next) && near(rotated.pose.e,-before.pose.e));
            CHECK(near(rotated.sigma_s,before.sigma_s) && near(rotated.sigma_e,before.sigma_e));
        } else {
            CHECK(near(rotated.pose.s,turn*before.pose.e) && near(rotated.pose.e,-turn*progress_from_next));
            CHECK(near(rotated.sigma_s,before.sigma_e) && near(rotated.sigma_e,before.sigma_s));
        }
    }
    Route next=route;
    next.origin_x+=4;next.origin_y+=4;
    CHECK(initial.begin_bridge(next));
    for(unsigned i=0;i<1000;++i) initial.advance(0,0);
    CHECK(!initial.active() && !initial.finish_bridge(0,next));
    initial.start(-30,0,next);
    CHECK(!initial.control_pose(pose) && !initial.bridging());
}

static void diagonal_feature_adapter()
{
    using namespace WallPositionCorrection;
    const t_pid_gain gain={1,0,0};
    for(unsigned side:{1U,2U})for(auto mode:{IrSensTask::WallEkfMode::Shadow,IrSensTask::WallEkfMode::Control})
    for(unsigned front_kind:{0U,1U,2U}) {
        const bool front_block=front_kind!=0;
        ShortestEntryHarness m;const t_turn_param_table table={.5f,26,30,40,45,side==1?Turn_R:Turn_L,0};
        m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
        m.pattern=side==1?Turn_in_R45:Turn_in_L45;m.control=DIAGONAL_STATE;m.motion_plan.turn_state.set(Post_Turn);
        m.sensors.WallEkfMode_Set(mode);m.sensors.WallEdgeMode_Set(mode==IrSensTask::WallEkfMode::Control?Mode::Control:Mode::Shadow);
        m.sensors.EnableIrSensDiagonal();m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);
        DiagonalPose::Route route;route.count=1;route.pillars[0]=side-1U;
        m.sensors.StageDiagonalPoseRoute(route);
        m.vehicle->ego.x_point.set(4);m.vehicle->ego.radian.set(.02f);m.handleTurnPost_Diagonal();
        for(auto &a:side_wall_test_adc)a=0;
        if(front_block) {side_wall_test_adc[sensor_fl]=sens_fl_table[2];side_wall_test_adc[sensor_fr]=sens_fr_table[2];}
        if(front_kind==2) {side_wall_test_adc[sensor_fl]=sens_fl_table[0]+1;side_wall_test_adc[sensor_fr]=sens_fr_table[0]+1;}
        auto &log=WallDiagnostics::instance();log.begin(m.sensors.WallEdgeMode_Get());
        const auto generation=m.sensors.wall_edge_generation;
        for(int i=1;i<=116;++i) {
            const float s=i*.5f,range=fminf(80,40+2*std::fabs(s-40));
            side_wall_test_adc[side==1?sensor_sl:sensor_sr]=side_adc_at(side==1?sens_sl_table:sens_sr_table,range);
            m.vehicle->ego.length.set(m.vehicle->ego.length.get()+.5f);
            m.sensors.IrSensorSet(m.vehicle,.5f,m.tick++);
            const float before=m.vehicle->ego.length.get(),x=m.vehicle->ego.x_point.get(),yaw=m.vehicle->ego.radian.get();
            const float omega=m.vehicle->ideal.rad_velo.get(),alpha=m.vehicle->ideal.rad_accel.get();
            m.record();
            CHECK(near(m.vehicle->ego.length.get(),before) && near(m.vehicle->ego.x_point.get(),x) && near(m.vehicle->ego.radian.get(),yaw));
            CHECK(near(m.vehicle->ideal.rad_velo.get(),omega) && near(m.vehicle->ideal.rad_accel.get(),alpha) && m.sensors.wall_correction==False);
            if(m.pattern!=Diagonal) {
                m.handleTurnPost_Diagonal();
                if(m.pattern==Run_Pause)m.resume_diagonal();
            }
        }
        const auto &pillars=PillarDiagnostics::instance();CHECK(pillars.size()==1);
        const auto &e=pillars.event(0);CHECK(e.axis==1 && e.phase==Phase::DiagonalRun && e.position_evaluated);
        CHECK(e.minimum.source_generation==generation && e.minimum.generation==e.confirmation.generation && e.minimum.rebase_mm< -40);
        CHECK(e.diagonal_model.shadow_only && !e.diagonal_model.model_ready && !e.reference_valid && !e.proposal_valid);
        CHECK(e.position.delta_mm==0 && e.position.after_mm==e.confirmation.length_mm);
        CHECK(near(e.minimum.model_left_mm,4) && near(e.confirmation.model_left_mm,4));
        CHECK(near(e.minimum.model_heading_rad,.02f) && near(e.confirmation.model_heading_rad,.02f));
        CHECK(e.diagonal_model.status==(front_block?DiagonalFeatureShadow::QualityRejected:DiagonalFeatureShadow::Uncalibrated));
        CHECK(e.diagonal_pose.status==(diagonal_pose_param.calibrated?DiagonalPose::Updated:DiagonalPose::Uncalibrated));
        CHECK(!e.diagonal_pose.lateral_heading_ready);
        CHECK(front_kind==1 ? (e.flags&PillarDiagnostics::FrontPairClose)!=0 : e.flags==0);
        CHECK(!front_block || (e.diagonal_model.quality_flags&PillarDiagnostics::FrontPairClose));
        // A repeated call from the same sample is not a second wall observation.
        m.sensors.r_wall_corner=True;m.sensors.r_corner_generation=m.sensors.wall_edge_generation;
        const auto n=log.size();m.ApplyWallPositionCorrection(0,Phase::DiagonalRun);m.ApplyWallPositionCorrection(0,Phase::DiagonalRun);
        CHECK(log.size()==n+1 && m.sensors.wall_correction==False);
        m.sensors.IrSensMotion_Set(TURN_IRSENS);
        if(side==2 && mode==IrSensTask::WallEkfMode::Control && !front_block)log.finish_and_print();else log.stop();
    }
    // The real turn handler seeds the logical-diagonal residual before it zeros ego.radian.
    ShortestEntryHarness m;const t_turn_param_table table={.5f,26,30,40,45,Turn_L,0};
    m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
    m.pattern=Turn_in_L45;m.sensors.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);
    m.sensors.WallEdgeMode_Set(Mode::Control);m.vehicle->ego.radian.set(DEG2RAD(45.0f)+.03f);
    m.handleTurnMain();m.handleTurnPost_Diagonal();
    for(auto &a:side_wall_test_adc)a=0;
    auto &log=WallDiagnostics::instance();log.begin(Mode::Control);
    m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);
    m.sensors.r_wall_corner=True;m.sensors.r_corner_generation=m.sensors.wall_edge_generation;
    m.ApplyWallPositionCorrection(0,Phase::DiagonalExit,2);
    CHECK(log.size()==1 && near(log.event(0).diagonal_model.heading_rad,.03f));
    CHECK(m.vehicle->ego.radian.get()==0 && log.event(0).correction.delta_mm==0);
    log.stop();
}



static void pose_frame_straight()
{
    using Mode=IrSensTask::WallEkfMode;
    for(auto mode:{Mode::Legacy,Mode::Shadow,Mode::Control}) {
        IrSensTask_type8 task{};Vehicle vehicle{};
        auto &frames=vehicle.pose;
        const PoseFrames::Pose initial={10,-20,.6f};CHECK(frames.set_start(initial));frames.start();frames.set_phase(STRAIGHT_STATE);
        task.WallEkfMode_Set(mode);task.EnableIrSensStraight();
        vehicle.ideal.velo.set(.5f);vehicle.ego.velo.set(.5f);vehicle.ego.x_point.set(6);
        side_wall_test_adc[sensor_fl]=side_wall_test_adc[sensor_fr]=0;
        side_wall_test_adc[sensor_sl]=side_adc_at(sens_sl_table,45);
        side_wall_test_adc[sensor_sr]=side_adc_at(sens_sr_table,45);
        for(unsigned i=0;i<160;++i) {
            frames.predict_body(.5f,0,0);vehicle.ego.length.set(vehicle.ego.length.get()+.5f);
            task.IrSensorSet(&vehicle,.5f,10000+i);task.SetWallControl_RadVelo(&vehicle,1);
            if(task.straight_wall_output.sync_estimator || task.straight_wall_ekf_used) {
                const auto local=PoseFrames::relative(initial,frames.snapshot().world);
                CHECK(std::fabs(local.y_mm-vehicle.ego.x_point.get()+6)<.001f);
            }
            const auto before=frames.snapshot();task.SetWallControl_RadVelo(&vehicle,1);
            CHECK(std::fabs(frames.snapshot().world.x_mm-before.world.x_mm)<.0001f);
            CHECK(std::fabs(frames.snapshot().world.y_mm-before.world.y_mm)<.0001f);
            if(i==80) {
                // Changing progress/gyro origins within the same passage is not motion.
                const auto at_reset=frames.snapshot();
                vehicle.ego.length.set(-.3f);vehicle.ego.radian.init();task.ResetStraightWallControl(true);
                CHECK(frames.snapshot().world.x_mm==at_reset.world.x_mm && frames.snapshot().world.y_mm==at_reset.world.y_mm);
            }
        }
        CHECK(frames.snapshot().corrections>0 && frames.snapshot().valid);
        if(mode==Mode::Control && task.FeatureEkfEnabled()) {
            const auto before=frames.snapshot();
            const auto result=task.ObserveFeature(&vehicle,vehicle.ego.length.get()+2,0,.000001f,false,WallPositionCorrection::Phase::StraightRun);
            CHECK(result.accepted() && result.delta_mm!=0);
            const auto expected=PoseFrames::compose({before.world.x_mm,before.world.y_mm,before.reference_yaw_rad},{result.delta_mm,0,0});
            CHECK(std::fabs(frames.snapshot().world.x_mm-expected.x_mm)<.0001f);
            CHECK(std::fabs(frames.snapshot().world.y_mm-expected.y_mm)<.0001f);
            const float progress=PoseFrames::relative(initial,frames.snapshot().world).x_mm;
            task.SetWallControl_RadVelo(&vehicle,1);
            CHECK(std::fabs(PoseFrames::relative(initial,frames.snapshot().world).x_mm-progress)<.0001f);
        }
        frames.stop();
    }
}

static void motion_heading_chain()
{
    using EkfMode=IrSensTask::WallEkfMode;
    const t_pid_gain gain={1,0,0};
    const float q=wall_ekf_param.heading_walk_rad_sqrt_s*wall_ekf_param.heading_walk_rad_sqrt_s;
    // The route EKF receives the same marginal without inventing a pillar.
    DiagonalPose::Filter filter;
    DiagonalPose::Route unknown;unknown.count=1;unknown.pillars[0]=DiagonalPose::Route::unknown;
    auto params=diagonal_pose_param;params.calibrated=true;
    filter.start(-30,.03f,unknown,.000004f);
    auto result=filter.observe(0,filter.odometry(),0,0,false,0,params);
    CHECK(result.status==DiagonalPose::NoMatch && result.used==0 && !result.lateral_heading_ready);
    CHECK(std::fabs(result.pose.heading-.03f)<.000001f);
    CHECK(std::fabs(result.sigma_heading-.002f)<.000001f && result.sigma_e==15);
    for(auto mode:{EkfMode::Legacy,EkfMode::Shadow,EkfMode::Control})
    for(float sign:{-1.0f,1.0f}) for(float entry_degree:{45.0f,135.0f})
    for(auto vpattern:{Turn_RV90,Turn_LV90,Long_turn_RV90,Long_turn_LV90}) {
        ShortestEntryHarness m;
        m.sensors.WallEkfMode_Set(mode);m.sensors.WallEdgeMode_Set(WallPositionCorrection::Mode::Control);
        m.sensors.EnableIrSensStraight();m.sensors.PrimeStraightTurnExit(.015f*sign,true,.0001f);
        m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);
        for(auto &adc:side_wall_test_adc)adc=0;
        float expected=.015f*sign,variance=.0001f;
        auto turn=[&](t_run_pattern pattern,float degree,bool from_diagonal,bool to_diagonal) {
            if(pattern==Long_turn_RV90 || pattern==Long_turn_LV90) m.init_long_v90(pattern==Long_turn_LV90);
            const t_turn_param_table table={.5f,26,30,46,degree,degree<0?Turn_R:Turn_L,0};
            m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
            m.pattern=pattern;m.motion_plan.turn_state.set(Prev_Turn);
            m.motion_plan.turn_time_ms.set(100);m.vehicle->ego.length.set(31);
            m.vehicle->ego.radian.set(.7f);m.vehicle->ego.rad_velo.set(.3f*sign);
            // Diagonal sensing precedes the entry handler; straight prediction follows it.
            if(from_diagonal)m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);
            if(mode==EkfMode::Control) {expected+=.0003f*sign;variance+=q*.001f;}
            if(from_diagonal)m.handleTurnPrev_Diagonal();else m.handleTurnPrev_Straight();
            m.handleTurnMain();
            const auto prior=m.sensors.TakeTurnHeadingPrior();
            CHECK(prior.usable()==(mode==EkfMode::Control));
            if(prior.usable()) {
                CHECK(std::fabs(prior.heading_rad-expected)<.00001f);
                CHECK(std::fabs(prior.variance_rad2-variance)<.00000002f);
            }
            CHECK(!m.sensors.TakeTurnHeadingPrior().usable());
            const float gyro=(DEG2RAD(degree)+.01f*sign)/.1f;
            for(unsigned j=1;j<=100;++j) {
                ++m.clock;m.vehicle->ego.rad_velo.set(gyro);
                m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);m.handleTurnMain();
            }
            if(mode==EkfMode::Control) {expected+=.01f*sign;variance+=q*.1f;}
            if(to_diagonal) {
                DiagonalPose::Route route;route.count=4;route.dx=1;route.dy=1;
                m.sensors.StageDiagonalPoseRoute(route);
                m.handleTurnPost_Diagonal();m.SetIdeal_wall_control();
                float heading=0,var=0;
                CHECK(m.sensors.DiagonalHeadingForTurn(heading,var)==(mode==EkfMode::Control));
                if(mode==EkfMode::Control) {
                    CHECK(std::fabs(heading-expected)<.00002f);
                    CHECK(std::fabs(var-variance)<.00000002f);
                    CHECK(std::fabs(m.sensors.diagonal_wall_output.estimate.heading_rad-expected)<.00002f);
                    CHECK(std::fabs(m.sensors.diagonal_wall_output.estimate.variance_heading_rad2-variance)<.00000002f);
                    CHECK(!m.sensors.diagonal_wall_output.estimate.observed);
                    CHECK(!m.sensors.DiagonalPoseControlsCorner()); // No fabricated pillars.
                }
                m.resume_diagonal(); // Same passage despite raw origin resets.
            } else {
                m.post();m.SetIdeal_wall_control();
                CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-(mode==EkfMode::Control?expected:0))<.00002f);
                if(mode==EkfMode::Control) CHECK(std::fabs(m.sensors.StraightHeadingVarianceForTurn()-variance)<.00000002f);
            }
            CHECK(!m.shortest_turn_heading.valid());
            m.vehicle->ego.rad_velo.set(0);
        };
        turn(sign<0?(entry_degree==45?Turn_in_R45:Turn_in_R135):(entry_degree==45?Turn_in_L45:Turn_in_L135),sign*entry_degree,false,true);
        // Continue through a diagonal pause; both elapsed yaw and variance survive.
        m.vehicle->ego.rad_velo.set(.2f*sign);
        for(unsigned i=0;i<12;++i) {m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);m.SetIdeal_wall_control();}
        if(mode==EkfMode::Control) {expected+=.0024f*sign;variance+=q*.012f;}
        const float vdegree=(vpattern==Turn_RV90 || vpattern==Long_turn_RV90)?-90:90;
        turn(vpattern,vdegree,true,true);
        turn(sign<0?Turn_out_R45:Turn_out_L45,45*sign,true,false);
        m.resume();
        CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-(mode==EkfMode::Control?expected:0))<.00002f);
    }
    // An inherited marginal expires without pillar updates, on bad samples,
    // disable, and on mode changes. It cannot seed a later unrelated turn.
    for(unsigned scenario=0;scenario<7;++scenario) {
        IrSensTask_type8 s{};Vehicle v{};
        s.WallEkfMode_Set(EkfMode::Control);s.WallEdgeMode_Set(WallPositionCorrection::Mode::Control);
        s.PrimeDiagonalPoseHeading({.02f,.0001f,true});s.EnableIrSensDiagonal();
        float h=0,p=0;CHECK(s.DiagonalHeadingForTurn(h,p));
        if(scenario==0)for(unsigned i=0;i<220;++i)s.IrSensorSet(&v,.5f,10000+i);
        if(scenario==1) {v.ego.rad_velo.set(NAN);s.IrSensorSet(&v,.5f,10000);}
        if(scenario==2)s.DisableIrSens();
        if(scenario==3)s.WallEkfMode_Set(EkfMode::Shadow);
        if(scenario==4)s.WallEdgeMode_Set(WallPositionCorrection::Mode::Shadow);
        if(scenario==5) {v.ideal.velo.set(.20f);s.SetWallControl_RadVelo(&v,1);}
        if(scenario==6)s.IrSensorSet(&v,6.1f,10000);
        CHECK(!s.DiagonalHeadingForTurn(h,p));
        CHECK(!s.TakeTurnHeadingPrior().usable());
    }
}

static void turn_heading_posterior()
{
    const t_pid_gain gain={1,0,0};
    for(auto mode:{TurnPose::Mode::Legacy,TurnPose::Mode::Shadow,TurnPose::Mode::Control})
    for(bool last_invalid:{false,true}) {
        ShortestEntryHarness m;
        m.sensors.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);m.sensors.EnableIrSensStraight();
        m.sensors.PrimeStraightTurnExit(.02f,true,.000004f);
        m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);
        const t_turn_param_table table={.5f,26,30,46,-90,Turn_R,0};
        m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
        m.pattern=Long_turnR90;m.motion_plan.turn_state.set(Prev_Turn);m.motion_plan.turn_time_ms.set(160);
        m.vehicle->ego.length.set(31);m.vehicle->ego.radian.set(.7f);
        m.handleTurnPrev_Straight();m.handleTurnMain();
        TurnPose::Tracker turn_pose_;turn_pose_.set_mode(mode);
        TurnPose::Input pose_input;
        pose_input.dt_s=.001f;pose_input.timing_valid=true;pose_input.quiet_command=true;
        pose_input.gyro_rad_s=.02f;pose_input.lateral_accel_m_s2=.1f;
        for(unsigned i=0;i<120;++i)turn_pose_.update(pose_input);
        pose_input.turning=true;pose_input.quiet_command=false;pose_input.pattern=Long_turnR90;
        pose_input.speed_m_s=pose_input.target_speed_m_s=.5f;
        const float gyro=DEG2RAD(-90)/.16f;
        auto &ir_sens=m.sensors;
        for(unsigned j=1;j<=160;++j) {
            pose_input.heading_before_rad=.7f+gyro*.001f*(j-1);
            pose_input.gyro_rad_s=gyro;pose_input.lateral_accel_m_s2=.5f*(gyro-.02f)+.1f;
            pose_input.legacy_left_mm=.5f*sinf(.7f+gyro*.001f*j);
            pose_input.legacy_forward_mm=.5f*cosf(.7f+gyro*.001f*j);
            pose_input.timing_valid=!(last_invalid && j==160);
#include "heading_prior_input.inc"
            turn_pose_.update(pose_input);
#include "heading_prior_output.inc"
            ++m.clock;m.vehicle->ego.rad_velo.set(gyro);m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);
            m.handleTurnMain();
        }
        const auto snapshot=turn_pose_.snapshot();
#if defined(MOUSE_A)
        const bool applied=mode==TurnPose::Mode::Control && !last_invalid;
        CHECK(snapshot.applied==applied);
#else
        const bool applied=false;
#endif
        const float expected=applied?snapshot.heading_rad-DEG2RAD(-90):.02f;
        CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-expected)<.00002f);
        if(applied) CHECK(std::fabs(m.sensors.StraightHeadingVarianceForTurn()-snapshot.sigma_heading_rad*snapshot.sigma_heading_rad)<.00000002f);
        CHECK(!m.sensors.TurnHeadingPosterior().usable());
    }
}

static void shortest_exit_heading()
{
    const t_pid_gain gain={1,0,0};
    for(auto pattern:{Long_turnR90,Long_turnL90,Long_turnR180,Long_turnL180})
    for(auto mode:{IrSensTask::WallEkfMode::Legacy,IrSensTask::WallEkfMode::Shadow,IrSensTask::WallEkfMode::Control})
    for(unsigned scenario=0;scenario<4;++scenario) {
        const bool right=pattern==Long_turnR90 || pattern==Long_turnR180;
        const float sign=right?-1.0f:1.0f;
        const float degree=(pattern==Long_turnR180 || pattern==Long_turnL180?180.0f:90.0f)*sign;
        const float incoming=.02f*sign, prior=.01f*.01f;
        // 0: normal; 1: invalid gyro mid-turn; 2: excessive residual; 3: no incoming prior.
        const float yaw_error=(scenario==2?.4f:.01f)*sign;
        ShortestEntryHarness m;
        const t_turn_param_table table={.5f,26,30,46,degree,right?Turn_R:Turn_L,2.0f*sign};
        m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
        m.pattern=pattern;m.motion_plan.turn_state.set(Prev_Turn);
        m.motion_plan.turn_time_ms.set(100);m.motion_plan.rad_max_velo.set(20*sign);
        m.sensors.WallEkfMode_Set(mode);m.sensors.EnableIrSensStraight();
        m.sensors.PrimeSearchTurnExit(incoming,true,prior);
        m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);
        for(auto &adc:side_wall_test_adc)adc=0;
        for(int j=0;j<12;++j) {
            m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);
            m.SetIdeal_wall_control();
        }
        if(scenario==3)m.sensors.ResetStraightWallControl();
        m.vehicle->ego.length.set(31);
        // Deliberately unrelated odometry yaw: the passage heading has its own origin.
        m.vehicle->ego.radian.set(.7f);
        m.vehicle->ego.rad_velo.set(.3f*sign);
        m.handleTurnPrev_Straight();m.handleTurnMain(); // Both execute in this IRQ.
        const float gyro=(DEG2RAD(degree)+yaw_error-.0003f*sign)/.1f;
        for(int j=1;j<=100;++j) {
            ++m.clock;
            m.vehicle->ego.rad_velo.set(scenario==1 && j==50?NAN:gyro);
            // Rotating samples must not seed the next passage's ten-point average.
            side_wall_test_adc[sensor_sl]=1000;side_wall_test_adc[sensor_sr]=1000;
            m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);
            m.handleTurnMain();
        }
        CHECK(m.motion_plan.turn_state.get()==Post_Turn);
        m.post();m.SetIdeal_wall_control();
        CHECK(!m.shortest_turn_heading.valid()); // No prior may leak into another turn.
        CHECK(m.vehicle->ego.radian.get()==0);
        const bool retained=mode==IrSensTask::WallEkfMode::Control && scenario==0;
        const float expected=retained?incoming+yaw_error:0.0f;
        CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-expected)<.00002f);
        if(mode==IrSensTask::WallEkfMode::Control) {
            CHECK(!m.sensors.straight_wall_ekf_active);
            CHECK(!m.sensors.straight_wall_ekf_used);
            CHECK(m.vehicle->ideal.rad_velo.get()==0);
        }
        m.vehicle->ego.rad_velo.set(0);
        for(auto &adc:side_wall_test_adc)adc=0;
        for(int j=1;j<=12;++j) {
            m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);m.SetIdeal_wall_control();
            CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-expected)<.00002f);
            if(mode==IrSensTask::WallEkfMode::Control && j<10)
                CHECK(!m.sensors.straight_wall_ekf_active);
        }
        if(mode==IrSensTask::WallEkfMode::Control) {
            const auto &e=m.sensors.straight_wall_estimate;
            CHECK(e.initialized && e.wall_updates==0 && !m.sensors.straight_wall_ekf_used);
            CHECK(std::fabs(e.heading_rad-expected)<.00002f);
            const float expected_prior=retained?prior:
                wall_ekf_param.initial_heading_std_rad*wall_ekf_param.initial_heading_std_rad;
            CHECK(e.variance_heading_rad2>=expected_prior);
            if(retained)CHECK(e.variance_heading_rad2<prior*1.1f);
            // Following straight initialization retains the same passage pose.
            m.resume();CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-expected)<.00002f);
            if(retained) {
                bool used=false, opposed=false;
                side_wall_test_adc[sensor_sl]=side_adc_at(sens_sl_table,45);
                side_wall_test_adc[sensor_sr]=side_adc_at(sens_sr_table,45);
                for(int j=0;j<100;++j) {
                    m.vehicle->ego.length.set(m.vehicle->ego.length.get()+.5f);
                    m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);m.SetIdeal_wall_control();
                    used=used || m.sensors.straight_wall_ekf_used;
                    opposed=opposed || (m.sensors.straight_wall_ekf_used
                        && sign*m.vehicle->ideal.rad_velo.get()<-.001f);
                }
                CHECK(used && opposed); // Accepted wall pose actually reaches yaw control.
            }
        }
    }
}

static void shortest_diagonal_exit_heading()
{
    using namespace WallPositionCorrection;
    const t_pid_gain gain={1,0,0};
    for(auto pattern:{Turn_out_R45,Turn_out_L45,Turn_out_R135,Turn_out_L135})
    for(auto mode:{IrSensTask::WallEkfMode::Legacy,IrSensTask::WallEkfMode::Shadow,IrSensTask::WallEkfMode::Control})
    for(unsigned scenario=0;scenario<6;++scenario) {
        const bool right=pattern==Turn_out_R45 || pattern==Turn_out_R135;
        const float sign=right?-1.0f:1.0f;
        const float degree=(pattern==Turn_out_R135 || pattern==Turn_out_L135?135.0f:45.0f)*sign;
        const float yaw_error=(scenario==2?.4f:.01f)*sign;
        ShortestEntryHarness m;
        const t_turn_param_table table={.5f,26,30,46,degree,right?Turn_R:Turn_L,2.0f*sign};
        m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
        m.sensors.WallEkfMode_Set(mode);
        m.sensors.WallEdgeMode_Set(mode==IrSensTask::WallEkfMode::Legacy?Mode::Legacy:Mode::Control);
        m.sensors.EnableIrSensDiagonal();m.control=DIAGONAL_STATE;m.pattern=Diagonal;
        m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);
        DiagonalPose::Route route;route.count=5;route.origin_x=2;route.origin_y=3;route.dx=1;route.dy=1;
        for(unsigned i=0;i<route.count;++i)route.pillars[i]=2U+(i%2);
        m.sensors.StageDiagonalPoseRoute(route);m.sensors.BeginDiagonalPoseRoute(m.vehicle,-33,0);
        FeatureContext frame;frame.valid=true;frame.phase=Phase::DiagonalRun;
        for(auto &adc:side_wall_test_adc)adc=0;
        // Machine-specific ADC conversion, detector, association and pose update.
        for(unsigned step=1;step<=460;++step) {
            const float s=-33+step*.5f;float range[2]={80,80};
            for(unsigned i=0;i<route.count;++i) {
                const unsigned side=i%2;
                const auto &model=diagonal_pose_param.pillar[side][route.pillars[i]>>1];
                const float center=(i+.5f)*DIAG_SECTION-(diagonal_pose_param.calibrated?model.offset_mm:20)
                    -(0.5f*DIAG_SECTION+(side?4:-4))*.700208f;
                range[side]=fminf(range[side],40+2*fabsf(s-center));
            }
            side_wall_test_adc[sensor_sl]=side_adc_at(sens_sl_table,range[0]);
            side_wall_test_adc[sensor_sr]=side_adc_at(sens_sr_table,range[1]);
            m.vehicle->ego.length.set(s);m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);
            m.sensors.RecordPillarCandidates(m.vehicle,Diagonal,DIAGONAL_STATE,0,false,frame);
            m.SetIdeal_wall_control();
        }
        if(scenario==3) {m.sensors.ResetStraightWallControl();m.sensors.EnableIrSensDiagonal();}
        if(scenario==5)m.sensors.WallEdgeMode_Set(Mode::Shadow);
        for(auto &adc:side_wall_test_adc)adc=0;
        if(scenario==4)for(unsigned j=0;j<225;++j)m.sensors.IrSensorSet(m.vehicle,0,++m.tick);
        // This nonzero entry sample is already in the diagonal posterior.
        m.vehicle->ego.rad_velo.set(.3f*sign);m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);
        float incoming=0, prior=0;
        const bool ready=m.sensors.DiagonalHeadingForTurn(incoming,prior);
        const bool eligible=diagonal_pose_param.calibrated && mode==IrSensTask::WallEkfMode::Control
            && scenario!=3 && scenario!=4 && scenario!=5;
        CHECK(ready==eligible);
        m.pattern=pattern;m.motion_plan.turn_state.set(Prev_Turn);
        m.motion_plan.turn_time_ms.set(100);m.motion_plan.rad_max_velo.set(20*sign);
        m.vehicle->ego.length.set(31);m.vehicle->ego.radian.set(.7f);
        m.handleTurnPrev_Diagonal();m.handleTurnMain();
        const float gyro=(DEG2RAD(degree)+yaw_error)/.1f;
        for(unsigned j=1;j<=100;++j) {
            ++m.clock;m.vehicle->ego.rad_velo.set(scenario==1 && j==50?NAN:gyro);
            side_wall_test_adc[sensor_sl]=1000;side_wall_test_adc[sensor_sr]=1000;
            m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);m.handleTurnMain();
        }
        m.post();m.SetIdeal_wall_control();
        const bool retained=eligible && scenario==0;
        const float expected=retained?incoming+yaw_error:0;
        CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-expected)<.00002f);
        if(retained) {
            const float noise=wall_ekf_param.heading_walk_rad_sqrt_s*wall_ekf_param.heading_walk_rad_sqrt_s*.1f;
            CHECK(std::fabs(m.sensors.StraightHeadingVarianceForTurn()-prior-noise)<.00000002f);
        }
        CHECK(!m.shortest_turn_heading.valid());
        m.vehicle->ego.rad_velo.set(0);for(auto &adc:side_wall_test_adc)adc=0;
        for(unsigned j=1;j<=12;++j) {
            m.sensors.IrSensorSet(m.vehicle,.5f,++m.tick);m.SetIdeal_wall_control();
            CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-expected)<.00002f);
            if(mode==IrSensTask::WallEkfMode::Control && j<10)CHECK(!m.sensors.straight_wall_ekf_active);
        }
        if(mode==IrSensTask::WallEkfMode::Control) {
            CHECK(m.sensors.straight_wall_ekf_active && !m.sensors.straight_wall_ekf_used);
            CHECK(std::fabs(m.sensors.straight_wall_estimate.heading_rad-expected)<.00002f);
            m.resume();CHECK(std::fabs(m.sensors.StraightHeadingForTurn()-expected)<.00002f);
        }
    }
}

static void shortest_feature_exit()
{
    using namespace WallPositionCorrection;
    const t_pid_gain gain={1,0,0};
    for(auto pattern:{Long_turnR90,Long_turnL90,Long_turnR180,Long_turnL180,Turn_out_R45,Turn_out_L45,Turn_out_R135,Turn_out_L135})
    for(auto mode:{IrSensTask::WallEkfMode::Control,IrSensTask::WallEkfMode::Shadow})
    for(float fix:{-3.0f,3.0f}) for(unsigned side:{1U,2U}) {
        ShortestEntryHarness m;
        const t_turn_param_table table={.5f,26,30,46,90,Turn_L,0};
        m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
        m.pattern=pattern;m.motion_plan.turn_state.set(Post_Turn);m.motion_plan.fix_post_run.set(fix);
        m.sensors.WallEkfMode_Set(mode);
        if(!wall_feature_ekf_param.enabled)m.sensors.WallEdgeMode_Set(mode==IrSensTask::WallEkfMode::Control?Mode::Control:Mode::Shadow);
        m.sensors.EnableIrSensStraight();m.sensors.PrimeSearchTurnExit(.02f,true);
        m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);
        auto &log=WallDiagnostics::instance();log.begin(m.sensors.WallEdgeMode_Get());
        for(auto &adc:side_wall_test_adc)adc=0;
        m.post();const float end=46+fix;
        const float node_reference=wall_feature_ekf_param.enabled ? wall_edge_trial_search_reference_mm-45.0f : 7.0f;
        CHECK(near(m.sensors.StraightRunFeatureContext().wall_reference_mm,end+node_reference));
        const auto generation=m.sensors.wall_edge_generation;
        unsigned pause_cycles=0;
        auto sample=[&](int16_t adc){
            side_wall_test_adc[side==1?sensor_sl:sensor_sr]=adc;
            m.vehicle->ego.length.set(m.vehicle->ego.length.get()+.5f);
            m.sensors.IrSensorSet(m.vehicle,.5f,m.tick++);m.record();
            if(m.pattern==Run_Pause) {
                m.pause();
                // B keeps its legacy pause behavior; A's feature-control path predicts in pause.
                if(!m.sensors.FeatureEkfEnabled())m.SetIdeal_wall_control();
                if(++pause_cycles==3)m.resume();
            } else {
                if(m.pattern!=Straight)m.post();else m.run_features();
                m.SetIdeal_wall_control();
            }
        };
        for(int j=0;j<static_cast<int>((end-10)*2);++j)sample(0);
        CHECK(near(m.vehicle->ego.length.get(),end-10));
        for(int adc:{250,400,650,800,800,650,400,250,120})for(int j=0;j<4;++j)sample(adc);
        const auto &pillars=PillarDiagnostics::instance();CHECK(pillars.size()==1);
        const auto &e=pillars.event(0);
        CHECK(m.pattern==Straight && e.phase==Phase::StraightRun && e.flags==0);
        CHECK(near(e.minimum.rebase_mm,-end) && e.minimum.source_generation==generation);
        CHECK(near(m.sensors.StraightRunFeatureContext().wall_reference_mm,node_reference));
        if(wall_feature_ekf_param.enabled) {
            CHECK(e.position_evaluated && e.position.ekf && near(e.reference_mm,node_reference-10));
            CHECK(std::fabs(e.position.delta_mm)<=4 && std::fabs(e.position.proposed_delta_mm)<=4);
            CHECK(mode==IrSensTask::WallEkfMode::Control?e.position.accepted():e.position.reason==Reason::Shadow);
            // A wall and the opposite pillar at this same boundary cannot be injected again.
            m.vehicle->ego.length.set(9);m.sensors.r_wall_corner=True;m.sensors.r_corner_delay_mm=1;
            m.sensors.r_corner_generation=m.sensors.wall_edge_generation;m.run_features();m.SetIdeal_wall_control();
            CHECK(log.size()==1 && log.event(0).correction.reason==Reason::NotSelected && near(m.vehicle->ego.length.get(),9));
            // The following 90 mm boundary remains eligible even while the legacy latch is true.
            m.vehicle->ego.length.set(100);m.run_features();m.SetIdeal_wall_control();
            CHECK(log.size()==2 && log.event(1).correction.ekf && near(log.event(1).correction.target_mm,91+node_reference));
            CHECK(mode==IrSensTask::WallEkfMode::Control?m.vehicle->ego.length.get()<100:near(m.vehicle->ego.length.get(),100));
            // A known origin shift retains consumption across a new motion generation.
            m.motion_plan.end_length.set(90);m.rebase();m.resume();
            m.vehicle->ego.length.set(10);m.sensors.r_wall_corner=True;m.sensors.r_corner_delay_mm=1;
            m.sensors.r_corner_generation=m.sensors.wall_edge_generation;m.run_features();m.SetIdeal_wall_control();
            CHECK(log.size()==3 && log.event(2).correction.reason==Reason::NotSelected && near(m.vehicle->ego.length.get(),10));
        } else CHECK(!e.reference_valid && !e.position_evaluated);
        m.sensors.IrSensMotion_Set(TURN_IRSENS);CHECK(!m.sensors.StraightRunFeatureContext().valid);
        if(pattern==Long_turnR90 && mode==IrSensTask::WallEkfMode::Control && fix==-3 && side==1)log.finish_and_print();
        else log.stop();
    }
    // Reference selection is based on the old feature, even after 50 mm confirmation travel.
    FeatureContext f;f.valid=true;f.phase=Phase::StraightRun;f.wall_reference_mm=7;f.period_mm=90;
    CHECK(near(feature_reference(f,-1,true),pillar_reference(f)));
    CHECK(near(feature_reference(f,90,true),pillar_reference(f)+90));
    CHECK(evaluate(0,8.01f,0,Phase::StraightRun).reason==Reason::OutsideWindow);
    CHECK(evaluate(0,1,wall_edge_param.maximum_pending_mm+.1f,Phase::StraightRun).reason==Reason::Invalid);
}

static void shortest_feature_entry()
{
    using namespace WallPositionCorrection;
    const t_pid_gain gain={1,0,0};
    for(auto pattern:{Long_turnR90,Long_turnL90,Long_turnR180,Long_turnL180,Turn_in_R45,Turn_in_L45,Turn_in_R135,Turn_in_L135})
    for(auto mode:{IrSensTask::WallEkfMode::Control,IrSensTask::WallEkfMode::Shadow})
    for(bool preserve:{false,true}) for(bool expected:{false,true}) {
        const bool right=pattern==Long_turnR90 || pattern==Long_turnR180 || pattern==Turn_in_R45 || pattern==Turn_in_R135;
        const unsigned side=(right==expected)?2U:1U;
        ShortestEntryHarness m;
        const t_turn_param_table table={.5f,26,30,20,right?-90.0f:90.0f,right?Turn_R:Turn_L,0};
        m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
        m.motion_plan.end_length.set(90);m.motion_plan.turn_time_ms.set(100);m.motion_plan.rad_max_velo.set(10);
        m.sensors.WallEkfMode_Set(mode);
        if(!wall_feature_ekf_param.enabled)m.sensors.WallEdgeMode_Set(mode==IrSensTask::WallEkfMode::Control?Mode::Control:Mode::Shadow);
        m.sensors.EnableIrSensStraight();m.sensors.PrimeSearchTurnExit(.025f,true);
        m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);m.vehicle->ego.length.set(50);
        for(auto &adc:side_wall_test_adc)adc=0;
        auto &log=WallDiagnostics::instance();log.begin(m.sensors.WallEdgeMode_Get());
        auto sample=[&](int16_t adc,bool entry){
            side_wall_test_adc[side==1?sensor_sl:sensor_sr]=adc;
            m.vehicle->ego.length.set(m.vehicle->ego.length.get()+.5f);
            m.sensors.IrSensorSet(m.vehicle,.5f,m.tick++);m.record();
            if(entry)m.update();else m.sensors.SetWallControl_RadVelo(m.vehicle,1);
        };
        for(int i=0;i<60;++i)sample(0,false);
        for(int adc:{250,400,650,800,800})for(int j=0;j<4;++j)sample(adc,false);
        CHECK(near(m.vehicle->ego.length.get(),90));
        const float heading=m.sensors.StraightHeadingForTurn();
        const auto generation=m.sensors.wall_edge_generation;
        m.rebase(); // Actual SetIdeal_straight completion fragment, including origin shift.
        CHECK(near(m.vehicle->ego.length.get(),0));
        m.sensors.ResetStraightWallControl(preserve);m.sensors.EnableIrSensStraight();m.sensors.Division_Wall_Correction_Reset(preserve);
        if(preserve)CHECK(near(m.sensors.StraightHeadingForTurn(),heading));
        m.pattern=pattern;m.motion_plan.turn_state.set(Prev_Turn);
        for(int adc:{650,400,250,120})for(int j=0;j<4;++j)sample(adc,true);
        const auto &pillars=PillarDiagnostics::instance();CHECK(pillars.size()==1);
        const auto &e=pillars.event(0);CHECK(e.phase==Phase::StraightEntry && e.minimum.rebase_mm==-90);
        if(preserve) {
            CHECK(e.flags==0 && e.minimum.generation==e.confirmation.generation && e.minimum.source_generation==generation);
            if(wall_feature_ekf_param.enabled) {
                CHECK(e.position_evaluated && near(e.reference_mm,wall_feature_ekf_param.pillar_reference_mm-45));
                if(expected) {
                    CHECK(e.feature_mm<0 && e.position.reason==Reason::Invalid && !e.position.ekf);
                    CHECK(!e.proposal_valid && e.position.proposed_delta_mm==0 && e.position.delta_mm==0);
                    CHECK(m.sensors.wall_correction==False);
                } else {
                    CHECK(e.position.reason==Reason::WrongSide && e.position.delta_mm==0 && m.sensors.wall_correction==False);
                }
                // A rejected preceding-straight pillar on either side must not consume the boundary.
                {
                    m.vehicle->ego.length.set(9.5f);
                    m.sensors.r_wall_corner=right?True:False;m.sensors.l_wall_corner=right?False:True;
                    m.sensors.r_corner_generation=m.sensors.l_corner_generation=m.sensors.wall_edge_generation;
                    m.sensors.r_corner_delay_mm=m.sensors.l_corner_delay_mm=.5f;
                    m.update();
                    CHECK(log.size()==1 && log.event(0).phase==static_cast<unsigned>(Phase::StraightEntry));
                    CHECK(log.event(0).correction.ekf && m.sensors.wall_correction==True);
                    CHECK(mode==IrSensTask::WallEkfMode::Control ? m.vehicle->ego.length.get()<9.5f : near(m.vehicle->ego.length.get(),9.5f));
                }
            } else CHECK(!e.reference_valid && !e.position_evaluated);
        } else CHECK((e.flags&PillarDiagnostics::FrameChanged) && !e.position_evaluated);
        log.stop();
    }
    // Actual shortest main handler must discard any candidate queued at the turn deadline.
    ShortestEntryHarness m;const t_turn_param_table table={.5f,26,10,20,-90,Turn_R,0};
    m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
    m.pattern=Turn_in_R45;m.motion_plan.turn_state.set(Prev_Turn);m.motion_plan.turn_time_ms.set(100);
    m.sensors.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);m.sensors.EnableIrSensStraight();
    m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);m.vehicle->ego.length.set(11);
    m.sensors.r_wall_corner=True;m.sensors.r_corner_generation=m.sensors.wall_edge_generation;
    m.sensors.QueueWallFeature(m.vehicle,7,Turn_in_R45,Phase::StraightEntry,2);
    m.update();CHECK(m.control==SLATURN_STATE && near(m.vehicle->ego.length.get(),11));
}

static void feature_coordinate_review()
{
    using namespace WallPositionCorrection;
    // Recorded in_L45: previously +3.829mm at 000616 and -2.01mm at 001507.
    for(auto sample: {std::pair<float,float>{2.068f,15.226f}, {4.0f,5.48f}}) {
        const auto old=evaluate(sample.first,-5.5f,sample.second,Phase::StraightEntry,50);
        CHECK(old.accepted());
        const auto guarded=evaluate(sample.first,-5.5f,sample.second,Phase::StraightEntry,50,true);
        CHECK(guarded.reason==Reason::Invalid && !guarded.accepted());
        CHECK(guarded.delta_mm==0 && guarded.proposed_delta_mm==0 && guarded.after_mm==sample.first);
    }
    CHECK(evaluate(2,-5.5f,2,Phase::StraightEntry,50,true).accepted()); // feature exactly at origin
    CHECK(evaluate(2,-5.5f,2.001f,Phase::StraightEntry,50,true).reason==Reason::Invalid);
    CHECK(evaluate(2,-5.5f,2.001f,Phase::StraightEntry,50).accepted()); // wall crossing unaffected
    CHECK(evaluate(2,-5.5f,2.001f,Phase::SearchStraight,50,true).accepted());
    // Preserve the approach history, but actually pass the peak AFTER origin.
    // This observation must still reach the real EKF/control adapter.
    if(wall_feature_ekf_param.enabled) {
        const t_pid_gain gain={1,0,0};ShortestEntryHarness m;
        const t_turn_param_table table={.5f,26,30,20,-45,Turn_R,0};
        m.turn_motion_param={&table,&gain,&gain};m.straight_motion_param={nullptr,&gain,&gain};
        m.motion_plan.end_length.set(90);m.motion_plan.turn_time_ms.set(100);m.motion_plan.rad_max_velo.set(10);
        m.sensors.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);m.sensors.EnableIrSensStraight();
        m.vehicle->ideal.velo.set(.5f);m.vehicle->ego.velo.set(.5f);m.vehicle->ego.length.set(50);
        for(auto &a:side_wall_test_adc)a=0;
        auto &log=WallDiagnostics::instance();log.begin(Mode::Control);
        auto sample=[&](int adc,bool entry) {
            side_wall_test_adc[sensor_sr]=adc;m.vehicle->ego.length.set(m.vehicle->ego.length.get()+.5f);
            m.sensors.IrSensorSet(m.vehicle,.5f,m.tick++);m.record();
            if(entry)m.update();else m.sensors.SetWallControl_RadVelo(m.vehicle,1);
        };
        for(int i=0;i<60;++i)sample(0,false);
        for(int adc:{250,400,650})for(int i=0;i<4;++i)sample(adc,false);
        for(int i=0;i<8;++i)sample(650,false);
        m.rebase();m.sensors.ResetStraightWallControl(true);m.sensors.EnableIrSensStraight();
        m.sensors.Division_Wall_Correction_Reset(true);m.pattern=Turn_in_R45;m.motion_plan.turn_state.set(Prev_Turn);
        for(int adc:{800,650,400,250,120})for(int i=0;i<4;++i)sample(adc,true);
        const auto &pillars=PillarDiagnostics::instance();CHECK(pillars.size()==1);
        const auto &e=pillars.event(0);
        CHECK(e.feature_mm>=0 && e.flags==0 && e.position.ekf && e.position.accepted());
        CHECK(e.proposal_valid && e.position.delta_mm<0 && m.sensors.wall_correction==True);
        log.stop();
    }
    // The proposed terminal anchor has the same phase for real 90/180mm runs.
    // It cannot make these measured crossings/minima pass without changing the model.
    for(float end: {90.0f,180.0f}) for(float feature: {5.2f,21.7f,64.0f,72.2f,154.4f}) {
        for(bool pillar:{false,true}) {
            FeatureContext f;f.valid=true;f.phase=Phase::StraightRun;f.wall_reference_mm=4.5f;f.period_mm=90;
            const float ref=feature_reference(f,feature,pillar);
            const float terminal=end+4.5f+(pillar ? wall_feature_ekf_param.pillar_reference_mm-wall_edge_trial_search_reference_mm : 0.0f);
            const float terminal_ref=terminal-90*std::round((terminal-feature)/90);
            CHECK(near(ref,terminal_ref));
        }
    }
    // Existing diagonal calibration is local to each phase: an exit-origin change
    // cancels for the run model. A common rotation-end model would need a new
    // reference convention/calibration, not just another end_length rebase.
    // Synthetic calibration only; shipped A/B models remain uncalibrated.
    using S=DiagonalFeatureShadow;
    for(float exit: {25.0f,38.0f}) {
        auto p=diagonal_feature_shadow_param;
        p.reference[1][1][0]={true,101.5f,63.63961f,0,0};
        S s;s.begin_exit(exit);s.advance(.5f,0,p);s.rebase(-exit);
        S::Input in;in.before_mm=101.5f+2;in.delay_mm=2;
        const auto d=s.observe(in,p);
        CHECK(d.model.status==S::Ready && near(d.position.innovation_mm,0));
        CHECK(near(d.model.nominal_reference_mm,101.5f) && d.position.delta_mm==0);
        CHECK(!S::apply_enabled());
    }
}

static void feature_ekf_math()
{
    StraightWallEkf f; f.start(0.0f,.08f);
    for (int i=0;i<150;++i) CHECK(f.predict(.0005f,0.0f,.001f));
    CHECK(f.estimate().longitudinal_error_m<0 && f.estimate().covariance_heading_longitudinal_m_rad<0);
    CHECK(f.observe_longitudinal(.004f,.000016f));
    CHECK(f.estimate().longitudinal_gain>0 && f.estimate().longitudinal_gain<=.5f);
    const float step=f.estimate().longitudinal_error_m; f.inject_longitudinal(step);
    CHECK(near(f.estimate().longitudinal_error_m,0.0f));
    auto prior=f.estimate();
    CHECK(!f.observe_longitudinal(.3f,.000016f));
    CHECK(f.estimate().longitudinal_updates==prior.longitudinal_updates);
    CHECK(!f.observe_longitudinal(0.0f,0.0f));
    CHECK(!f.observe_longitudinal(std::numeric_limits<float>::quiet_NaN(),.000016f));
    for (int i=0;i<1000;++i) {
        CHECK(f.predict(.0004f,i%2?.00001f:-.00001f,.001f));
        if (i%30==0) f.observe_longitudinal(.001f,.000016f);
        if (i%10==0) f.observe_walls(.044f,.046f,3,true);
        const auto &e=f.estimate();
        const float a=e.variance_lateral_m2,b=e.variance_heading_rad2,c=e.variance_longitudinal_m2;
        const float d=e.covariance_m_rad,x=e.covariance_lateral_longitudinal_m2,y=e.covariance_heading_longitudinal_m_rad;
        CHECK(a>=0 && b>=0 && c>=0 && a*b-d*d>=-1e-15f && a*c-x*x>=-1e-15f && b*c-y*y>=-1e-15f);
        CHECK(a*b*c+2*d*x*y-a*y*y-b*x*x-c*d*d>=-1e-20f);
    }
    prior=f.estimate(); f.reset_longitudinal();
    CHECK(f.estimate().lateral_m==prior.lateral_m && f.estimate().heading_rad==prior.heading_rad);
    CHECK(f.estimate().variance_lateral_m2==prior.variance_lateral_m2 && f.estimate().variance_heading_rad2==prior.variance_heading_rad2);
    CHECK(f.estimate().covariance_m_rad==prior.covariance_m_rad && f.estimate().longitudinal_error_m==0);
}

static void machine_pillar_width()
{
    // A:26mm, B:36mmの近い独立柱。Bの実測再生で起きたWall誤分類を防ぐ。
    const float width=straight_feature_history_param.maximum_excursion_mm>=40.0f?36.0f:26.0f;
    for (unsigned side=0;side<2;++side) {
        IrSensTask_type8 task{};Vehicle v{};task.WallEdgeMode_Set(WallPositionCorrection::Mode::Shadow);task.EnableIrSensStraight();
        v.ideal.velo.set(.37f);for(auto &adc:side_wall_test_adc)adc=0;
        auto &log=PillarDiagnostics::instance();log.begin();unsigned walls=0;int tick=0;
        auto sample=[&](int16_t adc) {
            side_wall_test_adc[side==0?sensor_sl:sensor_sr]=adc;v.ego.length.set(v.ego.length.get()+.5f);
            task.IrSensorSet(&v,.5f,5000+tick++);
            if(task.l_wall_corner==True || task.r_wall_corner==True)++walls;
            task.RecordPillarCandidates(&v,Search_st_section,STRAIGHT_STATE,0,true);
        };
        for(int i=0;i<60;++i)sample(0);
        const int16_t peak=side==0?sens_sl_table[3]:sens_sr_table[3];
        sample(peak/3);sample(peak*2/3); // 実波形の接近を含め、単発スパイクにしない。
        for(int i=0;i<static_cast<int>(width/.5f);++i)sample(peak);
        for(int i=0;i<30;++i)sample(0);
        CHECK(walls==0 && log.size()==1 && log.event(0).flags==0 && log.event(0).side==side+1);
        log.stop();
    }
}

static void feature_coordinate_correction()
{
    using namespace WallPositionCorrection;
    // 167916/624156 msの再現: 柱の極大から離反確認までに逆側の縦補正が入る。
    // 未知のジャンプは引き続き棄却し、既知のEKF注入だけ座標を揃える。
    for (unsigned side:{1U,2U}) for (float initial:{13.5f,23.5f})
    for (int scenario:{0,1,2,3}) {
        IrSensTask_type8 task{};Vehicle v{};
        task.WallEkfMode_Set(scenario==2?IrSensTask::WallEkfMode::Shadow:IrSensTask::WallEkfMode::Control);
        if(!wall_feature_ekf_param.enabled)task.WallEdgeMode_Set(scenario==2?Mode::Shadow:Mode::Control);
        task.EnableIrSensStraight();v.ideal.velo.set(.37f);v.ego.velo.set(.37f);v.ego.length.set(initial);
        auto &pillars=PillarDiagnostics::instance();auto &walls=WallDiagnostics::instance();walls.begin(task.WallEdgeMode_Get());
        for(auto &adc:side_wall_test_adc)adc=0;
        unsigned tick=0;bool corrected=false;float step=0;
        auto sample=[&](int16_t adc,bool falling=false) {
            side_wall_test_adc[side==1?sensor_sl:sensor_sr]=adc;
            v.ego.length.set(v.ego.length.get()+.5f);
            task.IrSensorSet(&v,.5f,5000+tick++);
            if (falling && !corrected && scenario!=3) {
                corrected=true;
                if (scenario==1) {step=-1.798f;v.ego.length.set(v.ego.length.get()+step);}
                else {
                    if(side==1) {task.r_wall_corner=True;task.r_corner_delay_mm=.5f;task.r_corner_generation=task.wall_edge_generation;}
                    else {task.l_wall_corner=True;task.l_corner_delay_mm=.5f;task.l_corner_generation=task.wall_edge_generation;}
                    task.QueueWallFeature(&v,49.5f,Search_st_section);
                }
            }
            task.RecordPillarCandidates(&v,Search_st_section,STRAIGHT_STATE,0,true);
            // 確定済みの柱が同一IRQで壁補正の後に評価される経路も確認する。
            if(scenario==3 && !corrected && pillars.size()) {
                corrected=true;
                if(side==1) {task.r_wall_corner=True;task.r_corner_delay_mm=.5f;task.r_corner_generation=task.wall_edge_generation;}
                else {task.l_wall_corner=True;task.l_corner_delay_mm=.5f;task.l_corner_generation=task.wall_edge_generation;}
                task.QueueWallFeature(&v,49.5f,Search_st_section);
            }
            const float before=v.ego.length.get();task.SetWallControl_RadVelo(&v,1);
            step+=v.ego.length.get()-before;
        };
        for(int i=0;i<40;++i)sample(120);
        for(int adc:{250,400,650,800,800})for(int i=0;i<4;++i)sample(adc);
        for(int adc:{650,400,250,120})for(int i=0;i<4;++i)sample(adc,true);
        CHECK(corrected && pillars.size()==1);
        const auto &e=pillars.event(0);
        if (scenario==1) CHECK(e.flags&PillarDiagnostics::PositionJump);
        else {
            CHECK(e.flags==0 && std::fabs(e.coordinate_jump_mm)<.01f);
            CHECK(std::fabs(e.minimum.rebase_mm-step)<.01f);
            if(wall_feature_ekf_param.enabled) {
                CHECK(walls.size()==1 && e.position_evaluated && e.position.reason==Reason::NotSelected);
                CHECK(near(e.reference_mm,39.5f) && e.position.delta_mm==0);
                CHECK(scenario==2 ? near(step,0) : std::fabs(step)>(scenario==3?.1f:1.0f));
            } else CHECK(near(step,0));
        }
        walls.stop();
    }
}

static void feature_ekf_adapter()
{
    using namespace WallPositionCorrection;
    for (auto mode:{IrSensTask::WallEkfMode::Shadow,IrSensTask::WallEkfMode::Control}) {
        IrSensTask_type8 task{};Vehicle v{};
        task.WallEkfMode_Set(mode);task.EnableIrSensStraight();
        CHECK(!wall_feature_ekf_param.enabled || task.WallEdgeMode_Get()==(mode==IrSensTask::WallEkfMode::Control?Mode::Control:Mode::Shadow));
        v.ideal.velo.set(.37f);v.ego.velo.set(.37f);
        for (auto &adc:side_wall_test_adc) adc=0;
        for (int i=0;i<50;++i) {task.IrSensorSet(&v,.5f,1000+i);task.SetWallControl_RadVelo(&v,1);}
        auto &log=WallDiagnostics::instance();log.begin(task.WallEdgeMode_Get());
        v.ego.length.set(53);task.r_wall_corner=True;task.r_corner_delay_mm=.5f;task.r_corner_generation=task.wall_edge_generation;
        task.QueueWallFeature(&v,49.5f,Search_st_section);task.SetWallControl_RadVelo(&v,1);
        if (wall_feature_ekf_param.enabled) {
            CHECK(log.size()==1 && log.event(0).correction.ekf && task.wall_correction==True);
            const auto &r=log.event(0).correction;
            CHECK(r.ekf_gain>0 && r.ekf_gain<=.5f && std::fabs(r.proposed_delta_mm+1.5f)<.02f);
            CHECK(mode==IrSensTask::WallEkfMode::Control ? std::fabs(v.ego.length.get()-51.5f)<.02f : near(v.ego.length.get(),53));
            const float before=v.ego.length.get();task.QueueWallFeature(&v,49.5f,Search_st_section);task.SetWallControl_RadVelo(&v,1);
            CHECK(near(v.ego.length.get(),before) && log.event(1).correction.reason==Reason::NotSelected);
            task.Division_Wall_Correction_Reset();v.ego.length.set(52);task.r_corner_generation=task.wall_edge_generation;task.r_wall_corner=True;
            task.QueueWallFeature(&v,49.5f,Search_st_section);task.SetWallControl_RadVelo(&v,1);
            CHECK(log.event(2).correction.ekf_gain>.49f); // New frame renews ds covariance.
        } else CHECK(near(v.ego.length.get(),53) && log.size()==0);
        log.stop();
    }
    // Real ADC -> minimum sample -> delayed confirmation -> 3-state EKF -> ego.
    for (int scenario:{0,1,2,3,4}) {
        const bool conflict=scenario==1;
        IrSensTask_type8 task{};Vehicle v{};task.WallEkfMode_Set(IrSensTask::WallEkfMode::Control);task.EnableIrSensStraight();
        // B still records the trial, but must not adopt A's reference.
        if (!wall_feature_ekf_param.enabled) task.WallEdgeMode_Set(Mode::Control);
        v.ideal.velo.set(.37f);v.ego.velo.set(.37f);v.ego.length.set(13.5f);
        for (auto &adc:side_wall_test_adc) adc=0;
        auto &log=WallDiagnostics::instance();log.begin(task.WallEdgeMode_Get());auto &pillars=PillarDiagnostics::instance();
        if (scenario==3) pillars.stop();
        if (scenario==4) {
            SpatialPillarPeak::Event peak{};peak.detected=peak.motion_clean=true;PillarDiagnostics::Sample sample{};
            for(unsigned i=0;i<pillars.capacity;++i) pillars.record(peak,sample,sample,0,1,Search_st_section,STRAIGHT_STATE,0,Mode::Control,true);
        }
        int n=0;bool queued_conflict=false;float applied=0.0f;
        auto sample=[&](int16_t adc) {
            side_wall_test_adc[sensor_sl]=adc;v.ego.length.set(v.ego.length.get()+.5f);
            task.IrSensorSet(&v,.5f,2000+n++);task.RecordPillarCandidates(&v,Search_st_section,STRAIGHT_STATE,0,true);
            if (conflict && pillars.size() && !queued_conflict) {
                queued_conflict=true;task.r_wall_corner=True;task.r_corner_delay_mm=.5f;task.r_corner_generation=task.wall_edge_generation;
                task.QueueWallFeature(&v,v.ego.length.get()+.5f,Search_st_section);
            }
            if (scenario==2 && pillars.size() && !queued_conflict) {queued_conflict=true;task.Division_Wall_Correction_Reset();}
            const float before=v.ego.length.get();
            task.SetWallControl_RadVelo(&v,1);
            applied+=v.ego.length.get()-before;
        };
        for(int i=0;i<40;++i) sample(120);
        for(int adc:{250,400,650,800,800,650,400,250,120}) for(int j=0;j<4;++j) sample(adc);
        if(scenario>=3) {
            CHECK(pillars.size()==(scenario==3?0U:pillars.capacity));
            CHECK(wall_feature_ekf_param.enabled ? applied<-.1f : applied==0.0f);
            log.stop();continue;
        }
        CHECK(pillars.size()==1 && !pillars.event(0).flags);
        const auto &e=pillars.event(0);
        if (wall_feature_ekf_param.enabled) {
            CHECK(e.position_evaluated && e.reference_valid);
            if (conflict) CHECK(e.position.reason==Reason::NotSelected && e.position.delta_mm==0 && log.size()==1);
            else if (scenario==2) CHECK(e.position.reason==Reason::StaleGeneration && e.position.delta_mm==0 && applied==0);
            else {
                CHECK(e.position.accepted() && e.position.ekf && e.position.delta_mm<0);
                CHECK(std::fabs(e.position.delta_mm)<=4 && e.position.ekf_gain<=.5f);
                CHECK(e.position.target_mm==wall_feature_ekf_param.pillar_reference_mm+e.peak.delay_mm);
            }
        } else CHECK(!e.position_evaluated && !e.reference_valid);
        if (scenario==0) pillars.finish_and_print(); else log.stop();
    }
}

int main(int argc, char **argv)
{
    if(argc==3 && std::strcmp(argv[1],"recorded-open-pillar")==0){recorded_open_pillar(argv[2]);return 0;}
    struct Case { const char *name; void (*run)(); };
    const Case cases[] = {
        {"shortest-feature-entry",shortest_feature_entry},
        {"feature-coordinate-review",feature_coordinate_review},
        {"shortest-feature-exit",shortest_feature_exit},
        {"pose-frame-straight",pose_frame_straight},
        {"motion-heading-chain",motion_heading_chain},
        {"turn-heading-posterior",turn_heading_posterior},
        {"shortest-exit-heading",shortest_exit_heading},
        {"shortest-diagonal-exit-heading",shortest_diagonal_exit_heading},
        {"feature-coordinate-correction",feature_coordinate_correction},
        {"diagonal-feature-model",diagonal_feature_model},
        {"diagonal-pose-route",diagonal_pose_route},
        {"diagonal-pose-handoff",diagonal_pose_handoff},
        {"diagonal-pose-steering",diagonal_pose_steering_runtime},
        {"diagonal-feature-adapter",diagonal_feature_adapter},
        {"machine-pillar-width", machine_pillar_width},
        {"feature-ekf-math", feature_ekf_math},
        {"feature-ekf-adapter", feature_ekf_adapter},
        {"wall-pillar-leds", wall_pillar_leds},
        {"search-turn-runtime", search_turn_runtime},
        {"wall-edge-trial-detector", wall_edge_trial_detector},
        {"straight-pillar-peak", straight_pillar_peak},
        {"straight-close-pillar-history", straight_close_pillar_history},
        {"straight-history-recovery", straight_history_recovery},
        {"open-origin-wall-control", open_origin_wall_control},
        {"diagonal-pillar-minimum", diagonal_pillar_minimum},
        {"pillar-quality-rearm", pillar_quality_rearm},
        {"pillar-ir-wire", pillar_ir_wire},
        {"wall-edge-trial-decision", wall_edge_trial_decision},
        {"wall-edge-trial-motion", wall_edge_trial_motion_application},
        {"wall-edge-trial-shell", wall_edge_trial_shell},
        {"wall-observation-diagnostics", wall_observation_diagnostics},
        {"window", window}, {"partial-window", partial_window},
        {"helper-speed", helper_speed}, {"linear-wall", linear_wall},
        {"decimal-boundaries", decimal_boundaries}, {"linear-activation", linear_activation},
        {"difference-limit", difference_limit}, {"stationary", stationary},
        {"reset", reset}, {"gap", gap}, {"integration-speed", integration_speed},
        {"integration-decimal", integration_decimal},
        {"integration-near", integration_near}, {"integration-front", integration_front},
        {"integration-loss", integration_loss}, {"integration-corner", integration_corner},
        {"edge-noise-pulse", straight_edge_noise_and_pulse},
        {"edge-clock", corner_clock_saturates},
        {"edge-position", edge_position_across_speeds},
        {"edge-invalid", edge_rejects_spikes_and_invalid_motion},
        {"edge-adapter", edge_adapter_symmetry_offsets_and_pair_validity},
        {"edge-adc-mode", edge_adc_rotation_and_mode_reentry},
        {"edge-recall-wall", edge_recalls_distant_and_short_walls},
        {"edge-recall-weak", edge_recalls_weak_distance_rises},
        {"edge-recall-zero", edge_recalls_crossing_between_encoder_pulses},
        {"edge-recall-adc", edge_recall_real_adc_at_sixty_five_mm},
        {"integration-disabled", integration_disabled},
        {"integration-speed-gate", integration_speed_gate}, {"integration-adc", integration_adc},
        {"search-turn-post-correction", search_turn_post_correction_sign_and_limits},
        {"search-turn-heading", search_turn_heading_handoff_math},
        {"search-turn-exit", search_turn_exit_reacquisition},
        {"search-turn-covariance", search_turn_heading_covariance_transfer},
        {"search-turn-covariance-ir", search_turn_covariance_real_ir_handoff},
        {"search-turn-guard", search_turn_front_confirmation_and_fallbacks},
        {"search-turn-ir", search_turn_guard_real_ir_adapter},
        {"search-turn-diagnostics", search_turn_diagnostics_bounds},
        {"zirconia-symmetry", zirconia_symmetry}, {"zirconia-independent", zirconia_independent},
        {"zirconia-spatial", zirconia_spatial_confidence}, {"zirconia-reset", zirconia_reset_and_sync},
        {"zirconia-adapter", zirconia_adc_adapter},
        {"ekf-math", ekf_math_and_dropout}, {"ekf-adapter", ekf_adapter_switching},
        {"ekf-exit-disagreement", ekf_turn_exit_heading_disagreement},
        {"diagonal-ekf-math", diagonal_ekf_symmetry_and_covariance},
        {"diagonal-ekf-sparse", diagonal_ekf_sparse_pillars},
        {"diagonal-ekf-gates", diagonal_ekf_observation_gates},
        {"diagonal-ekf-ir", diagonal_ekf_real_ir_modes},
        {"diagonal-ekf-reset", diagonal_ekf_reset_and_no_phantom_wall},
        {"diagonal-ekf-fallback-log", diagonal_fallback_logged_samples},
        {"diagonal-ekf-front-pillar", diagonal_front_pillar_observations},
        {"diagonal-ekf-trajectory", diagonal_ekf_keeps_trajectory_damping},
        {"diagonal-edge-position", diagonal_edge_spatial_position},
        {"diagonal-edge-plateau", diagonal_edge_clipped_plateau},
        {"diagonal-edge-invalid", diagonal_edge_noise_stop_and_reset},
        {"diagonal-edge-rearm", diagonal_edge_rearm_and_zero_counts},
        {"diagonal-edge-pair", diagonal_corner_pair_delays_and_symmetry},
        {"diagonal-edge-turn", diagonal_turn_delay_compensation},
        {"diagonal-edge-adc", diagonal_edge_real_adc_and_modes},
        {"diagonal-edge-rebound", diagonal_edge_recovers_a_short_rebound},
        {"ekf-corner-mode", ekf_corner_and_mode},
        {"ekf-shell", ekf_shell},
    };
    if (argc == 2) {
        for (const auto &test : cases) {
            if (std::strcmp(argv[1], test.name) == 0) { test.run(); return 0; }
        }
    }
    std::fprintf(stderr, "Unknown side-wall stability test case\n");
    return 2;
}
