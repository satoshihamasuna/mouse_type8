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

#include "Task/Inc/side_wall_stability.h"
#include "Task/Inc/sensing_task.h"
#include "Params/sens_table.h"
#include "Task/Inc/search_turn_heading.h"
#include "Subsys/Inc/search_turn_diagnostics.h"

int16_t side_wall_test_adc[5] = {};

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

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    std::exit(1); } } while (false)

static bool near(float actual, float expected)
{
    return std::fabs(actual - expected) < 0.0001f;
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
    for (int which = 0; which < 4; ++which) for (float delay : {0.0f, .5f, 3.2f}) {
        Vehicle state{}; Vehicle *vehicle = &state;
        IrSensTask_type8 sensors{}; IrSensTask *ir_sens = &sensors;
        sensors.r_corner_delay_mm = sensors.l_corner_delay_mm = delay;
        struct Fields { float Lend; } fields{20.0f};
        struct Ref { const Fields *param; } turn_motion_param{&fields};
        struct Plan { param_element fix_post_run; } motion_plan{};
        motion_plan.fix_post_run.set(3.0f);
        const float detect_wall_edge_di = 0.0f;
        vehicle->ego.length.set(10.0f + delay);
        #include "diagonal_turn_delay.inc"
        CHECK(near(vehicle->ego.length.get() - delay, which < 2 ? 5.0f : 16.5f));
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

int main(int argc, char **argv)
{
    struct Case { const char *name; void (*run)(); };
    const Case cases[] = {
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
