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

#include "Task/Inc/side_wall_stability.h"
#include "Task/Inc/sensing_task.h"
#include "Params/sens_table.h"

int16_t side_wall_test_adc[5] = {};

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    std::exit(1); } } while (false)

static bool near(float actual, float expected)
{
    return std::fabs(actual - expected) < 0.0001f;
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
    fixture.task.sen_r.prev_diff = -0.1f;
    fixture.task.sen_r.diff = 0.1f;
    fixture.sample(1.0f);
    CHECK(fixture.task.r_wall_corner == True);
    CHECK(fixture.task.sen_r.is_control == False && fixture.task.sen_l.is_control == True);
    CHECK(fixture.task.sen_r.control_distance == 0.0f);
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
    fixture.task.sen_r.distance = 52.0f;
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
    CHECK(fixture.task.control_ir.get() == 1.0f);
    CHECK(fixture.vehicle.ego.x_point.get() == 1.0f);
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
        {"integration-disabled", integration_disabled},
        {"integration-speed-gate", integration_speed_gate}, {"integration-adc", integration_adc},
    };
    if (argc == 2) {
        for (const auto &test : cases) {
            if (std::strcmp(argv[1], test.name) == 0) { test.run(); return 0; }
        }
    }
    std::fprintf(stderr, "Unknown side-wall stability test case\n");
    return 2;
}
