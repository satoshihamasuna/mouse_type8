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
        {"zirconia-symmetry", zirconia_symmetry}, {"zirconia-independent", zirconia_independent},
        {"zirconia-spatial", zirconia_spatial_confidence}, {"zirconia-reset", zirconia_reset_and_sync},
        {"zirconia-adapter", zirconia_adc_adapter},
    };
    if (argc == 2) {
        for (const auto &test : cases) {
            if (std::strcmp(argv[1], test.name) == 0) { test.run(); return 0; }
        }
    }
    std::fprintf(stderr, "Unknown side-wall stability test case\n");
    return 2;
}
