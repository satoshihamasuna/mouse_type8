#include "Task/Inc/straight_wall_accel_control.h"
#include "Task/Inc/straight_wall_accel_adapter.h"
#include <assert.h>
#include <float.h>
#include <initializer_list>
#include <stdio.h>
#include <string.h>
#include <type_traits>

using Controller = StraightWallAccelControl;

// 合成モデル検証専用。A/Bの実機採用パラメータではない。
static Controller::Parameters parameters()
{
    Controller::Parameters p;
    p.lookahead_m = 0.067f;
    p.error_kp_per_s2 = 400.0f;
    p.error_kd_per_s = 40.0f;
    p.omega_limit_radps = 1.2f;
    p.alpha_limit_radps2 = 20.0f;
    return p;
}

static Controller::Input sample()
{
    Controller::Input in;
    in.enabled = true;
    in.velocity_mps = 0.8f;
    return in;
}

static bool close(float a, float b, float tolerance = 0.0001f) { return fabsf(a - b) < tolerance; }

// 定加速度・定角加速度で短時間動かした実座標から、sの微分を数値的に調べる。
// 制御器内部のalpha計算式は再実装しない。
static double future_error(const Controller::Input &in, double length, double alpha, double t)
{
    const auto velocity = [&](double u) {
        const double heading = in.heading_rad + in.measured_omega_radps * u + 0.5 * alpha * u * u;
        return (in.velocity_mps + in.accel_mps2 * u) * sin(heading);
    };
    const double e = in.lateral_m + t / 6.0 * (velocity(0) + 4 * velocity(t / 2) + velocity(t));
    const double heading = in.heading_rad + in.measured_omega_radps * t + 0.5 * alpha * t * t;
    return e + length * heading;
}

static void model()
{
    auto p = parameters();
    p.omega_limit_radps = 100.0f;
    p.alpha_limit_radps2 = 10000.0f;
    for (float acceleration : {-20.0f, 0.0f, 20.0f}) {
        for (float heading : {-0.3f, 0.04f, 0.3f}) {
            Controller controller(p);
            auto in = sample();
            in.lateral_m = 0.0015f;
            in.heading_rad = heading;
            in.velocity_mps = 1.3f;
            in.accel_mps2 = acceleration;
            in.measured_omega_radps = -0.4f;
            const auto out = controller.update(in);
            assert(out.status == Controller::Status::Active);
            assert(!out.alpha_saturated && !out.omega_saturated);
            const double h = 0.0001;
            const double center = future_error(in, p.lookahead_m, out.alpha_radps2, 0);
            const double before = future_error(in, p.lookahead_m, out.alpha_radps2, -h);
            const double after = future_error(in, p.lookahead_m, out.alpha_radps2, h);
            const double first = (after - before) / (2 * h);
            const double second = (after - 2 * center + before) / (h * h);
            assert(fabs(first - out.error_rate_mps) < 0.00001);
            assert(fabs(second + p.error_kd_per_s * first + p.error_kp_per_s2 * center) < 0.001);
        }
    }
}

static void feedback()
{
    auto p = parameters();
    p.omega_limit_radps = 10;
    p.alpha_limit_radps2 = 1000;
    Controller a(p), b(p);
    assert(a.reset(0.2f) && b.reset(-0.2f));
    auto in = sample();
    in.lateral_m = 0.002f;
    in.heading_rad = 0.01f;
    in.measured_omega_radps = -0.03f;
    const auto oa = a.update(in), ob = b.update(in);
    assert(close(oa.requested_alpha_radps2, ob.requested_alpha_radps2));
    assert(close(oa.alpha_radps2, ob.alpha_radps2));
    assert(close(oa.omega_radps - ob.omega_radps, 0.4f));

    // 実測角速度が違えば、指令角速度の初期値が同じでもフィードバックは変わる。
    assert(a.reset() && b.reset());
    const auto before = a.update(in);
    in.measured_omega_radps += 0.1f;
    const auto after = b.update(in);
    assert(after.requested_alpha_radps2 < before.requested_alpha_radps2);

    // 構築後に元の構造体を書き換えても、固定Lは変更されない。
    assert(a.reset());
    const auto fixed = a.update(in);
    p.lookahead_m *= 2;
    assert(a.reset());
    assert(close(fixed.requested_alpha_radps2, a.update(in).requested_alpha_radps2));
}

static void limits()
{
    const auto p = parameters();
    for (float dt : {0.0005f, 0.001f, 0.007f}) {
        Controller left(p), right(p);
        float previous = 0;
        auto in = sample();
        in.dt_s = dt;
        in.lateral_m = 0.03f;
        Controller::Output out;
        for (unsigned i = 0; i < 500; ++i) {
            out = left.update(in);
            auto mirror = in;
            mirror.lateral_m *= -1;
            const auto reflected = right.update(mirror);
            assert(out.status == Controller::Status::Active);
            assert(fabsf(out.omega_radps) <= p.omega_limit_radps);
            assert(fabsf(out.alpha_radps2) <= p.alpha_limit_radps2 + 0.001f);
            assert(close(out.omega_radps - previous, out.alpha_radps2 * dt, 0.000001f));
            assert(close(out.omega_radps, -reflected.omega_radps));
            assert(close(out.alpha_radps2, -reflected.alpha_radps2));
            previous = out.omega_radps;
        }
        assert(close(out.omega_radps, -p.omega_limit_radps));
        assert(out.alpha_saturated && out.omega_saturated);
        assert(out.alpha_radps2 == 0);
        // 飽和中の積分を蓄積しないので、逆向きの誤差へ即座に応答する。
        in.lateral_m *= -1;
        const auto reverse = left.update(in);
        assert(reverse.omega_radps > out.omega_radps);
        assert(close(reverse.alpha_radps2, p.alpha_limit_radps2, 0.001f));
    }
}

static void stopped(const Controller::Output &out, Controller::Status status)
{
    assert(out.status == status);
    assert(out.omega_radps == 0 && out.alpha_radps2 == 0);
    assert(out.error_m == 0 && out.error_rate_mps == 0 && out.requested_alpha_radps2 == 0);
}

static void lifecycle()
{
    Controller controller(parameters());
    auto in = sample();
    assert(controller.reset(0.5f));
    in.enabled = false;
    stopped(controller.update(in), Controller::Status::Disabled);
    in.enabled = true;
    assert(controller.update(in).omega_radps == 0);
    for (float velocity : {-1.0f, 0.0f, 0.20f}) {
        assert(controller.reset(-0.5f));
        in.velocity_mps = velocity;
        stopped(controller.update(in), Controller::Status::LowSpeed);
        in.velocity_mps = 0.8f;
        assert(controller.update(in).omega_radps == 0);
    }
    assert(controller.reset(0.5f));
    assert(!controller.reset(1.3f));
    assert(controller.update(in).omega_radps == 0);
    assert(!controller.reset(NAN));
    assert(controller.update(in).omega_radps == 0);
}

static void invalid()
{
    const float bad_values[] = {NAN, INFINITY, -INFINITY};
    const auto p = parameters();
    Controller controller(p);
    const auto neutral = sample();
    float Controller::Input::*fields[] = {&Controller::Input::lateral_m, &Controller::Input::heading_rad,
        &Controller::Input::velocity_mps, &Controller::Input::accel_mps2,
        &Controller::Input::measured_omega_radps, &Controller::Input::dt_s};
    for (auto field : fields) for (float bad : bad_values) {
        assert(controller.reset(0.5f));
        auto in = neutral;
        in.*field = bad;
        stopped(controller.update(in), Controller::Status::InvalidInput);
        assert(controller.update(neutral).omega_radps == 0);
    }
    for (float dt : {-0.001f, 0.0f, 0.020f, 1.0f}) {
        auto in = neutral;
        in.dt_s = dt;
        stopped(controller.update(in), Controller::Status::InvalidInput);
    }
    auto overflow = neutral;
    overflow.lateral_m = FLT_MAX;
    stopped(controller.update(overflow), Controller::Status::InvalidInput);
    float Controller::Parameters::*positive[] = {&Controller::Parameters::lookahead_m,
        &Controller::Parameters::error_kp_per_s2, &Controller::Parameters::error_kd_per_s,
        &Controller::Parameters::omega_limit_radps, &Controller::Parameters::alpha_limit_radps2};
    for (auto field : positive) for (float bad : {0.0f, -1.0f, NAN, INFINITY}) {
        auto bad_param = p;
        bad_param.*field = bad;
        Controller bad_controller(bad_param);
        assert(!bad_controller.reset());
        stopped(bad_controller.update(neutral), Controller::Status::InvalidParameters);
    }
    for (float bad : {-1.0f, NAN, INFINITY}) {
        auto bad_param = p;
        bad_param.minimum_velocity_mps = bad;
        Controller bad_controller(bad_param);
        stopped(bad_controller.update(neutral), Controller::Status::InvalidParameters);
    }
    Controller unset(Controller::Parameters{});
    stopped(unset.update(neutral), Controller::Status::InvalidParameters);
}

static void trajectory()
{
    // 理想角速度追従の非線形モデル。20 m/s^2の加減速・定速を繰り返す。
    // 実機モータ/IR/横滑りや角速度PIDの帯域を模擬した評価ではない。
    const auto p = parameters();
    for (float dt : {0.0005f, 0.001f, 0.002f}) for (float sign : {-1.0f, 1.0f}) {
        Controller controller(p);
        double e = sign * 0.006, heading = sign * 0.025, omega = 0;
        const unsigned steps = static_cast<unsigned>(3.0 / dt + 0.5);
        for (unsigned i = 0; i < steps; ++i) {
            const double time = i * static_cast<double>(dt);
            const double phase = fmod(time, 1.0);
            double v = 0.4, accel = 0;
            if (phase >= 0.1 && phase < 0.16) { v += 20 * (phase - 0.1); accel = 20; }
            else if (phase >= 0.16 && phase < 0.4) v = 1.6;
            else if (phase >= 0.4 && phase < 0.46) { v = 1.6 - 20 * (phase - 0.4); accel = -20; }
            auto in = sample();
            in.lateral_m = static_cast<float>(e);
            in.heading_rad = static_cast<float>(heading);
            in.measured_omega_radps = static_cast<float>(omega);
            in.velocity_mps = static_cast<float>(v);
            in.accel_mps2 = static_cast<float>(accel);
            in.dt_s = dt;
            const auto out = controller.update(in);
            assert(out.status == Controller::Status::Active);
            assert(fabsf(out.omega_radps) <= p.omega_limit_radps);
            assert(fabsf(out.alpha_radps2) <= p.alpha_limit_radps2 + 0.001f);
            const double alpha = out.alpha_radps2;
            const auto lateral_velocity = [&](double t) {
                return (v + accel * t) * sin(heading + omega * t + 0.5 * alpha * t * t);
            };
            e += dt / 6.0 * (lateral_velocity(0) + 4 * lateral_velocity(dt / 2) + lateral_velocity(dt));
            heading += omega * dt + 0.5 * alpha * dt * dt;
            omega = out.omega_radps;
            assert(fabs(e) < 0.012 && fabs(heading) < 0.15);
        }
        if (fabs(e) >= 0.00001 || fabs(heading) >= 0.0002)
            fprintf(stderr, "dt:%g sign:%g final_e:%g final_heading:%g\n", dt, sign, e, heading);
        assert(fabs(e) < 0.00001 && fabs(heading) < 0.0002);
    }
}

using Adapter = StraightWallAccelAdapter;
static_assert(std::is_same<Adapter::Input, StraightWallControl::Input>::value, "shared input");
static_assert(std::is_same<Adapter::Output, StraightWallControl::Output>::value, "shared output");
static_assert(std::is_same<Adapter::Phase, StraightWallControl::Phase>::value, "shared phase");

static Adapter::Parameters adapter_parameters()
{
    Adapter::Parameters p;
    p.lookahead_m = parameters().lookahead_m;
    p.error_kp_per_s2 = parameters().error_kp_per_s2;
    p.error_kd_per_s = parameters().error_kd_per_s;
    return p;
}

static Adapter::Input walls()
{
    Adapter::Input in;
    in.enabled = in.sample_updated = true;
    in.velocity_mps = 0.8f;
    in.travel_m = 0.0008f;
    in.left_m = 0.043f;
    in.right_m = 0.047f;
    in.left_signal = in.right_signal = 1000;
    in.accel_mps2 = 6;
    in.measured_omega_radps = 0;
    return in;
}

static void same_observation(const Adapter::Output &a, const Adapter::Output &b)
{
    assert(a.phase == b.phase && a.pose_control == b.pose_control);
    assert(a.observed_mask == b.observed_mask && a.trusted_mask == b.trusted_mask);
    assert(a.sync_estimator == b.sync_estimator);
    assert(close(a.error_m, b.error_m, 0.000001f) && close(a.raw_error_m, b.raw_error_m, 0.000001f));
    assert(close(a.transition, b.transition) && close(a.phase_distance_m, b.phase_distance_m));
}

static void adapter_observations()
{
    for (bool estimated : {false, true}) {
        StraightWallControl legacy;
        Adapter adapter(adapter_parameters());
        bool applied = false;
        for (unsigned i = 0; i < 220; ++i) {
            auto in = walls();
            in.use_estimated_pose = estimated;
            in.prediction_trusted = estimated;
            in.estimated_error_m = -0.004f;
            if (i >= 100 && i < 110) in.right_m = 0.080f;
            if (i >= 110 && i < 150) {
                in.left_m = in.right_m = 0.080f;
                in.heading_rad = 0.02f;
                in.heading_only_gap = true;
            }
            if (i >= 150) { in.left_m = 0.050f; in.right_m = 0.040f; }
            if (i >= 170 && i < 180) in.range_valid_mask = 1;
            if (i >= 180 && i < 200) in.permitted_mask = 1;
            const auto reference = legacy.update(in), out = adapter.update(in);
            same_observation(reference, out);
            assert(adapter.status() == Adapter::Status::Active);
            if (i >= 170 && i < 180) assert(!adapter.using_acceleration_control());
            if (out.phase == Adapter::Estimated) {
                assert(!adapter.using_acceleration_control());
                assert(close(out.omega_radps, reference.omega_radps));
                assert(close(out.alpha_radps2, reference.alpha_radps2));
            }
            if (adapter.using_acceleration_control()) {
                applied = true;
                const auto &model = adapter.acceleration_output();
                assert(close(model.error_m, -out.error_m + adapter_parameters().lookahead_m * in.heading_rad));
                if (i < 100) assert(out.error_m < 0 && out.omega_radps <= 0);
            }
        }
        assert(applied);
        adapter.prime_tracking(0.043f, 0.047f);
        legacy.prime_tracking(0.043f, 0.047f);
        same_observation(legacy.update(walls()), adapter.update(walls()));
        adapter.reset();
        assert(adapter.status() == Adapter::Status::Disabled && !adapter.using_acceleration_control());
    }
}

static void adapter_handover()
{
    Adapter left(adapter_parameters()), right(adapter_parameters());
    float previous = 0;
    for (unsigned i = 0; i < 200; ++i) {
        auto in = walls();
        in.use_estimated_pose = true;
        in.estimated_error_m = -0.004f;
        if (i >= 100 && i < 150) in.measurement_mask = 0;
        auto reflected = in;
        reflected.left_m = in.right_m;
        reflected.right_m = in.left_m;
        reflected.estimated_error_m *= -1;
        const auto a = left.update(in), b = right.update(reflected);
        assert(close(a.omega_radps, -b.omega_radps));
        assert(close(a.alpha_radps2, -b.alpha_radps2, 0.001f));
        assert(close(a.omega_radps - previous, a.alpha_radps2 * in.dt_s, 0.000001f));
        const float bound = a.pose_control ? wall_control_param.tracking_alpha_limit : wall_control_param.alpha_limit;
        assert(fabsf(a.alpha_radps2) <= bound + 0.001f);
        assert(fabsf(a.omega_radps) <= wall_control_param.tracking_omega_limit);
        if (i == 99 || i == 199) assert(left.using_acceleration_control());
        if (i >= 100 && i < 150) assert(!left.using_acceleration_control());
        previous = a.omega_radps;
    }
}

static void adapter_guards()
{
    Adapter adapter(adapter_parameters());
    auto in = walls();
    in.accel_mps2 = in.measured_omega_radps = NAN;
    StraightWallControl legacy;
    assert(legacy.update(in).phase != Adapter::Disabled); // 現行制御は追加フィールドを使わない。
    assert(adapter.update(in).phase == Adapter::Disabled);
    assert(adapter.status() == Adapter::Status::InvalidInput);
    in = walls();
    for (unsigned i = 0; i < 100; ++i) adapter.update(in);
    assert(adapter.using_acceleration_control());
    in.enabled = false;
    assert(adapter.update(in).phase == Adapter::Disabled);
    assert(adapter.status() == Adapter::Status::Disabled);
    in = walls();
    in.velocity_mps = 0.20f;
    assert(adapter.update(in).phase == Adapter::Disabled);
    assert(adapter.status() == Adapter::Status::LowSpeed);
    in = walls();
    for (float invalid : {NAN, INFINITY}) {
        in.measured_omega_radps = invalid;
        assert(adapter.update(in).phase == Adapter::Disabled);
        assert(adapter.status() == Adapter::Status::InvalidInput);
    }
    in = walls();
    in.dt_s = 0.020f;
    assert(adapter.update(in).phase == Adapter::Disabled);
    assert(adapter.status() == Adapter::Status::InvalidInput);
    Adapter unconfigured;
    assert(unconfigured.update(walls()).phase == Adapter::Disabled);
    assert(unconfigured.status() == Adapter::Status::InvalidParameters);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "model")) model();
    else if (!strcmp(argv[1], "feedback")) feedback();
    else if (!strcmp(argv[1], "limits")) limits();
    else if (!strcmp(argv[1], "lifecycle")) lifecycle();
    else if (!strcmp(argv[1], "invalid")) invalid();
    else if (!strcmp(argv[1], "trajectory")) trajectory();
    else if (!strcmp(argv[1], "adapter-observations")) adapter_observations();
    else if (!strcmp(argv[1], "adapter-handover")) adapter_handover();
    else if (!strcmp(argv[1], "adapter-guards")) adapter_guards();
    else return 2;
    return 0;
}
