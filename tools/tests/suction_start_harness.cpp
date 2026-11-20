#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "Component/Inc/math_utils.h"
#include "Task/Inc/ctrl_task.h"
#include "Params/run_config.h"

t_encoder test_right{}, test_left{};
int test_fan_duty = 0;
extern "C" void Sensor_SetMotionActive(uint8_t) {}

namespace {
constexpr float OM_JERK_FF_LIMIT_V = 2.0f;
constexpr float SP_TURN_FF_LIMIT_V = 1.0f;
constexpr float SUCTION_OM_FB_LIMIT_V = 0.5f;
#if defined(MOUSE_A)
constexpr float default_slip_k_off = 41.28055f, default_slip_k_on = 254.90725f;
#else
constexpr float default_slip_k_off = 75.0f, default_slip_k_on = 250.0f;
#endif
void check(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "%s\n", message);
        std::exit(1);
    }
}
}

void Motion::Motion_error_handling() {}
#include "suction_functions.inc"

class TestControl : public CtrlTask {
public:
    using CtrlTask::CtrlTask;
    using Motion::SetIdeal_suction_start;
    using Motion::updateTurnKinematics;
};

static void slip_model_follows_output()
{
    Vehicle simulated;
    Vehicle &hardware = Vehicle_type8::getInstance();
    Vehicle *vehicles[] = {&simulated, &hardware};
    for (Vehicle *vehicle : vehicles) {
        vehicle->suctionStop();
        vehicle->ego.turn_slip_theta.set(0.12f);
        vehicle->ideal.turn_slip_theta.set(-0.03f);
        vehicle->pose.set_start({30.0f, 40.0f, 0.5f});
        vehicle->pose.start();
        vehicle->pose.predict_body(1.0f, 0.0f, 0.01f);
        const auto before = vehicle->pose.snapshot();
        check(vehicle->turn_slip_k_m_s2() == default_slip_k_off,
            "unpowered fan did not use this machine's non-suction default");
        vehicle->suction_flag_set(True);
        check(vehicle->turn_slip_k_m_s2() == default_slip_k_off,
            "startup flag selected suction K before any fan output");
        vehicle->suctionSetDuty(650);
        check(vehicle->turn_slip_k_m_s2() == default_slip_k_on && vehicle->suction_out == 650,
            "base/hardware vehicle disagrees on the powered fan slip model");
        if (vehicle == &hardware) check(test_fan_duty == 650, "fan output was not sent to hardware");
        vehicle->suctionSetDuty(0);
        check(vehicle->turn_slip_k_m_s2() == default_slip_k_off && vehicle->suction_flag_get() == True,
            "zero duty retained suction K while startup remained enabled");
        vehicle->suctionSetDuty(700);
        vehicle->suctionStop();
        check(vehicle->turn_slip_k_m_s2() == default_slip_k_off
            && vehicle->suction_flag_get() == False && vehicle->suction_out == 0 && test_fan_duty == 0,
            "fan stop retained stale output or suction K");
        const Vehicle &constant_vehicle = *vehicle;
        check(constant_vehicle.turn_slip_k_m_s2() == default_slip_k_off,
            "slip model was not available from a const vehicle");
        const auto after = vehicle->pose.snapshot();
        check(after.world.x_mm == before.world.x_mm && after.world.y_mm == before.world.y_mm
            && after.world.yaw_rad == before.world.yaw_rad && after.samples == before.samples
            && vehicle->ego.turn_slip_theta.get() == 0.12f
            && vehicle->ideal.turn_slip_theta.get() == -0.03f,
            "suction I/O modified pose or estimated slip state");
        vehicle->pose.stop();
    }
}

static void stationary_noise(const t_run_config *config, float noise)
{
    Vehicle vehicle{};
    IrSensTask sensors;
    TestControl control(&vehicle, &sensors, 1);
    vehicle.ego_initialize();
    vehicle.ideal_initialize();
    vehicle.battery.set(7.4f);
    test_right.wheel_speed = test_left.wheel_speed = 0.0f;
    control.motion_enable_set();
    const auto *gains = config->straight_control->bands[0].gain;
    const float voltage = config->suction / 1000.0f * 8.40f;
    const int stay_ms = static_cast<int>(voltage / SUCTION_ACC) + 300;
    control.Init_Motion_suction_start(voltage, stay_ms, &gains->sp_gain);
    check(vehicle.turn_slip_k_m_s2() == default_slip_k_off,
        "suction motion selected suction K before issuing fan duty");
    float previous = 0.0f;
    for (int tick = 0; tick < stay_ms + 15; ++tick) {
        // 車輪停止のままIMU由来の速度偏りだけが存在する。
        vehicle.ego.velo.set(noise);
        if (control.motion_pattern_get() == Suction_start) control.SetIdeal_suction_start();
        control.motion_control();
        check(vehicle.turn_slip_k_m_s2() == default_slip_k_on,
            "fan ramp output did not select the suction default coefficient");
        check(vehicle.sp_feedback.get() == 0.0f,
            "stationary wheels received speed feedback from acceleration noise");
        check(vehicle.motor_out_r == 0 && vehicle.motor_out_l == 0,
            "stationary wheels received a drive command during/after suction_start");
        check(vehicle.V_suction.get() >= previous && vehicle.V_suction.get() <= voltage,
            "suction voltage did not ramp monotonically to its target");
        previous = vehicle.V_suction.get();
    }
    check(std::fabs(previous - voltage) < 1e-6f && vehicle.suction_flag_get() == True,
        "suction was not held after startup");
    check(control.motion_exeStatus_get() == complete, "suction_start did not complete");
    check(sensors.steering_calls == 0, "stationary startup invoked moving wall steering");

    // 実際の前後移動は引き続き逆向きに抑える。
    const float speeds[] = {0.02f, -0.02f};
    for (float speed : speeds) {
        control.Init_Motion_suction_start(voltage, stay_ms, &gains->sp_gain);
        vehicle.ego.velo.set(0.0f);
        test_right.wheel_speed = speed;
        test_left.wheel_speed = -speed;
        control.SetIdeal_suction_start();
        control.motion_control();
        check(vehicle.sp_feedback.get() * speed < 0.0f, "wheel movement was not opposed");
        check(vehicle.V_r * speed < 0.0f && vehicle.V_l * speed > 0.0f,
            "left/right stationary braking signs are incorrect");
    }

    // 次の直進では従来の走行速度推定と吸引FFへ戻る。
    control.motion_pattern_set(Straight);
    control.motion_control_set(STRAIGHT_STATE);
    vehicle.Vehicle_controller.speed_ctrl.I_param_reset();
    vehicle.ideal.velo.set(0.5f);
    vehicle.ego.velo.set(0.49f);
    test_right.wheel_speed = 0.47f;
    test_left.wheel_speed = -0.47f;
    control.motion_control();
    check(std::fabs(vehicle.sp_feedback.get() - gains->sp_gain.Kp * 0.01f) < 1e-5f,
        "moving control no longer uses the normal fused velocity");
    const float expected_ff = ff_gain_straight_suction.sp_bias + ff_gain_straight_suction.sp_velo * 0.5f;
    check(std::fabs(vehicle.sp_feedforward.get() - expected_ff) < 1e-5f,
        "suction straight feedforward changed");
}

static void measured_and_ideal_slip_defaults()
{
    Vehicle &vehicle = Vehicle_type8::getInstance();
    IrSensTask sensors;
    TestControl control(&vehicle, &sensors, 1);
    const int duties[] = {0, 650};
    const float yaw_rates[] = {-0.3f, 0.3f};
    for (int duty : duties) {
        vehicle.suctionSetDuty(duty);
        const float k = duty ? default_slip_k_on : default_slip_k_off;
        for (float yaw_rate : yaw_rates) {
            vehicle.ego_initialize();
            vehicle.ideal_initialize();
            vehicle.ideal.velo.set(0.5f);
            vehicle.ideal.rad_velo.set(yaw_rate);
            for (int tick = 0; tick < 2000; ++tick) {
                step_measured_slip(0.5f, yaw_rate);
                control.updateTurnKinematics();
            }
            // Steady small-slip balance: K*beta + speed*yaw_rate = 0.
            const float expected = -0.5f * yaw_rate / k;
            check(std::fabs(vehicle.ego.turn_slip_theta.get() - expected) < 1e-6f,
                "measured slip did not converge using the configured default K");
            check(std::fabs(vehicle.ideal.turn_slip_theta.get() - expected) < 1e-6f,
                "ideal slip did not converge using the same default K");
            check(std::fabs(vehicle.ideal.horizon_accel.get()) < 2e-6f,
                "ideal lateral acceleration disagrees with the steady slip balance");
        }
    }
    vehicle.suctionStop();
}

static void reset_stale_targets()
{
    Vehicle vehicle{};
    IrSensTask sensors;
    TestControl control(&vehicle, &sensors, 1);
    vehicle.ego_initialize();
    vehicle.ideal_initialize();
    vehicle.battery.set(7.4f);
    vehicle.ideal.velo.set(0.5f);
    vehicle.ideal.accel.set(6.0f);
    vehicle.ideal.rad_velo.set(0.3f);
    vehicle.ideal.rad_accel.set(1.0f);
    vehicle.ideal.rad_jerk.set(2.0f);
    control.Init_Motion_suction_start(5.0f, 1000.0f);
    check(vehicle.ideal.velo.get() == 0.0f && vehicle.ideal.accel.get() == 0.0f
        && vehicle.ideal.rad_velo.get() == 0.0f && vehicle.ideal.rad_accel.get() == 0.0f
        && vehicle.ideal.rad_jerk.get() == 0.0f, "startup inherited moving targets");
    // 毎周期も静止目標を明示する。
    vehicle.ideal.rad_velo.set(0.3f);
    control.SetIdeal_suction_start();
    check(vehicle.ideal.rad_velo.get() == 0.0f, "startup failed to maintain a stationary yaw target");
    test_right.wheel_speed = test_left.wheel_speed = 0.0f;
    vehicle.ego.rad_velo.set(0.2f);
    control.motion_enable_set();
    control.motion_control();
    check(vehicle.om_feedback.get() < 0.0f, "gyro yaw damping was lost");
    check(vehicle.sp_feedforward.get() == 0.0f && vehicle.om_feedforward.get() == 0.0f,
        "stationary startup inherited feedforward");
}

static void heading_hold_survives_completion()
{
    const float rates[] = {-0.2f, 0.2f};
    for (float rate : rates) {
        Vehicle vehicle{};
        IrSensTask sensors;
        TestControl control(&vehicle, &sensors, 1);
        vehicle.ego_initialize();
        vehicle.ideal_initialize();
        vehicle.battery.set(7.4f);
        test_right.wheel_speed = test_left.wheel_speed = 0.0f;
        control.motion_enable_set();
        control.Init_Motion_suction_start(0.8f, 200.0f);
        for (int tick = 0; tick < 200; ++tick) {
            vehicle.ego.rad_velo.set(tick < 150 ? rate : 0.0f);
            vehicle.ego.radian.set(vehicle.ego.radian.get() + vehicle.ego.rad_velo.get()*0.001f);
            control.SetIdeal_suction_start();
            control.motion_control();
        }
        const float angle = vehicle.ego.radian.get();
        const float correction = vehicle.om_feedback.get();
        check(correction * angle < 0.0f, "startup did not oppose the accumulated heading error");
        while (control.motion_pattern_get() == Suction_start) {
            control.SetIdeal_suction_start();
            control.motion_control();
        }
        check(vehicle.ego.radian.get() == angle,
            "startup completion discarded the heading error before it was corrected");
        check(std::fabs(vehicle.om_feedback.get() - correction) < 1e-6f,
            "startup completion removed the holding torque");
        for (int tick = 0; tick < 30; ++tick) control.motion_control();
        check(std::fabs(vehicle.om_feedback.get() - correction) < 1e-6f,
            "completed startup failed to hold the initial direction in No_run");

        // 次の走行は既存のゲイン設定・積分リセットで通常制御へ戻る。
        control.motion_pattern_set(Straight);
        control.motion_control_set(STRAIGHT_STATE);
        vehicle.Vehicle_controller.omega_ctrl.Gain_Set(basic_om_gain);
        vehicle.Vehicle_controller.omega_ctrl.I_param_reset();
        control.motion_control();
        check(vehicle.om_feedback.get() == 0.0f, "startup yaw correction leaked into the next motion");
    }
}

static void startup_gains_and_feedback_limit()
{
    Vehicle vehicle{};
    IrSensTask sensors;
    TestControl control(&vehicle, &sensors, 1);
    vehicle.ego_initialize();
    vehicle.ideal_initialize();
    vehicle.battery.set(7.4f);
    test_right.wheel_speed = test_left.wheel_speed = 0.0f;
    control.motion_enable_set();
    control.Init_Motion_suction_start(0.8f, 200.0f);
    const auto &yaw = vehicle.Vehicle_controller.omega_ctrl;
    check(yaw.Kp == om_gain_suction_start.Kp && yaw.Ki == om_gain_suction_start.Ki,
        "startup did not select this machine's stationary gains");
    const float rates[] = {-20.0f, 20.0f};
    for (float rate : rates) {
        control.Init_Motion_suction_start(0.8f, 200.0f);
        vehicle.ego.rad_velo.set(rate);
        for (int tick = 0; tick < 50; ++tick) {
            control.SetIdeal_suction_start();
            control.motion_control();
            check(std::fabs(vehicle.om_feedback.get()) == SUCTION_OM_FB_LIMIT_V,
                "stationary heading feedback exceeded its voltage limit");
            check(vehicle.om_feedback.get()*rate < 0.0f, "limited yaw correction has the wrong sign");
        }
        check(vehicle.Vehicle_controller.omega_ctrl.get_I_peration() == 0.0f,
            "stationary saturation accumulated an integral kick");
        vehicle.ego.rad_velo.set(0.0f);
        control.motion_control();
        check(vehicle.om_feedback.get() == 0.0f,
            "saturation release retained a stationary yaw command");
    }
    const float angles[] = {-0.2f, 0.2f};
    for (float angle : angles) {
        vehicle.ego.radian.set(angle);
        for (int tick = 0; tick < 100; ++tick) {
            control.motion_control();
            check(vehicle.om_feedback.get()*angle < 0.0f
                && std::fabs(vehicle.om_feedback.get()) == SUCTION_OM_FB_LIMIT_V,
                "saturation lost the original heading reference");
        }
        vehicle.ego.radian.set(angle*0.1f);
        control.motion_control();
        const float expected = -angle*0.1f*om_gain_suction_start.Ki*1000.0f;
        check(std::fabs(vehicle.om_feedback.get()-expected) < 1e-6f,
            "heading correction did not recover from the absolute angle after saturation");
        vehicle.ego.radian.set(0.0f);
        control.motion_control();
        check(vehicle.om_feedback.get() == 0.0f,
            "returning to the initial heading retained a saturation kick");
    }
    control.motion_pattern_set(Straight);
    control.motion_control_set(STRAIGHT_STATE);
    vehicle.ego.rad_velo.set(20.0f);
    control.motion_control();
    check(std::fabs(vehicle.om_feedback.get()) > SUCTION_OM_FB_LIMIT_V,
        "stationary voltage limit leaked into moving control");
    control.motion_disable_set();
    control.motion_control();
    check(vehicle.motor_out_l == 0 && vehicle.motor_out_r == 0,
        "disabled control continued heading hold");

    const t_pid_gain explicit_gain = {0.15f,0.006f,0.0f};
    control.Init_Motion_suction_start(0.8f, 200.0f, &basic_sp_gain, &explicit_gain);
    check(yaw.Kp == explicit_gain.Kp && yaw.Ki == explicit_gain.Ki,
        "explicit startup tuning gains were ignored");
}

int main()
{
    slip_model_follows_output();
    measured_and_ideal_slip_defaults();
    for (const auto *config : run_config_table) {
        if (config->suction == 0) continue;
        stationary_noise(config, 0.015f);
        stationary_noise(config, -0.015f);
    }
    reset_stale_targets();
    heading_hold_survives_completion();
    startup_gains_and_feedback_limit();
    std::puts("suction_start stationary/noise/braking/launch checks passed");
}
