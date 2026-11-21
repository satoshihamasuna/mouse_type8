// Host-only driver for the firmware's turn tracker and world pose.
#define parameters source_parameters
#include "Params/turn_pose_param.h"
#undef parameters

namespace TurnPose {
static float suction_k;
static float suction_acceleration_std;
inline Parameters parameters(bool suction) {
    auto value = source_parameters(suction);
    if (suction && suction_k > 0.0f) {
        value.k_m_s2 = suction_k;
        value.acceleration_std_m_s2 = suction_acceleration_std;
    }
    return value;
}
}

#include "Task/Inc/turn_pose_ekf.h"
#include "Module/Inc/pose_frames.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>

int main(int argc, char **argv) {
    if (argc != 6) return 2;
    std::ifstream input(argv[1]);
    std::ofstream output(argv[2]);
    if (!input || !output) return 3;
    TurnPose::suction_k = std::strtof(argv[3], nullptr);
    TurnPose::suction_acceleration_std = std::strtof(argv[4], nullptr);
    TurnPose::Tracker tracker;
    if (std::atoi(argv[5]) != 0) tracker.set_mode(TurnPose::Mode::Legacy);
    PoseFrames::Tracker pose;
    output << std::setprecision(9)
           << "forward_mm,left_mm,yaw_rad,applied,reason,beta_rad,nis,sigma_beta_rad,updates,rejected,bias_ready\n";
    float period_us, previous_irq_us, velocity, accel, gyro, lateral, target, heading_before;
    float right_pulses, left_pulses, replay_dt, duty;
    int pattern, turn, quiet, first_moving, force_nominal, retained;
    while (input >> period_us >> previous_irq_us >> velocity >> accel >> gyro >> lateral
                 >> target >> heading_before >> right_pulses >> left_pulses >> replay_dt
                 >> duty >> pattern >> turn >> quiet >> first_moving >> force_nominal >> retained) {
        if (first_moving) pose.start();
        TurnPose::Input sample;
        sample.timing_valid = period_us >= 500.0f && period_us <= 2500.0f
                           && previous_irq_us >= 0.0f && previous_irq_us < .85f * period_us;
        const float nominal_irq_s = .001f;
        const float irq_s = period_us * 1.0e-6f;
        const float speed_denominator = sample.timing_valid ? irq_s*1000.0f : 1.0f;
        sample.dt_s = retained ? (sample.timing_valid ? irq_s : nominal_irq_s) : replay_dt;
        if (force_nominal) {
            sample.timing_valid = true;
            sample.dt_s = replay_dt;
        }
        sample.speed_m_s = velocity / (force_nominal ? replay_dt*1000.0f : speed_denominator);
        const float mean_wheel = velocity - .010f * accel;
        sample.wheel_speed_m_s = mean_wheel / speed_denominator;
        sample.target_speed_m_s = target;
        sample.gyro_rad_s = gyro;
        sample.lateral_accel_m_s2 = lateral;
        sample.forward_accel_m_s2 = accel;
        sample.heading_before_rad = heading_before;
        const float heading = heading_before + gyro * nominal_irq_s;
        sample.legacy_left_mm = velocity * sinf(heading);
        sample.legacy_forward_mm = velocity * cosf(heading);
        sample.suction = duty > 0.0f;
        sample.suction_duty = duty;
        sample.pattern = static_cast<uint8_t>(pattern);
        sample.turning = turn != 0;
        sample.quiet_command = quiet != 0;
        const auto estimate = tracker.update(sample);
        const auto state = tracker.snapshot();
        const float encoder_mm = (right_pulses - left_pulses) * MMPP / 2.0f;
        if (estimate.applied)
            pose.predict_turn(estimate.forward_mm, estimate.left_mm, estimate.source_heading_before_rad, estimate.yaw_delta_rad);
        else
            pose.predict_body(encoder_mm, 0.0f, gyro * nominal_irq_s);
        const auto world = pose.snapshot().world;
        output << world.x_mm << ',' << world.y_mm << ',' << world.yaw_rad << ','
               << estimate.applied << ',' << TurnPose::reason_name(state.reason) << ','
               << state.slip_rad << ',' << state.nis << ',' << state.sigma_slip_rad << ','
               << state.updates << ',' << state.rejected << ',' << state.bias_ready << '\n';
    }
    return input.eof() && output.good() ? 0 : 4;
}
