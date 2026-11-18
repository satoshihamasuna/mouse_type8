#ifndef PARAMS_TURN_POSE_PARAM_H_
#define PARAMS_TURN_POSE_PARAM_H_

#include "Params/mouse_config.h"

namespace TurnPose {
struct Parameters {
    bool calibrated;
    float k_m_s2, acceleration_std_m_s2, gyro_std_rad_s;
    float initial_gyro_bias_std_rad_s, initial_accel_bias_std_m_s2;
    float calibration_speed_m_s;
};

// A: 2026-11-18, 500/no suction (6 videos), 1600/suction650 (13 videos).
// tools/logs/turn_slip_conditions_A_20261118_v1/report.json
// Effective small-slip model ay=-K*beta. B has no matching calibration.
inline Parameters parameters(bool suction) {
#if defined(MOUSE_A)
    return suction ? Parameters{true,254.90725f,7.0f,0.65f,0.10f,0.6f,1.6f}
                   : Parameters{true,41.28055f,1.1f,0.04f,0.02f,0.3f,0.5f};
#else
    (void)suction;
    return {false,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
#endif
}
// Shared default for measured/ideal slip [m/s^2]. Use this machine's
// identified K when available; retain the old values for uncalibrated B.
// This does not expand the EKF's speed/duty/pattern eligibility.
inline float slip_k_m_s2(bool suction) {
    const auto p=parameters(suction);
    return p.calibrated ? p.k_m_s2 : (suction ? 250.0f : 75.0f);
}
constexpr float observation_interval_s=0.020f; // 20-point averages are correlated.
constexpr float observation_nis_limit=9.0f;
constexpr float heading_std_limit_rad=0.08726646f;
constexpr float slip_std_limit_rad=0.06981317f;
constexpr float slip_limit_rad=0.43633232f;
constexpr float yaw_disagreement_limit_rad=0.10471976f;
constexpr float maximum_position_difference_m=0.030f;
constexpr float maximum_step_correction_m=0.001f;
constexpr float bias_window_s=0.10f;
constexpr float bias_max_age_s=30.0f;
} // namespace TurnPose

#endif
