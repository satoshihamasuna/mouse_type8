/* Copyright 2026 hmsn
 * Modifications Copyright 2026 hmsn
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ZIRCONIA_WALL_CONTROL_H_
#define ZIRCONIA_WALL_CONTROL_H_

#include <stdint.h>
#include "Params/zirconia_wall_param.h"

// Adapted from program_zirconia/zirconia/software/Control/Src/wall_control.c
// (independent_side_pose_v3). No hardware or trajectory ownership here.
class ZirconiaWallControl
{
public:
    enum Phase : uint8_t { Disabled, Estimated, Confirming, AwayOnly, Tracking };
    struct Input {
        float left_m = 0.080f, right_m = 0.080f;
        float left_signal = 0.0f, right_signal = 0.0f;
        float travel_m = 0.0f, velocity_mps = 0.0f;
        float heading_rad = 0.0f, estimated_error_m = 0.0f;
        float base_omega_radps = 0.0f, dt_s = 0.001f;
        bool enabled = false, sample_updated = false;
    };
    struct Output {
        float error_m = 0.0f, raw_error_m = 0.0f;
        float omega_radps = 0.0f, alpha_radps2 = 0.0f;
        float transition = 0.0f, phase_distance_m = 0.0f;
        uint8_t observed_mask = 0, trusted_mask = 0;
        Phase phase = Disabled;
        bool pose_control = false, sync_estimator = false;
        bool alpha_saturated = false, omega_saturated = false;
    };
    void reset();
    void prime_tracking(float left_m, float right_m);
    Output update(const Input &input);

private:
    struct Side {
        float distance_m = 0.0f, signal = 0.0f;
        float travel_m = 0.0f, predicted_m = 0.0f, recovery_m = 0.0f;
        bool anchor = false, stable = false;
        void update(bool available, float distance, float signal_value,
                    float travel, float predicted, bool sample_updated);
    };
    Side left_, right_;
    Phase phase_ = Disabled;
    uint8_t observed_mask_ = 0;
    bool left_detected_ = false, right_detected_ = false;
    float phase_distance_m_ = 0.0f, transition_ = 0.0f, omega_ = 0.0f;
    float tracking_error_m_ = 0.0f;
    bool tracking_valid_ = false;
    float sync_error_m_ = 0.0f, sync_travel_m_ = 0.0f, sync_stable_m_ = 0.0f;
    uint8_t sync_mask_ = 0;
    bool sync_valid_ = false;
};

// Match zirconia's ten fresh ADC readings, independently of the legacy
// twenty-cycle distance history used by wall-edge detection.
class ZirconiaWallAverage
{
public:
    void update(int16_t value) {
        sum_ -= samples_[index_];
        samples_[index_] = value > 0 ? value : 0;
        sum_ += samples_[index_];
        index_ = (index_ + 1) % 10;
        if (count_ < 10) ++count_;
    }
    bool ready() const { return count_ == 10; }
    int16_t value() const { return count_ ? static_cast<int16_t>(sum_ / count_) : 0; }
private:
    int16_t samples_[10] = {};
    int32_t sum_ = 0;
    int index_ = 0, count_ = 0;
};
#endif
