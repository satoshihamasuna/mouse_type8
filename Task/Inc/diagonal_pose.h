#ifndef TASK_INC_DIAGONAL_POSE_H_
#define TASK_INC_DIAGONAL_POSE_H_

#include <math.h>
#include <stdint.h>
#include "Params/mouse_config.h"

// s follows the planned diagonal, e and heading are left positive.
namespace DiagonalPose {

struct Route {
    static constexpr unsigned capacity=2*MAZE_SIZE_X+2*MAZE_SIZE_Y;
    static_assert(capacity<256, "Route count must fit in one byte");
    uint8_t count=0;
    // Bit 0: right. Bits 1/2: forward/backward outward wall. Zero walls is a pillar.
    uint8_t pillars[capacity]={};
    static constexpr uint8_t unknown=0x80;
    // Route origin in half cells. Both axes follow the maze's right-handed frame.
    int16_t origin_x=0, origin_y=0;
    int8_t dx=0, dy=0;
    bool has_frame() const { return (dx==1 || dx==-1) && (dy==1 || dy==-1); }
    bool same_frame(const Route &other) const {
        return has_frame() && origin_x==other.origin_x && origin_y==other.origin_y && dx==other.dx && dy==other.dy;
    }
};

struct ObservationModel { float offset_mm, sigma_mm; };
struct Parameters {
    bool calibrated;
    float cot_angle;
    ObservationModel pillar[2][4]; // left/right, no wall / forward / backward / both
};

struct Pose { float s=0, e=0, heading=0; };

enum Status : uint8_t { NoRoute, Uncalibrated, Invalid, NoMatch, Ambiguous, Interval, Updated };

struct Result {
    Pose pose;
    float sigma_e=0, sigma_s=0, sigma_heading=0, innovation=0, nis=0;
    uint8_t index=0, used=0;
    Status status=NoRoute;
    bool lateral_heading_ready=false;
};

class Filter {
public:
    void stop() { active_=bridging_=false; }
    bool active() const { return active_; }
    bool bridging() const { return bridging_; }
    const Pose &odometry() const { return odom_; }
    Pose estimate_pose() const { return {x_[0],x_[1],x_[2]}; }
    bool control_pose(Pose &out) const;
    bool heading_for_turn(float &heading_rad, float &variance_rad2) const;
    void start(float s, float heading, const Route &route, float heading_variance_rad2=-1.0f);
    bool begin_bridge(const Route &next);
    bool finish_bridge(float nominal_s, const Route &next);
    void advance(float distance, float yaw_delta, bool turning=false);
    Result observe(unsigned side, const Pose &peak, float center_shift_mm,
        float width_mm, bool saturated, float gyro_radps, const Parameters &params);

private:
    // Shared across every diagonal passage, regardless of entry turn or speed.
    static constexpr float initial_progress_std_mm=20.0f;
    static constexpr float association_window_mm=0.75f*static_cast<float>(DIAG_SECTION);
    // Fourth state: common uncertainty in the effective peak offset. Carry it
    // in the covariance so repeated pillars cannot certify the absolute phase.
    static constexpr unsigned size=4;
    bool pose_ready() const;
    float x_[size]={}, p_[size][size]={};
    Pose odom_;
    Route route_;
    uint8_t consumed_[2]={}, mask_=0, used_=0;
    uint16_t ticks_since_update_=0;
    uint16_t bridge_ticks_=0;
    float travel_since_update_mm_=0;
    float bridge_distance_mm_=0;
    bool active_=false, bridging_=false, inherited_=false;
};

class Steering {
public:
    struct Command { float omega_radps, alpha_radps2; };
    void reset() { blend_=0; engaged_=false; }
    bool engaged() const { return engaged_; }
    float blend() const { return blend_; }
    // Returns the previous controller's target after the blend reaches zero.
    Command update(const Pose *pose, float speed_mps, float travel_mm, float dt_s,
        float previous_omega, const Command &fallback);

private:
    float blend_=0;
    float last_omega_=0;
    bool engaged_=false;
};

} // namespace DiagonalPose

#endif
