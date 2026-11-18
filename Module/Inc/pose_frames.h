#ifndef MODULE_INC_POSE_FRAMES_H_
#define MODULE_INC_POSE_FRAMES_H_

#include <math.h>
#include <stdint.h>
#include "Task/Inc/run_typedef.h"
#if defined(__arm__) || defined(__thumb__)
#include "core_cm33.h"
#endif

// Geometry shared by the independent estimators, not another EKF. By default
// world +X is the starting forward direction, +Y is left, yaw is counterclockwise.
// Every position is mm and every angle is rad. A frame is a fixed numerical
// reference: changing it never moves the estimated world pose.
namespace PoseFrames {
struct Pose { float x_mm=0, y_mm=0, yaw_rad=0; };
inline bool finite(const Pose &p) { return isfinite(p.x_mm) && isfinite(p.y_mm) && isfinite(p.yaw_rad); }
inline float wrap(float angle) { return atan2f(sinf(angle),cosf(angle)); }
inline Pose compose(const Pose &origin,const Pose &local) {
    const float c=cosf(origin.yaw_rad),s=sinf(origin.yaw_rad);
    return {origin.x_mm+c*local.x_mm-s*local.y_mm,origin.y_mm+s*local.x_mm+c*local.y_mm,
        wrap(origin.yaw_rad+local.yaw_rad)};
}
inline Pose relative(const Pose &origin,const Pose &world) {
    const float c=cosf(origin.yaw_rad),s=sinf(origin.yaw_rad);
    const float x=world.x_mm-origin.x_mm,y=world.y_mm-origin.y_mm;
    return {c*x+s*y,-s*x+c*y,wrap(world.yaw_rad-origin.yaw_rad)};
}
// Covariance of [x_mm,y_mm,yaw_rad] in a FIXED reference frame. This does not
// add uncertainty of an estimated origin or treat correlated EKFs as sensors.
inline void rotate_covariance(const float in[3][3],float angle,float out[3][3]) {
    const float c=cosf(angle),s=sinf(angle),r[3][3]={{c,-s,0},{s,c,0},{0,0,1}};
    float tmp[3][3]={},result[3][3]={};
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j)
        for(unsigned k=0;k<3;++k) tmp[i][j]+=r[i][k]*in[k][j];
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j)
        for(unsigned k=0;k<3;++k) result[i][j]+=tmp[i][k]*r[j][k];
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) out[i][j]=result[i][j];
}

class StateLock {
#if defined(__arm__) || defined(__thumb__)
    uint32_t mask_;
public:
    StateLock():mask_(__get_PRIMASK()) { __disable_irq(); }
    ~StateLock() { __set_PRIMASK(mask_); }
#else
public:
    StateLock() {}
#endif
};

struct State {
    Pose world,origin;
    float reference_yaw_rad=0;
    uint32_t motion=0,frame=0,samples=0,corrections=0,rejected=0;
    t_runControl phase=NOP_STATE;
    bool active=false,valid=true;
};
struct Snapshot : State {
    Pose local; // worldとoriginから算出する参照用の値。内部には重複保存しない。
};

class Tracker {
public:
    Snapshot snapshot() const {
        const StateLock lock; Snapshot out;
        static_cast<State &>(out)=state_;
        out.local=relative(out.origin,out.world); return out;
    }
    bool set_start(const Pose &origin) {
        const StateLock lock;
        if (state_.active || !finite(origin)) return false;
        initial_=origin; initial_.yaw_rad=wrap(initial_.yaw_rad); reset(false); return true;
    }
    void start() { const StateLock lock; reset(true); }
    void stop() { const StateLock lock; state_.active=false; }
    void new_motion() {
        const StateLock lock;
        if (state_.active) { ++state_.motion; new_frame_=true; }
    }
    void set_phase(t_runControl phase) {
        const StateLock lock;
        if (!state_.active || (!new_frame_ && phase==state_.phase)) return;
        state_.phase=phase; state_.origin={state_.world.x_mm,state_.world.y_mm,state_.reference_yaw_rad};
        ++state_.frame; new_frame_=false;
    }
    void complete_turn(float logical_rad) {
        const StateLock lock;
        if (!state_.active) return;
        if (!isfinite(logical_rad)) { reject(); return; }
        state_.reference_yaw_rad=wrap(state_.reference_yaw_rad+logical_rad);
        new_frame_=true;
    }
    // Sampled once in preprocess. Signed wheel travel also covers reverse.
    void predict_body(float forward_mm,float left_mm,float yaw_delta_rad) {
        const StateLock lock;
        if (!state_.active) return;
        if (!valid_step(forward_mm,left_mm,yaw_delta_rad)) { reject(); return; }
        const float mid=state_.world.yaw_rad+.5f*yaw_delta_rad;
        translate(forward_mm,left_mm,mid);
        state_.world.yaw_rad=wrap(state_.world.yaw_rad+yaw_delta_rad); ++state_.samples;
    }
    // Accepted turn-EKF increments are already integrated in their own fixed
    // frame. Rotate them directly; do not integrate distance or gyro again.
    void predict_turn(float forward_mm,float left_mm,float source_heading_before_rad,float yaw_delta_rad) {
        const StateLock lock;
        if (!state_.active) return;
        if (!valid_step(forward_mm,left_mm,yaw_delta_rad) || !isfinite(source_heading_before_rad)) { reject(); return; }
        translate(forward_mm,left_mm,state_.world.yaw_rad-source_heading_before_rad);
        state_.world.yaw_rad=wrap(state_.world.yaw_rad+yaw_delta_rad); ++state_.samples;
    }
    void correct_passage(float forward_mm,float left_mm) {
        const StateLock lock;
        if (!state_.active) return;
        if (!isfinite(forward_mm) || !isfinite(left_mm)) { reject(); return; }
        translate(forward_mm,left_mm,state_.reference_yaw_rad); ++state_.corrections;
    }
    void align_passage_heading(float heading_rad) {
        const StateLock lock;
        if (!state_.active) return;
        if (!isfinite(heading_rad)) { reject(); return; }
        state_.world.yaw_rad=wrap(state_.reference_yaw_rad+heading_rad);
    }
    // Bind an estimator's native progress/lateral coordinates at its start.
    // Its yaw is relative to the planned passage direction, not ego.radian.
    Pose passage_origin(const Pose &local) const {
        const StateLock lock;
        const float c=cosf(state_.reference_yaw_rad),s=sinf(state_.reference_yaw_rad);
        return {state_.world.x_mm-c*local.x_mm+s*local.y_mm,
            state_.world.y_mm-s*local.x_mm-c*local.y_mm,state_.reference_yaw_rad};
    }
    void adopt_world(const Pose &world) {
        const StateLock lock;
        if (!state_.active) return;
        if (!finite(world)) { reject(); return; }
        state_.world=world; state_.world.yaw_rad=wrap(world.yaw_rad); ++state_.corrections;
    }
    // Compatibility view for old ego.turn_x/y telemetry. Keep only its fixed
    // origin; both these values and local are derived from the same world pose.
    void reset_log_origin(float sensor_heading_rad) {
        const StateLock lock;
        if (!isfinite(sensor_heading_rad)) { reject(); return; }
        log_origin_={state_.world.x_mm,state_.world.y_mm,wrap(state_.world.yaw_rad-sensor_heading_rad)};
    }
    Pose log_position() const {
        const StateLock lock; return relative(log_origin_,state_.world);
    }
private:
    Pose initial_,log_origin_;
    State state_;
    bool new_frame_=false;
    void reset(bool active) {
        state_=State{}; state_.world=state_.origin=initial_;
        log_origin_=initial_;
        state_.reference_yaw_rad=initial_.yaw_rad; state_.active=active; new_frame_=false;
    }
    void reject() { ++state_.rejected; state_.valid=false; }
    static bool valid_step(float forward,float left,float yaw) {
        return isfinite(forward) && isfinite(left) && isfinite(yaw)
            && hypotf(forward,left)<=20.0f && fabsf(yaw)<=1.0f;
    }
    void translate(float forward,float left,float angle) {
        const float c=cosf(angle),s=sinf(angle);
        state_.world.x_mm+=c*forward-s*left; state_.world.y_mm+=s*forward+c*left;
    }
};
}

#endif
