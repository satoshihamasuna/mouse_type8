#ifndef TASK_INC_TURN_POSE_EKF_H_
#define TASK_INC_TURN_POSE_EKF_H_

#include <math.h>
#include <stdint.h>
#include "Params/turn_pose_param.h"
#include "Task/Inc/run_typedef.h"
#include "Task/Inc/motion_heading.h"

namespace TurnPose {
enum class Mode : uint8_t { Legacy, Shadow, Control };
enum class Reason : uint8_t { Inactive, Legacy, Uncalibrated, Unsupported, BiasNotReady,
    Invalid, ProfileChanged, Innovation, Covariance, Disagreement, Shadow, Applied };
inline const char *reason_name(Reason reason) {
    static const char *const names[]={"inactive","legacy","uncalibrated","unsupported","bias_not_ready",
        "invalid","profile_changed","innovation","covariance","disagreement","shadow","applied"};
    const unsigned index=static_cast<unsigned>(reason);
    return index<sizeof(names)/sizeof(names[0])?names[index]:"invalid";
}

struct Input {
    MotionHeading::Prior heading_prior;
    float dt_s=0, speed_m_s=0, wheel_speed_m_s=0, target_speed_m_s=0, gyro_rad_s=0, lateral_accel_m_s2=0;
    float forward_accel_m_s2=0, heading_before_rad=0, legacy_left_mm=0, legacy_forward_mm=0;
    float suction_duty=0;
    uint8_t pattern=0;
    bool turning=false, suction=false, timing_valid=false, quiet_command=false;
};
struct Output {
    float left_mm=0, forward_mm=0, slip_rad=0;
    bool applied=false;
    float source_heading_before_rad=0, yaw_delta_rad=0;
};
struct Snapshot {
    Mode mode=Mode::Control;
    Reason reason=Reason::Inactive;
    bool active=false, applied=false, bias_ready=false, suction=false, eligible=false;
    uint8_t pattern=0;
    float forward_mm=0, left_mm=0, heading_rad=0, slip_rad=0, sigma_slip_rad=0, sigma_heading_rad=0;
    float legacy_forward_mm=0, legacy_left_mm=0, nis=0, k_m_s2=0;
    uint32_t predictions=0, updates=0, rejected=0, applied_ticks=0;
};

// Fixed six-state [forward,left,yaw,slip,gyro bias,lateral-accel bias] EKF.
// All covariance/scratch storage belongs to the singleton, not the IRQ stack.
class Filter {
public:
    void start(float heading, float gyro_bias, float accel_bias, const Parameters &p, float heading_variance=-1.0f) {
        for (unsigned i=0;i<n;++i) { x_[i]=0; for (unsigned j=0;j<n;++j) p_[i][j]=0; }
        x_[2]=heading; x_[4]=gyro_bias; x_[5]=accel_bias;
        p_[0][0]=p_[1][1]=1.0e-6f;
        p_[2][2]=0.0012184697f; p_[3][3]=0.0027415568f; // 2deg / 3deg
        if (isfinite(heading_variance) && heading_variance>0) p_[2][2]=heading_variance;
        p_[4][4]=p.initial_gyro_bias_std_rad_s*p.initial_gyro_bias_std_rad_s;
        p_[5][5]=p.initial_accel_bias_std_m_s2*p.initial_accel_bias_std_m_s2;
    }
    bool predict(float u, float gyro, float dt, const Parameters &p) {
        if (!isfinite(u) || !isfinite(gyro) || !isfinite(dt) || dt<=0 || dt>=0.010f || !p.calibrated) return false;
        const float decay=expf(-p.k_m_s2/fmaxf(0.1f,fabsf(u))*dt);
        const float omega=gyro-x_[4], coupling=u/p.k_m_s2*(1.0f-decay);
        const float beta=decay*x_[3]-coupling*omega, mid_beta=(x_[3]+beta)*0.5f;
        if (fabsf(mid_beta)>0.7f) return false;
        const float angle=x_[2]+0.5f*omega*dt, c=cosf(angle), s=sinf(angle);
        const float lateral=u*tanf(mid_beta), cb=cosf(mid_beta), derivative=u/(cb*cb);
        const float dx=(c*u-s*lateral)*dt, dy=(s*u+c*lateral)*dt;
        for(unsigned i=0;i<n;++i) for(unsigned j=0;j<n;++j) f_[i][j]=i==j?1.0f:0.0f;
        f_[0][2]=-dy; f_[1][2]=dx;
        f_[0][3]=-s*derivative*0.5f*(1+decay)*dt;
        f_[1][3]= c*derivative*0.5f*(1+decay)*dt;
        f_[0][4]=dy*0.5f*dt-s*derivative*0.5f*coupling*dt;
        f_[1][4]=-dx*0.5f*dt+c*derivative*0.5f*coupling*dt;
        f_[2][4]=-dt; f_[3][3]=decay; f_[3][4]=coupling;
        x_[0]+=dx; x_[1]+=dy; x_[2]+=omega*dt; x_[3]=beta;
        multiply_covariance();
        const float distance_noise=0.03f*fabsf(u)*dt;
        next_[0][0]+=distance_noise*distance_noise;
        next_[1][1]+=distance_noise*distance_noise;
        next_[2][2]+=0.000027415568f*dt;
        next_[3][3]+=0.019495514f*dt; // 8deg/sqrt(s), force-model mismatch
        next_[4][4]+=0.000004f*dt;
        next_[5][5]+=(p.acceleration_std_m_s2>2.0f?0.01f:0.0025f)*dt;
        for (unsigned i=0;i<n;++i) for (unsigned j=0;j<n;++j) {
            const float gi=i==4?0.0f:-f_[i][4], gj=j==4?0.0f:-f_[j][4];
            next_[i][j]+=gi*gj*p.gyro_std_rad_s*p.gyro_std_rad_s;
        }
        store_covariance();
        return finite();
    }
    bool observe(float acceleration, const Parameters &p, float &nis) {
        const float h3=-p.k_m_s2;
        const float predicted=h3*x_[3]+x_[5], residual=acceleration-predicted;
        const float extra=0.25f*p.k_m_s2*x_[3];
        const float variance=p.acceleration_std_m_s2*p.acceleration_std_m_s2+extra*extra;
        const float innovation=h3*h3*p_[3][3]+h3*(p_[3][5]+p_[5][3])+p_[5][5]+variance;
        nis=residual*residual/innovation;
        if (!isfinite(nis) || innovation<=0 || nis>observation_nis_limit) return false;
        for(unsigned i=0;i<n;++i) gain_[i]=(p_[i][3]*h3+p_[i][5])/innovation;
        for(unsigned i=0;i<n;++i) {
            x_[i]+=gain_[i]*residual;
            for(unsigned j=0;j<n;++j) f_[i][j]=(i==j?1.0f:0.0f)-gain_[i]*(j==3?h3:j==5?1.0f:0.0f);
        }
        multiply_covariance();
        for(unsigned i=0;i<n;++i) for(unsigned j=0;j<n;++j) next_[i][j]+=gain_[i]*gain_[j]*variance;
        store_covariance();
        return finite();
    }
    float state(unsigned i) const { return x_[i]; }
    float variance(unsigned i) const { return p_[i][i]; }
    bool finite() const {
        for(unsigned i=0;i<n;++i) {
            if (!isfinite(x_[i]) || !isfinite(p_[i][i]) || p_[i][i]<0) return false;
            for(unsigned j=0;j<n;++j) if(!isfinite(p_[i][j])) return false;
        }
        return true;
    }
private:
    static constexpr unsigned n=6;
    float x_[n]={},p_[n][n]={},f_[n][n]={},temp_[n][n]={},next_[n][n]={},gain_[n]={};
    void multiply_covariance() {
        for(unsigned i=0;i<n;++i) for(unsigned j=0;j<n;++j) {
            temp_[i][j]=0; for(unsigned k=0;k<n;++k) temp_[i][j]+=f_[i][k]*p_[k][j];
        }
        for(unsigned i=0;i<n;++i) for(unsigned j=0;j<n;++j) {
            next_[i][j]=0; for(unsigned k=0;k<n;++k) next_[i][j]+=temp_[i][k]*f_[j][k];
        }
    }
    void store_covariance() {
        for(unsigned i=0;i<n;++i) for(unsigned j=0;j<n;++j) p_[i][j]=(next_[i][j]+next_[j][i])*0.5f;
    }
};

class Tracker {
public:
    void set_mode(Mode mode) { mode_=mode; active_=aborted_=inherited_heading_=false; snapshot_.mode=mode; snapshot_.applied=false; }
    Mode mode() const { return mode_; }
    void reset() {
        active_=aborted_=observed_=inherited_heading_=false; elapsed_=accepted_age_=legacy_forward_=legacy_left_=0;
        for(auto &b:bias_) { b.clear_window();b.ready=false;b.age=0;b.gyro=b.accel=0; }
        snapshot_=Snapshot{};snapshot_.mode=mode_;
    }
    Snapshot snapshot() const { return snapshot_; }
    MotionHeading::Prior heading_for_handoff() const {
        if (!active_ || aborted_ || !inherited_heading_ || !snapshot_.applied) return {};
        return {filter_.state(2),filter_.variance(2),true};
    }

    Output update(const Input &in) {
        snapshot_.applied=false; snapshot_.active=false; snapshot_.mode=mode_;
        if(isfinite(in.dt_s) && in.dt_s>0) for(auto &b:bias_) b.age+=in.dt_s;
        const auto p=parameters(in.suction);
        if (!in.turning) {
            active_=aborted_=inherited_heading_=false;
            if(in.timing_valid) calibrate(in);
            snapshot_.reason=Reason::Inactive;
            return {};
        }
        snapshot_.active=true;
        snapshot_.pattern=in.pattern;snapshot_.suction=in.suction;snapshot_.k_m_s2=p.k_m_s2;
        snapshot_.eligible=eligible(in,p);
        if (!p.calibrated) return reject(Reason::Uncalibrated);
        if (mode_==Mode::Legacy) return reject(Reason::Legacy);
        if (!valid(in)) { aborted_=true; return reject(Reason::Invalid); }
        if(active_ && (suction_!=in.suction || pattern_!=in.pattern)) {
            aborted_=true; return reject(Reason::ProfileChanged);
        }
        if(aborted_) return reject(Reason::Invalid);
        if(!active_ && mode_==Mode::Control && !eligible(in,p)) {
            snapshot_.pattern=in.pattern;snapshot_.suction=in.suction;snapshot_.eligible=false;
            return reject(Reason::Unsupported);
        }
        auto &bias=bias_[in.suction?1:0];
        if(!active_) {
            active_=true; suction_=in.suction; pattern_=in.pattern;
            inherited_heading_=in.heading_prior.usable();
            heading_offset_=inherited_heading_?in.heading_prior.heading_rad-in.heading_before_rad:0.0f;
            filter_.start(in.heading_before_rad+heading_offset_,bias.ready?bias.gyro:0.0f,bias.ready?bias.accel:0.0f,p,
                inherited_heading_?in.heading_prior.variance_rad2:-1.0f);
            elapsed_=observation_interval_s; accepted_age_=1.0f; observed_=false;
            legacy_forward_=legacy_left_=0;
            snapshot_.predictions=snapshot_.updates=snapshot_.rejected=snapshot_.applied_ticks=0;
        }
        snapshot_.pattern=in.pattern; snapshot_.suction=in.suction; snapshot_.k_m_s2=p.k_m_s2;
        snapshot_.eligible=eligible(in,p);
        snapshot_.bias_ready=bias.ready && bias.age<bias_max_age_s;
        const float previous_forward=filter_.state(0),previous_left=filter_.state(1);
        const float previous_heading=filter_.state(2);
        legacy_forward_+=in.legacy_forward_mm/1000.0f; legacy_left_+=in.legacy_left_mm/1000.0f;
        if(!filter_.predict(in.speed_m_s,in.gyro_rad_s,in.dt_s,p)) { aborted_=true; return reject(Reason::Invalid); }
        ++snapshot_.predictions; elapsed_+=in.dt_s; accepted_age_+=in.dt_s;
        if(elapsed_+1.0e-7f>=observation_interval_s) {
            elapsed_=0;
            if(filter_.observe(in.lateral_accel_m_s2,p,snapshot_.nis)) {
                observed_=true; accepted_age_=0; ++snapshot_.updates;
            } else ++snapshot_.rejected;
        }
        if(!filter_.finite()) { aborted_=true; return reject(Reason::Invalid); }
        snapshot_.forward_mm=filter_.state(0)*1000; snapshot_.left_mm=filter_.state(1)*1000;
        snapshot_.heading_rad=filter_.state(2); snapshot_.slip_rad=filter_.state(3);
        snapshot_.sigma_heading_rad=sqrtf(filter_.variance(2)); snapshot_.sigma_slip_rad=sqrtf(filter_.variance(3));
        const float c=cosf(heading_offset_),s=sinf(heading_offset_);
        snapshot_.legacy_forward_mm=(c*legacy_forward_-s*legacy_left_)*1000;
        snapshot_.legacy_left_mm=(s*legacy_forward_+c*legacy_left_)*1000;
        if(!snapshot_.eligible) return reject(Reason::Unsupported);
        if(!snapshot_.bias_ready) return reject(Reason::BiasNotReady);
        if(!observed_ || accepted_age_>0.040f || snapshot_.nis>observation_nis_limit) return reject(Reason::Innovation);
        if(snapshot_.sigma_heading_rad>heading_std_limit_rad || snapshot_.sigma_slip_rad>slip_std_limit_rad
            || fabsf(snapshot_.slip_rad)>slip_limit_rad) return reject(Reason::Covariance);
        const float dx=filter_.state(0)-previous_forward,dy=filter_.state(1)-previous_left;
        // The passage prior may have a different origin from ego.radian. Keep
        // legacy sensor-frame increments in their existing frame, including gates.
        const float raw_dx=c*dx+s*dy,raw_dy=-s*dx+c*dy;
        const float expected_heading=in.heading_before_rad+heading_offset_+in.gyro_rad_s*in.dt_s;
        if(fabsf(filter_.state(2)-expected_heading)>yaw_disagreement_limit_rad
            || hypotf(c*filter_.state(0)+s*filter_.state(1)-legacy_forward_,-s*filter_.state(0)+c*filter_.state(1)-legacy_left_)>maximum_position_difference_m
            || hypotf(raw_dx-in.legacy_forward_mm/1000,raw_dy-in.legacy_left_mm/1000)>maximum_step_correction_m)
            return reject(Reason::Disagreement);
        if(mode_==Mode::Shadow) return reject(Reason::Shadow);
        snapshot_.applied=true; snapshot_.reason=Reason::Applied; ++snapshot_.applied_ticks;
        return {raw_dy*1000,raw_dx*1000,filter_.state(3),true,previous_heading-heading_offset_,filter_.state(2)-previous_heading};
    }
private:
    struct Bias {
        float time=0,gyro_sum=0,accel_sum=0,duty_min=1000,duty_max=0,gyro=0,accel=0,age=0;
        unsigned count=0;
        bool ready=false;
        void clear_window() { time=gyro_sum=accel_sum=0;count=0;duty_min=1000;duty_max=0; }
    } bias_[2];
    Filter filter_;
    Snapshot snapshot_;
    Mode mode_=Mode::Control;
    bool active_=false,aborted_=false,suction_=false,observed_=false,inherited_heading_=false;
    uint8_t pattern_=0;
    float elapsed_=0,accepted_age_=0,legacy_forward_=0,legacy_left_=0,heading_offset_=0;
    Output reject(Reason reason) { snapshot_.reason=reason;return {}; }
    static bool valid(const Input &in) {
        return in.timing_valid && isfinite(in.dt_s) && in.dt_s>=0.0005f && in.dt_s<=0.006f
            && isfinite(in.speed_m_s) && in.speed_m_s>0.15f && in.speed_m_s<3.5f
            && isfinite(in.gyro_rad_s) && fabsf(in.gyro_rad_s)<80.0f
            && isfinite(in.lateral_accel_m_s2) && fabsf(in.lateral_accel_m_s2)<150.0f
            && isfinite(in.heading_before_rad) && isfinite(in.legacy_left_mm) && isfinite(in.legacy_forward_mm);
    }
    static bool eligible(const Input &in,const Parameters &p) {
        const bool pattern=in.pattern==Long_turnR90 || (in.suction && in.pattern==Long_turnR180);
        return p.calibrated && pattern && fabsf(in.target_speed_m_s-p.calibration_speed_m_s)<0.05f
            && fabsf(in.speed_m_s-p.calibration_speed_m_s)<(in.suction?0.30f:0.10f)
            && (!in.suction || (in.suction_duty>=600 && in.suction_duty<=700));
    }
    void calibrate(const Input &in) {
        if(bias_[in.suction?0:1].time>0) bias_[in.suction?0:1].clear_window();
        auto &b=bias_[in.suction?1:0];
        const bool quiet=in.quiet_command && isfinite(in.dt_s) && in.dt_s>=0.0005f && in.dt_s<=0.006f
            && isfinite(in.wheel_speed_m_s) && fabsf(in.wheel_speed_m_s)<0.02f && isfinite(in.suction_duty);
        if(!quiet) { b.clear_window(); return; }
        b.time+=in.dt_s;
        // Fan vibration can invalidate one averaged acceleration sample. Keep
        // the fixed time window, but never count that sample as stationary data.
        if(isfinite(in.gyro_rad_s) && fabsf(in.gyro_rad_s)<3.0f
            && isfinite(in.lateral_accel_m_s2) && fabsf(in.lateral_accel_m_s2)<2.0f
            && isfinite(in.forward_accel_m_s2) && fabsf(in.forward_accel_m_s2)<2.0f) {
            b.gyro_sum+=in.gyro_rad_s;b.accel_sum+=in.lateral_accel_m_s2;++b.count;
        }
        b.duty_min=fminf(b.duty_min,in.suction_duty); b.duty_max=fmaxf(b.duty_max,in.suction_duty);
        if(b.time+1.0e-6f>=bias_window_s) {
            const float gyro=b.count?b.gyro_sum/b.count:0.0f, accel=b.count?b.accel_sum/b.count:0.0f;
            if(b.count>=10 && b.count*in.dt_s+1.0e-6f>=0.05f && fabsf(gyro)<0.20f && fabsf(accel)<1.5f
                && b.duty_max-b.duty_min<=10.0f) {
                b.gyro=gyro; b.accel=accel; b.age=0; b.ready=true;
            }
            b.clear_window();
        }
    }
};
} // namespace TurnPose

#endif
