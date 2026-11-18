#include "Task/Inc/diagonal_pose.h"

namespace DiagonalPose {

void Filter::start(float s, float heading, const Route &route, float heading_variance_rad2)
{
    if (!route.count || route.count>Route::capacity || !isfinite(s) || !isfinite(heading)) {
        stop(); return;
    }
    route_=route;
    odom_={0,0,heading};
    x_[0]=s; x_[1]=0; x_[2]=heading; x_[3]=0;
    for (auto &row:p_) for (float &v:row) v=0;
    p_[0][0]=initial_progress_std_mm*initial_progress_std_mm;
    p_[1][1]=15.0f*15.0f;
    p_[2][2]=0.13962634f*0.13962634f; // 8 deg
    if (isfinite(heading_variance_rad2) && heading_variance_rad2>0) p_[2][2]=heading_variance_rad2;
    p_[3][3]=10.0f*10.0f;
    consumed_[0]=consumed_[1]=mask_=used_=0;
    ticks_since_update_=0;
    travel_since_update_mm_=0;
    bridging_=inherited_=false;
    active_=true;
}

bool Filter::begin_bridge(const Route &next)
{
    Pose pose;
    if (bridging_ || !next.count || next.count>Route::capacity
        || !route_.has_frame() || !next.has_frame() || route_.same_frame(next)
        || !control_pose(pose)) return false;
    bridge_ticks_=0;
    bridge_distance_mm_=0;
    bridging_=true;
    return true;
}

bool Filter::finish_bridge(float nominal_s, const Route &next)
{
    if (!bridging_ || !next.count || next.count>Route::capacity || !next.has_frame()) return false;

    // Rotate and translate from the previous diagonal into the next one.
    // Diagonal unit vectors differ by multiples of 90 degrees.
    const float c=0.5f*(route_.dx*next.dx+route_.dy*next.dy);
    const float t=0.5f*(route_.dx*next.dy-route_.dy*next.dx);
    constexpr float half_cell_projection_mm=static_cast<float>(DIAG_SECTION)*0.5f;
    const int offset_x=route_.origin_x-next.origin_x, offset_y=route_.origin_y-next.origin_y;
    const float s=c*x_[0]+t*x_[1]+half_cell_projection_mm*(next.dx*offset_x+next.dy*offset_y);
    const float e=-t*x_[0]+c*x_[1]+half_cell_projection_mm*(-next.dy*offset_x+next.dx*offset_y);
    const float angle=x_[2]-atan2f(t,c);
    const float heading=atan2f(sinf(angle),cosf(angle));
    if (!isfinite(s) || !isfinite(e) || !isfinite(heading) || !isfinite(nominal_s)
        || fabsf(s-nominal_s)>association_window_mm || fabsf(e)>0.5f*DIAG_SECTION
        || fabsf(heading)>0.52359878f) return false;

    const float ss=p_[0][0], ee=p_[1][1], se=p_[0][1];
    p_[0][0]=c*c*ss+2*c*t*se+t*t*ee;
    p_[1][1]=t*t*ss-2*c*t*se+c*c*ee;
    p_[0][1]=p_[1][0]=(c*c-t*t)*se+c*t*(ee-ss);
    // Angle and common optical bias keep their existing covariance,
    // including cross-correlations with both transformed coordinates.
    for (unsigned i=2;i<size;++i) {
        const float ps=p_[0][i], pe=p_[1][i];
        p_[0][i]=p_[i][0]=c*ps+t*pe;
        p_[1][i]=p_[i][1]=-t*ps+c*pe;
    }
    x_[0]=s; x_[1]=e; x_[2]=heading;
    odom_={0,0,heading};
    route_=next;
    consumed_[0]=consumed_[1]=mask_=used_=0;
    bridging_=false;
    inherited_=true;
    return true;
}

void Filter::advance(float distance, float yaw_delta, bool turning)
{
    if (!active_) return;
    if (!isfinite(distance) || distance<0 || distance>6 || !isfinite(yaw_delta)) {
        stop(); return;
    }
    if (bridging_) {
        bridge_distance_mm_+=distance;
        if (++bridge_ticks_>=1000U || bridge_distance_mm_>=6.0f*DIAG_SECTION) {
            stop(); return;
        }
    }
    const float mid=x_[2]+0.5f*yaw_delta;
    if (ticks_since_update_<220U) ++ticks_since_update_;
    travel_since_update_mm_=fminf(2.0f*static_cast<float>(DIAG_SECTION),travel_since_update_mm_+distance);
    const float ds=distance*cosf(mid), de=distance*sinf(mid);
    x_[0]+=ds; x_[1]+=de; x_[2]+=yaw_delta;
    const float odom_mid=odom_.heading+0.5f*yaw_delta;
    odom_.s+=distance*cosf(odom_mid); odom_.e+=distance*sinf(odom_mid);
    odom_.heading+=yaw_delta;

    // F P F^T for F=[I, (-de,ds,0,0) in the heading column].
    const float tangent[size]={-de,ds,0,0};
    float heading_cov[size];
    for (unsigned i=0;i<size;++i) heading_cov[i]=p_[i][2];
    for (unsigned i=0;i<2;++i) {
        for (unsigned j=i;j<size;++j) {
            p_[i][j]+=tangent[i]*heading_cov[j]+tangent[j]*heading_cov[i]
                +tangent[i]*tangent[j]*heading_cov[2];
            p_[j][i]=p_[i][j];
        }
    }
    p_[0][0]+=0.01f*distance; p_[1][1]+=0.0225f*distance;
    p_[2][2]+=2.741557e-7f*distance;
    if (turning) {
        // Wheel travel during slalom has additional slip and yaw uncertainty.
        // Scale it with measured rotation, independent of the turn profile.
        const float rotation=fabsf(yaw_delta);
        p_[0][0]+=4.0f*rotation;
        p_[1][1]+=4.0f*rotation;
        p_[2][2]+=0.00030461742f*rotation;
    }
}

Result Filter::observe(unsigned side, const Pose &peak, float center_shift_mm,
    float width_mm, bool saturated, float gyro_radps, const Parameters &params)
{
    Result out;
    if (!active_) return out;
    out.status=Uncalibrated;
    if (!params.calibrated) return out;
    out.status=Invalid;
    if (side>1 || !isfinite(peak.s) || !isfinite(peak.e) || !isfinite(peak.heading)
        || !isfinite(center_shift_mm) || !isfinite(width_mm) || width_mm<0
        || !isfinite(gyro_radps) || (saturated && width_mm<=0)) return out;

    // Evaluate the peak at its own pose, expressed as a rigid odometry offset
    // from the current pose. The Jacobian therefore updates the current pose
    // with the full lever arm. Timing noise covers the short confirmation delay.
    const float peak_s=peak.s+center_shift_mm*cosf(peak.heading);
    const float peak_e=peak.e+center_shift_mm*sinf(peak.heading);
    const float correction=x_[2]-odom_.heading;
    const float c=cosf(correction), t=sinf(correction);
    const float offset_s=c*(peak_s-odom_.s)-t*(peak_e-odom_.e);
    const float offset_e=t*(peak_s-odom_.s)+c*(peak_e-odom_.e);
    const float heading=peak.heading+correction;
    const float ch=cosf(heading), sh=sinf(heading);
    const float signed_cot=side==0 ? params.cot_angle : -params.cot_angle;
    const float h_s=-ch-signed_cot*sh, h_e=-sh+signed_cot*ch;
    float best=9.0f, second=INFINITY, best_gain[size]={}, best_innovation=0, best_total=0;
    int index=-1;
    for (unsigned i=consumed_[side];i<route_.count;++i) {
        const unsigned pillar=route_.pillars[i];
        if ((pillar&Route::unknown) || (pillar&1U)!=side) continue;
        const float corner_s=(i+0.5f)*static_cast<float>(DIAG_SECTION);
        const float corner_e=(side==0?0.5f:-0.5f)*static_cast<float>(DIAG_SECTION);
        const float ds=corner_s-x_[0]-offset_s, de=corner_e-x_[1]-offset_e;
        const auto &model=params.pillar[side][pillar>>1];
        const float residual=-h_s*ds-h_e*de-model.offset_mm-x_[3];
        const float half_width=saturated ? 0.5f*width_mm+model.sigma_mm : 0;
        // Nearby same-side corners are normally two diagonal steps apart.
        // Limit the geometric residual before ranking by covariance and NIS.
        if (fabsf(residual)>association_window_mm+half_width) continue;
        const float innovation=-copysignf(fmaxf(0,fabsf(residual)-half_width),residual);
        const float timing=gyro_radps*0.0025f;
        const float noise=model.sigma_mm*model.sigma_mm+timing*timing*(ds*ds+de*de)
            +width_mm*width_mm*(saturated ? 0.25f : 1.0f/12.0f);
        const float h[size]={h_s,h_e,h_e*(corner_s-x_[0])-h_s*(corner_e-x_[1]),-1};
        float ph[size]={};
        float total=noise;
        for (unsigned j=0;j<size;++j) {
            for (unsigned k=0;k<size;++k) ph[j]+=p_[j][k]*h[k];
            total+=h[j]*ph[j];
        }
        if (!(total>0) || !isfinite(total)) continue;
        const float nis=innovation*innovation/total;
        if (!(nis<=9)) continue;
        if (nis<=best) {
            second=index>=0 ? best : second;
            best=nis; best_innovation=innovation; best_total=total; index=static_cast<int>(i);
            for (unsigned j=0;j<size;++j) best_gain[j]=ph[j];
        } else if (nis<second) second=nis;
    }

    if (index<0) out.status=NoMatch;
    else if (second-best<2.0f) out.status=Ambiguous;
    else {
        consumed_[side]=index+1U;
        out.index=index+1U;
        out.nis=best; out.innovation=best_innovation;
        if (saturated && best_innovation==0) out.status=Interval;
        else {
            out.status=Updated;
            for (unsigned i=0;i<size;++i) {
                x_[i]+=best_gain[i]*best_innovation/best_total;
                for (unsigned j=i;j<size;++j) {
                    p_[i][j]-=best_gain[i]*best_gain[j]/best_total;
                    p_[j][i]=p_[i][j];
                }
            }
            mask_|=1U<<side;
            ++used_;
            ticks_since_update_=0;
            travel_since_update_mm_=0;
        }
    }
    out.pose={x_[0],x_[1],x_[2]}; out.used=used_;
    out.sigma_e=sqrtf(fmaxf(p_[1][1],0)); out.sigma_s=sqrtf(fmaxf(p_[0][0],0));
    out.sigma_heading=sqrtf(fmaxf(p_[2][2],0));
    out.lateral_heading_ready=pose_ready();
    return out;
}

bool Filter::pose_ready() const
{
    constexpr float heading_std_limit=0.05235988f;
    return active_ && used_>=(inherited_ ? 2U : 3U) && mask_==3
        && p_[1][1]>=0 && p_[1][1]<=36 && p_[2][2]>=0 && p_[2][2]<=heading_std_limit*heading_std_limit
        && fabsf(x_[1])<=40 && fabsf(x_[2])<=0.52359878f;
}

bool Filter::control_pose(Pose &out) const
{
    // Do not use an old pose indefinitely after losing the pillars. These
    // bounds are common to all speeds and routes; the distance is 1.5 cells.
    if (!pose_ready() || ticks_since_update_>=220U || travel_since_update_mm_>=1.5f*DIAG_SECTION
        || fabsf(x_[1])>15.0f || fabsf(x_[2])>0.13962634f) return false;
    out={x_[0],x_[1],x_[2]};
    return true;
}

bool Filter::heading_for_turn(float &heading_rad, float &variance_rad2) const
{
    Pose pose;
    if (!control_pose(pose)) return false;
    heading_rad=pose.heading;
    variance_rad2=p_[2][2];
    return true;
}

Steering::Command Steering::update(const Pose *pose, float speed_mps, float travel_mm, float dt_s,
    float previous_omega, const Command &fallback)
{
    if (!isfinite(dt_s) || dt_s<=0 || !isfinite(previous_omega) || !isfinite(fallback.omega_radps)) {
        reset(); return fallback;
    }
    const bool use_pose=pose && isfinite(speed_mps) && speed_mps>0.20f
        && isfinite(pose->e) && isfinite(pose->heading);
    const float step=dt_s/0.050f;
    blend_=use_pose ? fminf(1.0f,blend_+fminf(step,fmaxf(0,travel_mm)/25.0f))
                     : fmaxf(0,blend_-step);
    if (!engaged_ && blend_==0) return fallback;
    constexpr float lookahead_m=0.120f;
    float pose_omega=0;
    if (use_pose) {
        const float limit=fminf(0.9f,0.55f/speed_mps);
        const float demand=-2.0f*speed_mps/lookahead_m*(pose->e*0.001f/lookahead_m+pose->heading);
        pose_omega=fminf(limit,fmaxf(-limit,demand));
    }
    const float requested=use_pose ? (1.0f-blend_)*fallback.omega_radps+blend_*pose_omega : fallback.omega_radps;
    const float max_delta=18.0f*dt_s;
    const float start=engaged_ ? last_omega_ : previous_omega;
    const float delta=requested-start;
    const float bounded=fminf(max_delta,fmaxf(-max_delta,delta));
    last_omega_=start+bounded;
    engaged_=blend_>0 || fabsf(delta)>max_delta;
    return {last_omega_,bounded/dt_s};
}

} // namespace DiagonalPose
