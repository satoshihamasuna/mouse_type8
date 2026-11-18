#include "Task/Inc/turn_pose_ekf.h"
#include <assert.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <string.h>

using namespace TurnPose;

// Compile the real shell handler with only the hardware access guarded here.
static uint32_t irq_mask;
static uint32_t __get_PRIMASK() { return irq_mask; }
static void __disable_irq() { irq_mask=1; }
static void __set_PRIMASK(uint32_t value) { irq_mask=value; }
static int ntlibc_strcmp(const char *a,const char *b) { return strcmp(a,b); }
class CtrlTask_type8 {
public:
    bool busy=false;int status=complete;
    static CtrlTask_type8 &getInstance() { static CtrlTask_type8 x;return x; }
    int is_control_enable() const { return busy?1:0; }
    int motion_exeStatus_get() const { return status; }
};
class Interrupt {
public:
    static Interrupt &getInstance() { static Interrupt x;return x; }
    Tracker &turn_pose() { return tracker_; }
private:Tracker tracker_;
};
#define True 1
#include "turn_pose_shell.inc"
#undef True

static void shell_checks() {
    char command[]="turn_ekf", shadow[]="shadow", control[]="control", reset[]="reset", bad[]="bad", show[]="show";
    char *args[]={command,shadow};
    for(uint32_t prior=0;prior<=1;++prior) {
        irq_mask=prior;
        assert(usrcmd_turn_ekf(2,args)==0);
        assert(irq_mask==prior && Interrupt::getInstance().turn_pose().mode()==Mode::Shadow);
        CtrlTask_type8::getInstance().busy=true;args[1]=control;
        assert(usrcmd_turn_ekf(2,args)==-1);
        assert(irq_mask==prior && Interrupt::getInstance().turn_pose().mode()==Mode::Shadow);
        CtrlTask_type8::getInstance().busy=false;CtrlTask_type8::getInstance().status=execute;
        assert(usrcmd_turn_ekf(2,args)==-1);
        CtrlTask_type8::getInstance().status=complete;
        assert(usrcmd_turn_ekf(2,args)==0 && irq_mask==prior);
        args[1]=reset;assert(usrcmd_turn_ekf(2,args)==0 && irq_mask==prior);
        args[1]=bad;assert(usrcmd_turn_ekf(2,args)==-1);
        args[1]=show;assert(usrcmd_turn_ekf(2,args)==0 && irq_mask==prior);
        args[1]=shadow;
    }
}

static Input quiet(bool suction=false) {
    Input in;
    in.dt_s=0.001f;in.timing_valid=true;in.quiet_command=true;
    in.gyro_rad_s=0.02f;in.lateral_accel_m_s2=0.1f;
    in.suction=suction;in.suction_duty=suction?650.0f:0.0f;
    return in;
}
static void calibrate(Tracker &tracker,bool suction=false) {
    for(int i=0;i<120;++i) assert(!tracker.update(quiet(suction)).applied);
}
static Input turning(bool suction=false) {
    auto in=quiet(suction);
    in.turning=true;in.quiet_command=false;in.pattern=Long_turnR90;
    in.target_speed_m_s=in.speed_m_s=suction?1.6f:0.5f;
    in.gyro_rad_s=-9.98f;in.lateral_accel_m_s2=-10.0f*in.speed_m_s+0.1f;
    in.legacy_forward_mm=in.speed_m_s;
    return in;
}

static void heading_handoff_checks() {
    Parameters p={true,100.0f,1.0f,0.04f,0.02f,0.3f,0.5f};
    Filter filter;
    filter.start(.03f,0,0,p,.000004f);
    assert(fabsf(filter.variance(2)-.000004f)<1e-10f);
    Tracker first,rotated;
    calibrate(first);calibrate(rotated);
    auto a=turning(),b=a;
    for(int i=0;i<80;++i) {
        a.heading_before_rad=b.heading_before_rad=.7f-.01f*i;
        a.heading_prior=i==0?MotionHeading::Prior{.01f,.000004f,true}:MotionHeading::Prior{};
        b.heading_prior=i==0?MotionHeading::Prior{.05f,.000004f,true}:MotionHeading::Prior{};
        a.legacy_left_mm=b.legacy_left_mm=.5f*sinf(a.heading_before_rad-.01f);
        a.legacy_forward_mm=b.legacy_forward_mm=.5f*cosf(a.heading_before_rad-.01f);
        const auto x=first.update(a),y=rotated.update(b);
        assert(x.applied==y.applied);
        if(x.applied) {
            // A different passage yaw origin cannot rotate legacy turn_x/y.
            assert(fabsf(x.left_mm-y.left_mm)<.0001f);
            assert(fabsf(x.forward_mm-y.forward_mm)<.0001f);
        }
    }
#if defined(MOUSE_A)
    const auto x=first.heading_for_handoff(),y=rotated.heading_for_handoff();
    assert(x.usable() && y.usable());
    assert(fabsf(y.heading_rad-x.heading_rad-.04f)<.00001f);
    assert(x.variance_rad2>.000004f && x.variance_rad2<.0001f);
    a.timing_valid=false;first.update(a);
    assert(!first.heading_for_handoff().usable());
    rotated.set_mode(Mode::Shadow);
    assert(!rotated.heading_for_handoff().usable());
#else
    assert(!first.heading_for_handoff().usable() && !rotated.heading_for_handoff().usable());
#endif
    first.reset();calibrate(first);a=turning();
    first.update(a);
    assert(!first.heading_for_handoff().usable()); // No invented incoming frame.
    a.heading_prior={.02f,NAN,true};
    first.update(quiet());first.update(a);
    assert(!first.heading_for_handoff().usable());
}

static void checks() {
    shell_checks();
    heading_handoff_checks();
    Parameters p={true,100.0f,1.0f,0.04f,0.02f,0.3f,0.5f};
    Filter filter;
    filter.start(0,0.02f,0.1f,p);
    for(int i=0;i<500;++i) {
        assert(filter.predict(.5f,10.02f,.001f,p));
        if(i%20==0) { float nis=0;assert(filter.observe(5.1f,p,nis)); }
    }
    assert(fabsf(filter.state(3)+.05f)<.003f);
    assert(filter.state(0)>0 && filter.state(1)>0 && filter.finite());
    for(unsigned i=0;i<6;++i) assert(filter.variance(i)>=0);

    Tracker tracker;
    calibrate(tracker);
    auto in=turning();
    unsigned used=0;
    for(int i=0;i<100;++i) {
        in.heading_before_rad=-.01f*i;
        in.legacy_left_mm=in.speed_m_s*sinf(in.heading_before_rad-.01f);
        in.legacy_forward_mm=in.speed_m_s*cosf(in.heading_before_rad-.01f);
        if(tracker.update(in).applied) ++used;
    }
#if defined(MOUSE_A)
    assert(used>70);
    assert(tracker.snapshot().updates<=6);
    assert(tracker.snapshot().applied_ticks==used);
    assert(tracker.snapshot().slip_rad>0);
#else
    assert(used==0);
    assert(tracker.snapshot().reason==Reason::Uncalibrated);
#endif
    tracker.set_mode(Mode::Shadow);
    assert(!tracker.update(in).applied);
    tracker.set_mode(Mode::Legacy);
    assert(!tracker.update(in).applied);
    tracker.set_mode(Mode::Control);
    tracker.update(quiet());
    in=turning();in.pattern=Long_turnR180;
    for(int i=0;i<20;++i) assert(!tracker.update(in).applied); // known worse without suction
    tracker.update(quiet());
    in=turning();in.pattern=Long_turnL90;
    assert(!tracker.update(in).applied); // no left-turn measurement yet
    tracker.update(quiet());
    in=turning();in.dt_s=0.1f;
    assert(!tracker.update(in).applied);
    in=turning();
    assert(!tracker.update(in).applied); // invalid timing aborts the whole turn
    tracker.update(quiet());
    tracker.reset();
    assert(!tracker.update(turning()).applied); // no startup calibration
    tracker.update(quiet());calibrate(tracker);
    in=turning();in.lateral_accel_m_s2=100;
    assert(!tracker.update(in).applied);
#if defined(MOUSE_A)
    assert(tracker.snapshot().reason==Reason::Innovation);
#endif
    tracker.update(quiet());
    in=turning();in.gyro_rad_s=NAN;
    assert(!tracker.update(in).applied);
    tracker.update(quiet());calibrate(tracker,true);
    in=turning(true);
    const auto first=tracker.update(in);
#if defined(MOUSE_A)
    assert(first.applied);
#else
    assert(!first.applied);
#endif
    in.suction=false;
    assert(!tracker.update(in).applied);
    in.suction=true;
    assert(!tracker.update(in).applied); // no mid-turn profile restart
    tracker.update(quiet(true));
    in=turning(true);in.suction_duty=500;
    assert(!tracker.update(in).applied);
    std::cout<<"turn pose checks passed; tracker bytes="<<sizeof(Tracker)<<"\n";
}

int main(int argc,char **argv) {
    if(argc==1) { checks();return 0; }
    if(argc!=3) return 2;
    std::ifstream file(argv[1]);std::ofstream out(argv[2]);
    if(!file || !out) return 3;
    Tracker tracker;
    std::string line;
    out<<"left_mm,forward_mm,applied,reason,beta_rad,updates,bias_ready\n";
    while(std::getline(file,line)) {
        std::istringstream row(line);
        Input in;int pattern,turn,suction,timing,quiet_flag;
        row>>in.dt_s>>in.speed_m_s>>in.wheel_speed_m_s>>in.target_speed_m_s>>in.gyro_rad_s
            >>in.lateral_accel_m_s2>>in.forward_accel_m_s2>>in.heading_before_rad
            >>in.legacy_left_mm>>in.legacy_forward_mm>>in.suction_duty
            >>pattern>>turn>>suction>>timing>>quiet_flag;
        if(!row) return 4;
        in.pattern=static_cast<uint8_t>(pattern);in.turning=turn!=0;in.suction=suction!=0;
        in.timing_valid=timing!=0;in.quiet_command=quiet_flag!=0;
        auto result=tracker.update(in);auto status=tracker.snapshot();
        out<<result.left_mm<<","<<result.forward_mm<<","<<result.applied<<","<<static_cast<unsigned>(status.reason)
            <<","<<status.slip_rad<<","<<status.updates<<","<<status.bias_ready<<"\n";
    }
    return 0;
}
