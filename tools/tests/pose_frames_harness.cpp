#include "Module/Inc/vehicle.h"
#include "Task/Inc/turn_pose_ekf.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <initializer_list>

#undef assert
#define assert(condition) do { if (!(condition)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#condition); exit(1); } } while(false)

using namespace PoseFrames;
static bool close(float a,float b,float tolerance=.001f) { return fabsf(a-b)<tolerance; }
static void same(const Pose &a,const Pose &b,float tolerance=.001f) {
    assert(close(a.x_mm,b.x_mm,tolerance) && close(a.y_mm,b.y_mm,tolerance));
    assert(fabsf(wrap(a.yaw_rad-b.yaw_rad))<tolerance);
}
static uint32_t irq_mask;
static uint32_t __get_PRIMASK() { return irq_mask; }
static void __disable_irq() { irq_mask=1; }
static void __set_PRIMASK(uint32_t value) { irq_mask=value; }
static int ntlibc_strcmp(const char *a,const char *b) { return strcmp(a,b); }
class CtrlTask_type8 {
public:
    Vehicle storage;
    Vehicle *vehicle=&storage;
    bool busy=false; t_exeStatus status=complete;
    static CtrlTask_type8 &getInstance() { static CtrlTask_type8 ctrl; return ctrl; }
    bool is_control_enable() const { return busy; }
    t_exeStatus motion_exeStatus_get() const { return status; }
    Vehicle *return_vehicleObj() { return vehicle; }
};
#include "pose_shell.inc"

class MotionState {
public:
    Vehicle *vehicle;
    explicit MotionState(Vehicle *owner):vehicle(owner) {}
    t_runControl motion_control=NOP_STATE;
    t_exeStatus motion_exeStatus=complete;
#include "pose_setters.inc"
    void start_pose() {
#include "Motion_start_pose.inc"
    }
    void end_pose() {
#include "Motion_end_pose.inc"
    }
    void error_pose() {
#include "Motion_error_handling_pose.inc"
    }
};

static void geometry() {
    constexpr float pi=3.14159265358979323846f;
    for(int i=-8;i<=8;++i) {
        const Pose origin={37,-91,i*pi/4},local={123,-42,.73f};
        same(relative(origin,compose(origin,local)),local);
        const Pose world={-18,79,-2.4f};
        same(compose(origin,relative(origin,world)),world);
    }
    float p[3][3]={{4,1,.2f},{1,9,-.3f},{.2f,-.3f,.04f}},rotated[3][3],back[3][3];
    rotate_covariance(p,pi/2,rotated);
    assert(close(rotated[0][0],9) && close(rotated[1][1],4) && close(rotated[0][1],-1));
    assert(close(rotated[0][2],.3f) && close(rotated[1][2],.2f));
    rotate_covariance(rotated,-pi/2,back);
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) assert(close(p[i][j],back[i][j]));
    rotate_covariance(p,pi/2,p); // In-place conversion must also preserve cross terms.
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) assert(close(p[i][j],rotated[i][j]));
}

static void route() {
    constexpr float pi=3.14159265358979323846f;
    Tracker frames;
    assert(frames.set_start({45,45,pi/2})); frames.start();
    for(unsigned edge=0;edge<4;++edge) {
        frames.new_motion();frames.set_phase(STRAIGHT_STATE);
        same(frames.snapshot().local,{});
        for(unsigned j=0;j<90;++j) frames.predict_body(1,0,0);
        const Pose before=frames.snapshot().world;
        frames.new_motion();frames.set_phase(PIVTURN_STATE);
        same(before,frames.snapshot().world);
        for(unsigned j=0;j<100;++j) frames.predict_body(0,0,pi/200);
        const Pose turned=frames.snapshot().world;
        frames.complete_turn(pi/2);frames.set_phase(STRAIGHT_STATE);
        same(turned,frames.snapshot().world);
        const auto state=frames.snapshot();same(compose(state.origin,state.local),state.world);
    }
    same(frames.snapshot().world,{45,45,pi/2},.004f);
    // 45-degree diagonal, a right 135-degree exit, and reverse after reframe.
    frames.predict_body(0,0,-pi/4);frames.complete_turn(-pi/4);frames.set_phase(DIAGONAL_STATE);
    frames.predict_body(10,0,0);
    const auto diagonal=frames.snapshot();assert(close(diagonal.local.x_mm,10));
    for(unsigned i=0;i<200;++i) frames.predict_body(0,0,-3*pi/800);
    frames.complete_turn(-3*pi/4);frames.set_phase(STRAIGHT_STATE);
    const auto turn=frames.snapshot();same(compose(turn.origin,turn.local),turn.world);
    frames.predict_body(-7,0,0);assert(close(frames.snapshot().local.x_mm,-7));
    const auto valid=frames.snapshot();
    frames.predict_body(NAN,0,0);
    assert(!frames.snapshot().valid && frames.snapshot().rejected==1);same(frames.snapshot().world,valid.world);
    frames.predict_body(1,0,0);assert(!frames.snapshot().valid); // Missing travel stays visible.
}

static void lifecycle() {
    Vehicle storage;Vehicle *vehicle=&storage;
    auto &frames=vehicle->pose;assert(frames.set_start({12,-8,.4f}));
    MotionState motion(vehicle);motion.start_pose();
    motion.motion_exeStatus_set(execute);motion.motion_control_set(STRAIGHT_STATE);
    frames.predict_body(10,0,.02f);
    auto before=frames.snapshot();
    motion.motion_control_set(STRAIGHT_STATE);
    assert(frames.snapshot().frame==before.frame);same(frames.snapshot().local,before.local);
    motion.motion_exeStatus_set(complete);motion.motion_exeStatus_set(execute);
    motion.motion_control_set(STRAIGHT_STATE);
    auto next=frames.snapshot();assert(next.motion==2 && next.frame==before.frame+1);
    same(next.world,before.world);assert(close(next.local.x_mm,0) && close(next.local.y_mm,0));
    // A native EKF's replacement moves both coordinate views consistently.
    const Pose origin=frames.passage_origin({0,3,.02f});
    auto corrected=compose(origin,{7,4,.03f});frames.adopt_world(corrected);
    same(frames.snapshot().world,corrected);
    frames.correct_passage(2,-1);
    const auto state=frames.snapshot();same(compose(state.origin,state.local),state.world);
    assert(!frames.set_start({0,0,0}));
    motion.end_pose();
    const auto stopped=frames.snapshot();
    frames.predict_body(12,0,.3f);motion.motion_control_set(NOP_STATE);
    same(frames.snapshot().world,stopped.world);same(frames.snapshot().local,stopped.local);
    assert(!frames.snapshot().active);
    motion.start_pose();
    same(frames.snapshot().world,{12,-8,.4f});
    assert(frames.snapshot().samples==0);
    frames.predict_body(2,0,0);
    motion.error_pose();
    assert(!frames.snapshot().active);
}

static void irq() {
    Vehicle vehicle;
    auto &frames=vehicle.pose;
    for(float initial_yaw:{0.0f,1.23f}) {
        frames.stop();assert(frames.set_start({23,-7,initial_yaw}));frames.start();frames.align_passage_heading(.02f);
        TurnPose::Tracker tracker;
        TurnPose::Input in;in.dt_s=.001f;in.timing_valid=true;in.quiet_command=true;
        in.gyro_rad_s=.02f;in.lateral_accel_m_s2=.1f;
        for(unsigned i=0;i<120;++i)tracker.update(in);
        in.turning=true;in.quiet_command=false;in.pattern=Long_turnR90;
        in.speed_m_s=in.target_speed_m_s=.5f;in.gyro_rad_s=-.98f;in.lateral_accel_m_s2=-.4f;
        float enc_velo=.5f,rad_velo=in.gyro_rad_s;
        for(unsigned i=0;i<100;++i) {
            in.heading_before_rad=.7f-.00098f*i;
            in.heading_prior=i==0?MotionHeading::Prior{.02f,.000004f,true}:MotionHeading::Prior{};
            in.legacy_forward_mm=.5f*cosf(in.heading_before_rad-.00098f);
            in.legacy_left_mm=.5f*sinf(in.heading_before_rad-.00098f);
            const auto pose_output=tracker.update(in);
#include "pose_prediction.inc"
#if defined(MOUSE_A)
            assert(pose_output.applied);
            const auto estimate=tracker.snapshot();
            same(frames.snapshot().world,compose({23,-7,initial_yaw},{estimate.forward_mm,estimate.left_mm,estimate.heading_rad}),.002f);
#endif
            assert(frames.snapshot().samples==i+1);
        }
        const Pose before=frames.snapshot().world;
        frames.complete_turn(-.1f);frames.set_phase(STRAIGHT_STATE);same(frames.snapshot().world,before);
        // ego.radian can now reset to zero; the world increment needs no ego origin.
        const TurnPose::Output pose_output={};rad_velo=0;
        {
#include "pose_prediction.inc"
        }
        same(frames.snapshot().world,compose(before,{.5f,0,0}));
        assert(frames.snapshot().samples==101);
    }
}

static void ownership() {
    Vehicle first,second;
    const Vehicle &initial=first;
    assert(initial.ego.velo.get()==0 && initial.ideal.radian.get()==0 && initial.ego.turn_dir_state.get()==Turn_None);
    assert(first.pose.set_start({10,-20,.2f}));assert(second.pose.set_start({-60,30,-.4f}));
    MotionState a(&first),b(&second);a.start_pose();b.start_pose();
    a.motion_exeStatus_set(execute);a.motion_control_set(STRAIGHT_STATE);
    const auto unchanged=second.pose.snapshot();
    {
        Vehicle &vehicle=first;
        const TurnPose::Output pose_output={};float enc_velo=2,rad_velo=.1f;
#include "pose_prediction.inc"
    }
    assert(first.pose.snapshot().samples==1 && first.pose.snapshot().motion==1);
    same(second.pose.snapshot().world,unchanged.world);
    assert(second.pose.snapshot().samples==0 && second.pose.snapshot().frame==0);
    const auto first_before=first.pose.snapshot();
    b.motion_exeStatus_set(execute);b.motion_control_set(DIAGONAL_STATE);
    {
        Vehicle &vehicle=second;
        const TurnPose::Output pose_output={};float enc_velo=3,rad_velo=-.2f;
#include "pose_prediction.inc"
    }
    same(first.pose.snapshot().world,first_before.world);
    assert(first.pose.snapshot().samples==1 && second.pose.snapshot().samples==1);
    // The shared parameter initializers preserve the old reset scopes, and
    // resetting legacy control coordinates must leave both pose owners intact.
    first.ego.length.set(7);first.ego.x_point.set(8);first.ego.radian.set(.1f);
    first.ego.turn_slip_theta.set(.2f);
    first.ego.velo.set(.7f);first.ego.rad_velo.set(.3f);first.ego.rad_jerk.set(2);
    first.ideal.velo.set(.8f);first.ego_integral_init();
    assert(first.ego.length.get()==0 && first.ego.x_point.get()==0 && first.ego.radian.get()==0);
    assert(first.ego.turn_slip_theta.get()==0);
    assert(first.ego.velo.get()==.7f && first.ego.rad_velo.get()==.3f && first.ego.rad_jerk.get()==2);
    first.ego_initialize();assert(first.ego.velo.get()==0 && first.ego.rad_jerk.get()==0 && first.ideal.velo.get()==.8f);
    first.ideal.length.set(1);first.ideal_integral_init();assert(first.ideal.length.get()==0 && first.ideal.velo.get()==.8f);
    first.ideal_initialize();assert(first.ideal.velo.get()==0);
    same(first.pose.snapshot().world,first_before.world);
    const auto second_before=second.pose.snapshot();
    a.end_pose();assert(!first.pose.snapshot().active && second.pose.snapshot().active);
    a.start_pose();same(first.pose.snapshot().world,{10,-20,.2f});
    same(second.pose.snapshot().world,second_before.world);
    b.error_pose();assert(first.pose.snapshot().active && !second.pose.snapshot().active);
    // The shell follows the control task's injected Vehicle, not a global pose.
    auto &ctrl=CtrlTask_type8::getInstance();Vehicle *previous=ctrl.vehicle;ctrl.vehicle=&second;
    char pose[]="pose",reset[]="reset";char *args[]={pose,reset};
    assert(usrcmd_pose(2,args)==0);same(second.pose.snapshot().world,{});
    same(first.pose.snapshot().world,{10,-20,.2f});ctrl.vehicle=previous;
}


static void log_view() {
    auto &vehicle=Vehicle_type8::getInstance();auto &frames=vehicle.pose;
    frames.stop();assert(frames.set_start({10,20,1.2f}));frames.start();frames.set_phase(SLATURN_STATE);
    vehicle.ego_initialize();vehicle.ego.radian.set(.3f);vehicle.reset_turn_position_for_log();
    const Pose log_origin={10,20,.9f};
    frames.predict_body(10,0,0);
    auto read_log=[&]() {
        float sample[22]={};
        auto float_to_half=[](float value) { return value; };
#include "pose_log.inc"
        assert(sample[12]==0 && sample[14]==0); // Historical ideal position slots remain reserved zero.
        same({sample[15],sample[13],0},{frames.log_position().x_mm,frames.log_position().y_mm,0});
        return Pose{sample[15],sample[13],0};
    };
    const auto expected=relative(log_origin,frames.snapshot().world);
    same(read_log(),{expected.x_mm,expected.y_mm,0});
    assert(close(read_log().y_mm,10*sinf(.3f)) && close(read_log().x_mm,10*cosf(.3f)));
    const auto before=frames.snapshot();
    frames.set_phase(DIAGONAL_STATE); // The control frame changes independently of the log reference.
    same(frames.snapshot().world,before.world);same(read_log(),{expected.x_mm,expected.y_mm,0});
    assert(close(frames.snapshot().local.x_mm,0) && close(frames.snapshot().local.y_mm,0));
    frames.correct_passage(2,3); // Adopted wall corrections are visible without a second integral.
    const auto corrected=relative(log_origin,frames.snapshot().world);
    same(read_log(),{corrected.x_mm,corrected.y_mm,0});
    const auto at_reset=frames.snapshot();vehicle.reset_turn_position_for_log();
    same(read_log(),{});same(frames.snapshot().world,at_reset.world);same(frames.snapshot().local,at_reset.local);
    assert(frames.snapshot().samples==at_reset.samples);
    frames.predict_body(1,0,.01f);const auto after=frames.snapshot();vehicle.ego_integral_init();
    same(read_log(),{});same(frames.snapshot().world,after.world);same(frames.snapshot().local,after.local);
    frames.stop();vehicle.ego_initialize();same(read_log(),{});same(frames.snapshot().world,after.world);
}

static void shell() {
    auto &frames=CtrlTask_type8::getInstance().return_vehicleObj()->pose;frames.stop();
    char pose[]="pose",reset[]="reset",show[]="show",x[]="45",y[]="-9",angle[]="90",nan[]="nan",junk[]="4x";
    char *args[]={pose,reset,x,y,angle};
    for(uint32_t mask=0;mask<=1;++mask) {
        irq_mask=mask;assert(usrcmd_pose(5,args)==0 && irq_mask==mask);
        same(frames.snapshot().world,{45,-9,1.57079632679f});
        frames.start();same(frames.snapshot().world,{45,-9,1.57079632679f});frames.stop();
        CtrlTask_type8::getInstance().busy=true;
        assert(usrcmd_pose(2,args)==-1 && irq_mask==mask);
        CtrlTask_type8::getInstance().busy=false;CtrlTask_type8::getInstance().status=execute;
        assert(usrcmd_pose(5,args)==-1 && irq_mask==mask);
        CtrlTask_type8::getInstance().status=complete;
        for(char *bad:{nan,junk}) {args[2]=bad;assert(usrcmd_pose(5,args)==-1 && irq_mask==mask);}
        args[2]=x;args[1]=show;
        assert(usrcmd_pose(2,args)==0 && irq_mask==mask);
        assert(usrcmd_pose(5,args)==-1);
        args[1]=reset;
    }
    assert(usrcmd_pose(2,args)==0);same(frames.snapshot().world,{});
}

int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"geometry"))geometry();
    else if(!strcmp(argv[1],"route"))route();
    else if(!strcmp(argv[1],"lifecycle"))lifecycle();
    else if(!strcmp(argv[1],"ownership"))ownership();
    else if(!strcmp(argv[1],"log-view"))log_view();
    else if(!strcmp(argv[1],"irq"))irq();
    else if(!strcmp(argv[1],"shell"))shell();
    else return 2;
    return 0;
}
