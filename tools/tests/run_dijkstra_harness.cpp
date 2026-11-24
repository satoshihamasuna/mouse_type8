#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "Component/Inc/typedef.h"
#include "Subsys/Inc/make_path.h"
#include "Subsys/Src/calcRunTime.cpp"

struct Event { std::string name; std::vector<float> values; };
static std::vector<Event> events;
static void record(const char *name, std::initializer_list<float> values = {}) { events.push_back({name, values}); }
static void log_enable() { record("log.on"); }
static void log_disable() { record("log.off"); }
static void HAL_Delay(int ms) { record("delay", {float(ms)}); }
static void FAN_Motor_SetDuty(int duty) { record("fan", {float(duty)}); }
static bool cancel_requested() { record("cancel"); return true; }
static constexpr float SUCTION_ACC = 0.03f;
struct t_position { uint8_t x, y; };
enum t_direction { North };
struct Scalar { float value = 0; float get() const { return value; } };
struct TestVehicle { struct { Scalar velo; } ideal; };

class Motion {
public:
    TestVehicle vehicle;
    int fail_after = -1, moves = 0;
    t_exeStatus status = complete;
    TestVehicle *return_vehicleObj() { return &vehicle; }
    void Motion_start() { record("start"); vehicle.ideal.velo.value = 0; }
    void Motion_end() { record("end"); }
    void SetRunStartPosition(float length) { record("origin", {length}); }
    t_exeStatus motion_exeStatus_get() { return status; }
    void exe_Motion_suction_start(float voltage, int stay, const t_pid_gain *sp) {
        record("suction", {voltage, float(stay), sp->Kp});
    }
    void straight(const char *name, float length, float acc, float max, float end,
                  const t_pid_gain *sp, const t_pid_gain *om, const t_ff_gain *ff, float decel) {
        record(name, {length, acc, max, end, sp->Kp, sp->Ki, om->Kp, om->Ki, ff->sp_velo, decel});
        vehicle.ideal.velo.value = end;
        if (++moves == fail_after) status = error;
    }
    void exe_Motion_straight(float l, float a, float m, float e, const t_pid_gain *sp,
                            const t_pid_gain *om, const t_ff_gain *ff, float d) { straight("straight", l,a,m,e,sp,om,ff,d); }
    void exe_Motion_diagonal(float l, float a, float m, float e, const t_pid_gain *sp,
                            const t_pid_gain *om, const t_ff_gain *ff, float d) { straight("diagonal", l,a,m,e,sp,om,ff,d); }
    void turn(const char *name, const t_param *param, t_run_pattern pattern, float end,
              float acc, const t_pid_gain *sp, const t_pid_gain *om) {
        assert(param);
        record(name, {float(pattern), param->param->velo, param->param->r_min,
            param->param->Lstart, param->param->Lend, end, acc, sp->Kp, om->Kp});
        vehicle.ideal.velo.value = end;
        if (++moves == fail_after) status = error;
    }
#define TURN_METHOD(method, label) \
    void method(const t_param *p, t_run_pattern pattern, const t_pid_gain *sp, const t_pid_gain *om) { \
        turn(label ".fixed", p,pattern,p->param->velo,0,sp,om); } \
    void method(const t_param *p, t_run_pattern pattern, float end, float acc, const t_pid_gain *sp, const t_pid_gain *om) { \
        turn(label ".variable",p,pattern,end,acc,sp,om); }
    TURN_METHOD(exe_Motion_long_turn, "long")
    TURN_METHOD(exe_Motion_turn_in, "in")
    TURN_METHOD(exe_Motion_turn_out, "out")
    TURN_METHOD(exe_Motion_turn_v90, "v90")
    TURN_METHOD(exe_Motion_long_turn_v90, "longv90")
#undef TURN_METHOD
};
#include "run_support.inc"

static unsigned index_of(t_posDijkstra p) { return p.x + 32*p.y; }
static t_posDijkstra position(unsigned index) { return {uint16_t(index%32), uint16_t(index/32), C_pos, 0}; }
static t_direction dijkstra_pos_dir(t_posDijkstra) { return North; }
class Dijkstra : public calcRunTime {
    void run_Dijkstra_common(t_position, t_direction, t_position, uint8_t, const t_run_config *, Motion *, bool, uint8_t *);
public:
    std::vector<t_run_pattern> route;
    t_posDijkstra run_pos_buff[DIJKSTRA_PATH_MAX]{};
    uint8_t run_pt_buff[DIJKSTRA_PATH_MAX]{};
    bool (*abort_requested)() = nullptr;
    struct Node { t_posDijkstra position; } node;
    t_posDijkstra make_path_Dijkstra_priority_queue(t_position,t_direction,t_position,uint8_t) {
        record("plan"); return position(route.size());
    }
    t_posDijkstra last_expand(t_posDijkstra p,t_direction,t_position,uint8_t,t_bool) { record("expand"); return p; }
    t_posDijkstra conv_t_pos2t_posDijkstra(t_position,t_direction) { return position(0); }
    uint8_t get_run_pattern(t_posDijkstra p) { return route[route.size()-index_of(p)]; }
    Node *get_closure_inf(t_posDijkstra p) { node.position = p; return &node; }
    t_posDijkstra get_parent(const Node *p) { return position(index_of(p->position)-1); }
    uint16_t straight_section_num(t_posDijkstra,t_posDijkstra,t_direction) { return 3; }
    uint16_t diagonal_section_num(t_posDijkstra,t_posDijkstra,t_direction) { return 3; }
    void stage_diagonal_pose_route(Motion *,int current,int next) { record("route", {float(current),float(next)}); }
    void run_Dijkstra(t_position,t_direction,t_position,uint8_t,const t_run_config *,Motion *);
    void run_Dijkstra_suction(t_position,t_direction,t_position,uint8_t,const t_run_config *,Motion *);
    void run_Dijkstra_suction_acc(t_position,t_direction,t_position,uint8_t,const t_run_config *,Motion *);
};
#include "run_methods.inc"

static int event_index(const char *name) {
    for (unsigned i=0; i<events.size(); ++i) if (events[i].name==name) return i;
    return -1;
}
static int count(const char *name) { int n=0; for (const auto &e:events) n += e.name==name; return n; }
static void run_case(int mode, const t_run_config &config, const std::vector<t_run_pattern> &route,
                     const char *label, bool cancel=false, int fail_after=-1) {
    events.clear(); Dijkstra run; run.route=route; Motion motion;
    run.abort_requested = cancel ? cancel_requested : nullptr;
    motion.fail_after=fail_after;
    if (mode==0) run.run_Dijkstra({},North,{},1,&config,&motion);
    else if (mode==1) run.run_Dijkstra_suction({},North,{},1,&config,&motion);
    else run.run_Dijkstra_suction_acc({},North,{},1,&config,&motion);
    if (event_index("start")>=0) {
        assert(event_index("start")<event_index("origin"));
        if (mode) assert(event_index("start")<event_index("suction") && event_index("suction")<event_index("log.on"));
        else assert(event_index("log.on")<event_index("start"));
    }
    if (event_index("fan")>=0) {
        assert(count("fan")==1 && count("end")==1 && count("log.off")==1 && count("delay")==2);
        assert(events[events.size()-4].name=="end" && events[events.size()-3].name=="delay");
        assert(events[events.size()-2].name=="fan" && events.back().name=="delay");
    }
    if (cancel) {
        assert(event_index("start")==-1 && event_index("origin")==-1 && count("suction")==0);
        assert(count("log.on")==int(mode==0));
    }
    if (fail_after>0) assert(motion.moves==fail_after && count("end")==1);
    for (const auto &e:events) {
        std::printf("%s/%d/%s %s",config.key,mode,label,e.name.c_str());
        for (float value:e.values) std::printf(" %.6f",double(value));
        std::puts("");
    }
}

int main(int argc,char **argv) {
    std::setvbuf(stdout,nullptr,_IONBF,0);
    assert(argc==2);
    const std::string test=argv[1];
    const std::vector<t_run_pattern> mixed = {Straight,Turn_out_R45,Diagonal,Turn_RV90,
        Diagonal,Turn_in_L135,Straight,Long_turnL180,Straight}; // stored goal -> start
    if (test=="profiles") {
        for (const auto *config:run_config_table) run_case(config->type,*config,mixed,"mixed");
    } else if (test=="turns") {
        for (int mode=0; mode<3; ++mode) for (int p=Long_turnR90; p<=Turn_LV90; ++p) {
            run_case(mode,run_config_plain500,{Straight,t_run_pattern(p),Straight},"turn");
            assert(count("straight")==2 && count("end")==1);
        }
    } else if (test=="combined" || test=="combined-missing") {
        const t_param *low[32]={}, *high[32]={};
        for (int p=Long_turnR90;p<=Turn_LV90;++p) {
            low[p]=run_config_plain500.turn_mode[0][p]; high[p]=run_config_uniform1000.turn_mode[0][p];
        }
        low[Long_turn_RV90]=low[Turn_RV90]; low[Long_turn_LV90]=low[Turn_LV90];
        high[Long_turn_RV90]=high[Turn_RV90]; high[Long_turn_LV90]=high[Turn_LV90];
        if (test=="combined-missing") low[Long_turn_RV90]=low[Long_turn_LV90]=high[Long_turn_RV90]=high[Long_turn_LV90]=nullptr;
        const t_param *const *tables[]={low,high};
        auto config=run_config_acc1600_v1; config.turn_mode=tables; config.turn_mode_size=2;
        run_case(2,config,{Straight,Turn_in_R45,Turn_out_R45,Diagonal,Turn_in_L45,Turn_out_L45,Straight},test.c_str());
        if (test=="combined") assert(count("longv90.variable")==2);
        else assert(count("longv90.variable")==0 && count("in.variable")==2 && count("out.variable")==2);
    } else if (test=="initial-selection") {
        const t_param *low[32]={}, *high[32]={};
        for (int p=Long_turnR90;p<=Turn_LV90;++p) {
            low[p]=run_config_plain500.turn_mode[0][p]; high[p]=run_config_uniform1000.turn_mode[0][p];
        }
        const t_param *const *tables[]={low,high};
        auto config=run_config_plain500; config.turn_mode=tables; config.turn_mode_size=2;
        for (int mode=0;mode<3;++mode) {
            high[Long_turnL90]=mode==2 ? nullptr : run_config_uniform1000.turn_mode[0][Long_turnL90];
            for (auto p:{Long_turnR90,Long_turnL90,Turn_in_R45,Turn_in_L45,Turn_in_R135,Turn_in_L135}) {
                run_case(mode,config,{Long_turnL90,p},"first");
                const bool first_right=p==Long_turnR90 || p==Turn_in_R45 || p==Turn_in_R135;
                const t_param *expected=mode==2 && (first_right || !high[p]) ? low[p] : high[p];
                bool found=false;
                for (const auto &event:events) {
                    if (event.name=="long.variable" || event.name=="in.variable") {
                        assert(event.values[0]==p && event.values[1]==expected->param->velo);
                        assert(event.values[2]==expected->param->r_min);
                        found=true; break;
                    }
                }
                assert(found);
            }
        }
    } else if (test=="cancel") {
        for (int mode=0;mode<3;++mode) run_case(mode,run_config_plain500,mixed,"cancel",true);
    } else if (test=="invalid") {
        auto config=run_config_plain500; config.straight_speed=nullptr;
        for (int mode=0;mode<3;++mode) { run_case(mode,config,mixed,"invalid"); assert(events.empty()); }
    } else if (test=="unreachable") {
        t_speed_profile slow={.5f,.01f,.01f,0,.5f};
        auto config=run_config_plain500; config.straight_speed=&slow; config.diagonal_speed=&slow;
        for (int mode=0;mode<3;++mode) for (auto p:{Straight,Diagonal}) {
            run_case(mode,config,{Long_turnR90,p},"unreachable");
            assert(count("end")==1 && count("log.off")==1 && count("fan")==0);
        }
    } else if (test=="motion-error" || test=="initial-error") {
        for (int mode=0;mode<3;++mode) run_case(mode,run_config_plain500,
            test=="initial-error" ? std::vector<t_run_pattern>{Straight,Long_turnR90} : mixed,test.c_str(),false,1);
    } else if (test=="long-route") {
        for (int mode=0;mode<3;++mode) run_case(mode,run_config_plain500,
            std::vector<t_run_pattern>(DIJKSTRA_PATH_MAX,Straight),"long");
        assert(count("straight")==DIJKSTRA_PATH_MAX);
    } else assert(false);
}
