"""実際の探索ターン初期化・機体別Paramsで前後PIDの選択を検証する。"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command, parameter_sources

ROOT = Path(__file__).resolve().parents[2]


def function(text, signature):
    start = text.index(signature)
    opening = text.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


class SearchTurnParamsTests(unittest.TestCase):
    def test_real_initializer_uses_scoped_params_and_explicit_debug_overrides(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        implementation = function((ROOT/'Task/Src/init_motion.cpp').read_text(encoding='utf-8'),
                                  'void Motion::Init_Motion_search_turn')
        header = (ROOT/'Task/Inc/ctrl_task.h').read_text(encoding='utf-8')
        declaration = re.search(r'void Init_Motion_search_turn[^\n]+;', header).group(0)
        wrapper = function(header, 'inline t_exeStatus exe_Motion_search_turn')
        search_source = function((ROOT/'System/Src/myshell.cpp').read_text(encoding='utf-8'),
                                 'static const t_param *shell_debug_search_source')
        search_header = (ROOT/'Subsys/Inc/search_class.h').read_text(encoding='utf-8')
        begin = search_header.index('#if defined(MOUSE_A)')
        end = search_header.index('#endif', begin) + len('#endif')
        search_defaults = search_header[begin:end]
        search_reset = function(search_header, 'void search_param_init()')
        source = r'''
#include <cassert>
#include <cmath>
#include <cstddef>
#include "Params/run_param.h"
constexpr float accel_Integral = .7f;
struct Element {
    float value=0;
    void set(float v){value=v;}
    float get()const{return value;}
    void init(){value=0;}
};
struct Plan {
    Element velo,max_velo,end_velo,accel,deccel,end_length,length_accel,length_deccel;
    Element rad_accel,rad_deccel,rad_max_velo,end_radian,radian_accel,radian_deccel;
    Element turn_r_min,turn_state,turn_time_ms,fix_prev_run,fix_post_run;
};
struct Controller {
    t_pid_gain gain{};
    void Gain_Set(t_pid_gain g){gain=g;}
    void I_param_reset(){}
};
struct Vehicle {
    struct {Controller speed_ctrl,omega_ctrl;} Vehicle_controller;
    struct {Element length,radian,x_point;} ego,ideal;
};
struct IR {
    struct {bool is_wall=false;float distance=90;} sen_fr,sen_fl;
    bool CanContinueStraightWallPose(){return false;}
    void ResetStraightWallControl(bool){}
    void EnableIrSensStraight(){}
    void Division_Wall_Correction_Reset(){}
};
class Motion {
public:
    Plan motion_plan;
    t_straight_param straight_motion_param{};
    t_param turn_motion_param{};
    Vehicle storage,*vehicle=&storage;
    IR ir,*ir_sens=&ir;
    t_exeStatus status=complete;
    struct {void clear(){}} search_turn_heading;
    t_runControl motion_control_get(){return STRAIGHT_STATE;}
    t_bool motion_is_enable_get(){return False;}
    t_exeStatus motion_exeStatus_get(){return status;}
    void Motion_error_handling(){}
    void motion_pattern_set(t_run_pattern){}
    void motion_exeStatus_set(t_exeStatus v){status=v;}
    void motion_control_set(t_runControl){}
    void run_time_ms_reset(){}
    void run_time_limit_ms_reset(){}
    void error_counter_reset(){}
    t_exeStatus execute_Motion(){return complete;}
''' + declaration + '\n' + wrapper + '\n};\n' + implementation + '\n' + search_source + (
    '\nclass SearchDefaults {\npublic:\n' + search_defaults + '\n' + search_reset + '\n};\n') + r'''
static bool near(float a,float b){return std::fabs(a-b)<.00001f;}
static void gains(Motion& motion,const t_pid_gain* sp,const t_pid_gain* om) {
    assert(motion.straight_motion_param.sp_gain==sp);
    assert(motion.straight_motion_param.om_gain==om);
    assert(near(motion.vehicle->Vehicle_controller.speed_ctrl.gain.Kp,sp->Kp));
    assert(near(motion.vehicle->Vehicle_controller.omega_ctrl.gain.Ki,om->Ki));
}
static void legacy(const t_param& p) {
    assert(p.search_straight_sp_gain==nullptr && p.search_straight_om_gain==nullptr);
    Motion motion;
    motion.Init_Motion_search_turn(&p);
    gains(motion,&basic_sp_gain,&basic_om_gain);
    assert(motion.turn_motion_param.sp_gain==p.sp_gain);
    assert(motion.turn_motion_param.ff_gain==p.ff_gain);
    assert(motion.exe_Motion_search_turn(&p)==complete);
    gains(motion,&basic_sp_gain,&basic_om_gain);
}
int main() {
    SearchDefaults search;
    for(int reset=0;reset<2;++reset) {
        assert(near(search.search_st_param.param->max_velo,.37f));
        assert(near(search.param_L90_search.param->velo,.37f));
        assert(near(search.param_R90_search.param->velo,.37f));
        assert(search.param_L90_search.param==param_L90_search_370.param);
        assert(search.param_R90_search.param==param_R90_search_370.param);
        assert(search.param_L90_search.ff_gain==param_L90_search_370.ff_gain);
        assert(search.param_R90_search.ff_gain==param_R90_search_370.ff_gain);
        search.search_st_param=search_straight_slow.parameter();
        search.param_L90_search=param_L90_search_320;
        search.param_R90_search=param_R90_search_320;
        assert(near(search.search_st_param.param->max_velo,.32f));
        search.search_param_init(); // A new search restores normal speed after slow mode.
    }
    legacy(param_L90_search_280);legacy(param_R90_search_280);
    legacy(param_L90_search_300);legacy(param_R90_search_300);
    legacy(param_L90_search_320);legacy(param_R90_search_320);
    legacy(param_L90_search_350);legacy(param_R90_search_350);
    const int speeds[]={280,300,320,350,370};
    const t_param* left[]={&param_L90_search_280,&param_L90_search_300,
        &param_L90_search_320,&param_L90_search_350,&param_L90_search_370};
    const t_param* right[]={&param_R90_search_280,&param_R90_search_300,
        &param_R90_search_320,&param_R90_search_350,&param_R90_search_370};
    for(int i=0;i<5;++i) {
        assert(shell_debug_search_source(False,speeds[i])==left[i]);
        assert(shell_debug_search_source(True,speeds[i])==right[i]);
        assert(near(left[i]->param->velo,speeds[i]/1000.f));
        assert(near(right[i]->param->velo,speeds[i]/1000.f));
    }
    assert(shell_debug_search_source(False,360)==nullptr);
    assert(shell_debug_search_source(True,360)==nullptr);
#ifdef MOUSE_A
    assert(shell_debug_search_source(False,400)==&param_L90_search_400);
    assert(shell_debug_search_source(True,400)==&param_R90_search_400);
    legacy(param_L90_search_400);legacy(param_R90_search_400);
    assert(near(sp_gain_search_turn.Kp,2));
    const t_param* profiles[]={&param_L90_search_370,&param_R90_search_370};
    for(const auto* profile:profiles) {
        assert(near(profile->sp_gain->Kp,5) && near(profile->sp_gain->Ki,.04f));
        assert(near(profile->param->Lstart,9.46f));
        const bool left=profile->param->turn_dir==Turn_L;
        assert(near(profile->param->Lend,left?9.f:8.5f));
        assert(near(profile->param->degree,left?90.f:-90.f));
        assert(near(profile->param->degree_correction,0));
        assert(profile->ff_gain==(left?&ff_gain_search_turn_L_370:&ff_gain_search_turn_R_370));
        t_param copied=*profile; // Search stores a copy of the selected table.
        Motion motion;
        motion.Init_Motion_search_turn(&copied);
        gains(motion,profile->sp_gain,profile->om_gain);
        assert(motion.turn_motion_param.sp_gain==profile->sp_gain);
        assert(motion.exe_Motion_search_turn(&copied)==complete);
        gains(motion,profile->sp_gain,profile->om_gain);
        const t_pid_gain custom_sp={3,.02f,0},custom_om={.2f,.006f,0};
        motion.Init_Motion_search_turn(&copied,&custom_sp,&custom_om);
        gains(motion,&custom_sp,&custom_om); // Explicit debug edits win.
        motion.Init_Motion_search_turn(&copied,&custom_sp);
        gains(motion,&custom_sp,profile->om_gain);
        motion.Init_Motion_search_turn(&copied,nullptr,&custom_om);
        gains(motion,profile->sp_gain,&custom_om);
        copied.search_straight_om_gain=nullptr;
        motion.Init_Motion_search_turn(&copied);
        gains(motion,profile->sp_gain,&basic_om_gain);
    }
#else
    assert(near(sp_gain_search_turn.Kp,1.2f));
    assert(near(sp_gain_search_turn.Ki,.02f) && near(sp_gain_search_turn.Kd,0));
    assert(near(om_gain_search_turn.Kp,.1f) && near(om_gain_search_turn.Ki,.0015f));
    assert(near(om_gain_search_turn.Kd,0));
    assert(shell_debug_search_source(False,400)==nullptr);
    assert(shell_debug_search_source(True,400)==nullptr);
    const t_param* initial_profiles[]={&param_L90_search_350,&param_R90_search_350};
    for(const auto* profile:initial_profiles) {
        const bool left=profile->param->turn_dir==Turn_L;
        assert(profile->param->turn_dir==Turn_L || profile->param->turn_dir==Turn_R);
        assert(near(profile->param->r_min,left?26.f:-26.f));
        assert(near(profile->param->Lstart,9.46f) && near(profile->param->Lend,11.16f));
        assert(near(profile->param->degree,left?90.f:-90.f));
        assert(near(profile->param->degree_correction,0));
        assert(profile->sp_gain==&sp_gain_search_turn && profile->om_gain==&om_gain_search_turn);
        assert(profile->ff_gain==&ff_gain_default);
        legacy(*profile);
    }
    const t_param* profiles[]={&param_L90_search_370,&param_R90_search_370};
    for(const auto* profile:profiles) {
        const bool left=profile->param->turn_dir==Turn_L;
        assert(near(profile->param->r_min,left?26.f:-26.f));
        assert(near(profile->param->Lstart,left?7.3f:8.6f));
        assert(near(profile->param->Lend,left?11.5f:10.4f));
        assert(near(profile->param->degree,left?90.f:-90.f));
        assert(near(profile->param->degree_correction,left?0.f:-1.f));
        assert(profile->sp_gain==&sp_gain_search_turn_370 && profile->om_gain==&om_gain_search_turn);
        assert(near(profile->sp_gain->Kp,5) && near(profile->sp_gain->Ki,.02f));
        assert(near(profile->sp_gain->Kd,0));
        assert(profile->ff_gain==(left?&ff_gain_search_turn_L_370:&ff_gain_search_turn_R_370));
        const auto* ff=profile->ff_gain;
        assert(std::fabs(ff->om_accel-.000690f)<1e-9f);
        assert(std::fabs(ff->om_decel-.000555f)<1e-9f);
        assert(near(ff->sp_velo,ff_gain_default.sp_velo) && near(ff->sp_accel,ff_gain_default.sp_accel));
        assert(near(ff->sp_bias,ff_gain_default.sp_bias) && near(ff->om_velo,ff_gain_default.om_velo));
        assert(near(ff->om_bias,ff_gain_default.om_bias));
        assert(near(ff->om_jerk,0) && near(ff->sp_turn_accel_mag,0) && near(ff->sp_turn_velo_sq,0));
        t_param copied=*profile;
        Motion motion;
        motion.Init_Motion_search_turn(&copied);
        gains(motion,profile->sp_gain,profile->om_gain);
        assert(motion.turn_motion_param.ff_gain==profile->ff_gain);
        assert(motion.exe_Motion_search_turn(&copied)==complete);
        gains(motion,profile->sp_gain,profile->om_gain);
        const t_pid_gain custom_sp={3,.03f,0},custom_om={.2f,.006f,0};
        motion.Init_Motion_search_turn(&copied,&custom_sp,&custom_om);
        gains(motion,&custom_sp,&custom_om);
        motion.Init_Motion_search_turn(&copied,&custom_sp);
        gains(motion,&custom_sp,profile->om_gain);
        motion.Init_Motion_search_turn(&copied,nullptr,&custom_om);
        gains(motion,profile->sp_gain,&custom_om);
    }
#endif
    Motion motion;
    const t_pid_gain explicit_sp={3,.03f,0},explicit_om={.3f,.007f,0};
    motion.Init_Motion_search_turn(&param_L90_search_320,&explicit_sp,&explicit_om);
    gains(motion,&explicit_sp,&explicit_om);
}
'''
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target','x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='search-params-',dir=ROOT/'build') as folder:
            folder=Path(folder)
            stub=folder/'Module/Inc/vehicle.h'
            stub.parent.mkdir(parents=True)
            stub.write_text('#pragma once\n#include "Task/Inc/run_typedef.h"\n',encoding='utf-8')
            path=folder/'test.cpp';path.write_text(source,encoding='utf-8')
            for mouse in ('A','B'):
                with self.subTest(mouse=mouse):
                    executable=folder/('test'+mouse+('.exe' if os.name=='nt' else ''))
                    result=subprocess.run(command+['-std=c++14','-O0','-Wall','-Wextra',
                        '-fno-exceptions','-fno-rtti','-DMOUSE_'+mouse,
                        '-I',str(folder),'-I',str(ROOT),str(path),
                        *parameter_sources(mouse),'-o',str(executable)],
                        capture_output=True,env=environment,timeout=240)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
                    result=subprocess.run([str(executable)],capture_output=True,timeout=10)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))


if __name__=='__main__':
    unittest.main()
