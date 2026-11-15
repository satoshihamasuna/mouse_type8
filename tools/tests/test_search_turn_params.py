"""実際の探索ターン初期化・機体別Paramsで前後PIDの選択を検証する。"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command

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
''' + declaration + '\n' + wrapper + '\n};\n' + implementation + r'''
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
    legacy(param_L90_search_280);legacy(param_R90_search_280);
    legacy(param_L90_search_300);legacy(param_R90_search_300);
    legacy(param_L90_search_320);legacy(param_R90_search_320);
#ifdef MOUSE_A
    legacy(param_L90_search_350);legacy(param_R90_search_350);
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
                        '-I',str(folder),'-I',str(ROOT),str(path),'-o',str(executable)],
                        capture_output=True,env=environment,timeout=240)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
                    result=subprocess.run([str(executable)],capture_output=True,timeout=10)
                    self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))


if __name__=='__main__':
    unittest.main()
