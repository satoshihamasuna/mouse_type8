"""実際の撮影専用許可をホストC++で検証する。実機は使用しない。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command

ROOT = Path(__file__).resolve().parents[2]


class CameraRunPermitTests(unittest.TestCase):
    def test_search_camera_retains_turn_before_post_run_braking(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        text = (ROOT/'System/Src/myshell.cpp').read_text(encoding='utf-8')
        start = text.index('static int shell_debug_search_command(')
        start = text.index('if (ntlibc_strcmp(argv[3], "exe") == 0 || camera_run)', start)
        opening = text.index('{', start)
        depth, end = 1, opening + 1
        while depth:
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        implementation = text[opening + 1:end - 1]
        source = r'''
#include <cassert>
#include <cstdio>
#include <deque>
#include <algorithm>
#include <vector>
constexpr bool True=true, False=false;
struct Gain{};
struct Param{};
struct Table{float velo=.37f;};
struct Debug{
    Table table; Gain sp_gain,om_gain,ff_gain; Param param;
    float pre_accel=6.5f,acc=6.5f; int turn_count=1,suction_duty=650;
    bool suction_enable=false;
};
static Debug stored;
static Debug shell_debug_straight_param;
struct IrSensTask{};
struct LogData{
    bool log_enable=false; std::deque<int> samples;
    static LogData& getInstance(){static LogData log;return log;}
    void reset_log(){samples.clear();}
    void sample(int phase){
        if(!log_enable)return;
        if(samples.size()==400)samples.pop_front();
        samples.push_back(phase);
    }
};
struct Motion{
    std::vector<int> calls; IrSensTask ir;
    IrSensTask* return_irObj(){return &ir;}
    void Motion_start(){calls.push_back(1);}
    void Motion_end(){calls.push_back(5);}
    void ticks(int phase,int count){for(int i=0;i<count;++i)LogData::getInstance().sample(phase);}
    void exe_Motion_straight(float,float,float,float end,const Gain*,const Gain*,const Gain*){
        calls.push_back(end==0?4:2);
        ticks(1,end==0?170:150);
        // Real SetIdeal_straight starts a400ms brake before this call returns.
        if(end==0)ticks(25,400);
    }
    void exe_Motion_search_turn(const Param*,const Gain*,const Gain*){
        calls.push_back(3); ticks(24,215);
    }
};
struct CtrlTask_type8{static Motion& getInstance(){static Motion motion;return motion;}};
struct Permit{void clear(){}};
static Permit shell_camera_run_permit;
static bool allow_camera=true;
static int hand_waits=0;
static bool shell_debug_camera_begin(int,char**,int){return allow_camera;}
static void shell_debug_wait_sensor(IrSensTask*){++hand_waits;}
static void shell_debug_suction_start(Motion*,bool,int){}
static float shell_debug_accel_in_length(float,float,float acc){return acc;}
static void Indicate_LED(int){}
static void HAL_Delay(int){}
static int run(bool camera_run){
    int argc=5;char *argv[5]={};Debug *debug=&stored;
''' + implementation + r'''
    return -99;
}
int main(){
    auto& log=LogData::getInstance();auto& motion=CtrlTask_type8::getInstance();
    const std::vector<int> expected{1,2,3,4,5};
    assert(run(false)==0);
    assert(motion.calls==expected && hand_waits==1);
    assert(log.samples.size()==400);
    assert(std::count(log.samples.begin(),log.samples.end(),24)==0);
    motion.calls.clear();hand_waits=0;
    assert(run(true)==0);
    assert(motion.calls==expected && hand_waits==0);
    assert(std::count(log.samples.begin(),log.samples.end(),24)==215);
    assert(std::count(log.samples.begin(),log.samples.end(),25)==0);
    assert(!log.log_enable);
    motion.calls.clear();allow_camera=false;
    assert(run(true)==-1 && motion.calls.empty());
}
'''
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-global'))
            if os.name == 'nt':command += ['-target','x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='camera-search-log-',dir=ROOT/'build') as directory:
            directory=Path(directory)
            path=directory/'camera_search_log.cpp';path.write_text(source,encoding='utf-8')
            executable=directory/('shell.exe' if os.name=='nt' else 'shell')
            result=subprocess.run(command+['-std=c++14','-Wall','-Wextra','-fno-exceptions','-fno-rtti',
                str(path),'-o',str(executable)],capture_output=True,env=environment,timeout=240)
            self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
            result=subprocess.run([str(executable)],capture_output=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_real_shell_camera_entry_rejects_count_battery_and_repeated_tokens(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        text = (ROOT/'System/Src/myshell.cpp').read_text(encoding='utf-8')
        start = text.index('static bool shell_debug_camera_begin(')
        end = text.index('\nstatic void shell_debug_wait_sensor', start)
        implementation = text[start:end]
        source = r'''
#include "System/Inc/camera_run_permit.h"
#include <cassert>
#include <cstdio>
static CameraRunPermit shell_camera_run_permit;
static uint32_t tick=100;
static float voltage=8.0f;
static uint32_t HAL_GetTick(){return tick;}
static float Battery_GetAverageVoltage(){return voltage;}
static bool Battery_IsVoltageError(float value){return value<7.0f;}
''' + implementation + r'''
int main(){
    char token[]="123";
    char *args[]={nullptr,nullptr,nullptr,nullptr,token};
    assert(!shell_debug_camera_begin(5,args,1));
    shell_camera_run_permit.arm(123,tick);
    assert(shell_debug_camera_begin(5,args,1));
    assert(!shell_debug_camera_begin(5,args,1));
    shell_camera_run_permit.arm(123,tick);
    assert(!shell_debug_camera_begin(5,args,2));
    assert(!shell_debug_camera_begin(5,args,1));
    shell_camera_run_permit.arm(123,tick);
    voltage=6.5f;
    assert(!shell_debug_camera_begin(5,args,1));
    voltage=8.0f;
    assert(!shell_debug_camera_begin(5,args,1));
    shell_camera_run_permit.arm(123,tick);
    assert(!shell_debug_camera_begin(4,args,1));
    assert(!shell_debug_camera_begin(5,args,1));
    shell_camera_run_permit.arm(123,tick);
    tick+=CameraRunPermit::ttl_ms;
    assert(!shell_debug_camera_begin(5,args,1));
}
'''
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR',str(ROOT/'build/ir-confidence-zig-global'))
            if os.name == 'nt':command += ['-target','x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='camera-shell-',dir=ROOT/'build') as directory:
            directory=Path(directory)
            path=directory/'camera_shell.cpp';path.write_text(source,encoding='utf-8')
            executable=directory/('shell.exe' if os.name=='nt' else 'shell')
            result=subprocess.run(command+['-std=c++14','-Wall','-Wextra','-fno-exceptions','-fno-rtti',
                '-I',str(ROOT),str(path),'-o',str(executable)],capture_output=True,env=environment,timeout=240)
            self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
            result=subprocess.run([str(executable)],capture_output=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))

    def test_one_shot_token_expiry_wrap_and_invalid_arguments(self):
        command = compiler_command()
        if command is None:
            self.skipTest('Host C++ compiler is required')
        environment = os.environ.copy()
        if Path(command[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT / 'build/ir-confidence-zig-global'))
            if os.name == 'nt':
                command += ['-target', 'x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='camera-permit-', dir=ROOT / 'build') as directory:
            executable = Path(directory) / ('permit.exe' if os.name == 'nt' else 'permit')
            result = subprocess.run(command + [
                '-std=c++14', '-Wall', '-Wextra', '-fno-exceptions', '-fno-rtti',
                '-I', str(ROOT), str(ROOT / 'tools/tests/camera_run_permit_harness.cpp'),
                '-o', str(executable),
            ], capture_output=True, env=environment, timeout=240)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            result = subprocess.run([str(executable)], capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))


if __name__ == '__main__':
    unittest.main()
