"""実LogDataの周期カウンタ・リング・バイナリ出力の時間軸をホストで検証する。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_side_wall_stability import compiler_command

ROOT = Path(__file__).resolve().parents[2]


class LogPeriodTests(unittest.TestCase):
    def test_sampling_ring_wrap_binary_time_and_default_reset(self):
        compiler = compiler_command()
        if compiler is None:
            self.skipTest('Host C++ compiler is required')
        logger = (ROOT/'Module/Src/log_data.cpp').read_text(encoding='utf-8')
        start = logger.index('void LogData::reset_log(')
        reset = logger[start:logger.index('void LogData::indicate_data()', start)]
        binary = logger[logger.index('void LogData::indicate_data_binary()'):logger.index('void LogData::logging()')]
        tick = logger[logger.rindex('\t\tif(log_enable == True)'):logger.index('\t\tif(stream_due) LogStream::enqueue_sample')]
        source = r'''
#include <cassert>
#include <cstdio>
#include <vector>
#define private public
#include "Module/Inc/log_data.h"
#undef private
static uint32_t __get_PRIMASK(){return 0;}
static void __disable_irq(){}
static void __set_PRIMASK(uint32_t){}
namespace InterruptTiming{static void reset(){}}
static const char *labels[LOG_DATA_NUM+1];
static std::vector<log_frame_t> frames;
static void Communicate_TxPushBuffer(uint8_t *data,unsigned size){
    assert(size==sizeof(log_frame_t));
    log_frame_t frame; memcpy(&frame,data,size); frames.push_back(frame);
}
''' + reset + binary + r'''
static void sample_tick(LogData &log,unsigned number){
    auto &buffer=log.buffer; auto &log_tick=log.log_tick;
    auto &saved_period_ms=log.saved_period_ms; auto &log_enable=log.log_enable;
    half_float sample[LOG_DATA_NUM]={}; sample[0].bitPattern=number;
''' + tick + r'''
}
int main(){
    for(auto &label:labels)label="x";
    auto &log=LogData::getInstance();
    for(unsigned period:{1,2,5,100}){
        log.reset_log(period); log.log_enable=True;
        const unsigned ticks=430*period+3;
        for(unsigned i=0;i<ticks;++i)sample_tick(log,i);
        const unsigned total=(ticks+period-1)/period;
        log.log_enable=False;
        sample_tick(log,65000);
        assert(log.buffer.size()==400 && log.period_ms()==period);
        frames.clear(); log.indicate_data_binary();
        assert(frames.size()==401 && frames.back().magic==LOG_MAGIC_END);
        for(unsigned i=0;i<400;++i){
            assert(frames[i].magic==LOG_MAGIC && frames[i].index==i*period);
            assert(frames[i].data[0].bitPattern==(total-400+i)*period);
        }
    }
    log.reset_log(); assert(log.period_ms()==1 && log.buffer.size()==0 && log.log_tick==0);
    log.reset_log(0); assert(log.period_ms()==1);
    log.reset_log(101); assert(log.period_ms()==1);
}
'''
        environment = os.environ.copy()
        if Path(compiler[0]).stem == 'zig':
            environment.setdefault('ZIG_LOCAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-cache'))
            environment.setdefault('ZIG_GLOBAL_CACHE_DIR', str(ROOT/'build/ir-confidence-zig-global'))
            if os.name == 'nt': compiler += ['-target', 'x86_64-windows-gnu']
        with tempfile.TemporaryDirectory(prefix='log-period-', dir=ROOT/'build') as name:
            folder = Path(name)
            path, executable = folder/'period.cpp', folder/('period.exe' if os.name == 'nt' else 'period')
            path.write_text(source, encoding='utf-8')
            result = subprocess.run(compiler+['-std=c++14', '-I', str(ROOT), str(path), '-o', str(executable)],
                capture_output=True, env=environment, timeout=240)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))
            result = subprocess.run([str(executable)], capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors='replace'))


if __name__ == '__main__': unittest.main()
