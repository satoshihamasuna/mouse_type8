"""実モーション・PIDで吸引立上げ中の停止指令を検証する。"""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_side_wall_stability import compiler_command, parameter_sources

ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent


def function_source(source, signature):
    start = source.index(signature)
    end = source.index("\n}", start) + 2
    return source[start:end]


class SuctionStartTest(unittest.TestCase):
    def test_stationary_feedback_and_launch_for_both_mice(self):
        command = compiler_command()
        if command is None:
            self.skipTest("No host C++ compiler: set CXX or install Zig locally")
        build = ROOT / "build"
        build.mkdir(exist_ok=True)
        environment = os.environ.copy()
        target = []
        if Path(command[0]).stem == "zig":
            environment.setdefault("ZIG_LOCAL_CACHE_DIR", str(build / "ir-confidence-zig-cache"))
            environment.setdefault("ZIG_GLOBAL_CACHE_DIR", str(build / "ir-confidence-zig-global"))
            if os.name == "nt":
                target = ["-target", "x86_64-windows-gnu"]
        with tempfile.TemporaryDirectory(prefix="suction-start-", dir=build) as directory:
            work = Path(directory)
            peripheral = work / "Peripheral/Inc/peripheral.h"
            peripheral.parent.mkdir(parents=True)
            peripheral.write_text('''#pragma once
#include <stdint.h>
struct t_encoder { float wheel_speed; int sp_pulse; };
extern t_encoder test_right, test_left;
extern int test_fan_duty;
inline t_encoder Encoder_GetProperty_Right() { return test_right; }
inline t_encoder Encoder_GetProperty_Left() { return test_left; }
inline void Motor_SetDuty_Right(int) {}
inline void Motor_SetDuty_Left(int) {}
inline void FAN_Motor_SetDuty(int duty) { test_fan_duty = duty; }
''', encoding="utf-8")
            sensing = work / "Task/Inc/sensing_task.h"
            sensing.parent.mkdir(parents=True)
            sensing.write_text('''#pragma once
#include "Component/Inc/singleton.h"
class Vehicle;
class IrSensTask {
public:
    int steering_calls = 0;
    void ResetStraightWallControl() {}
    void EnableIrSensStraight() {}
    void EnableIrSens() {}
    void Division_Wall_Correction_Reset() {}
    void SetWallControl_RadVelo(Vehicle *, float) { ++steering_calls; }
};
class IrSensTask_type8 : public IrSensTask, public Singleton<IrSensTask_type8> {};
''', encoding="utf-8")
            functions = []
            for filename, signature in (
                ("Task/Src/init_motion.cpp", "void Motion::Init_Motion_suction_start"),
                ("Task/Src/motion.cpp", "void Motion::SetIdeal_suction_start"),
                ("Task/Src/turn_motion.cpp", "void Motion::updateTurnKinematics()"),
                ("Task/Src/ctrl_task.cpp", "void CtrlTask::motion_control()"),
            ):
                source = (ROOT / filename).read_text(encoding="utf-8")
                functions.append(function_source(source, signature))
            irq = (ROOT / "Module/Src/interrupt.cpp").read_text(encoding="utf-8")
            begin = irq.index("\tfloat acc_mean")
            end = irq.index("\tfloat rad_velo", begin)
            functions.append("static void step_measured_velocity(float acc_sum, float x_acc_sum, float z_acc_sum)\n{\n"
                             "    const auto Renc = Encoder_GetProperty_Right();\n"
                             "    const auto Lenc = Encoder_GetProperty_Left();\n"
                             + irq[begin:end] + "\n}")
            begin = irq.index("\tfloat set_velo = velo;")
            last = "Vehicle_type8::getInstance().ego.turn_slip_theta.set(slip_theta);"
            end = irq.index(last, begin) + len(last)
            functions.append("static void step_measured_slip(float velo, float rad_velo)\n{\n"
                             + irq[begin:end] + "\n}")
            (work / "suction_functions.inc").write_text("\n".join(functions), encoding="utf-8")
            failures = []
            for mouse in ("A", "B"):
                with self.subTest(mouse=mouse):
                    exe = work / ("suction_" + mouse + (".exe" if os.name == "nt" else ""))
                    compile_result = subprocess.run(command + target + [
                        "-std=c++14", "-fno-exceptions", "-fno-rtti", "-DMOUSE_" + mouse,
                        "-I", str(work), "-I", str(ROOT),
                        str(TEST_DIR / "suction_start_harness.cpp"),
                        str(ROOT / "Component/Src/controller.cpp"),
                    ] + parameter_sources(mouse) + ["-o", str(exe)],
                        capture_output=True, text=True, errors="replace", env=environment, timeout=240)
                    self.assertEqual(compile_result.returncode, 0, compile_result.stdout + compile_result.stderr)
                    result = subprocess.run([str(exe)], capture_output=True, text=True,
                        errors="replace", env=environment, timeout=30)
                    if result.returncode:
                        failures.append(mouse + ": " + result.stdout + result.stderr)
            self.assertEqual(failures, [])


if __name__ == "__main__":
    unittest.main()
