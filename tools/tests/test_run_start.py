"""最短走行の予備加速から先頭モーションまでを実C++で検証する。"""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_side_wall_stability import compiler_command, parameter_sources
from test_suction_start import function_source

ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent


class RunStartTest(unittest.TestCase):
    def test_continuous_initial_acceleration_for_both_mice(self):
        command = compiler_command()
        if command is None:
            self.skipTest("No host C++ compiler")
        build = ROOT / "build"
        build.mkdir(exist_ok=True)
        environment = os.environ.copy()
        target = []
        if Path(command[0]).stem == "zig":
            environment.setdefault("ZIG_LOCAL_CACHE_DIR", str(build / "ir-confidence-zig-cache"))
            environment.setdefault("ZIG_GLOBAL_CACHE_DIR", str(build / "ir-confidence-zig-global"))
            if os.name == "nt":
                target = ["-target", "x86_64-windows-gnu"]
        with tempfile.TemporaryDirectory(prefix="run-start-", dir=build) as directory:
            work = Path(directory)
            peripheral = work / "Peripheral/Inc/peripheral.h"
            peripheral.parent.mkdir(parents=True)
            peripheral.write_text('''#pragma once
#include <stdint.h>
inline void Motor_SetDuty_Right(int) {}
inline void Motor_SetDuty_Left(int) {}
inline void FAN_Motor_SetDuty(int) {}
''', encoding="utf-8")
            sensing = work / "Task/Inc/sensing_task.h"
            sensing.parent.mkdir(parents=True)
            sensing.write_text('''#pragma once
#include "Module/Inc/vehicle.h"
#include "Task/Inc/wall_position_correction.h"
#include "Task/Inc/motion_heading.h"
class IrSensTask {
public:
    param_element r_corner_length{}, l_corner_length{};
    bool CanContinueStraightWallPose() { return false; }
    float StraightHeadingForTurn() { return 0.0f; }
    float StraightHeadingVarianceForTurn() { return 0.02f; }
    bool DiagonalHeadingForTurn(float &, float &) { return false; }
    void BeginTurnHeading(const MotionHeading::Prior &) {}
    bool FeatureEkfEnabled() { return false; }
    WallPositionCorrection::FeatureContext StraightRunFeatureContext() { return {}; }
    float StraightNodeFeatureReference(float value) { return value; }
    void ResetStraightWallControl(bool = false, bool = false) {}
    void EnableIrSensStraight() {}
    void EnableIrSensDiagonal() {}
    void EnableIrSens() {}
    void DisableIrSens() {}
    void Division_Wall_Correction_Reset(bool = false) {}
    void RebaseStraightFeatureCandidates(float) {}
    void RebaseDiagonalFeatureCandidates(float, float, float = 0) {}
};
class IrSensTask_type8 : public IrSensTask, public Singleton<IrSensTask_type8> {};
''', encoding="utf-8")
            header = (ROOT / "Task/Inc/ctrl_task.h").read_text(encoding="utf-8")
            # 前景のbusy waitだけを模擬IRQに置き換え、宣言・ラッパーは実ヘッダーを使う。
            header = 'class Motion; void host_control_tick(Motion *);\n' + header.replace(
                'while(motion_exeStatus_get() == execute);',
                'while(motion_exeStatus_get() == execute) host_control_tick(this);')
            (work / "Task/Inc/ctrl_task.h").write_text(header, encoding="utf-8")
            functions = []
            init = (ROOT / "Task/Src/init_motion.cpp").read_text(encoding="utf-8")
            for name in ("straight", "long_turn", "turn_in", "turn_out", "turn_v90"):
                functions.append(function_source(init, "void Motion::Init_Motion_" + name))
            runtime = (ROOT / "Task/Src/motion.cpp").read_text(encoding="utf-8")
            functions.append(function_source(runtime, "void Motion::SetIdeal_straight()"))
            turn = (ROOT / "Task/Src/turn_motion.cpp").read_text(encoding="utf-8")
            functions.append(function_source(turn, "void Motion::handleTurnPrev_Straight()"))
            functions.append(function_source(turn, "void Motion::handleTurnPrev_Diagonal()"))
            (work / "motion_functions.inc").write_text("\n".join(functions), encoding="utf-8")
            path = (ROOT / "Subsys/Inc/make_path.h").read_text(encoding="utf-8")
            runtime_header = work / "Subsys/Inc/make_path.h"
            runtime_header.parent.mkdir(parents=True)
            runtime_header.write_text('#pragma once\n#include "Params/run_config.h"\n'
                + path[path.index("class calcRunTime\n"):path.index("\nclass Dijkstra:")], encoding="utf-8")
            run = (ROOT / "Subsys/Src/make_path.cpp").read_text(encoding="utf-8")
            support = ""
            if "static constexpr float RUN_START_LENGTH_MM" in run:
                start = run.index("static constexpr float RUN_START_LENGTH_MM")
                support = run[start:run.index("#define DIJKSTRA_MAX_TIME", start)]
            (work / "run_start_support.inc").write_text(support, encoding="utf-8")
            for method, label in (("run_Dijkstra", "plain"), ("run_Dijkstra_suction", "suction"),
                                  ("run_Dijkstra_suction_acc", "variable")):
                source = function_source(run, "void Dijkstra::" + method + "(")
                stop = source.index("\n\tuint16_t section_count")
                start = source.rfind("\tlog_enable();", 0, stop)
                (work / (label + "_setup.inc")).write_text(source[start:stop], encoding="utf-8")
                start = source.index("for(int i = tail")
                end = source.index("\n\t\tswitch(run_pt_buff[i])", start)
                # 可変ターンの次モーション速度選択は別の既存テストで検証する。
                prefix = source[source.index("{", start) + 1:end] if label != "variable" else ""
                turn_cases = []
                for pattern in ('Long_turnR90', 'Long_turnL90', 'Long_turnR180', 'Long_turnL180',
                                'Turn_in_R45', 'Turn_in_L45', 'Turn_in_R135', 'Turn_in_L135'):
                    start = source.index('case ' + pattern + ':', end)
                    stop = source.index('\n\t\t\t\tbreak;', start) + len('\n\t\t\t\tbreak;')
                    turn_cases.append(source[start:stop])
                start = source.index("section_end = ", source.index("case Straight:", end))
                stop = source.index("\n\t\t\t\tbreak;", start)
                straight_case = source[start:stop]
                body = "for(int i = tail; i >= tail; --i) {\n" + prefix + '\nswitch (run_pt_buff[i]) {\n'
                body += '\n'.join(turn_cases) + '\ncase Straight:\n' + straight_case + '\nbreak;\ndefault: break;\n}\n}\n'
                (work / (label + "_first.inc")).write_text(body, encoding="utf-8")
            failures = []
            for mouse in ("A", "B"):
                exe = work / ("start_" + mouse + (".exe" if os.name == "nt" else ""))
                result = subprocess.run(command + target + ["-std=c++14", "-fno-exceptions", "-fno-rtti",
                    "-DMOUSE_" + mouse, "-I", str(work), "-I", str(ROOT),
                    str(TEST_DIR / "run_start_harness.cpp"), str(ROOT / "Task/Src/init_motion2.cpp"),
                    str(ROOT / "Component/Src/controller.cpp")]
                    + parameter_sources(mouse) + ["-o", str(exe)], capture_output=True, text=True,
                    errors="replace", env=environment, timeout=240)
                self.assertEqual(result.returncode, 0, (result.stdout + result.stderr)[-12000:])
                try:
                    result = subprocess.run([str(exe)], capture_output=True, text=True,
                        errors="replace", env=environment, timeout=10)
                except subprocess.TimeoutExpired as error:
                    output = error.stderr or ""
                    if isinstance(output, bytes):
                        output = output.decode("utf-8", errors="replace")
                    self.fail(mouse + ": " + output[-1800:])
                if result.returncode:
                    failures.append(mouse + ": " + (result.stdout + result.stderr)[-1800:])
            self.assertFalse(failures, "\n".join(failures))


if __name__ == "__main__":
    unittest.main()
