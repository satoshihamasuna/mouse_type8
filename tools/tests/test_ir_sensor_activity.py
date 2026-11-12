"""実IR DMAドライバ・手かざし判定・走行setterで発光状態を検証する。"""
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_side_wall_stability import compiler_command

ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent


class IrSensorActivityTest(unittest.TestCase):
    def test_driver_demo_waits_error_recovery_and_fresh_adc(self):
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
        with tempfile.TemporaryDirectory(prefix="ir-activity-", dir=build) as directory:
            work = Path(directory)
            fake_core = work / "Core/Inc"
            fake_core.mkdir(parents=True)
            for name in ("main", "adc", "gpdma", "icache", "lptim", "spi", "tim", "gpio"):
                (fake_core / (name + ".h")).write_text('#include "ir_sensor_test_hal.h"\n', encoding="utf-8")
            sensor = (ROOT / "Task/Src/ir_sens_task.cpp").read_text(encoding="utf-8")
            functions = []
            for signature in ("int16_t IrSensTask::IrSensor_Avg(", "bool IrSensTask::HandStartDetected("):
                start = sensor.index(signature)
                stop = sensor.index("\n}", start) + 2
                functions.append(sensor[start:stop])
            (work / "hand_start_detected.inc").write_text("\n".join(functions), encoding="utf-8")
            motion = (ROOT / "Task/Inc/ctrl_task.h").read_text(encoding="utf-8")
            setters = []
            for name in ("motion_enable_set", "motion_disable_set"):
                match = re.search(r"inline\s+void\s+" + name + r"\(\)\s*\{[^}]+\}", motion)
                self.assertIsNotNone(match)
                setters.append(match.group())
            (work / "motion_activity_setters.inc").write_text("\n".join(setters), encoding="utf-8")
            def function_body(source, signature):
                start = source.index(signature)
                body = source.index("{", start)
                depth = 1
                end = body + 1
                while depth:
                    depth += (source[end] == "{") - (source[end] == "}")
                    end += 1
                return source[start:end]

            lifecycle = (ROOT / "Task/Src/init_motion.cpp").read_text(encoding="utf-8")
            (work / "motion_activity_lifecycle.inc").write_text("\n".join(
                function_body(lifecycle, "void Motion::" + name + "()")
                for name in ("Motion_start", "Motion_end", "Motion_error_handling")), encoding="utf-8")
            interrupt = (ROOT / "Module/Src/interrupt.cpp").read_text(encoding="utf-8")
            (work / "ir_activity_interrupt.inc").write_text(
                function_body(interrupt, "void Interrupt::main()"), encoding="utf-8")
            menu = (ROOT / "System/Src/wall_histry_check.cpp").read_text(encoding="utf-8")
            (work / "wall_history_menu.inc").write_text(
                "namespace Mode {\n" + function_body(menu, "void Wall_Histry_Check(wall_class *wall_data)")
                + "\n}\n", encoding="utf-8")
            menu = (ROOT / "System/Src/search_time_select.cpp").read_text(encoding="utf-8")
            (work / "search_time_menu.inc").write_text(
                "namespace Mode {\n" + function_body(menu, "int32_t Seach_Time_Select()")
                + "\n}\n", encoding="utf-8")
            obj = work / "ir_driver.o"
            exe = work / ("ir_activity.exe" if os.name == "nt" else "ir_activity")
            demo_exe = work / ("ir_demo.exe" if os.name == "nt" else "ir_demo")
            includes = ["-I", str(work), "-I", str(TEST_DIR), "-I", str(ROOT)]
            invocations = [
                command + target + ["-x", "c", "-std=c11", "-c"] + includes +
                [str(TEST_DIR / "ir_sensor_activity_harness.c"), "-o", str(obj)],
                command + target + ["-std=c++14", "-fno-exceptions", "-fno-rtti"] + includes +
                [str(TEST_DIR / "ir_sensor_activity_harness.cpp"), str(obj), "-o", str(exe)],
                command + target + ["-std=c++14", "-fno-exceptions", "-fno-rtti"] + includes +
                [str(TEST_DIR / "ir_sensor_demo_harness.cpp"), str(obj), "-o", str(demo_exe)],
                [str(exe)],
                [str(demo_exe)],
            ]
            for invocation in invocations:
                result = subprocess.run(invocation, capture_output=True, text=True,
                    errors="replace", env=environment, timeout=240)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
