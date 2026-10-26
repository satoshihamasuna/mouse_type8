# Copyright 2026 hmsn
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Compile the real side-wall helper and IR task for host regression checks.

Use CXX, a c++/g++/clang++ on PATH, or the optional local Zig installation in
build/ir-confidence-tools. Missing compilers skip these tests. Both machine
calibrations are built; only hardware access and the Vehicle container are stubbed.
"""

import os
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent


def compiler_command():
    configured = os.environ.get("CXX", "").strip()
    if configured:
        if Path(configured).is_file():
            command = [configured]
        else:
            command = shlex.split(configured, posix=os.name != "nt")
            if os.name == "nt":
                command = [arg[1:-1] if arg[:1] == arg[-1:] == '"' else arg
                           for arg in command]
        if command and (Path(command[0]).is_file() or shutil.which(command[0])):
            if Path(command[0]).stem == "zig" and len(command) == 1:
                command.append("c++")
            return command
    for name in ("c++", "g++", "clang++"):
        executable = shutil.which(name)
        if executable:
            return [executable]
    for name in ("zig.exe", "zig"):
        executable = ROOT / "build" / "ir-confidence-tools" / "ziglang" / name
        if executable.is_file():
            return [str(executable), "c++"]
    return None


class SideWallStabilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        command = compiler_command()
        if command is None:
            raise unittest.SkipTest("No host C++ compiler: set CXX or install Zig locally")
        build_dir = ROOT / "build"
        build_dir.mkdir(exist_ok=True)
        cls.work = tempfile.TemporaryDirectory(prefix="side-wall-test-", dir=build_dir)
        cls.addClassCleanup(cls.work.cleanup)
        environment = os.environ.copy()
        if Path(command[0]).stem == "zig":
            environment.setdefault("ZIG_LOCAL_CACHE_DIR", str(build_dir / "ir-confidence-zig-cache"))
            environment.setdefault("ZIG_GLOBAL_CACHE_DIR", str(build_dir / "ir-confidence-zig-global"))
            if os.name == "nt":
                command += ["-target", "x86_64-windows-gnu"]
        cls.executables = {}
        for mouse in ("A", "B"):
            executable = Path(cls.work.name) / ("side-wall-" + mouse + (".exe" if os.name == "nt" else ""))
            invocation = command + [
                "-std=c++14", "-O0", "-Wall", "-Wextra", "-fno-exceptions", "-fno-rtti",
                "-DMOUSE_" + mouse, "-I", str(TEST_DIR / "side_wall_stubs"),
                "-I", str(ROOT), str(TEST_DIR / "side_wall_stability_harness.cpp"),
                str(ROOT / "Task" / "Src" / "ir_sens_task.cpp"),
                str(ROOT / "Task" / "Src" / "zirconia_wall_control.cpp"), "-o", str(executable),
            ]
            result = subprocess.run(invocation, capture_output=True, text=True,
                                    errors="replace", env=environment, timeout=240)
            if result.returncode:
                raise AssertionError("Host build failed for Mouse" + mouse +
                                     " (exit " + str(result.returncode) + "):\n" +
                                     result.stdout + result.stderr)
            cls.executables[mouse] = executable

    def run_case(self, case):
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), case], capture_output=True,
                                        text=True, errors="replace", timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_complete_six_mm_window_precedes_ten_mm_confirmation(self):
        self.run_case("window")

    def test_confirmation_does_not_count_the_window_filling_part_of_a_step(self):
        self.run_case("partial-window")

    def test_constant_wall_qualifies_at_the_same_physical_position(self):
        self.run_case("helper-speed")

    def test_six_mm_average_of_a_linear_wall_is_independent_of_sample_step(self):
        self.run_case("linear-wall")

    def test_decimal_steps_do_not_delay_exact_spatial_boundaries(self):
        self.run_case("decimal-boundaries")

    def test_linear_wall_qualifies_at_sixteen_mm_even_when_sample_steps_change(self):
        self.run_case("linear-activation")

    def test_difference_limit_is_strict_for_the_configured_threshold(self):
        self.run_case("difference-limit")

    def test_stopping_does_not_confirm_and_can_revoke_stability(self):
        self.run_case("stationary")

    def test_invalid_reverse_and_nonfinite_samples_clear_history(self):
        self.run_case("reset")

    def test_gaps_over_six_mm_require_a_new_spatial_window(self):
        self.run_case("gap")

    def test_ir_task_extends_control_at_the_same_position_for_different_steps(self):
        self.run_case("integration-speed")

    def test_ir_task_qualifies_at_sixteen_mm_for_point_two_half_and_one_point_six_mm_steps(self):
        self.run_case("integration-decimal")

    def test_near_wall_control_keeps_its_immediate_fallback(self):
        self.run_case("integration-near")

    def test_front_threshold_resets_only_its_own_side(self):
        self.run_case("integration-front")

    def test_wall_loss_and_reacquisition_leave_the_other_side_qualified(self):
        self.run_case("integration-loss")

    def test_diagonal_corner_revokes_only_the_affected_side(self):
        self.run_case("integration-corner")

    def test_disabled_control_keeps_observing_without_applying_wall_errors(self):
        self.run_case("integration-disabled")

    def test_diagonal_low_speed_gate_and_velocity_gain_are_preserved(self):
        self.run_case("integration-speed-gate")

    def test_zirconia_steering_symmetry_heading_damping_and_limits(self):
        self.run_case("zirconia-symmetry")

    def test_zirconia_rejects_a_pillar_without_resetting_the_remaining_wall(self):
        self.run_case("zirconia-independent")

    def test_zirconia_spatial_confidence_stop_and_expected_heading_motion(self):
        self.run_case("zirconia-spatial")

    def test_zirconia_disable_invalid_input_and_estimator_sync(self):
        self.run_case("zirconia-reset")

    def test_zirconia_real_adc_adapter_front_wall_mode_and_speed_transitions(self):
        self.run_case("zirconia-adapter")

    def test_real_adc_path_can_qualify_before_the_old_time_window_or_count_gate(self):
        self.run_case("integration-adc")


if __name__ == "__main__":
    unittest.main()
