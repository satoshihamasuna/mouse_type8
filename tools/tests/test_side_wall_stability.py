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
        shell = (ROOT / "System/Src/myshell.cpp").read_text(encoding="utf-8")
        start = shell.index("static int usrcmd_wall_ekf(int argc, char **argv)\n{")
        stop = shell.index("static void shell_irsens_usage", start)
        (Path(cls.work.name) / "wall_ekf_command.inc").write_text(shell[start:stop], encoding="utf-8")
        motion = (ROOT / "Task/Src/init_motion.cpp").read_text(encoding="utf-8")
        function = motion.index("void Motion::Init_Motion_search_turn")
        start = motion.index("\tfloat diff = vehicle->ego.x_point.get()", function)
        stop = motion.index("\t(motion_plan.end_radian.get() > 0)", start)
        (Path(cls.work.name) / "search_turn_post_correction.inc").write_text(motion[start:stop], encoding="utf-8")
        turn = (ROOT / "Task/Src/turn_motion.cpp").read_text(encoding="utf-8")
        setters = [line.strip() for line in turn.splitlines()
                   if 'vehicle->ego.length.set(' in line and 'detect_wall_edge_di' in line]
        assert len(setters) == 4
        code = 'switch (which) {\n' + ''.join('case {}: {}; break;\n'.format(i, line.rstrip(';'))
                                              for i, line in enumerate(setters)) + '}\n'
        (Path(cls.work.name) / "diagonal_turn_delay.inc").write_text(code, encoding="utf-8")
        for mouse in ("A", "B"):
            executable = Path(cls.work.name) / ("side-wall-" + mouse + (".exe" if os.name == "nt" else ""))
            invocation = command + [
                "-std=c++14", "-O0", "-Wall", "-Wextra", "-fno-exceptions", "-fno-rtti",
                "-DMOUSE_" + mouse, "-I", str(TEST_DIR / "side_wall_stubs"),
                "-I", str(ROOT), "-I", cls.work.name, str(TEST_DIR / "side_wall_stability_harness.cpp"),
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

    def test_turn_exit_range_yaw_disagreement_uses_parallel_legacy_control(self):
        self.run_case("ekf-exit-disagreement")

    def test_diagonal_edge_uses_a_spatial_reference_and_compensates_confirmation_delay(self):
        self.run_case("diagonal-edge-position")

    def test_diagonal_edge_recalls_the_clipped_zero_difference_plateau(self):
        self.run_case("diagonal-edge-plateau")

    def test_diagonal_edge_rejects_noise_stationary_reverse_invalid_and_rotating_inputs(self):
        self.run_case("diagonal-edge-invalid")

    def test_diagonal_edge_rearms_only_for_a_new_pillar_and_allows_quantized_zero_counts(self):
        self.run_case("diagonal-edge-rearm")

    def test_diagonal_corner_pair_requires_both_sides_and_compensates_delays_symmetrically(self):
        self.run_case("diagonal-edge-pair")

    def test_diagonal_turn_before_and_after_use_the_actual_delayed_progress_correction(self):
        self.run_case("diagonal-edge-turn")

    def test_diagonal_edge_real_machine_adc_calibration_rotation_and_mode_reset(self):
        self.run_case("diagonal-edge-adc")

    def test_diagonal_edge_defers_confirmation_but_recovers_a_short_rising_side_rebound(self):
        self.run_case("diagonal-edge-rebound")

    def test_diagonal_ekf_left_right_symmetry_and_no_false_yaw_certainty(self):
        self.run_case("diagonal-ekf-math")

    def test_diagonal_ekf_does_not_treat_gaps_between_pillars_as_centre_observations(self):
        self.run_case("diagonal-ekf-sparse")

    def test_diagonal_ekf_rejects_stationary_stale_far_rotating_and_invalid_observations(self):
        self.run_case("diagonal-ekf-gates")

    def test_diagonal_ekf_uses_actual_adc_calibration_and_preserves_shadow_legacy_equivalence(self):
        self.run_case("diagonal-ekf-ir")

    def test_diagonal_ekf_resets_at_frame_changes_and_never_observes_unseen_walls(self):
        self.run_case("diagonal-ekf-reset")

    def test_diagonal_ekf_preserves_legacy_trajectory_response_for_recorded_unaccepted_states(self):
        self.run_case("diagonal-ekf-fallback-log")

    def test_diagonal_ekf_accepts_a_single_front_pillar_but_rejects_a_close_front_pair(self):
        self.run_case("diagonal-ekf-front-pillar")

    def test_diagonal_ekf_changes_only_lateral_input_and_keeps_legacy_trajectory_damping(self):
        self.run_case("diagonal-ekf-trajectory")

    def test_turn_heading_covariance_propagates_without_asserting_a_new_wall(self):
        self.run_case("search-turn-covariance")

    def test_turn_heading_covariance_waits_for_fresh_adc_and_preserves_unknown_priors(self):
        self.run_case("search-turn-covariance-ir")

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

    def test_straight_edges_reject_stationary_and_threshold_noise_and_pulse_once(self):
        self.run_case("edge-noise-pulse")

    def test_corner_age_saturates_instead_of_wrapping_to_a_recent_event(self):
        self.run_case("edge-clock")

    def test_wall_edge_interpolation_preserves_crossing_position_across_speeds(self):
        self.run_case("edge-position")

    def test_wall_edge_rejects_spikes_reverse_rotation_gaps_and_stationary_confirmation(self):
        self.run_case("edge-invalid")

    def test_wall_edge_adapter_symmetry_delay_offsets_and_valid_corner_pairs(self):
        self.run_case("edge-adapter")

    def test_wall_edge_real_adc_rotation_rejection_and_mode_reentry(self):
        self.run_case("edge-adc-mode")

    def test_wall_edge_detects_short_walls_and_walls_at_sixty_two_to_sixty_five_mm(self):
        self.run_case("edge-recall-wall")

    def test_wall_edge_detects_weak_rises_without_waiting_for_sixty_eight_mm(self):
        self.run_case("edge-recall-weak")

    def test_wall_edge_keeps_crossings_between_encoder_pulses_but_rejects_long_stops(self):
        self.run_case("edge-recall-zero")

    def test_wall_edge_real_machine_calibration_detects_sixty_five_mm_wall_during_zero_count(self):
        self.run_case("edge-recall-adc")

    def test_disabled_control_keeps_observing_without_applying_wall_errors(self):
        self.run_case("integration-disabled")

    def test_diagonal_low_speed_gate_and_velocity_gain_are_preserved(self):
        self.run_case("integration-speed-gate")

    def test_search_turn_exit_correction_preserves_sign_limits_and_left_right_symmetry(self):
        self.run_case("search-turn-post-correction")

    def test_search_turn_heading_preserves_residual_in_the_new_passage_frame(self):
        self.run_case("search-turn-heading")

    def test_search_turn_exit_discards_rotating_adc_history_without_erasing_heading(self):
        self.run_case("search-turn-exit")

    def test_search_turn_front_gate_confirms_travel_and_retains_uncertain_or_large_corrections(self):
        self.run_case("search-turn-guard")

    def test_search_turn_front_gate_uses_actual_machine_calibration_and_control_mode(self):
        self.run_case("search-turn-ir")

    def test_search_turn_diagnostics_bound_records_but_count_all_decisions(self):
        self.run_case("search-turn-diagnostics")

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

    def test_two_state_ekf_heading_observability_covariance_and_wall_dropout(self):
        self.run_case("ekf-math")

    def test_ekf_real_adc_wall_loss_reacquisition_and_straight_frame_continuity(self):
        self.run_case("ekf-adapter")

    def test_ekf_corner_updates_once_shadow_equivalence_and_mode_reset(self):
        self.run_case("ekf-corner-mode")

    def test_wall_ekf_shell_rejects_busy_changes_and_preserves_irq_mask(self):
        self.run_case("ekf-shell")


if __name__ == "__main__":
    unittest.main()
