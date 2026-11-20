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
import re
import shlex
import shutil
import subprocess
import sys
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


def parameter_sources(mouse):
    return [str(path) for path in sorted((ROOT / "Params" / ("Param_" + mouse)).rglob("*.cpp"))]


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
        start = shell.index("static int usrcmd_wall_edge(int argc, char **argv)\n{")
        stop = shell.index("static int usrcmd_wall_ekf(int argc, char **argv)\n{", start)
        (Path(cls.work.name) / "wall_edge_command.inc").write_text(shell[start:stop], encoding="utf-8")
        motion = (ROOT / "Task/Src/init_motion.cpp").read_text(encoding="utf-8")
        function = motion.index("void Motion::Init_Motion_search_turn")
        start = motion.index("\tfloat diff = vehicle->ego.x_point.get()", function)
        stop = motion.index("\t(motion_plan.end_radian.get() > 0)", start)
        (Path(cls.work.name) / "search_turn_post_correction.inc").write_text(motion[start:stop], encoding="utf-8")
        runtime = (ROOT / "Task/Src/motion.cpp").read_text(encoding="utf-8")
        begin = runtime.index('{', runtime.index('MotionHeading::Prior Motion::HeadingForLog()')) + 1
        end = runtime.index('\n}', begin)
        (Path(cls.work.name) / 'heading_for_log.inc').write_text(runtime[begin:end], encoding='utf-8')
        led = (ROOT / "Peripheral/Src/interface.c").read_text(encoding="utf-8")
        (Path(cls.work.name) / "led_interface.inc").write_text(
            led[led.index('static uint8_t led_requested_state;'):led.index('t_bool Button_Status()')], encoding="utf-8")
        for signature, filename in (("void  Motion::SetIdeal_search_turn()", "search_turn_runtime.inc"),
                                    ("void Motion::SetIdeal_wall_control()", "search_wall_control.inc")):
            start = runtime.index('{', runtime.index(signature)) + 1
            stop = runtime.index('\n}\n', start)
            (Path(cls.work.name) / filename).write_text(runtime[start:stop], encoding="utf-8")
        turn = (ROOT / "Task/Src/turn_motion.cpp").read_text(encoding="utf-8")
        irq = (ROOT / "Module/Src/interrupt.cpp").read_text(encoding="utf-8")
        for prefix, filename in [('pose_input.heading_prior =', 'heading_prior_input.inc'),
                                 ('ir_sens.UpdateTurnHeadingPosterior(', 'heading_prior_output.inc')]:
            begin = irq.index(prefix)
            end = irq.index(';', begin) + 1
            (Path(cls.work.name) / filename).write_text(irq[begin:end], encoding='utf-8')
        setters = [call for call in re.findall(r'ApplyWallPositionCorrection\([^;]+;', turn)
                   if 'detect_wall_edge_di' in call and 'void Motion::' not in call]
        assert len(setters) == 2
        code = 'switch (which) {\n' + ''.join('case {}: {}; break;\n'.format(i, line.rstrip(';'))
                                              for i, line in enumerate(setters)) + '}\n'
        (Path(cls.work.name) / "diagonal_turn_delay.inc").write_text(code, encoding="utf-8")
        start = turn.index('{', turn.index('void Motion::ApplyWallPositionCorrection(')) + 1
        stop = turn.index('\n}\n', start)
        (Path(cls.work.name) / "wall_position_apply.inc").write_text(turn[start:stop], encoding="utf-8")
        for method,filename in [('handleTurnPrev_Straight','shortest_prev.inc'),('handleTurnPrev_Diagonal','shortest_diagonal_prev.inc'),('handleTurnMain','shortest_main.inc'),('SetIdeal_turn_in','shortest_update.inc'),('handleTurnPost_Straight','shortest_post.inc')]:
            begin=turn.index('{',turn.index('void Motion::'+method))+1
            end=turn.index('\n}',begin)
            (Path(cls.work.name)/filename).write_text(turn[begin:end],encoding='utf-8')
        begin=turn.index('{',turn.index('void Motion::handleTurnPost_Diagonal'))+1
        end=turn.index('\n}',begin)
        (Path(cls.work.name)/'diagonal_post.inc').write_text(turn[begin:end],encoding='utf-8')
        begin=motion.index('if (continue_diagonal_pose) ir_sens->RebaseDiagonalFeatureCandidates')
        end=motion.index('\n\tmotion_pattern_set(Diagonal)',begin)
        reset=motion.index('if (!continue_diagonal_pose)',end)
        stop=motion.index('\n\tir_sens->r_corner_length',reset)
        (Path(cls.work.name)/'diagonal_resume.inc').write_text(motion[begin:end]+'\n'+motion[reset:stop],encoding='utf-8')
        for source,filename in [(motion,'long_v90_init.inc'),
                                ((ROOT/'Task/Src/init_motion2.cpp').read_text(encoding='utf-8'),'variable_long_v90_init.inc')]:
            begin=source.index('void Motion::Init_Motion_long_turn_v90')
            prior=source.index('const bool continue_diagonal_pose',begin)
            prior_end=source.index(';',prior)+1
            begin=source.index('if (!continue_diagonal_pose) ir_sens->ResetStraightWallControl',prior_end)
            end=source.index('\n\terror_counter_reset();',begin)
            (Path(cls.work.name)/filename).write_text(source[prior:prior_end]+'\n'+source[begin:end],encoding='utf-8')
        begin=runtime.index('{',runtime.index('void Motion::RecordPillarCandidates()'))+1
        end=runtime.index('\n}',begin)
        (Path(cls.work.name)/'shortest_record.inc').write_text(runtime[begin:end],encoding='utf-8')
        begin=runtime.index('ir_sens->RebaseStraightFeatureCandidates(-motion_plan.end_length.get());')
        end=runtime.index('\n',runtime.index('vehicle->ego.length.set',begin))
        (Path(cls.work.name)/'shortest_rebase.inc').write_text(runtime[begin:end],encoding='utf-8')
        begin=runtime.index('const auto feature_frame=',runtime.index('void Motion::SetIdeal_straight()'))
        end=runtime.index('\n\tmotion_control_set',begin)
        (Path(cls.work.name)/'shortest_run.inc').write_text(runtime[begin:end],encoding='utf-8')
        begin=motion.index('const bool continue_wall_pose',motion.index('void Motion::Init_Motion_straight'))
        end=motion.index('\n\tif(',begin)
        reset=motion.index('ir_sens->ResetStraightWallControl(continue_wall_pose)',end)
        stop=motion.index('\n\tir_sens->r_corner_length',reset)
        (Path(cls.work.name)/'shortest_resume.inc').write_text(motion[begin:end]+'\n'+motion[reset:stop],encoding='utf-8')
        ctrl=(ROOT/'Task/Src/ctrl_task.cpp').read_text(encoding='utf-8')
        begin=ctrl.index('case Run_Pause:')+len('case Run_Pause:')
        end=ctrl.index('\n\t\t\tbreak;',begin)
        (Path(cls.work.name)/'shortest_pause.inc').write_text(ctrl[begin:end],encoding='utf-8')
        for mouse in ("A", "B"):
            executable = Path(cls.work.name) / ("side-wall-" + mouse + (".exe" if os.name == "nt" else ""))
            invocation = command + [
                "-std=c++14", "-O0", "-Wall", "-Wextra", "-fno-exceptions", "-fno-rtti",
                "-DMOUSE_" + mouse, "-I", str(TEST_DIR / "side_wall_stubs"),
                "-I", str(ROOT), "-I", cls.work.name, str(TEST_DIR / "side_wall_stability_harness.cpp"),
                str(ROOT / "Task" / "Src" / "ir_sens_task.cpp"),
                str(ROOT / "Task" / "Src" / "straight_wall_control.cpp"),
                str(ROOT / "Task" / "Src" / "diagonal_pose.cpp"), "-o", str(executable),
                *parameter_sources(mouse),
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

    def test_wall_and_pillar_diagnostic_slots_reset_defaults_after_full_buffer_reuse(self):
        self.run_case("wall-diagnostics-reuse")

    def test_turn_wall_detections_keep_opposite_and_missing_edges_even_after_correction(self):
        sys.path.insert(0, str(ROOT/'tools'))
        from ble import wall_diagnostics
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), 'turn-wall-detections'], capture_output=True,
                                        text=True, errors='replace', timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                data = wall_diagnostics.empty_pillar_diagnostics()
                for line in result.stdout.splitlines():
                    if not line.strip():
                        continue
                    tag, *items = line.split()
                    wall_diagnostics.receive_detection(data, tag, dict(item.split(':', 1) for item in items))
                self.assertTrue(wall_diagnostics.validate_detection(data), data)
                self.assertEqual([e['kind'] for e in data['events']], ['edge', 'phase_end', 'phase_end'])
                before = data['events'][1]
                self.assertEqual((before['phase'], before['expected_side'], before['expected_seen']), ('before', '2', '0'))
                self.assertEqual((before['left_count'], before['right_count']), ('1', '0'))
                self.assertEqual(data['events'][2]['end'], 'stop')

    def test_turn_exit_range_yaw_disagreement_uses_parallel_legacy_control(self):
        self.run_case("ekf-exit-disagreement")

    def test_feature_ekf_nonlinear_prediction_covariance_rebase_and_outliers(self):
        self.run_case("feature-ekf-math")

    def test_machine_specific_near_pillar_width_does_not_fire_wall_edges_on_either_side(self):
        self.run_case("machine-pillar-width")

    def test_shortest_entry_uses_real_motion_context_rebase_side_gate_and_rotation_cutoff(self):
        self.run_case("shortest-feature-entry")

    def test_shortest_cardinal_turn_retains_measured_exit_heading_once_with_fresh_ranges(self):
        self.run_case("shortest-exit-heading")

    def test_connected_straight_in_diagonal_v90_out_straight_heading_and_covariance(self):
        self.run_case("motion-heading-chain")

    def test_real_turn_tracker_replaces_heading_marginal_without_double_fusing(self):
        self.run_case("turn-heading-posterior")

    def test_adopted_heading_log_selects_passage_turn_diagonal_and_rejects_invalid_state(self):
        self.run_case("heading-log")

    def test_local_straight_pose_updates_world_without_reapplying_resets_or_travel(self):
        self.run_case("pose-frame-straight")

    def test_diagonal_out_turn_seeds_straight_heading_only_from_calibrated_fresh_pose(self):
        self.run_case("shortest-diagonal-exit-heading")

    def test_diagonal_pose_route_count_gaps_and_ambiguity(self):
        self.run_case("diagonal-pose-route")

    def test_diagonal_pose_turn_handoff_rotates_pose_covariance_and_expires(self):
        self.run_case("diagonal-pose-handoff")

    def test_diagonal_pose_steering_on_real_ir_and_shadow_fallback(self):
        self.run_case("diagonal-pose-steering")

    def test_feature_coordinate_review_negative_entry_and_existing_run_origins(self):
        self.run_case("feature-coordinate-review")

    def test_open_origin_wide_pillar_does_not_steer_as_a_wall_and_preserves_opposite_wall(self):
        self.run_case("open-origin-wall-control")

    def test_recorded_mouse_a_open_pillars_keep_features_without_false_wall_control(self):
        # A measured ADC data must not be reused as a B calibration.
        result = subprocess.run([str(self.executables['A']), 'recorded-open-pillar',
                                 str(TEST_DIR/'fixtures/straight_open_pillar_20261118.csv')],
                                capture_output=True, text=True, errors='replace', timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_known_feature_injection_preserves_pending_peak_but_unknown_jump_is_rejected(self):
        self.run_case("feature-coordinate-correction")

    def test_shortest_exit_handoff_periodic_features_and_duplicate_suppression(self):
        sys.path.insert(0,str(ROOT/'tools'))
        from ble import wall_diagnostics
        for mouse,executable in self.executables.items():
            with self.subTest(mouse=mouse):
                r=subprocess.run([str(executable),'shortest-feature-exit'],capture_output=True,text=True,errors='replace',timeout=15)
                self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                walls=wall_diagnostics.empty_diagnostics();pillars=wall_diagnostics.empty_pillar_diagnostics()
                for line in r.stdout.splitlines():
                    if not line.strip():continue
                    tag,*items=line.split();values=dict(item.split(':',1) for item in items)
                    if tag in ('PILLAR_STATS','PILLAR_EDGE'):wall_diagnostics.receive_pillar(pillars,tag,values)
                    else:wall_diagnostics.receive(walls,tag,values)
                self.assertTrue(wall_diagnostics.validate(walls),walls)
                self.assertTrue(wall_diagnostics.validate_pillar(pillars),pillars)
                self.assertEqual(pillars['summary']['applied'],'1' if mouse=='A' else '0')
                self.assertEqual(pillars['events'][0]['position_phase'],'5')

    def test_diagonal_shadow_model_calibration_pose_gates_and_kalman_proposal_never_apply(self):
        self.run_case('diagonal-feature-model')

    def test_diagonal_shadow_real_motion_handoff_preserves_pose_and_diagnostics(self):
        sys.path.insert(0,str(ROOT/'tools'))
        from ble import wall_diagnostics
        for mouse,executable in self.executables.items():
            with self.subTest(mouse=mouse):
                r=subprocess.run([str(executable),'diagonal-feature-adapter'],capture_output=True,text=True,errors='replace',timeout=15)
                self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                walls=wall_diagnostics.empty_diagnostics();pillars=wall_diagnostics.empty_pillar_diagnostics()
                for line in r.stdout.splitlines():
                    if not line.strip():continue
                    tag,*items=line.split();values=dict(item.split(':',1) for item in items)
                    if tag in ('PILLAR_STATS','PILLAR_EDGE'):wall_diagnostics.receive_pillar(pillars,tag,values)
                    else:wall_diagnostics.receive(walls,tag,values)
                self.assertTrue(wall_diagnostics.validate(walls),walls)
                self.assertTrue(wall_diagnostics.validate_pillar(pillars),pillars)
                self.assertEqual(pillars['summary']['applied'],'0')
                self.assertEqual(pillars['events'][0]['position_status'],'uncalibrated')
                self.assertEqual(pillars['events'][0]['position_phase'],'6')
                self.assertEqual(pillars['events'][0]['diag_pose_status'],'6' if mouse=='A' else '1')
                self.assertEqual(pillars['events'][0]['diag_pose_pillar'],'1' if mouse=='A' else '0')

    def test_feature_ekf_real_adc_application_delay_shared_boundary_and_machine_gate(self):
        sys.path.insert(0, str(ROOT / "tools"))
        from ble import wall_diagnostics
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result=subprocess.run([str(executable),'feature-ekf-adapter'],capture_output=True,text=True,errors='replace',timeout=15)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                data=wall_diagnostics.empty_pillar_diagnostics()
                for line in result.stdout.splitlines():
                    if not line.strip():continue
                    tag,*items=line.split()
                    if tag in ('PILLAR_STATS','PILLAR_EDGE'):
                        wall_diagnostics.receive_pillar(data,tag,dict(item.split(':',1) for item in items))
                self.assertTrue(wall_diagnostics.validate_pillar(data),data)
                self.assertEqual(data['summary']['applied'],'1' if mouse=='A' else '0')

    def test_all_wall_and_pillar_detectors_indicate_valid_sides_without_latching_or_log_dependency(self):
        self.run_case("wall-pillar-leds")

    def test_search_turn_runtime_blocks_rotating_edges_and_primes_exit_after_mode_change(self):
        self.run_case("search-turn-runtime")

    def test_wall_position_limits_range_states_and_diagnostic_bounds(self):
        self.run_case("wall-observation-diagnostics")

    def test_wall_edge_trial_rejects_pillar_grazes_but_preserves_continuous_wall_arming(self):
        self.run_case("wall-edge-trial-detector")

    def test_wall_edge_trial_selects_before_latching_and_keeps_shadow_coordinates_unchanged(self):
        self.run_case("wall-edge-trial-decision")

    def test_wall_edge_trial_actual_motion_application_does_not_consume_rejected_candidates(self):
        sys.path.insert(0, str(ROOT / "tools"))
        from ble import wall_diagnostics
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), "wall-edge-trial-motion"], capture_output=True,
                                        text=True, errors="replace", timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                data = wall_diagnostics.empty_diagnostics()
                for line in result.stdout.splitlines():
                    if not line.strip():
                        continue
                    tag, *items = line.split()
                    if tag in ('WALL_CORR_STATS', 'WALL_RANGE_STATS', 'WALL_CORR'):
                        wall_diagnostics.receive(data, tag, dict(item.split(':', 1) for item in items))
                self.assertTrue(wall_diagnostics.validate(data), data)
                self.assertEqual(data['summary']['shadow'], '1')
                self.assertEqual(data['summary']['stale_generation'], '1')
                shadow = next(e for e in data['events'] if e['reason'] == 'shadow')
                self.assertEqual(float(shadow['delta_mm']), 0.0)
                self.assertEqual(float(shadow['proposed_after_mm']), 11.0)

    def test_wall_edge_trial_shell_changes_only_while_idle_and_resets_old_candidates(self):
        self.run_case("wall-edge-trial-shell")

    def test_straight_pillar_adc_peaks_need_open_before_and_after_and_work_beyond_range_limit(self):
        self.run_case("straight-pillar-peak")

    def test_close_open_pillar_is_retained_and_never_becomes_a_wall_edge_on_either_side(self):
        self.run_case("straight-close-pillar-history")

    def test_legacy_adopts_separate_pillar_pairs_and_calibrated_direct_position_without_ekf(self):
        self.run_case("legacy-pillar-adoption")

    def test_legacy_pillar_direct_correction_roundtrips_actual_firmware_diagnostics(self):
        sys.path.insert(0,str(ROOT/'tools'))
        from ble import wall_diagnostics
        for mouse,executable in self.executables.items():
            with self.subTest(mouse=mouse):
                r=subprocess.run([str(executable),'legacy-pillar-wire'],capture_output=True,text=True,timeout=15)
                self.assertEqual(r.returncode,0,r.stdout+r.stderr)
                data=wall_diagnostics.empty_pillar_diagnostics()
                for line in r.stdout.splitlines():
                    if not line.strip():continue
                    tag,*items=line.split()
                    wall_diagnostics.receive_pillar(data,tag,dict(item.split(':',1) for item in items))
                self.assertTrue(wall_diagnostics.validate_pillar(data),data)
                self.assertEqual(data['summary']['applied'],'1' if mouse=='A' else '0')
                self.assertEqual(data['events'][0]['mode'],'legacy')
                self.assertEqual(data['events'][0]['ekf'],'0')

    def test_straight_feature_history_expires_on_long_walls_and_recovers_after_stops_and_invalid_samples(self):
        self.run_case("straight-history-recovery")

    def test_diagonal_pillar_keeps_flat_bottom_and_detects_early_peak_without_ten_mm_fill(self):
        self.run_case("diagonal-pillar-minimum")

    def test_pillar_rearm_requires_global_separation_and_taints_rotating_candidates(self):
        self.run_case("pillar-quality-rearm")

    def test_pillar_actual_adc_pose_front_snapshots_and_shadow_wire_roundtrip(self):
        sys.path.insert(0, str(ROOT / "tools"))
        from ble import wall_diagnostics
        for mouse, executable in self.executables.items():
            with self.subTest(mouse=mouse):
                result = subprocess.run([str(executable), "pillar-ir-wire"], capture_output=True,text=True,timeout=15)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                data=wall_diagnostics.empty_pillar_diagnostics()
                for line in result.stdout.splitlines():
                    parts=line.split()
                    if parts and parts[0] in ('PILLAR_STATS','PILLAR_EDGE'):
                        wall_diagnostics.receive_pillar(data,parts[0],dict(p.split(':',1) for p in parts[1:]))
                self.assertTrue(wall_diagnostics.validate_pillar(data),data)
                self.assertEqual(data['summary']['diagonal'],'1')
                self.assertEqual(data['events'][0]['feature'],'diagonal_pillar_minimum')
                self.assertEqual(float(data['events'][0]['e_mm']),4.0)
                self.assertLess(int(data['events'][0]['minimum_ms']),int(data['events'][0]['ms']))
                self.assertEqual(data['events'][0]['reason'],'shadow')

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

    def test_straight_wall_steering_symmetry_heading_damping_and_limits(self):
        self.run_case("straight-wall-symmetry")

    def test_straight_wall_rejects_a_pillar_without_resetting_the_remaining_wall(self):
        self.run_case("straight-wall-independent")

    def test_straight_wall_spatial_confidence_stop_and_expected_heading_motion(self):
        self.run_case("straight-wall-spatial")

    def test_straight_wall_disable_invalid_input_and_estimator_sync(self):
        self.run_case("straight-wall-reset")

    def test_straight_wall_real_adc_adapter_front_wall_mode_and_speed_transitions(self):
        self.run_case("straight-wall-adapter")

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
