"""未使用stageに、操作者指定の概略開始確認を設定する。実機操作なし。"""
from pathlib import Path
import argparse
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--stage', type=Path, required=True)
a = p.parse_args()
stage = a.stage.resolve()
assert stage.is_relative_to(ROOT / 'tools/logs/pixel_camera')
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
identifier = read(stage / 'phase.json')['request_id']
assert not (stage / 'local_execution/jobs' / (identifier + '.json')).exists()
assert not (stage / 'local_execution' / (identifier + '.stdout.txt')).exists()
wrapper = stage / 'run_recorded_once.py'
source = wrapper.read_text(encoding='utf-8')
assert 'original_precision_result_path' not in source, '過去の配置を参照するwrapperはコピー元に使いません'
old = """    if checked['passed'] is not True:
        raise RuntimeError(f'開始位置または向きを照合できません: {result}')
    return checked
"""
new = """    if checked['passed'] is not True:
        # 今回撮影した実観測で概略開始位置を確認する。原本の不合格は保持。
        rough_ok = (checked['observed'] >= 30
            and checked['calibration']['per_frame_registration_required']
            and {2, 3}.issubset(checked['calibration']['used_marker_ids'])
            and checked['max_position_error_mm'] <= placement['position_tolerance_mm']
            and checked['representative_heading_error_deg'] <= placement['heading_tolerance_deg']
            and checked['heading_jitter_max_deg'] <= placement['heading_tolerance_deg']
            and checked['heading_jitter_rms_deg'] <= placement['heading_tolerance_deg']/4
            and checked['heading_half_window_difference_deg'] <= placement['heading_tolerance_deg']/4)
        if not rough_ok:
            raise RuntimeError(f'概略の開始位置または向きを照合できません: {result}')
        rough = dict(passed=True, acceptance_scope='operator_requested_approximate_initial_placement_only',
            original_precision_result=checked, original_precision_result_path=str(result),
            center_board_mm=checked['center_board_mm'], heading_deg=checked['heading_deg'],
            observed=checked['observed'], frames=checked['frames'], observed_fraction=checked['observed_fraction'],
            placement=placement, fixed_board_accepted=False, precise_start_accepted=False,
            observations_imputed=False, motion_analysis_thresholds_changed=False,
            evidence_source='fresh_current_trial_static_video')
        tuning.save_json(folder/'rough_start_acceptance.json', rough)
        return rough
    return checked
"""
assert source.count(old) == 1
changed = source.replace(old, new)
changed = changed.replace('position_tolerance_mm=5., heading_tolerance_deg=2.)',
                          'position_tolerance_mm=5., heading_tolerance_deg=5.)')
assert 'position_tolerance_mm=5., heading_tolerance_deg=5.)' in changed
compile(changed, str(wrapper), 'exec')
policy = dict(operator_instruction='初期配置をおおよそ確認して，走行までさせてください．',
    initial_position_tolerance_mm=5., initial_heading_tolerance_deg=5.,
    minimum_actual_initial_observations=30, fresh_trial_static_video_required=True,
    cached_previous_trial_pose_used=False, original_precision_failures_preserved=True,
    motion_analysis_thresholds_changed=False, motion_observation_minimum=.95,
    fixed_board_accepted=False, parameter_adoption_allowed=False,
    previous_wrapper_sha256=hashlib.sha256(source.encode('utf-8')).hexdigest(),
    wrapper_sha256=hashlib.sha256(changed.encode('utf-8')).hexdigest())
with (stage / 'approximate_start_policy.json').open('x', encoding='utf-8') as f:
    json.dump(policy, f, ensure_ascii=False, indent=2)
wrapper.write_text(changed, encoding='utf-8', newline='\n')
print('Fresh approximate-start check prepared; no connection or hardware command.')
