"""実マーカー全必要コマ照合後の別解析。元CSV/試行/旧レポートは保持。"""
from pathlib import Path
from datetime import datetime, timezone
import copy
import hashlib
import json
import math
import subprocess
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
base = root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
out = base/'geometry_same_candidate_three_review_v2_manual_pairs'
assert not out.exists()
out.mkdir()
read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')

sys.path[:0] = [str(root/'tools'), str(root/'tools/analysis')]
from pixel_camera_relative_pose import required_motion_quality

old_path = base/'geometry_same_candidate_three_review_v1/review.json'
old = read(old_path)
manifest_path = base/'all_required_marker_pairs_review_v1/manifest.json'
manifest = read(manifest_path)
assert [len(r['frames']) for r in manifest['runs']] == [160,160,159]
reviewer_path = root/'tmp/review_turn1600_B_outr135_midpoint_20261121.py'
reviewer = reviewer_path.read_text(encoding='utf-8-sig')
rejected = {1: [], 2: [], 3: [2287,2289]}
manual = dict(state='ALL_REQUIRED_FRAMES_VISUALLY_REVIEWED',
    reviewed_required_frames=479, rejected_frames_by_run=rejected,
    evidence=str(manifest_path), evidence_sha256=sha(manifest_path),
    findings='必要区間の全12シートを目視。3本目2287/2289は前点が床模様、後点が基板上。その他の元有効点は実前緑/後赤に重なる。欠測は回復・補間しない。',
    source_data_modified=False, original_archives_preserved=True,
    mask_only=True, acceptance_thresholds_changed=False,
    marker_detector_profile_changed=False, hardware_operations=0)
save(out/'manual_visual_review.json',manual)
rows = copy.deepcopy(old['runs'])
for number, row in enumerate(rows,1):
    run = Path(row['analysis_trial']).parent
    folder = out/f'run{number}'
    folder.mkdir()
    pose_path = run/'full/apparent_floor_marker_pose.csv'
    assert sha(pose_path) == manifest['runs'][number-1]['pose_sha256']
    assert sha(Path(row['geometry_review'])) == row['geometry_review_sha256']
    pose = pd.read_csv(pose_path)
    prior = read(Path(row['geometry_review']))
    excluded = pose.frame.isin(rejected[number]).to_numpy()
    assert excluded.sum() == len(rejected[number])
    times = pose.time_video_s.to_numpy()
    for label in ('entry_window_pts_s','exit_window_pts_s'):
        low,high = prior[label]
        assert not (excluded & (times>=low) & (times<=high)).any(),label
    late = prior['late_stationary']['window_pts_s']
    assert not (excluded & (times>=late[0]) & (times<=late[1])).any()
    # 計測点の補間両端にも誤対応がないことを先に検査する。
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    valid_indices = np.flatnonzero(valid)
    stamp = prior['Lend_video_pts_s']
    location = np.searchsorted(times[valid],stamp)
    assert not excluded[valid_indices[location-1:location+1]].any()
    changed = reviewer
    replacements = {
        "OUT=RUN/'midpoint_raw_wheel_clock_v1'": f'OUT=Path({str(folder/"midpoint_raw_wheel_clock_v1")!r})',
        'observed=pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)':
            'observed=pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)\n'
            f'observed &= ~np.isin(pose.frame.to_numpy(), {rejected[number]!r})',
    }
    for before,after in replacements.items():
        assert changed.count(before) == 1
        changed = changed.replace(before,after)
    executable = folder/'midpoint_reviewer_with_explicit_mask.py'
    executable.write_text(changed,encoding='utf-8')
    # 元スクリプトのROOTは配置依存なので、実行用だけ明示rootへ置換し記録する。
    before = 'ROOT=Path(__file__).resolve().parents[1]'
    assert changed.count(before) == 1
    changed = changed.replace(before, f'ROOT=Path({str(root)!r})')
    executable.write_text(changed,encoding='utf-8')
    with (folder/'review.stdout.txt').open('x',encoding='utf-8') as stream:
        completed = subprocess.run([sys.executable,str(executable),str(run.relative_to(base))],
            stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
    assert completed.returncode == 0, (number,completed.returncode)
    new_path = folder/'midpoint_raw_wheel_clock_v1/midpoint_review.json'
    geometry = read(new_path)
    for key in ('exit_path_direction_change_deg','midpoint_Lend_right_forward_mm',
                'Lstart_delta_from_exit_line_mm','clock','late_stationary'):
        assert geometry[key] == prior[key],key
    assert geometry['independent_optical_geometry_eligible']
    trial = read(run/'trial.json')
    stream = pd.read_csv(run/'telemetry.csv')
    full_quality = prior['original_full_required_motion_quality']
    valid &= ~excluded
    recomputed_full_quality = required_motion_quality(times, valid,
        pose.calibration_valid.to_numpy(bool), stream.time_ms.to_numpy(), trial,
        full_quality['video_motion_start_s']-(trial['log_quality']['motion_start_time_ms']-float(stream.time_ms.iloc[0]))/1000,1)
    assert recomputed_full_quality['accepted_for_relative_review']
    independent = geometry['independent_clock_required_motion_quality']
    required = (times>=independent['required_interval_pts_s'][0]) & (times<=independent['required_interval_pts_s'][1])
    save(folder/'exclusion_and_quality.json',dict(number=number,rejected_frames=rejected[number],
        reason='実際の前後マーカー以外を結んだ誤対応のみ。位置残差・角度目標で除外点を選んでいない。',
        full_model_clock_quality=recomputed_full_quality, independent_clock_quality=independent,
        required_frames=int(required.sum()),valid_required_frames=int((valid&required).sum()),
        metric_fit_windows_unchanged=True,Lend_interpolation_neighbors_unchanged=True,
        original_reviewer=str(reviewer_path),original_reviewer_sha256=sha(reviewer_path),
        executed_reviewer_sha256=sha(executable),source_pose_sha256=sha(pose_path),
        source_geometry_review=str(row['geometry_review']),
        source_geometry_review_sha256=row['geometry_review_sha256'],
        source_video_sha256=read(run/'full/apparent_floor_marker_pose.provenance.json')['video_sha256'],
        source_data_modified=False,hardware_operations=0))
    row.update(previous_geometry_review=row['geometry_review'],
        previous_geometry_review_sha256=row['geometry_review_sha256'],
        geometry_review=str(new_path),geometry_review_sha256=sha(new_path),
        required_observation_fraction=independent['observation_rate'],
        registration_fraction=independent['calibration_rate'],
        rejected_false_pair_frames=rejected[number],manual_reviewed_required_frames=int(required.sum()))
    print(json.dumps(dict(run=number,observation=independent['observation_rate'],rejected=rejected[number],metrics_unchanged=True)),flush=True)

result = copy.deepcopy(old)
result.update(recorded_at=datetime.now(timezone.utc).isoformat(),
    state='THREE_RUN_RELATIVE_SINGLE_TURN_CANDIDATE_AFTER_MANUAL_PAIR_QA',
    runs=rows, supersedes_comparison=str(old_path), supersedes_comparison_sha256=sha(old_path),
    manual_pair_QA=str(out/'manual_visual_review.json'),
    numerical_angle_Lend_and_stop_metrics_unchanged=True,
    superseding_reason='3本目2コマの実マーカー以外の誤対応を無効化。全必要区間479コマを目視確認。')
save(out/'review.json',result)
plt.rcParams['font.family']='Yu Gothic'
fig,ax=plt.subplots(figsize=(6,7),layout='constrained')
for number,row in enumerate(rows,1):
    pose=pd.read_csv(out/f'run{number}/midpoint_raw_wheel_clock_v1/midpoint_relative_trajectory.csv')
    valid=pose.registered_midpoint_valid.astype(bool)
    ax.plot(pose.right_mm.where(valid),pose.forward_mm.where(valid),label=f'{number}本目 {row["exit_angle_deg"]:.3f}°')
    ax.scatter(*row['Lend_right_forward_mm']);ax.scatter(*row['stop_right_forward_mm'],marker='+')
ax.scatter(67.5*math.sqrt(2),112.5*math.sqrt(2),marker='x',color='black',label='Lend目標')
ax.scatter(112.5*math.sqrt(2),67.5*math.sqrt(2),marker='s',facecolors='none',edgecolors='black',label='停止目標（別評価）')
ax.set(xlabel='入口に対する右 [mm]',ylabel='入口に対する前 [mm]',aspect='equal',title='機体B1600 右135度出口 / 誤対応除外後')
ax.grid(alpha=.3);ax.legend(fontsize=8);fig.savefig(out/'three_trajectories.png',dpi=140);plt.close(fig)
text='# 機体B1600 右135°出口・3本比較（実マーカー照合後）\n\n'
text+='候補：**半径−41 mm、Lstart 5.3 mm／Lend 31.0 mm、角度補正＋1.0°**、速度1.6 m/s、吸引650。PID/FF・実機モード・前後走設定は[review.json](review.json)に記録した。\n\n'
text+='必要区間の全479コマを目視確認。3本目のframe 2287・2289で、床模様を前マーカー、基板上の点を後マーカーとした誤対応を見つけた。元動画・CSV・旧レポート・アーカイブは保持し、別解析でこの2点のみ無効化した。欠測点の補間回復や検出閾値の変更は行っていない。元の3本比較v1は本レポートで更新する。\n\n'
text+='|試行|出口角 [°]|Lend目標差 [mm]|必要区間の有効観測率|\n|---|---:|---:|---:|\n'
for number,row in enumerate(rows,1):
    text+=f'|{number}|{row["exit_angle_deg"]:.3f}|{row["Lend_error_mm"]:.3f}|{row["required_observation_fraction"]*100:.3f}%|\n'
stats=result['statistics']
text+=f'\n誤対応2点は入口時計・出口直線・Lend補間・停止窓の外にあり、再計算した各数値は元結果と完全一致。3本平均角 **{stats["mean_exit_angle_deg"]:.3f}°**、Lend目標差最大 **{stats["maximum_Lend_error_mm"]:.3f} mm**。全評価フレームの実ArUco登録100%、実マーカー観測95%以上を維持した。3本とも走行ログ欠落/CRC0、出力飽和0、操作者確認は両方OFF・接触なし。\n\n'
text+=f'停止点の目標差は{stats["stop_error_range_mm"][0]:.2f}〜{stats["stop_error_range_mm"][1]:.2f} mm。後走90 mm指令に対し中点移動は約96 mmであり、ターン長と別の未解決事項として残す。\n\n'
text+='![誤対応を無効化した3軌跡](three_trajectories.png)\n\n'
text+='実測入口方向に合わせた相対マーカー中点の単体候補。絶対車軸・機軸・盤面精度、左出口、連続接続、BのEKF係数は未確認。出口値のソース反映・実機書込みは未実施。入口in135は別途、左右のソース反映と機体Bビルドを完了済み（左は右の鏡映、未測定）。\n\n'
text+='BLEは標準手順で今回1回取得できた。停止中の20/10/5 ms通信試験も通過したが、切断原因の特定や長期安定の確認は完了していない。元summaryのDETAILED_REVIEW_REQUIRED・採用制限は維持する。追加走行は自動予定していない。\n\n'
text+='照合画像と全コマ一覧は[manifest.json](../all_required_marker_pairs_review_v1/manifest.json)、除外根拠は[manual_visual_review.json](manual_visual_review.json)。再現用スクリプト・入力ハッシュ・除外後の品質は各runフォルダに保存した。\n'
(out/'README.md').write_text(text,encoding='utf-8')
state_path=base/'resume_state.json';state=read(state_path)
state.update(state='THREE_RUN_RELATIVE_CANDIDATE_MANUAL_PAIR_QA_RECORDED_BOTH_OFF',
    candidate_status=result['state'],comparison_review=str(out/'review.json'),
    comparison_superseding_review_pending=None,
    video_required_observation_fraction=rows[-1]['required_observation_fraction'],
    next_action='照合済み右out135候補と停止点差を報告する。追加走行やソース反映は自動予定しない。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n追加の全必要コマ目視で3本目frame2287/2289の誤対応を発見し、別解析で無効化した。旧比較v1を保持し、geometry_same_candidate_three_review_v2_manual_pairs/README.mdで更新する。角度/Lend/停止数値は不変、補正後も観測95%以上・実ArUco100%。両方OFFのまま保存済み資料だけを処理した。\n')
print(json.dumps(dict(state=result['state'],statistics=result['statistics']),ensure_ascii=False),flush=True)
