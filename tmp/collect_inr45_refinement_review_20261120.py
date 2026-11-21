"""保存済みの右45入口1800の比較資料を新しい出力先に集約する。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import csv
import hashlib
import json
import shutil
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1800_A_inr45_20261120_v1'
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'tools/analysis')]
from pixel_camera_tuning import setting_differences
from video_marker_pose import board_to_pose_transform, transform_points

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def save(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')

def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

INPUTS = set()

def load(path):
    INPUTS.add(Path(path))
    return read(path)

def project_confirmed_stop(run, analysis, review):
    full = run/'full_core_v1'
    pose = pd.read_csv(full/'apparent_floor_marker_pose.csv')
    reference = load(full/'marker_reference_used.json')
    geometry = load(full/'body_geometry_used.json')
    inverse = np.linalg.inv(board_to_pose_transform(reference))
    def midpoint(table):
        front = transform_points(table[['blue_x_mm','blue_y_mm']].to_numpy(), inverse).astype(float)
        rear = transform_points(table[['yellow_x_mm','yellow_y_mm']].to_numpy(), inverse).astype(float)
        return ((1-geometry['front_height_mm']/1500)*front+(1-geometry['rear_height_mm']/1500)*rear)/2
    points = midpoint(pose)
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    t = pose.time_video_s.to_numpy()
    start = review['entry_window_pts_s'][0]-.010
    baseline = valid & (t>=start-.18) & (t<=start-.04)
    assert baseline.sum() >= 10
    origin = np.median(points[baseline],axis=0)
    lo, hi = review['entry_window_pts_s']
    incoming = points[valid & (t>=lo) & (t<=hi)]
    d = np.linalg.svd(incoming-incoming.mean(0),full_matrices=False)[2][0]
    if d@(incoming[-1]-incoming[0])<0: d=-d
    basis = np.column_stack(([-d[1],d[0]],d))
    existing = pd.read_csv(analysis/'midpoint_review/midpoint_relative_trajectory.csv')
    difference = float(np.max(np.abs(((points-origin)@basis)[valid]-existing[['right_mm','forward_mm']].to_numpy()[valid])))
    assert difference < 1e-5
    stopped = pd.read_csv(analysis/'confirmed_stop_review/pose.csv')
    stop_valid = stopped.marker_detected_raw.to_numpy(bool) & stopped.calibration_valid.to_numpy(bool)
    endpoint = np.median(((midpoint(stopped)-origin)@basis)[stop_valid],axis=0)
    INPUTS.update([full/'apparent_floor_marker_pose.csv',full/'apparent_floor_marker_pose.provenance.json',
        analysis/'midpoint_review/midpoint_relative_trajectory.csv',analysis/'confirmed_stop_review/pose.csv'])
    return endpoint, difference

def current_row(label, stage, identifier):
    run = BASE/stage/'local_execution'/('run_'+identifier)
    analysis = BASE/'parallel_analysis'/identifier
    trial = load(run/'trial.json')
    actual = load(run/'saved_1ms.settings.json')
    requested = load(run/'requested.settings.json')
    review = load(analysis/'midpoint_review/midpoint_review.json')
    stopped = load(analysis/'confirmed_stop_review/review.json')
    tracking = load(analysis/'tracking_1ms_review/verification.json')
    assert trial['state']=='CAPTURED_REVIEW_REQUIRED' and trial['saved_log']['status']=='saved'
    assert trial['execution_write_attempted'] is True and trial['execution_resend_count']==0
    assert trial['log_quality']['lost_frames']==trial['log_quality']['crc_errors_during_trial']==0
    assert not setting_differences(actual,requested,quantized=True)
    assert review['clock']['passed'] and review['independent_clock_required_motion_quality']['accepted_for_relative_review']
    assert stopped['passed']
    assert all(review['segment_observation_quality'][s]['observation_rate']>=.95 for s in ('pure_motion','entry','exit'))
    stop, reconstruction = project_confirmed_stop(run,analysis,review)
    INPUTS.update(run/name for name in ('saved_1ms.csv','telemetry.csv','saved_1ms.settings.json',
        'requested.settings.json','marker_reference_used.json','body_geometry_used.json','archive.json'))
    residual = review['midpoint_Lend_residual_mm']
    return dict(label=label,trial_id=identifier,lstart_mm=actual['parameters']['lstart'],
        lend_mm=actual['parameters']['lend'],angle_deg=review['exit_path_direction_change_deg'],
        error_right_mm=residual[0],error_forward_mm=residual[1],error_norm_mm=float(np.linalg.norm(residual)),
        stop_right_mm=float(stop[0]),stop_forward_mm=float(stop[1]),
        stop_error_right_mm=float(stop[0]-135),stop_error_forward_mm=float(stop[1]-270),
        battery_min_v=tracking['battery_min_v'],pulse_speed_mm_s=tracking['pulse_speed_measured_time_mean_mm_s'],
        saturation_pct=tracking['nominal_1ms_tracking_metrics']['saturation_pct'],
        native_summary=review['original_summary'],original_early_stop_eligibility=review['independent_optical_geometry_eligible'],
        combined_motion_and_fixed_stop_passed=True,reconstruction_difference_mm=reconstruction,
        source_trial=str(run/'trial.json'),source_analysis=str(analysis),precise_Lend_available=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-name',required=True)
    parser.add_argument('--include-identity-checked',action='store_true')
    parser.add_argument('--include-postcharge-baseline',action='store_true')
    args=parser.parse_args()
    if args.include_postcharge_baseline and not args.include_identity_checked:
        parser.error('--include-postcharge-baseline also requires --include-identity-checked')
    assert args.output_name.isidentifier()
    out=BASE/args.output_name
    out.mkdir(exist_ok=False)
    original=load(BASE/'single_turn_candidate_3run_review_v1/review.json')
    rows=[]
    for i,row in enumerate(original['trials'],1):
        rows.append(dict(label=f'B{i}',trial_id=Path(row['source_trial']).parent.name.removeprefix('run_'),
            lstart_mm=2.,lend_mm=40.,angle_deg=row['exit_angle_deg'],
            error_right_mm=row['lend_error_right_mm'],error_forward_mm=row['lend_error_forward_mm'],
            error_norm_mm=row['lend_error_norm_mm'],stop_right_mm=row['stop_right_mm'],
            stop_forward_mm=row['stop_forward_mm'],stop_error_right_mm=row['stop_error_right_mm'],
            stop_error_forward_mm=row['stop_error_forward_mm'],battery_min_v=row['battery_min_v'],
            pulse_speed_mm_s=row['pulse_speed_mm_s'],saturation_pct=row['saturation_pct'],
            source_trial=row['source_trial'],precise_Lend_available=True))
    specs=[('C1','refine_Ls1_Le39p3_01','A_INR45_1800_Ls1_Le39p3_20261120_001'),
           ('B4','matched_baseline_retry1915_01','A_INR45_1800_matched_baseline_retry1915_20261120_001'),
           ('M1','mid_status_retry_02','A_INR45_1800_mid_status_retry_20261120_002'),
           ('M2','mid_exact_repeat_02','A_INR45_1800_mid_exact_repeat_20261120_002')]
    if args.include_identity_checked:
        specs.append(('M3','mid_identity_checked_01','A_INR45_1800_mid_identity_checked_20261121_001'))
    if args.include_postcharge_baseline:
        specs.append(('B5','postcharge_baseline_01','A_INR45_1800_postcharge_baseline_20261121_001'))
    rows.extend(current_row(*spec) for spec in specs)
    middle=[r for r in rows if r['label'].startswith('M')]
    baseline=[r for r in rows if r['label'].startswith('B')]
    def stats(data):
        return dict(n=len(data),mean_error_right_mm=float(np.mean([r['error_right_mm'] for r in data])),
            mean_error_forward_mm=float(np.mean([r['error_forward_mm'] for r in data])),
            mean_error_norm_mm=float(np.mean([r['error_norm_mm'] for r in data])),
            maximum_error_norm_mm=max(r['error_norm_mm'] for r in data),
            maximum_absolute_angle_error_deg=max(abs(r['angle_deg']-45) for r in data),
            minimum_angle_deg=min(r['angle_deg'] for r in data),maximum_angle_deg=max(r['angle_deg'] for r in data),
            minimum_battery_v=min(r['battery_min_v'] for r in data))
    failed=BASE/'mid_final_repeat_01/local_execution/run_A_INR45_1800_mid_final_repeat_20261120_001/trial.json'
    failed_trial=load(failed)
    assert failed_trial['state']=='FAILED' and failed_trial['saved_log']['status']=='failed'
    assert failed_trial['execution_write_attempted'] and failed_trial['execution_resend_count']==0
    code_before=load(BASE/'refine_Ls1_Le39p3_assets/analysis_code_before_matched_baseline.json')
    identity={Path(k).as_posix():dict(before=v,current=digest(k),same=v==digest(k)) for k,v in code_before['files'].items()}
    assert all(v['same'] for v in identity.values())
    spatial_path=BASE/'spatial_lines_diagnostic_v3/review.json'
    spatial=load(spatial_path) if spatial_path.exists() else None
    identity=None
    acquisition=None
    charged_pair=None
    if args.include_identity_checked:
        local=BASE/'mid_identity_checked_01/local_execution'
        identity=load(local/'firmware_identity.json')
        assert identity['verified'] and identity['actual_mouse']=='A' and identity['state']=='idle'
        final_run=local/'run_A_INR45_1800_mid_identity_checked_20261121_001'
        final_trial=load(final_run/'trial.json')
        operator=load(local/'operator_results/A_INR45_1800_mid_identity_checked_20261121_001.json')
        assert operator['both_powered_off'] and operator['contact_detected'] is False
        events=[json.loads(line) for line in (final_run/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        saved=next(e['at'] for e in events if e['event']=='saved_log_saved')
        transferred=next(e['at'] for e in events if e['event']=='recording_transferred')
        assert saved<transferred and final_trial['saved_log_capture_phase']=='after_confirmed_postroll_before_video_transfer'
        acquisition=dict(saved_log_at=saved,video_transferred_at=transferred,
            saved_log_capture_phase=final_trial['saved_log_capture_phase'],real_capture_verified=True,
            preset_parameters_before_apply=load(final_run/'before.settings.json')['parameters'])
        INPUTS.add(final_run/'events.jsonl')
        original_request=load(BASE/'mid_status_retry_02/candidate.settings.json')
        for label,stage,identifier in specs:
            if label.startswith('M'):
                request=load(BASE/stage/'local_execution'/('run_'+identifier)/'requested.settings.json')
                assert all(request[k]==original_request[k] for k in ('parameters','pid','feedforward','suction','turn_count'))
    if args.include_postcharge_baseline:
        local=BASE/'postcharge_baseline_01/local_execution'
        post_run=local/'run_A_INR45_1800_postcharge_baseline_20261121_001'
        post_trial=load(post_run/'trial.json')
        post_identity=load(local/'firmware_identity.json')
        post_operator=load(local/'operator_results/A_INR45_1800_postcharge_baseline_20261121_001.json')
        assert post_identity['verified'] and post_identity['actual_mouse']=='A'
        assert post_operator['both_powered_off'] and post_operator['contact_detected'] is False
        assert post_trial['saved_log_capture_phase']=='after_confirmed_postroll_before_video_transfer'
        post_actual=load(post_run/'saved_1ms.settings.json')
        middle_run=BASE/'mid_identity_checked_01/local_execution/run_A_INR45_1800_mid_identity_checked_20261121_001'
        middle_actual=load(middle_run/'saved_1ms.settings.json')
        assert all(post_actual[k]==middle_actual[k] for k in ('pid','feedforward','suction','turn_count'))
        assert all(post_actual['parameters'][k]==middle_actual['parameters'][k]
            for k in post_actual['parameters'] if k not in ('lstart','lend'))
        first=next(r for r in rows if r['label']=='M3')
        second=next(r for r in rows if r['label']=='B5')
        charged_pair=dict(first=first,second=second,
            battery_difference_v=second['battery_min_v']-first['battery_min_v'],
            pulse_speed_difference_mm_s=second['pulse_speed_mm_s']-first['pulse_speed_mm_s'],
            error_norm_difference_mm=second['error_norm_mm']-first['error_norm_mm'],
            angle_difference_deg=second['angle_deg']-first['angle_deg'],
            actual_pid_ff_suction_equal=True,causal_parameter_effect_identified=False,
            reason='One observation each; measured voltage and speed differ. Preserve the observed comparison without a causal claim.')
    report=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),mouse='A',motion='in_r45',speed_mm_s=1800,
        status='POSTCHARGE_CONTROL_COMPLETE_NO_NEW_ADOPTION' if args.include_postcharge_baseline else ('COMPARISON_COMPLETE_NEW_CANDIDATE_NOT_SELECTED' if args.include_identity_checked else 'CHARGING_AWAITING_THIRD_PRECISE_RUN'),
        requested_candidate=load(BASE/'mid_status_retry_02/candidate.settings.json'),
        baseline_stats=stats(baseline),midpoint_stats=stats(middle),measurements=rows,
        excluded_precise_trial=dict(source=str(failed),state=failed_trial['state'],
            execution_write_attempted=True,execution_resend_count=0,reason=failed_trial['saved_log'],
            video=failed_trial['video_file'],continuous_motion_retained=failed_trial['log_quality']['full_motion_retained']),
        supplementary_spatial_directions=spatial,analysis_code_identity=identity,
        evaluation_frame='Height-corrected marker midpoint relative to the measured entry only.',
        absolute_axle_calibrated=False,body_yaw_calibrated=False,board_fixed_alignment_claimed=False,
        firmware_table_modified=False,firmware_written=False,continuous_or_maze_validation=False,
        new_ekf_parameters_identified=False,parameter_adoption_allowed=False,
        firmware_identity=identity,acquisition_order_real_verification=acquisition,
        postcharge_pair=charged_pair,
        preferred_single_turn_candidate=dict(lstart_mm=2.,lend_mm=40.,r_min_mm=-53.,correction_deg=0.,
            source='Previously repeated candidate retained; no new firmware adoption',
            reason='New candidate has no demonstrated consistent improvement across the observed repeat/voltage conditions.'),
        tested_candidate_selected=False,precise_midpoint_runs=len(middle),midpoint_physical_runs=4 if args.include_identity_checked else 3,
        wrong_machine_static_capture=load(BASE/'mid_pretransfer_01/local_execution/preflight_before_ble_A_INR45_1800_mid_pretransfer_20261120_001/operator_machine_correction.json') if args.include_identity_checked else None,
        power_state='Operator confirmed stopped, no contact, both OFF' if args.include_identity_checked else 'Last operator confirmation: both OFF; battery charging',
        next_prepared_stage=None if args.include_identity_checked else str(BASE/'mid_identity_checked_01'),
        next_trial_id=None if args.include_identity_checked else 'A_INR45_1800_mid_identity_checked_20261121_001',
        host_tests=dict(capture=25,retained_log_and_one_shot=26,cli=27,mcp=7,ble_transport=38,identity_guard_cases=7,skipped=0))
    save(out/'review.json',report)
    fields=['label','trial_id','lstart_mm','lend_mm','angle_deg','error_right_mm','error_forward_mm','error_norm_mm',
            'stop_error_right_mm','stop_error_forward_mm','battery_min_v','pulse_speed_mm_s','saturation_pct']
    with (out/'measurements.csv').open('x',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    fig,axes=plt.subplots(1,2,figsize=(11,4.8),layout='constrained')
    colours={'B':'#777777','C':'#dd8800','M':'#1976d2'}
    for row in rows:
        colour=colours[row['label'][0]]
        axes[0].scatter(row['error_right_mm'],row['error_forward_mm'],color=colour,s=40)
        axes[0].annotate(row['label'],(row['error_right_mm'],row['error_forward_mm']),xytext=(5,5),textcoords='offset points')
        axes[1].scatter(row['battery_min_v'],row['angle_deg'],color=colour,s=40)
        axes[1].annotate(row['label'],(row['battery_min_v'],row['angle_deg']),xytext=(5,5),textcoords='offset points')
    axes[0].axhline(0,color='.6',linewidth=.8);axes[0].axvline(0,color='.6',linewidth=.8)
    axes[0].set(xlabel='Lend error right [mm]',ylabel='Lend error forward [mm]',
        title='Entry-relative marker midpoint\nB=2/40, C=1/39.3, M=1.5/39.6',
        xlim=(min(r['error_right_mm'] for r in rows)-.5,max(r['error_right_mm'] for r in rows)+.8),
        ylim=(min(r['error_forward_mm'] for r in rows)-.5,max(r['error_forward_mm'] for r in rows)+.5))
    axes[0].set_aspect('equal',adjustable='box')
    axes[1].axhline(45,color='.6',linewidth=.8)
    axes[1].set(xlabel='Loaded battery minimum [V]',ylabel='Exit path direction change [deg]',title='Observed conditions; no causal fit',
        xlim=(min(r['battery_min_v'] for r in rows)-.03,max(r['battery_min_v'] for r in rows)+.06),
        ylim=(min(r['angle_deg'] for r in rows)-.12,max(r['angle_deg'] for r in rows)+.15))
    for axis in axes:axis.grid(alpha=.25)
    fig.savefig(out/'comparison.png',dpi=180)
    table=['|試行|Lstart / Lend mm|出口方向°|右ずれmm|前ずれmm|ずれ距離mm|負荷時最低V|',
           '|---|---|---:|---:|---:|---:|---:|']
    table.extend(f"|{r['label']}|{r['lstart_mm']:.1f} / {r['lend_mm']:.1f}|{r['angle_deg']:.3f}|{r['error_right_mm']:+.3f}|{r['error_forward_mm']:+.3f}|{r['error_norm_mm']:.3f}|{r['battery_min_v']:.3f}|" for r in rows)
    text='\n'.join(['# 機体A・右45°入口1800の追加比較','',
        '1.5 / 39.6 mmは現在、有効な精密測定2本。走行済みの3本目は1 ms保存ログが欠け、精密Lendには数えない。充電完了後の新しい追加試行を準備済み。' if not args.include_identity_checked else '1.5 / 39.6 mmの有効な精密測定3本を取得し、比較を完了した。前方向の残差は小さくなったが、左右・角度にばらつきがあり、この値への変更は採用保留とする。単体候補は従来の2.0 / 40.0 mmを優先して残す。数値は入口に合わせた相対評価。',
        '',*table,'',
        'B1〜B3は従来候補2.0 / 40.0 mmの3本、B4は同時期の取り直し対照、B5は再開後の充電後対照、C1は1.0 / 39.3 mm、Mは1.5 / 39.6 mm。最良の1本だけを選ばず、悪化した試行も含める。',
        '',
        '基準は高さ補正済みマーカー中点と実測入口方向。盤面固定の設置誤差・絶対車軸・車体姿勢の校正はしていない。標準GUIの要確認判定は保持し、独立の動作品質と固定した停止後区間の検査を併記する。動画の観測率・毎フレームの盤面参照・時刻合わせの品質条件は維持した。',
        '',
        '1.5 / 39.6 mmでは前方向の残差は小さくなったが、左右・角度の再現性に差がある。電圧と速度も記録し、長さ設定だけの因果差とは断定しない。1 ms欠損試行はFAILEDのまま動画・連続CSV・設定をDドライブへ照合保存済み。参考の空間直線評価を精密Lendへ代用しない。',
        '',
        'ファームウェア採用値の変更・書き込み、連続走行、最短走行、新しいEKF係数の採用は行っていない。取得手順は停止・後撮り確認後に1 msログを先に取得するよう修正し、関連85テストが成功（スキップ0）。現在のBLE中継38テストと運用ラッパーの機体識別7ケースも成功。BLE実切断の恒久復旧は未確認。',
        '',
        '最後の操作者確認は停止・接触なし・両方OFF。その後の追加走行はしていない。M3は実機のmouse:A・idleを確認後に単体1回を実行し、動画転送より先に1 msログ158件・取得落ち0を保存した。今回読み返した実機プリセットの長さは1.33 / 46.18 mmで、推奨する比較候補2.0 / 40.0 mmをファームへ書き込んだ意味ではない。recommended_candidate.settings.jsonは単体比較用の候補である。誤って配置された機体Bの静止映像は別記録し、A用設定・走行指令は未送信、Aの測定には含めない。' if args.include_identity_checked else '最後の操作者確認は停止・接触なし・両方OFF、その後「充電中」。充電後の新しい配置・両方ON・手離し完了を受けてから単体1回を実行する。',
        '',
        ('充電後の対照B5は、入口相対の出口残差が右-0.812 mm・前-0.892 mm、距離1.206 mm、出口方向45.132°。M3の1.780 mm・45.530°と比較し、PID・FF・吸引は実機読戻値で一致した。ただし最低電圧7.477 V対7.621 V、パルス速度1870.3 mm/s対1904.7 mm/sの差と各1回という制限がある。今回の対照も従来候補のばらつき範囲内で、2.0 / 40.0 mmを優先する判断を維持する。長さ変更だけの効果や統計的優位は断定しない。' if args.include_postcharge_baseline else ''),
        '', '![比較](comparison.png)',''])
    (out/'README.md').write_text(text,encoding='utf-8')
    for source in [BASE/'mid_status_retry_02/candidate.settings.json',Path(__file__),BASE/'mid_status_retry_assets/pretransfer_preparation2036.json']:
        shutil.copy2(source,out/source.name)
        INPUTS.add(source)
    recommended=BASE/'single_turn_candidate_3run_review_v1/candidate.settings.json'
    shutil.copy2(recommended,out/'recommended_candidate.settings.json')
    shutil.copy2(BASE/'mid_status_retry_02/candidate.settings.json',out/'tested_candidate.settings.json')
    INPUTS.add(recommended)
    save(out/'inputs_manifest.json',dict(files={str(p.resolve()):digest(p) for p in sorted(INPUTS)},robot_commands_sent=0))
    print(json.dumps(dict(output=str(out),baseline=report['baseline_stats'],midpoint=report['midpoint_stats']),indent=2),flush=True)

if __name__=='__main__':main()

