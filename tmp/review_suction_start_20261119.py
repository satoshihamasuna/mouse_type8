"""保存動画の吸引開始区間を、毎フレームの盤面登録とMCU時刻で照合する。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/analysis'))
from video_marker_pose import read_video_pose


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def review(path, output, resume=False):
    trial = read(path)
    run = path.parent
    assert trial['mouse'] == 'A' and trial['suction_enabled'] and trial['suction_duty'] == 650
    settings = read(run / 'motion.settings.json')
    assert settings['mouse'] == 'A' and settings['suction'] == dict(enabled=True, duty=650)
    stream = pd.read_csv(run / 'telemetry.csv')
    motion = pd.read_csv(run / 'motion.csv')
    report = read(run / 'full/report.json')
    boundary = trial['log_quality']['motion_start_time_ms']
    segment = stream.loc[stream.time_ms.between(boundary-1800, boundary+50)].copy()
    spin = segment.loc[segment['motion.pattern'] == 28]
    assert len(spin) > 100 and (spin['ideal.velo'] == 0).all()
    start, end = float(spin.time_ms.iloc[0]), float(spin.time_ms.iloc[-1])
    origin = float(motion.time_ms.iloc[0])
    offset = report['log_time_zero_at_video_s']
    video_s = lambda mcu: offset + (mcu-origin)/1000
    reference = read(run/'marker_reference_used.json')
    geometry = read(run/'body_geometry_used.json')
    target = output/run.name
    target.mkdir(exist_ok=resume)
    if resume and (target/'pose.csv').is_file():
        pose = pd.read_csv(target/'pose.csv')
        # Only resumes this script's explicitly selected, freshly computed output.
        calibration = dict(method='registered_marker_vertices_per_evaluated_frame',
            reference_id=reference['reference_id'], per_frame_registration_required=True,
            missing_registration_policy='exclude_frame', calibration_frame_rate=float(pose.calibration_valid.mean()))
    else:
        pose, _, _, _ = read_video_pose(Path(trial['video_file']), geometry['front_colour'],
            marker_reference=reference, video_window=(video_s(start)-.35, video_s(boundary)+.04),
            marker_detection=geometry.get('marker_detection'), frame_stride=4,
            rear_colour=geometry['rear_colour'], nominal_marker_span_mm=geometry['nominal_marker_span_mm'],
            marker_span_limits_mm=[40,65])
        calibration = pose.attrs['marker_calibration']
        pose.to_csv(target/'pose.csv', index=False)
    valid = pose.marker_detected_raw.to_numpy(bool) & pose.calibration_valid.to_numpy(bool)
    t = pose.time_video_s.to_numpy()
    baseline = valid & (t >= video_s(start)-.30) & (t <= video_s(start)-.05)
    final = valid & (t >= video_s(boundary)-.15) & (t <= video_s(boundary)-.02)
    assert baseline.sum() >= 8 and final.sum() >= 5
    # pose plane is right-handed; positive heading is the robot's left turn.
    angle = np.degrees(pose.heading_rad.to_numpy())
    base = float(np.median(angle[baseline]))
    delta = (angle-base+180)%360-180
    center = pose[['center_x_mm','center_y_mm']].to_numpy()
    displacement = np.median(center[final],axis=0)-np.median(center[baseline],axis=0)
    stats = dict(trial=str(path.resolve()), video=trial['video_file'], mouse='A', duty=650,
        start_mcu_ms=start, end_mcu_ms=end, launch_mcu_ms=boundary,
        spin_duration_ms=end-start, start_video_s=video_s(start), launch_video_s=video_s(boundary),
        video_heading_change_deg=float(np.median(delta[final])),
        video_final_scatter_deg=float(np.std(delta[final])),
        gyro_heading_change_deg=float(np.degrees(spin['ego.radian'].iloc[-1])),
        initial_gyro_bias_rad_s=float(segment.loc[segment.time_ms.between(start-300,start-30),'ego.rad_velo'].median()),
        final_gyro_bias_rad_s=float(spin.tail(25)['ego.rad_velo'].mean()),
        initial_angle_noise_std_deg=float(np.std(delta[baseline])),
        observed_fraction=float(valid.mean()), calibration=calibration,
        maximum_gap_ms=float(spin.time_ms.diff().max()), lost_frames=int(spin.lost_frames.sum()),
        baseline_frames=int(baseline.sum()), final_frames=int(final.sum()),
        midpoint_shift_mm=float(np.linalg.norm(displacement)),
        final_voltage_v=float(spin.tail(25)[['V_r','V_l']].mean().mean()),
        input_sha256={str(p):sha(p) for p in [path,run/'telemetry.csv',run/'motion.settings.json',
            run/'marker_reference_used.json',run/'body_geometry_used.json']},
        video_sha256=sha(Path(trial['video_file'])), script_sha256=sha(Path(__file__)),
        time_alignment='existing trajectory fit extended to spin-up; not independent hardware sync',
        height_correction='apparent marker axis; camera foot unknown; fixed-start angle difference only')
    (target/'summary.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf-8')
    fig, axes = plt.subplots(2,1,figsize=(9,6),sharex=True)
    axes[0].plot(t[valid]-video_s(start),delta[valid],'.-',label='Video (registered markers)',ms=3)
    axes[0].plot((spin.time_ms-start)/1000,np.degrees(spin['ego.radian']),label='Gyro integrated')
    axes[0].axhline(0,color='gray',lw=.7)
    axes[0].set_ylabel('Heading change (deg, left +)')
    axes[0].legend()
    for name in ['V_r','V_l','om_feedback']:
        axes[1].plot((segment.time_ms-start)/1000,segment[name],label=name)
    axes[1].set_ylim(-.25,.25)
    axes[1].set_ylabel('Voltage (V)')
    axes[1].set_xlabel('Time from fan spin-up (s)')
    axes[1].legend()
    for ax in axes:
        ax.axvline(0,color='gray',ls='--')
        ax.axvline((boundary-start)/1000,color='red',ls='--',label='Launch')
        ax.set_xlim(-.3,(boundary-start)/1000+.02)
        ax.grid(alpha=.25)
    fig.suptitle(run.name)
    fig.tight_layout()
    fig.savefig(target/'comparison.png',dpi=150)
    plt.close(fig)
    # Save the same fixed crop before and after spin-up for visual inspection.
    frames=[]
    cap=cv2.VideoCapture(trial['video_file'])
    cap.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    chosen=[float(np.median(t[baseline])),float(np.median(t[final]))]
    for stamp in chosen:
        cap.set(cv2.CAP_PROP_POS_MSEC,stamp*1000)
        ok,frame=cap.read()
        assert ok
        frames.append(frame)
    cap.release()
    fig,axes=plt.subplots(1,2,figsize=(12,6))
    for ax,frame,stamp in zip(axes,frames,chosen):
        ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
        ax.set_title(f'{stamp:.3f} s')
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(target/'before_after.png',dpi=150)
    plt.close(fig)
    print(json.dumps({k:stats[k] for k in ['trial','video_heading_change_deg','gyro_heading_change_deg',
        'initial_angle_noise_std_deg','midpoint_shift_mm','observed_fraction']},ensure_ascii=False),flush=True)
    return stats


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(exist_ok=args.resume)
    results=[review(path,args.output,args.resume) for path in args.trial]
    (args.output/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    main()
