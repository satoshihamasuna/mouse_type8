"""機体A・右45°出口の動画と1 msログから出口直線・Lend終端を評価する。"""
import argparse
import json
import os
from pathlib import Path
from statistics import median

import numpy as np
import pandas as pd

from review_turn1600_A_inr45_followup_20261119 import align, image, line_fit, onboard_positions, plt, read, table
from video_marker_pose import board_to_pose_transform, transform_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('out', type=Path)
    parser.add_argument('--baseline', type=Path, help='基準走行のrepeat_summary.json')
    parser.add_argument('--board-target-mm', type=float, nargs=2, metavar=('X', 'Y'),
                        help='盤面に固定したLend終端の目標点')
    args = parser.parse_args()
    run, out = args.run.resolve(), args.out.resolve()
    trial = read(run/'trial.json')
    settings = read(run/'saved_1ms.settings.json')
    assert (trial['mouse'], trial['motion'], trial['preset']) == ('A', 'out_r45', 1600)
    full = run/'full'
    saved, stream, pose, relative = map(pd.read_csv, (run/'saved_1ms.csv', run/'telemetry.csv',
        full/'apparent_floor_marker_pose.csv', full/'relative_marker_midpoint.csv'))
    report = read(full/'report.json')
    time = relative.time_video_s.to_numpy()
    valid = relative.detected.to_numpy(bool)
    xy = relative[['center_dx_mm', 'center_dy_mm']].to_numpy()
    assert trial['saved_log']['drop_count'] == 0 and np.all(np.diff(saved.cnt) == 1)
    reference = read(run/'marker_reference_used.json')
    pts, period, origin, rms, motion_rms = align(trial, saved, stream, pose, xy, valid, reference)
    start, stop = (trial['log_quality'][k] for k in ('motion_start_time_ms', 'motion_end_time_ms'))
    turning = np.flatnonzero(saved['motion.pattern'].to_numpy() == 9)
    segment_start_pts = pts(origin+period*saved.cnt.iloc[turning[0]])
    end_pts = pts(origin+period*saved.cnt.iloc[-1])
    before = valid & (time >= pts(start+8)) & (time <= segment_start_pts-.003)
    after = valid & (time >= end_pts+.008) & (time <= pts(stop-8))
    entry, entry_stats = line_fit(xy[before])
    exit_axis, exit_stats = line_fit(xy[after])
    basis = np.column_stack([[-entry[1], entry[0]], entry])
    stationary = valid & (time < pts(start)-.025) & (time >= pts(start)-.25)
    assert stationary.sum() >= 12
    local = (xy-np.median(xy[stationary], axis=0)) @ basis
    direction = exit_axis @ basis
    angle = float(np.degrees(np.arctan2(*direction)))
    entry_board_angle = float(np.degrees(np.arctan2(entry[1],entry[0])))
    # 90 mmセルの右45°出口変位（出口軸で左45/前90 mm）を入口基準へ回転。
    # 斜め前走2区間を足し、直線後走90 mmを始める直前を目標にする。
    target = np.array([45/np.sqrt(2), 315/np.sqrt(2)])
    normal = direction[::-1]*[1, -1]
    line_shift = float((target@normal-np.median(local[after]@normal))/normal[1])

    def sample_by_time(coordinates,offset):
        return [float(np.interp(end_pts+offset,time[valid],axis[valid])) for axis in coordinates.T]

    def by_time(offset):
        return sample_by_time(local,offset)

    endpoint = by_time(0)
    quality = report['required_motion_window_quality']
    accepted = bool(quality['accepted_for_relative_review'] and rms <= 8 and motion_rms <= 8)
    params = settings['parameters']
    data = dict(trial=str(run/'trial.json'), relative_pose_report=str(full/'report.json'), settings=settings,
        target_Lend_right_forward_mm=target.tolist(), Lend_end_right_forward_mm=endpoint,
        timing_sensitivity_right_forward_mm={'minus5ms':by_time(-.005),'plus5ms':by_time(.005)},
        exit_direction_deg=angle, exit_direction_error_deg=angle-45, entry_direction_board_deg=entry_board_angle,
        exit_line_forward_correction_mm=line_shift, approximate_Lstart_candidate_mm=params['lstart']+line_shift,
        optical_rate=quality['observation_rate'], relative_review_eligible=accepted,
        entry_line=entry_stats, exit_line=exit_stats, motion_model_rmse_mm=motion_rms, model_rmse_mm=rms,
        parameter_adoption_allowed=False)
    blue = pose['turn_led_blue_pixels'].to_numpy()
    rising = np.flatnonzero((blue[:-2] < 30) & (blue[1:-1] >= 30) & (blue[2:] >= 30))+1
    rising = rising[np.abs(time[rising]-segment_start_pts) <= .015]
    limits = None
    if len(rising) == 1:
        frame = rising[0]
        limits = (time[frame-1]-segment_start_pts, time[frame]-segment_start_pts)
        data['turn_LED_onset_timing_check'] = dict(
            onset_video_pts_s=[float(time[frame-1]),float(time[frame])],
            differences_from_trajectory_fit_ms=[float(offset*1000) for offset in limits],
            estimated_end_right_forward_if_aligned_to_onset_mm=[by_time(offset) for offset in limits],
            note='Turn-segment start LED bracket only. Frame exposure and control-log sampling limit timing.')
    board = None
    if args.board_target_mm is not None:
        to_board = np.linalg.inv(board_to_pose_transform(reference))
        floor_front = transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(),to_board)
        floor_rear = transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(),to_board)
        floor_center = (floor_front+floor_rear)/2
        board = dict(target_mm=args.board_target_mm,apparent_start_mm=np.median(floor_center[stationary],axis=0).tolist(),
            entry_direction_deg=entry_board_angle,exit_direction_deg=float(np.degrees(np.arctan2(exit_axis[1],exit_axis[0]))),
            trajectory_timed_apparent_Lend_mm=sample_by_time(floor_center,0),
            note='Apparent marker midpoint projected to the board. Marker height and camera foot affect the absolute point.')
        if limits is not None:
            interval = [sample_by_time(floor_center,offset) for offset in limits]
            board.update(LED_timed_apparent_Lend_interval_mm=interval,
                LED_timed_apparent_Lend_midpoint_mm=np.mean(interval,axis=0).tolist())
        data['board_target_comparison'] = board
    baseline = read(args.baseline) if args.baseline else None
    if baseline is not None:
        samples = [r for r in baseline['trials'] if r['eligible']]
        prior_xy = np.median([r['endpoint_right_forward_mm'] for r in samples], axis=0)
        data['baseline_comparison'] = dict(source=str(args.baseline.resolve()), runs=len(samples),
            previous_parameters=baseline['settings'], exit_angle_change_deg=angle-baseline['median_exit_angle_deg'],
            endpoint_change_right_forward_mm=(np.asarray(endpoint)-prior_xy).tolist(),
            previous_median_endpoint_right_forward_mm=prior_xy.tolist(),
            previous_median_exit_line_forward_correction_mm=baseline['median_exit_line_forward_correction_mm'])
    try:
        onboard, reset = onboard_positions(saved, params['lend'], pre_pattern=2, turn_pattern=9)
        data['onboard_Lend_end_right_forward_mm'] = onboard[-1].tolist()
    except (AssertionError, IndexError):
        onboard = None
        data['onboard_path_note'] = 'Unexpected phase transition; raw 1 ms data preserved.'

    out.mkdir(exist_ok=False)
    (out/'review.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n',encoding='utf-8')
    use = (time >= pts(start)-.035) & (time <= pts(stop)+.33)
    plotted = np.where(valid[:,None], local, np.nan)
    fig, ax = plt.subplots(figsize=(6.3, 6.8), layout='constrained')
    ax.plot(*plotted[use].T, lw=2, label='Video marker midpoint')
    if onboard is not None:
        ax.plot(*onboard[:reset].T, color='#ad4823', label='Onboard position')
        ax.plot(*onboard[reset-1:].T, color='#ad4823', ls='--', label='Onboard Lend joined by gyro')
    ax.scatter(*endpoint, s=38, label='Estimated Lend end')
    ax.scatter(*target, marker='x', s=56, label='Target Lend end')
    ax.set(xlabel='Right [mm]', ylabel='Forward [mm]', aspect='equal')
    ax.legend(fontsize=8)
    ax.grid(alpha=.2)
    chart = image(fig, '右45度出口の動画と内部位置ログ')
    board_chart = ''
    if board is not None:
        board_trace = np.where(valid[:,None],floor_center,np.nan)
        fig, ax = plt.subplots(figsize=(6.6,6.2),layout='constrained')
        ax.plot(*board_trace[use].T,lw=2,label='Apparent marker midpoint')
        ax.scatter(*board['target_mm'],marker='x',s=65,color='#ab3528',label='Board target')
        if 'LED_timed_apparent_Lend_interval_mm' in board:
            ax.plot(*np.array(board['LED_timed_apparent_Lend_interval_mm']).T,marker='o',lw=2.5,
                    color='#008070',label='LED timing frame interval')
        ax.scatter(*board['trajectory_timed_apparent_Lend_mm'],s=35,color='#946434',label='Trajectory-timed Lend')
        ax.set(xlabel='Board X [mm]',ylabel='Board Y [mm]',xlim=(-10,370),ylim=(370,-10),aspect='equal',
               xticks=range(0,361,45),yticks=range(0,361,45))
        ax.legend(fontsize=8)
        ax.grid(alpha=.2)
        board_chart = f'<figure>{image(fig,"盤面に投影した軌跡とLend終端の目標点")}</figure>'
    columns = ['指標', '今回', '目標']
    rows = [
        ['出口角', f'{angle:.2f}°', '45.00°'],
        ['Lend終端 右 / 前', f'{endpoint[0]:.2f} / {endpoint[1]:.2f} mm', f'{target[0]:.2f} / {target[1]:.2f} mm'],
        ['出口直線を目標に合わせる前後補正', f'{line_shift:+.2f} mm', '0 mm'],
        ['走行窓の動画観測率', f'{quality["observation_rate"]:.1%}', '95%以上']]
    baseline_link = ''
    if baseline is not None:
        columns.insert(1, '基準3本の中央値')
        for row, previous in zip(rows, (f'{baseline["median_exit_angle_deg"]:.2f}°',
                f'{prior_xy[0]:.2f} / {prior_xy[1]:.2f} mm',
                f'{baseline["median_exit_line_forward_correction_mm"]:+.2f} mm',
                f'{median(r["optical_rate"] for r in samples):.1%}')):
            row.insert(1, previous)
        link = Path(os.path.relpath(args.baseline.with_name('compare_repeats.html').resolve(), out)).as_posix()
        baseline_link = f' ・ <a href="{link}">基準3本の比較</a>'
    if board is not None:
        point = board.get('LED_timed_apparent_Lend_midpoint_mm',board['trajectory_timed_apparent_Lend_mm'])
        board_rows=[
            ['盤面X / YのLend終端（見かけのマーカー中点）',f'{point[0]:.2f} / {point[1]:.2f} mm',
             f'{args.board_target_mm[0]:.2f} / {args.board_target_mm[1]:.2f} mm'],
            ['盤面上の進入方向',f'{entry_board_angle:.2f}°','135.00°']]
        for row in board_rows:
            if baseline is not None:
                row.insert(1,'—')
            rows.append(row)
    measures = table(columns, rows)
    page = f"""<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>機体A turn1600 右45°出口</title><style>body{{max-width:850px;margin:20px auto;padding:0 16px;
color:#183442;font:16px/1.6 system-ui}}img{{max-width:100%}}td,th{{padding:5px 13px;text-align:left;
border-bottom:1px solid #cbdade}}table{{border-collapse:collapse}}</style><h1>機体A turn1600 右45°出口</h1>
<p>単体測定。半径 {params['r_min']:g} mm、Lstart {params['lstart']:.2f} mm、Lend {params['lend']:.2f} mm、
角度補正 {params['correction']:+.2f}°。</p>
{measures}<figure>{chart}</figure>{board_chart}<p>進入直線基準。Lend終端は直線後走の前にあり、動画とログの軌跡で時刻を合わせて推定しました。</p>
<p><a href="review.json">評価値と入力ログ</a>{baseline_link}</p></html>"""
    (out/'compare.html').write_text(page, encoding='utf-8')
    print(json.dumps({k:v for k,v in data.items() if k not in ('settings', 'trial')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
