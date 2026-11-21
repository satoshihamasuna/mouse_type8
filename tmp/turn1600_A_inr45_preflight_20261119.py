"""機体Aの右45°入口の静止確認と、斜め減速プロファイルの照合。"""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tools/analysis')]


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def write(path, data):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(data,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')


def capture(output, config):
    from pixel_camera_capture import camera_workspace, record_trial
    sys.path.insert(0,str(camera_workspace()/'python'))
    from pixel_camera import PixelCameraClient
    output.mkdir(exist_ok=False)
    camera=PixelCameraClient(connection='wifi',port=8765,auto_start=False)
    status=camera.get_status()
    if not status.get('connected') or status.get('device',{}).get('phase')!='IDLE':
        raise RuntimeError('カメラの接続・待機状態を確認できません')
    manifest,result=record_trial(camera,ROOT,seconds=3,mouse='A',
        label='turn1600_A_inr45_static_20261119',camera_settings=config['camera_settings'],capture_profile='tuning')
    write(output/'static_capture.json',dict(manifest=str(manifest),**result))
    print(json.dumps({k:result.get(k) for k in ('state','video_file','error')},ensure_ascii=False),flush=True)
    if result.get('state')!='PASSED':
        raise RuntimeError('カメラの静止撮影を確認できません')


def inspect(output, config, plan):
    import cv2
    import numpy as np
    from video_marker_pose import read_video_pose, board_to_pose_transform, transform_points
    capture=read(output/'static_capture.json')
    reference=read(Path(config['reference']))
    geometry=read(Path(config['geometry']))
    path=Path(capture['video_file'])
    video=cv2.VideoCapture(str(path))
    video.set(cv2.CAP_PROP_ORIENTATION_AUTO,0)
    video.set(cv2.CAP_PROP_POS_MSEC,2000)
    ok,frame=video.read()
    video.release()
    if not ok or not cv2.imwrite(str(output/'static_frame.jpg'),frame):
        raise RuntimeError('静止フレームを保存できません')
    pose,_,_,_=read_video_pose(path,'red',marker_reference=reference,video_window=(1.,2.95),
        marker_detection=geometry['marker_detection'],frame_stride=4,rear_colour='yellow',
        nominal_marker_span_mm=50.,marker_span_limits_mm=[40,65])
    valid=pose.marker_detected_raw.to_numpy(bool)&pose.calibration_valid.to_numpy(bool)
    if not valid.any():
        raise RuntimeError('登録済み静止位置を取得できません')
    inverse=np.linalg.inv(board_to_pose_transform(reference))
    front=transform_points(pose[['blue_x_mm','blue_y_mm']].to_numpy(),inverse)
    rear=transform_points(pose[['yellow_x_mm','yellow_y_mm']].to_numpy(),inverse)
    midpoint=(front+rear)/2
    course=np.degrees(np.arctan2((front-rear)[:,1],(front-rear)[:,0]))
    center=np.median(midpoint[valid],axis=0)
    median_heading=float(np.median(course[valid]))
    target=plan.get('planned_start_board_mm')
    nominal=plan.get('planned_heading_image_deg')
    position_scatter=float(np.linalg.norm(midpoint[valid]-center,axis=1).max())
    heading_scatter=float(np.abs((course[valid]-median_heading+180)%360-180).max())
    maximum_position_error=None if target is None else float(np.linalg.norm(midpoint[valid]-target,axis=1).max())
    maximum_heading_error=None if nominal is None else float(np.abs((course[valid]-nominal+180)%360-180).max())
    result=dict(mouse='A',motion='in_r45',measurements=int(valid.sum()),frames=len(pose),
        valid_rate=float(valid.mean()),calibration=pose.attrs['marker_calibration'],
        target_mm=target,target_heading_deg=nominal,center_board_mm=center.tolist(),
        heading_deg=median_heading,maximum_position_deviation_from_median_mm=position_scatter,
        maximum_heading_deviation_from_median_deg=heading_scatter,
        maximum_position_error_mm=maximum_position_error,maximum_heading_error_deg=maximum_heading_error,
        placement_gate_passed=None if target is None or nominal is None else
            bool(valid.all() and len(pose)>50 and maximum_position_error<=5 and maximum_heading_error<=2))
    write(output/'static_check.json',result)
    print(json.dumps(result,indent=2),flush=True)


def device(output, config, plan):
    import math
    import time
    sys.path.insert(0,str(ROOT/'.tools/ble_python311'))
    from myshell_debug import DebugGui, PARAM_RE
    from pixel_camera_turn_capture import Shell
    from pixel_camera_tuning import LiveSession
    fields=('distance','acc','max_velo','end_velo','sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd',
            'ff_sp_velo','ff_sp_accel','ff_sp_bias','ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias')

    def parse(line):
        match=PARAM_RE.search(line)
        if not match:
            raise ValueError('斜め設定を解析できません')
        values={k:float(match[k]) for k in fields}
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('斜め設定に非有限値があります')
        return dict(values,suction=int(match['suction']),duty=int(match['duty']))

    def command(value):
        return 'debug diagonal set '+' '.join(f'{value[k]:.7g}' for k in (*fields,'suction','duty'))

    output.mkdir(exist_ok=False)
    session=LiveSession(ROOT,output/'transport')
    result=dict(state='PREPARE_FAILED',execution_commands_sent=0)
    try:
        session.prepare(config['address'])
        with (output/'shell.bin').open('xb') as trace, (output/'events.jsonl').open('x',encoding='utf-8') as events:
            def event(name,**values):
                events.write(json.dumps(dict(event=name,**values))+'\n')
                events.flush()
                if name=='battery':
                    result['battery']=values
            shell=Shell(session.connection,trace,event,char_delay=config['char_delay'])
            shell.send('shell')
            time.sleep(.5)
            shell.check_battery()
            for key,prefix,query in (
                ('camera_capability','DEBUG_CAMERA_CAP ','debug camera show'),
                ('wall_ekf','WALL_EKF ','wall_ekf show'),('wall_edge','WALL_EDGE ','wall_edge show'),
                ('turn_ekf','TURN_EKF ','turn_ekf show'),
            ):
                result[key]=shell.query(query,lambda line:line.startswith(prefix))
            predicate=lambda line:line.startswith('DEBUG_PARAM diagonal ')
            before=parse(shell.query('debug diagonal show',predicate))
            result.update(diagonal_before=before,restore_command=command(before))
            desired=dict(before,acc=float(plan['post_diagonal_accel_m_s2_to_verify']))
            if before!=desired:
                shell.send(command(desired))
                shell.wait_line(lambda line:line=='DEBUG_PARAM_SET_DONE',timeout=8)
            line=shell.query('debug diagonal show',predicate)
            after=parse(line)
            if desired!=after:
                raise RuntimeError('斜め減速設定の読み戻しが一致しません')
            result.update(diagonal_after=after,diagonal_after_raw=line,state='PREPARED_NO_EXECUTION')
    except Exception as error:
        result['error']=str(error)
    finally:
        session.close()
        result['ble_disconnected']=True
        write(output/'result.json',result)
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    if result['state']!='PREPARED_NO_EXECUTION':
        raise SystemExit(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step',choices=('capture','inspect','device'))
    parser.add_argument('stage',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    config=read(args.stage/'config.json')
    plan=read(args.stage/'preparation.json')
    if args.step=='capture':
        capture(args.output,config)
    elif args.step=='inspect':
        inspect(args.output,config,plan)
    else:
        device(args.output,config,plan)


if __name__=='__main__':
    main()
