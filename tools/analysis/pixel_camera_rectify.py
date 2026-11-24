"""各フレームの登録済みArUcoから盤面を正対動画へ変換する。機体の追跡・機器操作なし。"""
from __future__ import annotations

import argparse
import csv
from fractions import Fraction
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pixel_camera_environment import initialize_camera_environment
from pixel_camera_detection import file_sha256
from pixel_camera_reference import load_marker_reference, rectify_board
from pixel_camera_position import prepare_reference
from video_marker_pose import marker_registration


def rectify(video, reference_path, output, *, stride=1, start=0., end=None, size=720):
    if stride not in (1,4) or isinstance(stride,bool):
        raise ValueError('stride must be 1 or 4')
    if not isinstance(size,int) or isinstance(size,bool) or not 128<=size<=2160 or size%2:
        raise ValueError('出力辺は128〜2160の偶数画素で指定してください')
    if not np.isfinite(start) or start<0 or (end is not None and (not np.isfinite(end) or end<=start)):
        raise ValueError('動画の開始・終了秒を確認してください')
    video,reference_path,output=map(Path,(video,reference_path,output))
    reference=load_marker_reference(reference_path)
    initialize_camera_environment()
    import av
    output.mkdir(parents=True,exist_ok=False)
    def save(name,value):
        with (output/name).open('x',encoding='utf-8') as stream:
            json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    summary=dict(analysis_mode='rectify_only',rectification_available=False,
        video_file=str(video.resolve()),video_sha256=file_sha256(video),
        reference_id=reference['reference_id'],reference_sha256=file_sha256(reference_path),
        frame_stride=stride,video_window_s=[start,end],output_size_px=[size,size],
        board_side_mm=reference['board_side_mm'],coordinate_system='board TL origin; X right; Y down',
        per_frame_registration_required=True,minimum_registered_markers=2,
        invalid_frame_policy='black frame with AR UNAVAILABLE; no previous transform or interpolation',
        body_tracking_used=False,log_used=False,height_correction_applied=False,
        metric_accuracy_verified=False,robot_commands_sent=False,
        calibration_thresholds_changed=False,audio_copied=False)
    save('marker_reference_input.json',reference)
    try:
        reference,search=prepare_reference(video,reference,start,end)
        summary['reference_search']=search
        save('marker_reference_used.json',reference)
        rows=[]; origin=None; previous=None; valid_count=0
        target=output/'rectified.mp4'
        with av.open(str(video)) as source:
            stream=source.streams.video[0]
            time_base=Fraction(stream.time_base)
            rate=stream.average_rate
            if not rate or time_base<=0:
                raise ValueError('動画の時刻刻み・フレームレートが不明です')
            with av.open(str(target),'w') as destination:
                encoded=destination.add_stream('libx264',rate=Fraction(rate)/stride)
                encoded.width=encoded.height=size
                encoded.pix_fmt='yuv420p'
                encoded.time_base=encoded.codec_context.time_base=time_base
                encoded.codec_context.max_b_frames=0
                encoded.codec_context.thread_count=2
                encoded.options={'crf':'18','preset':'veryfast'}
                for index,frame in enumerate(source.decode(stream)):
                    if frame.pts is None or not frame.time_base:
                        raise ValueError('動画フレームにPTSがありません')
                    stamp=frame.pts*Fraction(frame.time_base)
                    if previous is not None and stamp<=previous:
                        raise ValueError('動画PTSが増加していません')
                    previous=stamp
                    if end is not None and stamp>end:
                        break
                    if stamp<start or index%stride:
                        continue
                    if origin is None:
                        origin=stamp
                    image=frame.to_ndarray(format='bgr24')
                    registration=marker_registration(image,reference)
                    ids=registration.get('used_marker_ids',[]) if registration.get('available') else []
                    valid=bool(registration.get('available') and len(ids)>=2)
                    if valid:
                        canvas=rectify_board(image,registration,size)
                        if valid_count==0:
                            if not cv2.imwrite(str(output/'rectified_preview.png'),canvas):
                                raise OSError('補正画像を保存できません')
                        valid_count+=1
                    else:
                        canvas=np.zeros((size,size,3),np.uint8)
                        cv2.putText(canvas,'AR UNAVAILABLE',(max(5,size//16),size//2),
                            cv2.FONT_HERSHEY_SIMPLEX,size/900,(255,255,255),1,cv2.LINE_AA)
                    out_frame=av.VideoFrame.from_ndarray(canvas,format='bgr24')
                    out_frame.time_base=time_base
                    out_frame.pts=round((stamp-origin)/time_base)
                    out_frame.duration=max(1,round(frame.duration*Fraction(frame.time_base)*stride/time_base)) if frame.duration else 0
                    for packet in encoded.encode(out_frame):
                        destination.mux(packet)
                    rows.append(dict(frame=index,time_video_s=float(stamp),time_output_s=float(stamp-origin),
                        source_pts=frame.pts,source_time_base=str(frame.time_base),
                        calibration_valid=valid,registered_marker_count=len(ids),
                        used_marker_ids=json.dumps(ids),
                        decoded_marker_ids=json.dumps(registration.get('detected_marker_ids',[])),
                        reason='' if valid else registration.get('reason','insufficient_registered_markers'),
                        homography_image_to_board_mm=json.dumps(registration['homography_image_to_board_mm']) if valid else ''))
                    if len(rows)%240==0:
                        print(f'rectify: {len(rows)} frames, PTS {float(stamp):.3f} s',flush=True)
                if not rows:
                    raise ValueError('指定した区間に動画フレームがありません')
                for packet in encoded.encode():
                    destination.mux(packet)
        with (output/'rectification.csv').open('x',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        # 保存動画の枚数とPTSを再復号して照合し、速度変更や欠落を成功扱いにしない。
        with av.open(str(target)) as checked:
            stamps=[float(frame.pts*frame.time_base) for frame in checked.decode(video=0)]
        expected=np.asarray([row['time_output_s'] for row in rows])
        if len(stamps)!=len(rows) or not np.allclose(stamps,expected,rtol=0,atol=float(time_base)*1.01):
            raise ValueError('補正動画のフレーム数・PTSが元動画と一致しません')
        if file_sha256(video)!=summary['video_sha256']:
            raise ValueError('解析中に元動画が変わりました')
        summary.update(frames=len(rows),valid_frames=valid_count,invalid_frames=len(rows)-valid_count,
            registration_rate=valid_count/len(rows),rectification_available=valid_count>0,
            output_pts_origin_video_s=float(origin),source_nominal_fps=float(rate),
            output_video_file=str(target.resolve()),output_video_sha256=file_sha256(target),
            output_timebase_verified=True,status='RECTIFIED_REVIEW_REQUIRED' if valid_count else 'RECTIFICATION_UNAVAILABLE')
        if not valid_count:
            raise ValueError('盤面ARの登録が全フレームで不成立でした。rectification.csvを確認してください')
    except Exception as error:
        summary.update(rectification_available=False,status='RECTIFICATION_UNAVAILABLE',error=type(error).__name__+': '+str(error))
        save('summary.json',summary)
        raise
    save('summary.json',summary)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path)
    parser.add_argument('--video',type=Path)
    parser.add_argument('--reference',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--stride',type=int,choices=(1,4),default=1)
    parser.add_argument('--start',type=float,default=0.)
    parser.add_argument('--end',type=float)
    parser.add_argument('--size',type=int,default=720)
    args=parser.parse_args()
    if args.trial:
        trial=json.loads(args.trial.read_text(encoding='utf-8'))
        args.video=args.video or Path(trial['video_file'])
        args.reference=args.reference or args.trial.parent/'marker_reference_used.json'
        args.output=args.output or args.trial.parent/('full' if args.stride==1 else 'quick')
    if not all((args.video,args.reference,args.output)):
        parser.error('--trial または --video --reference --output を指定してください')
    rectify(args.video,args.reference,args.output,stride=args.stride,start=args.start,end=args.end,size=args.size)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
