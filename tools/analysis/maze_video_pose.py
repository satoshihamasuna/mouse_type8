"""探索・最短の保存動画を盤面へ登録し、前後マーカーとBLE実時刻を比較する。

座標は盤面TL原点、x右、y下。高さ未補正のマーカー中点を車軸位置と断定しない。
機体・カメラへの接続、採用パラメータの書き換えは行わない。
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import html
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pixel_camera_framing import inspect_rois
from analysis.wall_event_review import review_rows
from marker_detection import colour_mask


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def json_save(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def project(points, matrix):
    return cv2.perspectiveTransform(np.float32(points).reshape(-1, 1, 2), matrix).reshape(-1, 2)


def detector():
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))


class DuplicateMarkerIds(ValueError):
    pass


def detected_markers(frame, aruco):
    corners, ids, _ = aruco.detectMarkers(frame)
    if ids is not None and len(set(map(int,ids.ravel()))) != len(ids):
        raise DuplicateMarkerIds('同じArUco IDを複数検出しました')
    return {} if ids is None else {str(int(k)): q.reshape(4, 2) for k, q in zip(ids.ravel(), corners)}


def make_reference(frame_path, grid_report, output, seconds=1.5):
    """確認済み9×9格子と、同画像のArUco全頂点を関連付ける。"""
    if Path(output).exists():
        raise ValueError('既存の盤面基準を上書きしません。新しい出力名を指定してください')
    report = json.loads(Path(grid_report).read_text(encoding='utf-8'))
    entry = min(report['frames'], key=lambda f: abs(f['seconds']-seconds))
    if abs(entry['seconds']-seconds) > .001 or sha256(frame_path) != entry['image_sha256']:
        raise ValueError('格子校正と画像の時刻・SHA256が一致しません')
    frame = cv2.imread(str(frame_path))
    markers = detected_markers(frame, detector())
    if len(markers) != 4:
        raise ValueError('基準画像では4個のArUcoが必要です')
    matrix = np.asarray(entry['homography_image_to_rectified_px'], float) / 100
    matrix[2] *= 100  # 900 px square -> 9 cell square; homogeneous denominator unchanged.
    reference = dict(schema=1, rows=9, columns=9, dictionary='DICT_4X4_50',
        axes='board TL origin, x toward TR, y toward BL; heading clockwise from +x',
        source_frame=str(Path(frame_path).resolve()), source_sha256=sha256(frame_path),
        grid_report=str(Path(grid_report).resolve()), grid_report_sha256=sha256(grid_report),
        image_width=frame.shape[1], image_height=frame.shape[0],
        source_markers_image_px={k:q.tolist() for k,q in markers.items()},
        markers_cell={k: project(q, matrix).tolist() for k, q in markers.items()},
        wall_segments=entry.get('wall_segments', []),
        limitations=['格子・壁上端から作成した見かけの平面。床面・レンズ歪み・高さの精密校正は未検証'])
    json_save(output, reference)
    return reference


def registration(frame, aruco, reference, search_matrix=None):
    try:
        found = detected_markers(frame, aruco)
    except DuplicateMarkerIds:
        return None, dict(marker_ids=[],reason='duplicate_aruco_ids')
    # Search positions never replace observed vertices. Each recovered ID must be
    # decoded in this frame; clipped tags remain missing.
    search = reference.get('source_markers_image_px',{})
    if search_matrix is not None:
        inverse=np.linalg.inv(search_matrix)
        search={k:project(q,inverse) for k,q in reference['markers_cell'].items()}
    for key,quad in search.items():
        if key in found:
            continue
        q=np.asarray(quad)
        if q.min(axis=0).min()<1 or q[:,0].max()>=frame.shape[1]-1 or q[:,1].max()>=frame.shape[0]-1:
            continue
        lo,hi=q.min(axis=0)-35,q.max(axis=0)+35
        box=[max(0,lo[0]),max(0,lo[1]),min(frame.shape[1],hi[0]),min(frame.shape[0],hi[1])]
        for item in inspect_rois(frame,{'search':box}).values():
            for recovered,vertices in zip(item['ids'],item['corners_px']):
                if str(recovered)==key:
                    found[key]=np.asarray(vertices,np.float32)
    ids = sorted(set(found) & set(reference['markers_cell']))
    if len(ids) < 2:
        return None, dict(marker_ids=ids, reason='fewer_than_two_aruco')
    source = np.concatenate([found[k] for k in ids]).astype(np.float32)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    refined=source.reshape(-1,1,2).copy()
    if (refined[:,:,0].min()>4 and refined[:,:,1].min()>4 and
            refined[:,:,0].max()<frame.shape[1]-4 and refined[:,:,1].max()<frame.shape[0]-4):
        cv2.cornerSubPix(gray,refined,(3,3),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,30,.01))
        if np.max(np.linalg.norm(refined.reshape(-1,2)-source,axis=1))<3:
            source=refined.reshape(-1,2)
    target = np.concatenate([reference['markers_cell'][k] for k in ids]).astype(np.float32)
    if np.linalg.norm(target.max(axis=0)-target.min(axis=0)) < 4:
        return None, dict(marker_ids=ids, reason='aruco_baseline_too_short')
    matrix, mask = cv2.findHomography(source, target, cv2.RANSAC, .025)
    if matrix is None:
        return None, dict(marker_ids=ids, reason='homography_failed')
    residual = np.linalg.norm(project(source, matrix)-target, axis=1)
    # A single marker must not be the only surviving RANSAC support.
    inliers = mask.ravel().astype(bool)
    supporting = [ids[i] for i in range(len(ids)) if inliers[i*4:i*4+4].sum() >= 3]
    rms = float(np.sqrt(np.mean(residual[inliers]**2)))
    if len(supporting) < 2 or rms > .025:
        return None, dict(marker_ids=ids, reason='registration_residual')
    quad = project([[0,0],[9,0],[9,9],[0,9]], np.linalg.inv(matrix))
    area = cv2.contourArea(quad.astype(np.float32), oriented=True)
    if not np.isfinite(quad).all() or area < 20000:
        return None, dict(marker_ids=ids, reason='invalid_board_quad')
    return matrix, dict(marker_ids=ids, supporting_marker_ids=supporting, residual_cell=rms, reason='ok')


def components(hsv, colour, matrix, detection=None):
    defaults = dict(hue_ranges=[[0, 11], [169, 179]] if colour == 'red' else [[22, 39]],
                    minimum_saturation=60, minimum_value=65)
    mask, minimum_area = colour_mask(hsv, colour, detection, defaults=defaults)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pixel_area = 'minimum_area_px' in (detection or {}).get(colour, {})
    result = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        local = None
        if pixel_area:
            local = np.zeros((h,w),np.uint8)
            cv2.drawContours(local,[contour-np.array([[[x,y]]])],-1,255,-1)
            local = cv2.bitwise_and(local,mask[y:y+h,x:x+w])
            area = cv2.countNonZero(local)
        if not minimum_area <= area <= 350:
            continue
        if max(w,h)/max(1,min(w,h)) > 2.7:
            continue
        perimeter = cv2.arcLength(contour, True)
        circularity = 4*math.pi*cv2.contourArea(contour)/max(perimeter**2, 1)
        if circularity < .35:
            continue
        moments = cv2.moments(local,True) if pixel_area else cv2.moments(contour)
        pixel = np.array([moments['m10']/moments['m00'],moments['m01']/moments['m00']])
        if pixel_area:
            pixel += [x,y]
        cell = project([pixel], matrix)[0]
        if (cell >= -.2).all() and (cell <= 9.2).all():
            result.append(dict(pixel=pixel, cell=cell, area=int(area), circularity=circularity))
    return result


def select_pair(fronts, rears, previous=None, dt=None, span_cell=50/90, span_limits=None):
    candidates = []
    for front in fronts:
        for rear in rears:
            vector = front['cell']-rear['cell']
            span = np.linalg.norm(vector)
            low, high = span_limits or (.65*span_cell, 1.4*span_cell)
            if not low <= span <= high:
                continue
            centre = (front['cell']+rear['cell'])/2
            heading = math.atan2(vector[1], vector[0])
            score = -(span/span_cell-1)**2*18 + front['circularity']+rear['circularity']
            if previous is not None and dt is not None and dt <= .25:
                step = np.linalg.norm(centre-previous['centre'])
                turn = abs(math.atan2(math.sin(heading-previous['heading']), math.cos(heading-previous['heading'])))
                if step > 20*dt+.10 or turn > 30*dt+.35:
                    continue
                score -= step*2 + turn*.3
            candidates.append(dict(front=front,rear=rear,centre=centre,heading=heading,score=score))
    candidates.sort(key=lambda c:c['score'], reverse=True)
    if not candidates:
        return None, 'no_pair'
    if len(candidates) > 1 and candidates[0]['score']-candidates[1]['score'] < .20:
        return None, 'ambiguous_pair'
    return candidates[0], 'ok'


def marker_colours(mouse):
    """実機設定の機体名から、前後マーカーの色を決める。"""
    if mouse == 'A':
        return 'red', 'yellow'
    if mouse == 'B':
        return 'yellow', 'red'
    raise ValueError('マーカーの前後を選ぶには、実機設定JSONのfirmware.mouseにAまたはBが必要です')


def track(video, reference, stride=4, cell_mm=90., progress=True, mouse='A', detection_profile=None):
    front_colour, rear_colour = marker_colours(mouse)
    profile = detection_profile or {}
    if profile.get('mouse', mouse) != mouse:
        raise ValueError('色検出プロファイルの機体とfirmware.mouseが一致しません')
    if profile.get('front_colour', front_colour) != front_colour or profile.get('rear_colour', rear_colour) != rear_colour:
        raise ValueError('色検出プロファイルの前後色が実機設定と一致しません')
    detection = profile.get('marker_detection', {})
    span = profile.get('nominal_marker_span_mm', 50.)
    limits = profile.get('marker_span_limits_mm', [.65*span, 1.4*span])
    if (not math.isfinite(span) or span <= 0 or len(limits) != 2
            or not all(math.isfinite(v) for v in limits) or not 0 < limits[0] <= span <= limits[1]):
        raise ValueError('色検出プロファイルの目印間隔を確認してください')
    for colour in (front_colour, rear_colour):
        colour_mask(np.zeros((1, 1, 3), np.uint8), colour, detection)
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError('動画を開けません: '+str(video))
    capture.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
    aruco = detector()
    rows, previous, last_pts, last_good = [], None, None, None
    index = 0
    search_matrix = None
    try:
        while capture.grab():
            # Decode the original stream, preserve original frame numbers and PTS.
            if index % stride:
                index += 1
                continue
            ok, frame = capture.retrieve()
            if not ok:
                raise ValueError('動画の復号に失敗しました')
            pts = float(capture.get(cv2.CAP_PROP_POS_MSEC))/1000
            if not math.isfinite(pts) or (last_pts is not None and pts <= last_pts):
                raise ValueError('動画PTSが単調増加しません。行番号/fpsでは代用しません')
            last_pts = pts
            matrix, quality = registration(frame, aruco, reference, search_matrix)
            if matrix is not None:
                search_matrix = matrix
            row = dict(frame=index, time_video_s=pts, calibration_valid=matrix is not None,
                       marker_ids=','.join(quality['marker_ids']), registration_rms_cell=quality.get('residual_cell'),
                       pose_valid=False, reason=quality['reason'])
            if matrix is not None:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                pair, reason = select_pair(components(hsv,front_colour,matrix,detection), components(hsv,rear_colour,matrix,detection),
                    previous, None if last_good is None else pts-last_good,
                    span/(cell_mm or 90), [value/(cell_mm or 90) for value in limits])
                row['detection_mode'] = 'selected_profile' if detection else 'standard'
                if pair is None and reason == 'no_pair' and not detection:
                    # Keep an already unambiguous standard pair. Only missing
                    # pairs get a second observation with small/pale disk settings.
                    small = dict(red=dict(minimum_area_px=3,minimum_saturation=35,hue_ranges=[[0,11],[160,179]]),
                                 yellow=dict(minimum_area_px=3,hue_ranges=[[22,45]]))
                    pair, reason = select_pair(components(hsv,front_colour,matrix,small),components(hsv,rear_colour,matrix,small),
                        previous,None if last_good is None else pts-last_good,
                        span/(cell_mm or 90),[value/(cell_mm or 90) for value in limits])
                    row['detection_mode'] = 'small_marker_fallback'
                row['reason'] = reason
                if pair is not None:
                    centre, heading = pair['centre'], pair['heading']
                    row.update(pose_valid=True, x_cell=float(centre[0]),y_cell=float(centre[1]),
                        heading_cw_rad=heading, front_image_x=float(pair['front']['pixel'][0]),
                        front_image_y=float(pair['front']['pixel'][1]),rear_image_x=float(pair['rear']['pixel'][0]),
                        rear_image_y=float(pair['rear']['pixel'][1]),
                        front_x_cell=float(pair['front']['cell'][0]),front_y_cell=float(pair['front']['cell'][1]),
                        rear_x_cell=float(pair['rear']['cell'][0]),rear_y_cell=float(pair['rear']['cell'][1]))
                    pixel = (pair['front']['pixel']+pair['rear']['pixel'])/2
                    radius = max(6, int(np.linalg.norm(pair['front']['pixel']-pair['rear']['pixel'])*.8))
                    x,y = np.round(pixel).astype(int)
                    region = hsv[max(0,y-radius):y+radius+1,max(0,x-radius):x+radius+1]
                    row['blue_led_pixels'] = int(((region[:,:,0]>=82)&(region[:,:,0]<=140)&(region[:,:,1]>=90)&(region[:,:,2]>=120)).sum())
                    if cell_mm:
                        row.update(x_apparent_mm=float(centre[0]*cell_mm),y_apparent_mm=float(centre[1]*cell_mm))
                    previous, last_good = pair, pts
            rows.append(row)
            index += 1
            if progress and len(rows)%600 == 0:
                print(f'追跡 {pts:.1f} s / 有効 {sum(r["pose_valid"] for r in rows)}/{len(rows)}', flush=True)
    finally:
        capture.release()
    derivatives(rows, cell_mm)
    return rows


def derivatives(rows, cell_mm):
    """欠測を跨がず、約80 ms窓の回帰で見かけ速度・角速度を求める。"""
    t = np.array([r['time_video_s'] for r in rows])
    good = np.array([r['pose_valid'] for r in rows])
    indices = np.flatnonzero(good)
    groups = np.split(indices, np.flatnonzero(np.diff(indices)>1)+1)
    for group in groups:
        if len(group) < 4:
            continue
        angle = np.unwrap([rows[i]['heading_cw_rad'] for i in group])
        xy = np.array([[rows[i]['x_cell'],rows[i]['y_cell']] for i in group])
        for k,i in enumerate(group):
            window = np.flatnonzero(abs(t[group]-t[i])<=.045)
            if len(window)<4 or np.ptp(t[group[window]])<.04:
                continue
            local_t = t[group[window]]-t[i]
            design = np.column_stack([local_t,np.ones(len(local_t))])
            slope = np.linalg.lstsq(design,np.column_stack([xy[window],angle[window]]),rcond=None)[0][0]
            rows[i]['omega_video_ccw_rad_s'] = float(-slope[2])
            rows[i]['speed_video_cell_s'] = float(np.linalg.norm(slope[:2]))
            if cell_mm:
                rows[i]['speed_video_apparent_m_s'] = float(np.linalg.norm(slope[:2])*cell_mm/1000)


def read_ble(trial):
    settings = json.loads(Path(trial['run_settings']).read_text(encoding='utf-8'))
    lo,hi = [settings[k]['time_ms'] for k in ('telemetry_start','telemetry_end')]
    with (Path(trial['ble_directory'])/'telemetry.csv').open(encoding='utf-8') as source:
        raw = [r for r in csv.DictReader(source) if lo<=int(r['time_ms'])<=hi]
    fields = ['time_ms','sequence','ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo',
              'sen_l.distance','sen_r.distance','sen_fl.distance','sen_fr.distance',
              'sp_feedback','om_feedback','Battery','V_l','V_r']
    optional = ('sen_l.value', 'sen_r.value', 'sen_fl.value', 'sen_fr.value', 'ir.range_states', 'motion.pattern')
    fields += [key for key in optional if raw and key in raw[0]]
    rows = [{key:float(r.get(key,'nan')) for key in fields} for r in raw]
    t = np.array([r['time_ms']/1000 for r in rows])
    if len(t)<3 or np.any(np.diff(t)<=0):
        raise ValueError('BLEの実time_msが単調増加しません')
    for field in ('ideal.velo','ego.velo','ideal.rad_velo','ego.rad_velo'):
        if not all(math.isfinite(r[field]) for r in rows):
            raise ValueError('必須BLE列に欠測があります: '+field)
    moving = next((r for r in rows if abs(r['ideal.velo'])>.005 or abs(r['ideal.rad_velo'])>.05), None)
    if moving is None:
        raise ValueError('走行区間がありません')
    return rows, settings, int(moving['time_ms'])


def clock_offset(trial, zero):
    anchor = trial.get('motion_anchor')
    clocks = [c for c in trial.get('clock_samples',[]) if c['recording_id']==trial['recording_id']]
    if not anchor or not clocks:
        return None
    pc = anchor['host_monotonic']+(zero-anchor['time_ms'])/1000
    clock = min(clocks,key=lambda c:abs((c['pc_before']+c['pc_after'])/2-pc))
    return clock['camera_elapsed_s']+pc-(clock['pc_before']+clock['pc_after'])/2


def fit_sync(pose, ble, zero, prior):
    """符号付き映像角速度とジャイロを比較する。空間軌跡は同期に使わない。"""
    base = dict(mcu_origin_ms=zero,video_time_at_mcu_origin_s=prior or 0.,
                clock_scale=1.,source='PC clock coarse estimate',accepted=False,manual_alignment_required=True)
    if prior is None:
        return dict(base,reason='no_clock_prior')
    samples = [p for p in pose if 'omega_video_ccw_rad_s' in p]
    if len(samples)<60:
        return dict(base,reason='insufficient_video_heading')
    tv = np.array([p['time_video_s'] for p in samples])
    wv = np.array([p['omega_video_ccw_rad_s'] for p in samples])
    tb = np.array([(r['time_ms']-zero)/1000 for r in ble])
    wb = np.array([r['ego.rad_velo'] for r in ble])
    scores = []
    for offset in np.arange(prior-2,prior+2.0001,.005):
        target = tv-offset
        j = np.searchsorted(tb,target).clip(1,len(tb)-1)
        mask = (target>=tb[0])&(target<=tb[-1])&((tb[j]-tb[j-1])<=.025)
        interpolated = np.interp(target,tb,wb)
        mask &= (np.abs(wv)>.6)|(np.abs(interpolated)>.6)
        if mask.sum()<60 or np.std(wv[mask])<.5 or np.std(interpolated[mask])<.5:
            scores.append((-1.,float(offset),0))
            continue
        score = float(np.corrcoef(wv[mask],interpolated[mask])[0,1])
        scores.append((score,float(offset),int(mask.sum())))
    peak = max(scores)
    competitor = max((s[0] for s in scores if abs(s[1]-peak[1])>.15),default=1.)
    accepted = peak[0]>.75 and peak[0]-competitor>.08 and abs(peak[1]-prior)<1.98
    result = dict(base,source='video heading derivative vs signed gyro; constant offset search',
        candidate_offset_s=peak[1],correlation=peak[0],competing_correlation=competitor,
        sample_count=peak[2],search_prior_s=prior,search_half_width_s=2.,step_s=.005,
        accepted=accepted,manual_alignment_required=not accepted,
        reason='accepted_constant_offset' if accepted else 'weak_or_ambiguous_correlation',
        uncertainty='5 ms search grid; exposure, tracking noise and clock drift are not calibrated')
    if accepted:
        result['video_time_at_mcu_origin_s'] = peak[1]
    return result


def fit_clock_drift(pose, ble, zero, initial):
    """区間ごとの旋回一致から時計の比率を求める。空間位置には合わせない。"""
    prior=initial.get('search_prior_s',initial['video_time_at_mcu_origin_s'])
    samples=[p for p in pose if 'omega_video_ccw_rad_s' in p]
    if len(samples)<120:
        return initial
    tv=np.array([p['time_video_s'] for p in samples])
    wv=np.array([p['omega_video_ccw_rad_s'] for p in samples])
    tb=np.array([(r['time_ms']-zero)/1000 for r in ble])
    wb=np.array([r['ego.rad_velo'] for r in ble])
    moving=[r for r in ble if abs(r['ideal.velo'])>.01 or abs(r['ideal.rad_velo'])>.1]
    if not moving:
        return initial
    duration=(moving[-1]['time_ms']-zero)/1000
    if duration<30:
        return dict(initial,drift_reason='run_shorter_than_30_seconds')
    windows=np.linspace(prior,max(prior+1,prior+duration),6)
    anchors=[]
    for lo,hi in zip(windows[:-1],windows[1:]):
        select=(tv>=lo)&(tv<hi)
        if select.sum()<40:
            continue
        local_t,local_w=tv[select],wv[select]
        candidates=[]
        for offset in np.arange(prior-2,prior+2.0001,.005):
            target=local_t-offset
            j=np.searchsorted(tb,target).clip(1,len(tb)-1)
            omega=np.interp(target,tb,wb)
            good=((abs(local_w)>.6)|(abs(omega)>.6))&(target>=tb[0])&(target<=tb[-1])&((tb[j]-tb[j-1])<=.025)
            if good.sum()<30 or np.std(local_w[good])<.5 or np.std(omega[good])<.5:
                continue
            score=float(np.corrcoef(local_w[good],omega[good])[0,1])
            candidates.append((score,float(offset),int(good.sum()),float(np.median(local_t[good]-offset))))
        if not candidates:
            continue
        peak=max(candidates)
        competitor=max((c[0] for c in candidates if abs(c[1]-peak[1])>.15),default=1.)
        accepted=peak[0]>.75 and peak[0]-competitor>.06 and abs(peak[1]-prior)<1.98
        anchors.append(dict(video_window_s=[float(lo),float(hi)],correlation=peak[0],offset_s=peak[1],
            count=peak[2],mcu_relative_s=peak[3],competing_correlation=competitor,accepted=accepted))
    usable=[a for a in anchors if a['accepted']]
    details=dict(local_clock_anchors=anchors)
    if len(usable)<3 or np.ptp([a['mcu_relative_s'] for a in usable])<25:
        return dict(initial,**details,drift_reason='insufficient_unique_local_turn_matches')
    x=np.array([a['mcu_relative_s'] for a in usable]); y=np.array([a['offset_s'] for a in usable])
    coefficients=np.polyfit(x,y,1)
    residual=y-np.polyval(coefficients,x)
    keep=abs(residual-np.median(residual))<=max(.025,3*float(np.median(abs(residual-np.median(residual)))))
    if keep.sum()<3 or np.ptp(x[keep])<25:
        return dict(initial,**details,drift_reason='inconsistent_local_clock_matches')
    drift,offset=np.polyfit(x[keep],y[keep],1)
    rms=float(np.sqrt(np.mean((y[keep]-(offset+drift*x[keep]))**2)))
    if abs(drift)>.02 or rms>.04:
        return dict(initial,**details,drift_reason='clock_rate_or_residual_out_of_range',local_offset_fit_rms_s=rms)
    def score_model(off,scale):
        target=(tv-off)/scale
        j=np.searchsorted(tb,target).clip(1,len(tb)-1)
        omega=np.interp(target,tb,wb)
        good=((abs(wv)>.6)|(abs(omega)>.6))&(target>=tb[0])&(target<=tb[-1])&((tb[j]-tb[j-1])<=.025)
        if good.sum()<90 or np.std(wv[good])<.5 or np.std(omega[good])<.5:
            return -1.,0
        return float(np.corrcoef(wv[good],omega[good])[0,1]),int(good.sum())
    fits=[]
    for delta_offset in np.arange(-.03,.0301,.005):
        for delta_scale in np.arange(-.0006,.00061,.0001):
            scale=1+float(drift)+float(delta_scale)
            off=float(offset)+float(delta_offset)
            corr,count=score_model(off,scale)
            fits.append((corr,off,scale,count))
    corr,offset,scale,count=max(fits)
    accepted=corr>.8 and (corr>initial.get('correlation',-1)+.12 or abs(scale-1)<.001)
    if not accepted:
        return dict(initial,**details,drift_reason='affine_clock_model_not_significantly_better',affine_candidate_correlation=corr)
    return dict(initial,**details,accepted=True,manual_alignment_required=False,
        source='signed video/gyro turn matching; affine clock model',
        video_time_at_mcu_origin_s=offset,clock_scale=scale,clock_rate_difference_ppm=(scale-1)*1e6,
        correlation=corr,sample_count=count,local_offset_fit_rms_s=rms,
        local_fit_used_count=int(keep.sum()),local_fit_span_s=float(np.ptp(x[keep])),
        constant_offset_correlation=initial.get('correlation'),
        model='video_time_s = offset_s + clock_scale * (time_ms - mcu_origin_ms) / 1000',
        reason='accepted_affine_clock_candidate',
        uncertainty='offset grid 5 ms; scale grid 100 ppm; exposure and physical clock accuracy uncalibrated')


def turn_groups(ble):
    active = [i for i,r in enumerate(ble) if abs(r['ideal.rad_velo'])>.4]
    groups = []
    for i in active:
        if not groups or ble[i]['time_ms']-ble[groups[-1][-1]]['time_ms']>60:
            groups.append([])
        groups[-1].append(i)
    return groups


def summarize_turns(ble, pose, sync):
    result = []
    offset,zero = sync['video_time_at_mcu_origin_s'], sync['mcu_origin_ms']
    scale=sync.get('clock_scale',1.)
    for number,group in enumerate(turn_groups(ble),1):
        first,last = group[0],group[-1]
        t = np.array([r['time_ms']/1000 for r in ble[first:last+1]])
        rows = ble[first:last+1]
        if len(t)<3:
            continue
        start,end = (t[0]-zero/1000)*scale+offset,(t[-1]-zero/1000)*scale+offset
        observed = [p for p in pose if start<=p['time_video_s']<=end]
        valid = [p for p in observed if p['pose_valid']]
        errors = [r['ego.rad_velo']-r['ideal.rad_velo'] for r in rows]
        item = dict(turn=number,start_video_s=start,end_video_s=end,start_mcu_ms=int(t[0]*1000),
            direction='L' if sum(r['ideal.rad_velo'] for r in rows)>0 else 'R',
            command_angle_deg=float(np.trapezoid([r['ideal.rad_velo'] for r in rows],t)*180/math.pi),
            gyro_angle_deg=float(np.trapezoid([r['ego.rad_velo'] for r in rows],t)*180/math.pi),
            gyro_tracking_rms_rad_s=float(np.sqrt(np.mean(np.square(errors)))),
            video_coverage=len(valid)/max(1,len(observed)),
            ble_max_gap_ms=float(np.max(np.diff(t))*1000),
            min_side_distance_mm=min(min(r['sen_l.distance'],r['sen_r.distance']) for r in rows),
            max_speed_tracking_error_m_s=max(abs(r['ego.velo']-r['ideal.velo']) for r in rows))
        if valid:
            item.update(start_x_cell=valid[0]['x_cell'],start_y_cell=valid[0]['y_cell'],
                        end_x_cell=valid[-1]['x_cell'],end_y_cell=valid[-1]['y_cell'])
        if (sync['accepted'] and len(valid)>=4 and item['video_coverage']>=.85
                and valid[0]['time_video_s']-start<.03 and end-valid[-1]['time_video_s']<.03):
            angle = np.unwrap([p['heading_cw_rad'] for p in valid])
            item['video_apparent_angle_deg'] = float(-(angle[-1]-angle[0])*180/math.pi)
        result.append(item)
    return result


def write_csv(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def review_wall_map(reference, path=None):
    """座標登録用の基準を保持し、描画する壁配置だけを明示的に選ぶ。"""
    if path is None:
        raise ValueError('壁配置の明示指定が必要です。省略時は今回の動画から検出してください')
    selected = json.loads(Path(path).read_text(encoding='utf-8'))
    walls = selected.get('wall_segments', [])
    keys = set()
    for wall in walls:
        key = (wall.get('axis'), wall.get('grid_line'), wall.get('cell_segment'))
        if (key[0] not in ('H', 'V') or type(key[1]) is not int or not 0 <= key[1] <= 9
                or type(key[2]) is not int or not 0 <= key[2] < 9
                or wall.get('state') not in ('wall', 'open', 'unknown') or key in keys):
            raise ValueError('壁配置JSONの座標・状態・重複を確認してください')
        keys.add(key)
    if len(walls) != 180:
        raise ValueError('9×9の壁配置JSONには180辺が必要です')
    metadata = dict(revision=selected.get('wall_revision', '更新確認なし'),
                    source=str(Path(path).resolve()), sha256=sha256(path),
                    edge_count=len(walls), selection='explicit_wall_map')
    return walls, metadata


def analyze(trial_path, reference_path, output, stride=4, cell_mm=None, manual_sync=None, wall_map=None,
            detection_profile=None):
    if stride not in (1,2,4):
        raise ValueError('strideは1,2,4から選択してください')
    if cell_mm is not None and (not math.isfinite(cell_mm) or not 10<=cell_mm<=300):
        raise ValueError('1マスの寸法が不正です')
    trial=json.loads(Path(trial_path).read_text(encoding='utf-8'))
    reference=json.loads(Path(reference_path).read_text(encoding='utf-8'))
    if reference.get('schema')!=1 or reference.get('rows')!=9 or reference.get('columns')!=9:
        raise ValueError('9×9用の登録済み基準JSONを指定してください')
    if wall_map is not None:
        walls, wall_map_metadata = review_wall_map(reference, wall_map)
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    # Keep a reproducible source snapshot even if the working tree is edited
    # while a long decode is in progress.
    source_snapshot=Path(__file__).read_bytes()
    (output/'analysis_source.py').write_bytes(source_snapshot)
    video=Path(trial['video_file'])
    source_hash=sha256(video)
    if trial.get('sha256') and trial['sha256']!=source_hash:
        raise ValueError('trialと動画のSHA256が一致しません')
    ble,settings,zero=read_ble(trial)
    mouse=settings.get('firmware',{}).get('mouse')
    front_colour,rear_colour=marker_colours(mouse)
    profile = json.loads(Path(detection_profile).read_text(encoding='utf-8')) if detection_profile else None
    pose=track(video,reference,stride,cell_mm,mouse=mouse,detection_profile=profile)
    if wall_map is None:
        from analysis.maze_wall_detection import detect_video_walls
        walls, wall_map_metadata = detect_video_walls(video, reference, registration, detector)
    json_save(output/'wall_map_used.json', dict(wall_revision=wall_map_metadata['revision'],
                                              wall_segments=walls, detection=wall_map_metadata))
    if profile is not None:
        json_save(output/'marker_detection_used.json', profile)
    sync=fit_clock_drift(pose,ble,zero,fit_sync(pose,ble,zero,clock_offset(trial,zero)))
    if manual_sync:
        manual=json.loads(Path(manual_sync).read_text(encoding='utf-8'))
        offset=manual['video_time_at_mcu_origin_s']
        scale=manual.get('clock_scale',1.)
        if manual['mcu_origin_ms']!=zero or not math.isfinite(offset) or not math.isfinite(scale) or not .98<=scale<=1.02:
            raise ValueError('手動同期のMCU原点またはoffsetが一致しません')
        sync.update(video_time_at_mcu_origin_s=offset,clock_scale=scale,accepted=True,manual_alignment_required=False,
                    source='explicit manual sync file',manual_sync=str(manual_sync))
    turns=summarize_turns(ble,pose,sync)
    good=sum(p['pose_valid'] for p in pose)
    report=dict(schema=1,created_at=datetime.now().astimezone().isoformat(),
        trial=str(Path(trial_path).resolve()),mouse=settings.get('firmware',{}).get('mouse','unknown'),
        run_type=settings.get('run_type','unknown'),options=settings.get('options'),
        source_video=str(video),video_sha256=source_hash,run_settings_sha256=sha256(trial['run_settings']),
        ble_csv_sha256=sha256(Path(trial['ble_directory'])/'telemetry.csv'),
        reference=str(Path(reference_path).resolve()),reference_sha256=sha256(reference_path),wall_map=wall_map_metadata,
        analysis_source_sha256=hashlib.sha256(source_snapshot).hexdigest(),opencv_version=cv2.__version__,numpy_version=np.__version__,
        stride=stride,cell_mm=cell_mm,front_colour=front_colour,rear_colour=rear_colour,
        marker_detection_modes={mode:sum(p.get('detection_mode')==mode for p in pose)
            for mode in ('standard','small_marker_fallback','selected_profile')},
        nominal_marker_span_mm=(profile or {}).get('nominal_marker_span_mm',50.),
        marker_detection_profile=profile,
        marker_detection_profile_sha256=sha256(detection_profile) if detection_profile else None,
        position_kind='apparent_front_rear_marker_midpoint_on_reference_plane',
        axes=reference['axes'],height_correction=False,camera_height_mm=None,
        sampled_frames=len(pose),valid_frames=good,pose_coverage=good/max(1,len(pose)),
        calibration_coverage=sum(p['calibration_valid'] for p in pose)/max(1,len(pose)),
        ble_quality=trial.get('ble_quality'),
        ble_period_ms=settings.get('period_ms'),ble_max_gap_ms=max(np.diff([r['time_ms'] for r in ble])),
        sync=sync,turns=turns,
        limitations=['カメラ高さ・真下位置・レンズ歪みは未校正。mm値は見かけの平面投影',
          '前後マーカー中点を追跡。車軸の絶対位置・真の横滑り角は未確定',
          '欠測は補間しない。壁付近の赤・電池の黄色との誤識別は画像重畳で要確認',
          '同期は旋回波形からのオフセット・時計比率の候補。露光・物理的な時計精度は未校正',
          '旋回番号は角速度指令区間の順序。180度・複合ターンの名称は経路と照合が必要',
          '制御改善候補の観察資料。採用Paramsは変更しない'])
    write_csv(output/'pose.csv',pose)
    write_csv(output/'turns.csv',turns)
    json_save(output/'sync.json',sync)
    json_save(output/'analysis.json',report)
    matched=[]
    times=np.array([r['time_ms'] for r in ble])
    for p in pose:
        mcu=zero+(p['time_video_s']-sync['video_time_at_mcu_origin_s'])/sync.get('clock_scale',1.)*1000
        i=int(np.clip(np.searchsorted(times,mcu),0,len(times)-1))
        if i and abs(times[i-1]-mcu)<abs(times[i]-mcu): i-=1
        # Never fill a missing or out-of-run BLE sample with the nearest value.
        if abs(times[i]-mcu)<=12:
            matched.append(dict(p,time_mcu_estimated_ms=mcu,ble_time_error_ms=times[i]-mcu,
                                **{'ble_'+k:v for k,v in ble[i].items()}))
    write_csv(output/'matched.csv',matched)
    build_review(output,trial,ble,pose,dict(reference, wall_segments=walls),report)
    lines=[f'# {report["mouse"]} {report["run_type"]} 動画位置解析',
        f'追跡率 {report["pose_coverage"]:.1%}、盤面登録 {report["calibration_coverage"]:.1%}、旋回区間 {len(turns)}。',
        f'同期: {sync["source"]} / 採用可能フラグ {sync["accepted"]}。',
        f'BLE最大間隔 {report["ble_max_gap_ms"]:.0f} ms。stride={stride}。',
        f'前後マーカー: 前={front_colour} / 後={rear_colour}（実機設定JSONのfirmware.mouseから選択）。',
        '## 確認する順序',
        '1. review.htmlでマーカーへの画像重畳と軌跡の飛びを確認する。',
        '2. 青LED・旋回波形で同期を確認し、必要なら手動同期JSONで再解析する。',
        '3. turns.csvの角速度追従誤差・映像観測率・左右壁距離を旋回ごとに比較する。',
        '4. 入口位置／出口角度／直進中のずれを切り分けて追加測定し、再現後に制御候補を決める。',
        '## 制約',*['- '+s for s in report['limitations']]]
    (output/'README.md').write_text('\n\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('mouse','run_type','pose_coverage','calibration_coverage','ble_max_gap_ms')},ensure_ascii=False),flush=True)
    print(json.dumps(sync,ensure_ascii=False),flush=True)
    return report


def build_review(output, trial, ble, pose, reference, report):
    # Missing values become null for JS. Pose is never interpolated across occlusion.
    settings = json.loads(Path(trial['run_settings']).read_text(encoding='utf-8'))
    events = review_rows(settings, report['sync'])
    write_csv(output/'wall_events.csv', events)
    # IR edges need the original saved sampling interval, even when motion
    # traces are thinned for display. Missing old columns stay null.
    wall_fields = ('time_ms', 'sen_l.distance', 'sen_r.distance', 'sen_fl.distance', 'sen_fr.distance',
                   'sen_l.value', 'sen_r.value', 'sen_fl.value', 'sen_fr.value', 'ir.range_states', 'motion.pattern')
    data=dict(pose=pose,ble=ble[::4],report=report,walls=reference.get('wall_segments',[]),wall_events=events,
              wall_signal_fields=wall_fields,wall_signals=[[r.get(k) for k in wall_fields] for r in ble])
    payload=json.dumps(data,ensure_ascii=False,allow_nan=False).replace('</','<\\/')
    template=Path(__file__).with_suffix('.html').read_text(encoding='utf-8')
    video=html.escape(Path(trial['video_file']).as_uri(),quote=True)
    (output/'review.html').write_text(template.replace('VIDEO_URI',video).replace('PAYLOAD',payload),encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path)
    parser.add_argument('--reference',type=Path)
    parser.add_argument('--wall-map',type=Path,help='走行時の9×9物理壁配置JSON。ArUco座標登録とは別に表示へ適用')
    parser.add_argument('--marker-detection',type=Path,help='機体別の色・小さい目印の検出設定JSON。高さ・盤面登録は流用しない')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--stride',type=int,choices=[1,2,4],default=4)
    parser.add_argument('--cell-mm',type=float)
    parser.add_argument('--sync',type=Path,help='比較画面で保存した手動同期JSON')
    parser.add_argument('--register-frame',type=Path)
    parser.add_argument('--grid-report',type=Path)
    args=parser.parse_args()
    if args.register_frame:
        make_reference(args.register_frame,args.grid_report,args.output)
    else:
        if not args.trial or not args.reference:
            parser.error('--trial と --reference が必要です')
        analyze(args.trial,args.reference,args.output,args.stride,args.cell_mm,args.sync,args.wall_map,args.marker_detection)


if __name__=='__main__':
    main()
