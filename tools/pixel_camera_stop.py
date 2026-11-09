"""走行後の停止を観測し、録画だけを早期終了する。走行コマンドは送らない。"""
from __future__ import annotations

import math
import time


def video_time_base(path):
    """MP4の実際の時刻単位を読む。推測したtimescaleを検査へ使わない。"""
    import av
    with av.open(str(path)) as container:
        return float(container.streams.video[0].time_base)


def owned_recording(status, client, recording_id):
    device = status.get('device', {})
    return (status.get('connected') is True and device.get('recording_id') == recording_id
            and device.get('owner') == client.client_id and device.get('phase') == 'RECORDING')


def wait_motion_stop(client, recording_id, seconds, *, tail, health, baseline_crc,
                     alive, stop_evidence, event, now=None, sleep=None):
    """200msの停止確認後、PC・MCU・動画時刻で各1秒以上を待つ。異常時は上限へ任せる。"""
    now, sleep = now or time.monotonic, sleep or time.sleep
    deadline, anchor = now()+seconds+1, None
    value = dict(mode='motion_end_plus_postroll', postroll_seconds=1.0, request_stop=False)
    def fallback(reason):
        value.update(fallback_reason=reason)
        event('recording_stop_fallback', reason=reason)
        return value
    try:
        while now() < deadline:
            if not alive():
                return fallback('ble_disconnected')
            status = client.get_status()
            if not owned_recording(status, client, recording_id):
                return fallback('recording_no_longer_owned_or_active')
            elapsed = float(status['device']['elapsed_seconds'])
            if not math.isfinite(elapsed) or elapsed < 0:
                return fallback('invalid_camera_clock')
            if elapsed >= seconds-.5:
                return fallback('duration_cap_near')
            samples = tail.snapshot()
            if now()-tail.latest_at > .5:
                return fallback('stale_csv')
            ledger = health()
            if not 0 <= now()-ledger['monotonic'] <= 3 or ledger['crc_errors'] != baseline_crc:
                return fallback('stale_health_or_crc_error')
            steps = [(int(b['time_ms'])-int(a['time_ms'])) % 2**32 for a,b in zip(samples,samples[1:])]
            sequences = [(int(b['sequence'])-int(a['sequence'])) % 2**32 for a,b in zip(samples,samples[1:])]
            if (any(not 0 < step <= 25 for step in steps) or any(step != 1 for step in sequences)
                    or any(int(row['lost_frames']) != 0 for row in samples)):
                return fallback('discontinuous_csv')
            evidence = stop_evidence(samples)
            if evidence.get('reason') in ('invalid_motion_log', 'nonfinite_motion_log'):
                return fallback(evidence['reason'])
            if not evidence['confirmed']:
                anchor = None
            elif anchor is None or anchor['stationary_start_time_ms'] != evidence['stationary_start_time_ms']:
                anchor = dict(pc=now(), camera=elapsed, mcu=int(samples[-1]['time_ms']),
                              sequence=int(samples[-1]['sequence']), **evidence)
                event('recording_postroll', stationary_ms=evidence['stationary_ms'], postroll_seconds=1.0)
            else:
                pc_span, video_span = now()-anchor['pc'], elapsed-anchor['camera']
                mcu_span = (int(samples[-1]['time_ms'])-anchor['mcu']) % 2**32
                covered = (int(ledger['latest_sequence'])-anchor['sequence']) % 2**32 < 2**31
                if pc_span >= 1 and video_span >= 1 and 1000 <= mcu_span < 60000 and covered:
                    value.update(request_stop=True, stationary_ms=evidence['stationary_ms'],
                        postroll_elapsed_seconds=pc_span, postroll_mcu_ms=mcu_span,
                        postroll_camera_seconds=video_span, confirmation_camera_seconds=anchor['camera'],
                        decision_camera_seconds=elapsed, log_stop_evidence=evidence)
                    event('recording_postroll_complete', **value)
                    return value
            sleep(.1)
    except Exception as error:
        return fallback(type(error).__name__+': '+str(error))
    return fallback('monitor_timeout')


def inspect_controlled_stop(inspection, metadata, seconds, stop):
    """既存の全復号・ハッシュ検査を維持し、意図した短縮録画の時間条件だけを置換する。"""
    inspection = dict(inspection, recording_stop_mode='duration_cap')
    if metadata.get('stop_reason') != 'USER_OR_ACTIVITY_STOP':
        return inspection
    errors = []
    def require(condition, code):
        if not condition:
            errors.append(code)
    def number(value):
        return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)
    window = metadata.get('recording_window', {})
    require(stop.get('request_stop') is True and stop.get('stop_write_attempted') is True,
            'UNCONFIRMED_CONTROLLED_STOP')
    require(all(number(stop.get(key)) and stop[key] >= minimum for key, minimum in (
        ('postroll_elapsed_seconds', 1), ('postroll_mcu_ms', 1000),
        ('postroll_camera_seconds', 1), ('stationary_ms', 1200))), 'INSUFFICIENT_STOP_POSTROLL')
    require(metadata.get('requested_duration_seconds') == seconds
            and window.get('duration_limit_us') == seconds*1_000_000
            and window.get('origin') == 'first_encoded_frame_pts'
            and window.get('timestamps_rescaled') is False and window.get('limit_reached') is False,
            'CONTROLLED_STOP_WINDOW_MISMATCH')
    span, duration = inspection.get('pts_span_seconds'), inspection.get('duration_seconds')
    last_pts, requested_at = window.get('last_output_relative_pts_us'), stop.get('request_camera_seconds')
    require(number(span) and number(duration) and 1.2 <= span < seconds
            and 0 <= duration-span <= .05, 'CONTROLLED_STOP_DURATION_MISMATCH')
    # エンコーダのµsとMP4のtickは異なる。復号PTSはMediaExtractorのMP4 PTSと照合する。
    # tickへの丸めが各sample durationへ蓄積するため、元のµsとの完全一致を要求しない。
    extracted = metadata.get('mp4', {}).get('sample_timing', {}).get('span_seconds')
    encoded = metadata.get('encoded_timing', {}).get('span_seconds')
    tick, frames = inspection.get('mp4_time_base_seconds'), inspection.get('decoded_frames')
    require(number(span) and number(extracted) and abs(span-extracted) <= .00001,
            'CONTROLLED_STOP_MP4_PTS_MISMATCH')
    require(number(encoded) and number(last_pts) and abs(encoded-last_pts/1_000_000) <= .000001,
            'CONTROLLED_STOP_ENCODER_PTS_MISMATCH')
    limit = (frames-1)*tick+.000002 if (number(frames) and frames > 1 and number(tick)
                                       and 0 < tick <= 1/240) else None
    require(limit is not None and number(encoded) and number(extracted)
            and abs(encoded-extracted) <= limit, 'CONTROLLED_STOP_TIMEBASE_MISMATCH')
    require(number(span) and number(requested_at) and requested_at > 0
            and requested_at-1/240 <= span <= requested_at+2,
            'CONTROLLED_STOP_REQUEST_TIME_MISMATCH')
    replaceable = {'UNEXPECTED_STOP_REASON', 'FIRST_FRAME_DURATION_NOT_CONFIRMED',
                   'VIDEO_DURATION_MISMATCH', 'PTS_SPAN_MISMATCH'}
    original = inspection['errors']
    remaining = [code for code in original if code not in replaceable] if not errors else list(original)
    inspection.update(recording_stop_mode='motion_end_plus_postroll', fixed_duration_errors=list(original),
                      encoder_mp4_span_delta_seconds=encoded-extracted if number(encoded) and number(extracted) else None,
                      muxer_quantization_bound_seconds=limit,
                      errors=remaining+errors, passed=not (remaining+errors))
    return inspection
