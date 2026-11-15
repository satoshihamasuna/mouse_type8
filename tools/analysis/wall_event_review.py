"""保存済みMCU判断時刻を、採用した動画同期へ写像する（補間なし）。"""
import math

from ble.wall_diagnostics import decoded_row


def review_rows(settings, sync):
    rows = []
    for key, kind in (('turn_diagnostics', 'turn_decision'), ('wall_diagnostics', 'wall_correction')):
        diagnostics = settings.get(key) or {}
        for event in diagnostics.get('events', []):
            row = decoded_row(event)
            row.update(kind=kind, diagnostics_complete=diagnostics.get('complete', False), video_s=None,
                       sync_accepted=sync.get('accepted', False))
            try:
                ms = float(row['ms'])
                video = sync['video_time_at_mcu_origin_s'] + sync.get('clock_scale', 1.) * (ms-sync['mcu_origin_ms'])/1000
                if math.isfinite(video) and math.isfinite(ms):
                    row['video_s'] = video
            except (KeyError, ValueError, TypeError):
                pass
            rows.append(row)
    return rows
