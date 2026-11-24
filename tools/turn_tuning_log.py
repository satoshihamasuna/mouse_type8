"""本体保存ログを実際の記録周期で描画する。機器への通信は行わない。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def render_log(trial_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from analysis.log_compare import load_log, plot_comparisons

    trial_path = Path(trial_path)
    trial = json.loads(trial_path.read_text(encoding='utf-8'))
    saved = trial.get('saved_log', {})
    if saved.get('status') != 'saved':
        raise ValueError('描画できる本体保存ログがありません')
    period = saved['period_ms']
    if type(period) is not int or not 1 <= period <= 100:
        raise ValueError('本体ログの記録周期が不正です')
    source = trial_path.parent / saved['csv']
    settings = json.loads((trial_path.parent / saved['settings_json']).read_text(encoding='utf-8'))
    frame = load_log(source)
    # 本体のcntは経過ms。既存の描画関数はサンプル番号×周期を使う。
    if 'cnt' not in frame or len(frame) != saved['frame_count'] or frame.empty:
        raise ValueError('本体ログの行数・時間軸が不正です')
    import numpy as np
    if not np.array_equal(frame['cnt'].to_numpy(), np.arange(len(frame)) * period):
        raise ValueError('本体ログの時刻と記録周期が一致しません')
    frame['cnt'] = frame['cnt'] / period
    figure = plot_comparisons(frame,
        f"Mouse {trial['mouse']} / {trial['motion']} / {period} ms / {len(frame)} samples",
        period_ms=period, settings=settings,
        jerk_gain=settings.get('feedforward', {}).get('ff_om_jerk', 0), jerk_gain_source='settings.json')
    output = trial_path.parent / 'log.png'
    try:
        figure.savefig(output, dpi=130)
        # GUIには縮小版を表示し、全解像度の画像は同じフォルダーへ残す。
        figure.savefig(trial_path.parent / 'log_preview.png', dpi=42)
    finally:
        plt.close(figure)
    trial['log_image'] = str(output)
    trial['log_image_period_ms'] = period
    trial_path.write_text(json.dumps(trial, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    print(render_log(parser.parse_args().trial))
