"""調整用と9×9迷路用の撮影条件を別々に保存し、録画ごとに照合する。"""
from __future__ import annotations

import json
import math
from pathlib import Path
import re

LOGS = Path(__file__).resolve().parent / 'logs/pixel_camera'
DEFAULTS = {
    'tuning': dict(ev=-2.5, torch=False, focus_lock=True, zoom_ratio=1.0),
    'maze_9x9': dict(ev=-2.0, torch=False, focus_lock=True, zoom_ratio=1.0),
}


def validate_camera_settings(settings):
    if not isinstance(settings, dict) or set(settings)-{'ev', 'torch', 'focus_lock', 'zoom_ratio', 'exposure_compensation_steps'}:
        raise ValueError('不明な撮影設定です')
    for key in ('ev', 'zoom_ratio'):
        if key in settings:
            value = settings[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('撮影EV・ズームは有限の数値で指定してください')
    if 'ev' in settings and not -4 <= settings['ev'] <= 4:
        raise ValueError('撮影EVは−4～4で指定してください')
    if 'zoom_ratio' in settings and settings['zoom_ratio'] < 1:
        raise ValueError('ズーム倍率は1以上で指定してください')
    for key in ('torch', 'focus_lock'):
        if key in settings and type(settings[key]) is not bool:
            raise ValueError('ライト・AF固定はON/OFFで指定してください')
    if 'exposure_compensation_steps' in settings:
        if type(settings['exposure_compensation_steps']) is not int or 'ev' in settings:
            raise ValueError('露出はEVまたは整数ステップの一方で指定してください')
    return dict(settings)


def profile_path(profile, directory=None):
    if profile not in DEFAULTS:
        raise ValueError('撮影用途はtuning / maze_9x9で指定してください')
    return Path(directory or LOGS) / 'capture_profiles' / (profile + '.json')


def load_camera_profile(profile, directory=None):
    path = profile_path(profile, directory)
    settings = dict(DEFAULTS[profile])
    if path.is_file():
        value = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(value, dict) or value.get('schema') != 1 or value.get('capture_profile') != profile:
            raise ValueError('撮影設定の用途・形式が一致しません: ' + str(path))
        saved = validate_camera_settings(value['camera_settings'])
        if 'exposure_compensation_steps' in saved:
            settings.pop('ev')
        settings.update(saved)
    elif profile == 'maze_9x9':
        # 以前の静止比較値は読取りだけに使い、測定の根拠ファイルを上書きしない。
        legacy = path.parent.parent / 'explore_capture_preferences.json'
        try:
            value = json.loads(legacy.read_text(encoding='utf-8'))
            saved = validate_camera_settings(value['camera_settings'])
            if value.get('schema') == 1:
                settings.update(saved)
        except (OSError, ValueError, KeyError, TypeError):
            pass
    return settings


def save_camera_profile(profile, settings, directory=None):
    settings = validate_camera_settings(settings)
    required = {'torch', 'focus_lock', 'zoom_ratio'}
    if not required <= set(settings) or not ({'ev', 'exposure_compensation_steps'} & set(settings)):
        raise ValueError('用途別の撮影条件をすべて指定してください')
    path = profile_path(profile, directory)
    path.parent.mkdir(parents=True, exist_ok=True)
    value = dict(schema=1, capture_profile=profile, camera_settings=settings, apply_on_gui_open=False)
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def configure_camera(client, settings=None):
    """受付不一致では録画へ進まない。明るさや合焦の実測確認とは区別する。"""
    settings = validate_camera_settings(settings or {})
    requested = {key: value for key, value in settings.items() if key != 'ev'}
    evidence = dict(selection=settings, physical_brightness_effect_verified=False)
    if 'ev' in settings:
        from pixel_camera_framing import ev_to_steps
        cameras = [camera for camera in client.get_capabilities().get('cameras', [])
                   if camera.get('target_fixed_240_supported') and camera.get('back_facing')]
        if len(cameras) != 1:
            raise ValueError('240fpsカメラの露出刻みを一意に確認できません')
        camera = cameras[0]
        limits = [int(value) for value in re.findall(r'-?\d+', str(camera['ev_steps_range']))]
        if len(limits) != 2:
            raise ValueError('カメラの露出補正範囲を確認できません')
        requested['exposure_compensation_steps'] = ev_to_steps(settings['ev'], camera['ev_step'], limits)
        evidence.update(ev_step=camera['ev_step'], ev_steps_range=limits)
    response = client.configure(**requested)
    if requested and not isinstance(response, dict):
        raise RuntimeError('カメラの撮影設定の受付を確認できません。走行を開始しません')
    if requested and any(type(response.get(key)) is not type(value) or response[key] != value
                         for key, value in requested.items() if key != 'zoom_ratio'):
        raise RuntimeError('カメラの撮影設定の受付が一致しません。走行を開始しません')
    if 'zoom_ratio' in requested:
        actual = response.get('zoom_ratio')
        if isinstance(actual, bool) or not isinstance(actual, (int, float)) or not math.isclose(actual, requested['zoom_ratio'], abs_tol=1e-6):
            raise RuntimeError('カメラのズーム設定の受付が一致しません。走行を開始しません')
    evidence.update(requested=requested, response=response)
    return evidence
