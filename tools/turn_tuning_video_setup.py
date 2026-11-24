"""機体別の前後目印を登録する。カメラや機体の操作は行わない。"""
from datetime import datetime
import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import ttk
import uuid

COLOURS = {'赤': 'red', '黄': 'yellow', '緑': 'green', '青': 'blue', 'ピンク': 'magenta'}


def body_geometry(mouse, reference, front, rear, front_height, rear_height, camera_height, span=None):
    if mouse not in ('A', 'B') or not reference.get('reference_id'):
        raise ValueError('機体A/Bと認識済みの盤面基準を選択してください')
    if front not in COLOURS.values() or rear not in COLOURS.values() or front == rear:
        raise ValueError('前後の目印に異なる色を指定してください')
    if isinstance(camera_height, bool) or not isinstance(camera_height, (int, float)) or not math.isfinite(camera_height) or camera_height <= 20:
        raise ValueError('床からレンズまでの高さをmmで指定してください')
    if any(isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height)
            or not 0 <= height < camera_height for height in (front_height, rear_height)):
        raise ValueError('目印の床からの高さを0以上、カメラ高さ未満のmmで指定してください')
    if span is not None and (not math.isfinite(span) or span <= 0):
        raise ValueError('前後目印の間隔は正のmmで指定してください')
    return dict(schema_version=1, mouse=mouse, reference_id=reference['reference_id'],
        front_colour=front, rear_colour=rear, front_height_mm=front_height, rear_height_mm=rear_height,
        nominal_marker_span_mm=span, axle_offset_from_marker_midpoint_mm=None,
        camera_height_policy='per_measurement', source='operator_entered_gui',
        evaluation_scope='relative marker midpoint; absolute axle position not calibrated')


def validate_video_inputs(mouse, reference_path, geometry_path, camera_height):
    from pixel_camera_detection import load_detection_geometry
    reference = json.loads(Path(reference_path).read_text(encoding='utf-8'))
    geometry = load_detection_geometry(geometry_path)
    if geometry.get('mouse') != mouse or geometry.get('reference_id') != reference.get('reference_id'):
        raise ValueError('選択した機体・盤面に対応する映像解析設定を登録してください')
    body_geometry(mouse, reference, geometry.get('front_colour'), geometry.get('rear_colour'),
        geometry.get('front_height_mm', float('nan')), geometry.get('rear_height_mm', float('nan')),
        camera_height, geometry.get('nominal_marker_span_mm'))
    return geometry


def open_video_setup(panel):
    mouse = panel.mouse.get()
    if mouse not in ('A', 'B'):
        panel.status.set('先に機体A/Bを選択してください')
        return
    try:
        reference = json.loads(Path(panel.reference.get()).read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        panel.status.set('盤面基準を選択してください: '+str(error))
        return
    try:
        saved = json.loads(Path(panel.geometry.get()).read_text(encoding='utf-8'))
        if saved.get('mouse') != mouse:
            saved = {}
    except (OSError, ValueError):
        saved = {}
    window = tk.Toplevel(panel.gui)
    window.title(f'機体{mouse} — 映像解析の初回設定')
    form = ttk.Frame(window, padding=14); form.pack(fill=tk.BOTH, expand=True)
    names = {value: key for key, value in COLOURS.items()}
    front = tk.StringVar(value=names.get(saved.get('front_colour'), ''))
    rear = tk.StringVar(value=names.get(saved.get('rear_colour'), ''))
    front_height = tk.StringVar(value=str(saved.get('front_height_mm', '')))
    rear_height = tk.StringVar(value=str(saved.get('rear_height_mm', '')))
    try:
        camera_height = str(panel.height.get())
    except tk.TclError:
        camera_height = ''
    height = tk.StringVar(value=camera_height)
    span = tk.StringVar(value=str(saved.get('nominal_marker_span_mm') or ''))
    for row, (label, variable) in enumerate((('前の目印の色', front), ('後ろの目印の色', rear),
            ('前の目印の床からの高さ [mm]', front_height), ('後ろの目印の床からの高さ [mm]', rear_height),
            ('床からレンズまでの高さ [mm]', height), ('前後目印の間隔 [mm]（任意）', span))):
        ttk.Label(form, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        widget = (ttk.Combobox(form, textvariable=variable, values=tuple(COLOURS), state='readonly', width=12)
                  if row < 2 else ttk.Entry(form, textvariable=variable, width=15))
        widget.grid(row=row, column=1, sticky=tk.W, padx=8)
    error_text = tk.StringVar()
    ttk.Label(form, text='前後の目印の中点を使った相対軌跡を解析します。\n車軸の絶対位置は未校正です。').grid(row=6, column=0, columnspan=2, sticky=tk.W, pady=8)
    ttk.Label(form, textvariable=error_text, wraplength=440).grid(row=7, column=0, columnspan=2, sticky=tk.W)

    def save():
        try:
            camera_height = float(height.get())
            value = body_geometry(mouse, reference, COLOURS.get(front.get()), COLOURS.get(rear.get()),
                float(front_height.get()), float(rear_height.get()), camera_height,
                float(span.get()) if span.get().strip() else None)
            if all(saved.get(key) == value[key] for key in ('mouse', 'front_colour', 'rear_colour')):
                for key in ('marker_detection', 'detector_adjustment_source'):
                    if key in saved:
                        value[key] = saved[key]
            path = panel.preferences.parent/f'body_geometry_{mouse}_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:4]}.json'
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('x', encoding='utf-8') as output:
                json.dump(value, output, ensure_ascii=False, indent=2)
            panel.geometry.set(str(path)); panel.height.set(camera_height)
            panel.geometry_by_mouse[mouse] = str(path)
            panel.save_preferences()
            panel.status.set(f'機体{mouse}の映像解析設定を保存しました')
            window.destroy()
        except (OSError, ValueError) as error:
            error_text.set(str(error))
    ttk.Button(form, text='保存する', command=save).grid(row=8, column=0, columnspan=2, pady=10)
