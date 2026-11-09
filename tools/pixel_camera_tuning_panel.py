"""myshell_debugの撮影タブ。Tk変数はUIスレッドでだけ取得する。"""
from __future__ import annotations
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import threading
import tkinter as tk
from tkinter import ttk
import uuid

from pixel_camera_tuning import (LiveSession, TrialRequest, analysis_python,
    capture_connected, hidden_process)
from pixel_camera_turn_capture import write_json


def gui_expected_settings(gui, mouse):
    return dict(mouse=mouse, motion_name=gui.turn_type.get(), preset_speed_mm_s=gui.turn_preset_speed.get(),
        turn_count=gui.turn_count.get(), parameters=dict(velo=gui.values['distance'].get(),
        r_min=gui.values['acc'].get(), lstart=gui.values['max_velo'].get(), lend=gui.values['end_velo'].get(),
        degree=gui.values['degree'].get(), correction=gui.values['degree_correction'].get(), pre_accel=gui.pre_accel.get()),
        pid={name:gui.values[name].get() for name in ('sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd')},
        feedforward={name:gui.values[name].get() for name in ('ff_sp_velo','ff_sp_accel','ff_sp_bias',
            'ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')},
        suction=dict(enabled=gui.suction_enable.get(), duty=gui.suction_duty.get()))


class PixelTuningPanel:
    def __init__(self, gui, notebook):
        self.gui, self.project = gui, Path(__file__).resolve().parents[1]
        self.session = None
        self.rows, self.summaries, self.widget_states = {}, {}, []
        self.latest_folder, self.photo = None, None
        self.analysis_lock = threading.Lock()
        base = self.project/'tools/logs/pixel_camera'
        self.preferences = base/'tuning_ui.json'
        def read(path):
            try:
                return json.loads(path.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                return {}
        saved = read(self.preferences)
        previous = read(base/'current_ui_tuning_session.json')
        last = Path(previous.get('session_folder', str(base)))
        reference = last/'reference_visible_spread.json'
        geometry = last/'body_geometry_detection.json'
        self.mouse = tk.StringVar(value=saved.get('mouse', previous.get('mouse', '')))
        self.height = tk.DoubleVar(value=saved.get('height_mm', 800))
        self.seconds = tk.IntVar(value=20)
        self.address = tk.StringVar(value=saved.get('address', 'C1:B5:60:87:91:73'))
        self.reference = tk.StringVar(value=saved.get('reference', str(reference if reference.exists() else base/'marker_reference.json')))
        self.geometry = tk.StringVar(value=saved.get('geometry', str(geometry if geometry.exists() else base/'body_geometry_A.json')))
        self.ready = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='未接続。撮影用接続を準備し、Load defaultsで実機プリセットを取得してください')
        self.detail_status = tk.StringVar(value='停止確認後1秒以上を撮影。確認できない場合は録画上限まで保存します。詳細検証は速報後に実行します')
        self.tab = ttk.Frame(notebook, padding=6)
        notebook.add(self.tab, text='Pixel撮影・結果')
        conditions = ttk.Frame(notebook, padding=10)
        notebook.add(conditions, text='撮影条件')
        row = ttk.Frame(self.tab)
        row.pack(fill=tk.X)
        ttk.Label(row, text='機体（自己申告）').pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.mouse, values=('A','B'), state='readonly', width=3).pack(side=tk.LEFT, padx=4)
        ttk.Label(row, text='今回の高さ[mm]').pack(side=tk.LEFT, padx=(8,0))
        ttk.Entry(row, textvariable=self.height, width=7).pack(side=tk.LEFT, padx=4)
        ttk.Label(row, text='録画上限[秒]').pack(side=tk.LEFT, padx=(8,0))
        ttk.Combobox(row, textvariable=self.seconds, values=(20,30), state='readonly', width=4).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(row, text='設置・停止・固定カメラと画角・手離しを確認', variable=self.ready).pack(side=tk.LEFT, padx=10)
        buttons = ttk.Frame(self.tab)
        buttons.pack(fill=tk.X, pady=4)
        ttk.Button(buttons, text='撮影用接続を準備', command=self.prepare).pack(side=tk.LEFT)
        ttk.Button(buttons, text='撮影して1回走行', command=self.run).pack(side=tk.LEFT, padx=6)
        ttk.Button(buttons, text='結果フォルダーを開く', command=self.open_result).pack(side=tk.LEFT, padx=6)
        ttk.Label(self.tab, textvariable=self.status, wraplength=1150).pack(anchor=tk.W, pady=2)
        ttk.Label(self.tab, textvariable=self.detail_status, wraplength=1150).pack(anchor=tk.W, pady=2)
        content = ttk.Frame(self.tab)
        content.pack(fill=tk.BOTH, expand=True)
        self.table = ttk.Treeview(content, columns=('trial','lend','forward','right','stop','full'), show='headings', height=5)
        for name, label, width in (('trial','試行',100),('lend','Lend[mm]',80),('forward','前後[mm]',85),
                                    ('right','右[mm]',85),('stop','停止確認',110),('full','詳細検証',120)):
            self.table.heading(name, text=label)
            self.table.column(name, width=width, stretch=False)
        self.table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.table.bind('<<TreeviewSelect>>', lambda _event: self.select_result())
        self.preview = ttk.Label(content, text='速報の軌跡をここに表示します', anchor=tk.CENTER)
        self.preview.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8)
        for index, (label, variable) in enumerate((('HY0020 BLEアドレス', self.address),
                ('盤面の登録参照JSON', self.reference), ('機体・色検出プロファイルJSON', self.geometry))):
            ttk.Label(conditions, text=label).grid(row=index, column=0, sticky=tk.W, pady=5)
            ttk.Entry(conditions, textvariable=variable, width=95).grid(row=index, column=1, sticky=tk.EW, padx=8)
        conditions.columnconfigure(1, weight=1)
        ttk.Label(conditions, text='BLE 8766／カメラ8765。参照・目印高さと色条件は撮影ごとにコピーします。\n'
            '画面のターン設定を照合してから撮影します。プリセットは撮影時に読み直しません。\n'
            '速報は約60fpsの暫定結果、詳細は240fps全フレーム。自動の値採用・連続走行・実行再送はしません。',
            wraplength=1100).grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=10)

    def _busy(self, value):
        self.gui._camera_busy = value
        if value:
            def disable(parent):
                for widget in parent.winfo_children():
                    if isinstance(widget, (ttk.Button, ttk.Entry, ttk.Combobox, ttk.Spinbox, ttk.Checkbutton)):
                        self.widget_states.append((widget, str(widget.cget('state'))))
                        widget.configure(state='disabled')
                    disable(widget)
            disable(self.gui)
        else:
            for widget, state in self.widget_states:
                if widget.winfo_exists():
                    widget.configure(state=state)
            self.widget_states.clear()

    def _launch(self, operation, callback):
        if self.gui._camera_busy or not self.gui.lock.acquire(blocking=False):
            self.status.set('通信処理中です。完了を待ってください')
            return False
        self._busy(True)
        def worker():
            try:
                result = operation()
                self.gui.events.put(('pixel_done', callback, result, None))
            except Exception as error:
                self.gui.events.put(('pixel_done', callback, None, str(error)))
            finally:
                self.gui.lock.release()
        threading.Thread(target=worker, daemon=True).start()
        return True

    def prepare(self):
        if self.gui.ser and self.gui.ser.is_open:
            self.status.set('既存の接続をDisconnectしてから準備してください')
            return
        if self.session:
            self.session.close()
        saved = dict(mouse=self.mouse.get(), height_mm=self.height.get(), address=self.address.get(),
                     reference=self.reference.get(), geometry=self.geometry.get())
        self.preferences.parent.mkdir(parents=True, exist_ok=True)
        write_json(self.preferences, saved)
        address = self.address.get().strip()
        folder = self.preferences.parent/('tuning_gui_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        session = LiveSession(self.project, folder)
        self.status.set('Pixel待機状態とBLE接続、連続CSVを準備しています…')
        def attached(result):
            self.session = result
            self.gui.ser = result.connection
            self.gui._connection_generation += 1
            self.gui._shell_ready = False
            self.gui.port.set('BLE')
            self.gui.connect_button.configure(text='Disconnect')
            self.status.set('接続準備完了。Load defaultsで実機プリセットを取得し、値を編集してください')
        self._launch(lambda: session.prepare(address), attached)

    def run(self):
        if self.gui._camera_busy or self.gui.lock.locked():
            self.status.set('通信・撮影処理の完了を待ってください')
            return
        try:
            if self.gui.motion.get() != 'turn':
                raise ValueError('撮影連動はMotion=turnの単体1回に対応します')
            if not self.session or self.gui.ser is not self.session.connection:
                raise ValueError('先に「撮影用接続を準備」を押してください')
            request = TrialRequest(gui_expected_settings(self.gui, self.mouse.get()), self.gui._set_command(),
                self.gui.char_delay.get(), self.height.get(), Path(self.reference.get()), Path(self.geometry.get()),
                self.seconds.get(), self.ready.get())
            request.validate()
        except Exception as error:
            self.status.set(str(error))
            return
        self.ready.set(False)  # 一回分。例外・タイムアウト後も自動で再設定しない。
        folder = self.session.folder/('run_'+datetime.now().strftime('%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.latest_folder = folder
        self.rows[str(folder)] = self.table.insert('',tk.END, values=(folder.name,request.expected['parameters']['lend'],'—','—','準備中','未実施'))
        generation = self.gui._connection_generation
        def progress(name, fields):
            self.gui.events.put(('pixel_progress', name, fields))
        def operation():
            self.gui._assert_connection(generation)
            result = capture_connected(self.session, request, folder, self.gui._parse_device_settings, progress)
            quick = None
            if result['state'] == 'CAPTURED_REVIEW_REQUIRED':
                self.gui.events.put(('pixel_progress','quick_analysis',{}))
                self.analyze(folder,4)
                path = folder/'quick/summary.json'
                if path.exists():
                    quick = json.loads(path.read_text(encoding='utf-8'))
                # UIを閉じても、すでに依頼した保存データの詳細検証は完了させる。
                threading.Thread(target=self._full, args=(folder,), daemon=False).start()
            return folder, result, quick
        self.status.set('設定照合・録画準備中。録画中の確認後に1回だけ走行します')
        if not self._launch(operation, self.completed):
            self.table.delete(self.rows.pop(str(folder)))

    def analyze(self, folder, stride):
        with (folder/('quick.stdout.txt' if stride==4 else 'full.stdout.txt')).open('w',encoding='utf-8') as output:
            return subprocess.call([str(analysis_python()), str(self.project/'tools/analysis/pixel_camera_tuning_report.py'),
                '--trial', str(folder/'trial.json'), '--stride', str(stride)], stdout=output, stderr=subprocess.STDOUT,
                env=dict(os.environ,PYTHONUTF8='1'), creationflags=hidden_process())

    def _full(self, folder):
        with self.analysis_lock:
            self.gui.events.put(('pixel_full',str(folder),'解析中'))
            try:
                code = self.analyze(folder,1)
                path = folder/'full/summary.json'
                result = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
                status = '要目視確認' if code==0 and result.get('auto_stop_confirmed') else '検証不成立'
            except Exception:
                status = '解析失敗'
            self.gui.events.put(('pixel_full',str(folder),status))

    def completed(self, result):
        folder, trial, quick = result
        row = self.rows[str(folder)]
        values = list(self.table.item(row,'values'))
        if quick:
            self.summaries[str(folder)] = quick
            values[2:5] = [f"{quick[k]:.2f}" if quick.get(k) is not None else '—'
                           for k in ('endpoint_forward_mm','endpoint_right_mm')]+['速報で確認' if quick.get('auto_stop_confirmed') else '要確認']
        else:
            values[4] = '要確認'
        values[5] = '待機中' if trial['state']=='CAPTURED_REVIEW_REQUIRED' else trial['state']
        self.table.item(row,values=values)
        self.table.selection_set(row)
        self.status.set('ログ・映像で停止確認（速報）。置き直し・手離しを確認して次の1回へ進めます'
            if quick and quick.get('auto_stop_confirmed') else '機体の停止・位置を確認してください。自動再送はしません')
        self.select_result()

    def select_result(self):
        selected = self.table.selection()
        if not selected:
            return
        self.latest_folder = next((Path(path) for path,row in self.rows.items() if row==selected[0]),self.latest_folder)
        path = self.latest_folder/'quick/relative_trajectory.png'
        if path.exists():
            self.photo = tk.PhotoImage(file=str(path)).subsample(3)
            self.preview.configure(image=self.photo,text='')

    def show_saved(self, trial_path):
        folder = Path(trial_path).resolve().parent
        trial = json.loads(Path(trial_path).read_text(encoding='utf-8'))
        settings = json.loads(Path(trial['type8_settings_json']).read_text(encoding='utf-8'))
        quick_path, full_path = folder/'quick/summary.json', folder/'full/summary.json'
        quick = json.loads(quick_path.read_text(encoding='utf-8')) if quick_path.exists() else {}
        full = json.loads(full_path.read_text(encoding='utf-8')) if full_path.exists() else {}
        summary = quick or full
        self.mouse.set(trial['mouse'])
        self.height.set(trial['camera_height_mm'])
        self.gui._queue_turn_params(settings['raw_response'])
        coordinates = [f'{summary[key]:.2f}' if summary.get(key) is not None else '—'
                       for key in ('endpoint_forward_mm','endpoint_right_mm')]
        self.rows[str(folder)] = self.table.insert('',tk.END, values=('保存済み',settings['parameters']['lend'],*coordinates,
            '速報で確認' if summary.get('auto_stop_confirmed') else '要確認', '要目視確認' if full.get('auto_stop_confirmed') else '未検証'))
        self.table.selection_set(self.rows[str(folder)])
        self.select_result()
        self.status.set('保存済み結果を表示中。実機未接続・設定未送信。再開時は実機から設定を取得してください')

    def open_result(self):
        if self.latest_folder and self.latest_folder.exists():
            os.startfile(str(self.latest_folder))

    def handle(self, item):
        if item[0] == 'pixel_done':
            self._busy(False)
            _, callback, result, error = item
            if error:
                self.status.set('処理を停止しました: '+error+'。実行は再送しません')
            else:
                callback(result)
        elif item[0] == 'pixel_progress':
            name, fields = item[1:]
            labels = dict(recording_requested='停止後1秒で終了する録画を準備中…', camera_permit_confirmed='録画中を確認。走行用の一回許可を確認しました',
                camera_motion_start_confirmed='1回の走行を開始。CSVで停止を確認しています',
                recording_postroll='停止を200 ms確認。あと1秒以上撮影して終了します',
                recording_postroll_complete='停止後の余白を確認。録画終了・転送・照合を待っています',
                recording_stop_fallback='停止条件を確認できません。録画上限で終了します（実行再送なし）',
                quick_analysis='動画とCSVを保存済み。速報解析中…')
            if name in labels:
                self.status.set(labels[name])
            if name == 'settings_verified':
                self.status.set('実機設定を照合済み（'+('変更なし・設定再送を省略' if fields['write_count']==0 else '反映後に読み返し')+'）')
                self.gui.pending_suction_override = None
                self.gui._queue_turn_params(fields['settings']['raw_response'])
        elif item[0] == 'pixel_full':
            _, path, state = item
            if path in self.rows:
                row = self.rows[path]
                values = list(self.table.item(row,'values'))
                values[5] = state
                self.table.item(row,values=values)
            self.detail_status.set(Path(path).name+'：240fps全フレーム '+state+'。採用値は自動更新しません')

    def close(self):
        if self.session:
            threading.Thread(target=self.session.close,daemon=True).start()
