"""myshell_debugの撮影タブ。Tk変数はUIスレッドでだけ取得する。"""
from __future__ import annotations
from datetime import datetime
import json
import os
from pathlib import Path
import threading
import tkinter as tk
from tkinter import ttk, filedialog
import uuid

from pixel_camera_tuning import (LiveSession, TrialRequest, analyze_trial,
    capture_connected, check_log_environment, render_trial_log, apply_connected_settings, setting_differences)
from pixel_camera_turn_capture import write_json
from pixel_camera_archive import archive_trial, archive_for_trial, write_index
from pixel_camera_profiles import load_camera_profile, save_camera_profile


def gui_expected_settings(gui, mouse):
    search = gui.motion.get() == 'search_turn'
    motion = 'search_turn_'+gui.direction.get() if search else gui.turn_type.get()
    preset = gui.search_turn_preset_speed.get() if search else gui.turn_preset_speed.get()
    return dict(mouse=mouse, motion_name=motion, preset_speed_mm_s=preset,
        turn_count=gui.turn_count.get(), parameters=dict(velo=gui.values['distance'].get(),
        r_min=gui.values['acc'].get(), lstart=gui.values['max_velo'].get(), lend=gui.values['end_velo'].get(),
        degree=gui.values['degree'].get(), correction=gui.values['degree_correction'].get(), pre_accel=gui.pre_accel.get()),
        pid={name:gui.values[name].get() for name in ('sp_kp','sp_ki','sp_kd','om_kp','om_ki','om_kd')},
        feedforward={name:gui.values[name].get() for name in ('ff_sp_velo','ff_sp_accel','ff_sp_bias',
            'ff_om_velo','ff_om_accel','ff_om_decel','ff_om_bias','ff_om_jerk','ff_sp_turn_accel_mag','ff_sp_turn_velo_sq')},
        suction=dict(enabled=gui.suction_enable.get(), duty=gui.suction_duty.get()))


BLE_TARGETS = {
    'A': ('HY0020-MOUSE-A', 'C1:B5:60:87:91:73'),
    'B': ('HY0020-MOUSE-B', 'E1:7F:86:21:84:73'),
}
ANALYSIS_MODES = {'録画のみ（解析なし）': 'record_only', '位置だけ（盤面XY）': 'position_only',
    'AR正対補正のみ': 'rectify_only', '位置＋外観学習（試験）': 'position_learned', 'ログと比較': 'compare'}


def mouse_ble_target(mouse, address):
    if mouse not in BLE_TARGETS:
        raise ValueError('接続する機体A/Bを選択してください')
    name, default = BLE_TARGETS[mouse]
    address = address.strip().upper() or default
    for other, (_, known) in BLE_TARGETS.items():
        if other != mouse and address == known:
            raise ValueError(f'機体{mouse}に機体{other}のBLEアドレスが指定されています')
    return name, address


class PixelTuningPanel:
    def __init__(self, gui, notebook):
        self.gui, self.project = gui, Path(__file__).resolve().parents[1]
        self.session = None
        self.guided = False
        self.verified_settings = None
        self.verified_generation = None
        self.settings_folder = None
        self.workflow_phase = 'placement'
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
        self._saved_analysis_mode = saved.get('analysis_mode')
        self.analysis_mode = tk.StringVar(value=next((label for label, mode in ANALYSIS_MODES.items()
            if mode == self._saved_analysis_mode), 'ログと比較'))
        previous = read(base/'current_ui_tuning_session.json')
        last = Path(previous.get('session_folder', str(base)))
        reference = last/'reference_visible_spread.json'
        geometry = last/'body_geometry_detection.json'
        self.mouse = tk.StringVar(value=saved.get('mouse', previous.get('mouse', '')))
        self.connected_mouse = None
        self._selected_mouse = self.mouse.get()
        self.addresses = {mouse: target[1] for mouse, target in BLE_TARGETS.items()}
        for mouse, address in saved.get('addresses', {}).items():
            if mouse in BLE_TARGETS:
                try:
                    self.addresses[mouse] = mouse_ble_target(mouse, address)[1]
                except ValueError:
                    pass
        if self._selected_mouse in BLE_TARGETS and saved.get('address'):
            try:
                self.addresses[self._selected_mouse] = mouse_ble_target(self._selected_mouse, saved['address'])[1]
            except ValueError:
                pass  # 旧GUIで別機体のアドレスが残った設定は引き継がない。
        self.height = tk.DoubleVar(value=saved.get('height_mm', 800))
        self.seconds = tk.IntVar(value=20)
        self.record_video = tk.BooleanVar(value=saved.get('record_video', True))
        self.log_period = tk.IntVar(value=saved.get('log_period_ms', 1))
        self.address = tk.StringVar(value=self.addresses.get(self._selected_mouse, ''))
        self.device_name = tk.StringVar(value=BLE_TARGETS.get(self._selected_mouse, ('', ''))[0])
        self.connection_target = tk.StringVar()
        self.reference = tk.StringVar(value=saved.get('reference', str(reference if reference.exists() else base/'marker_reference.json')))
        self.geometry = tk.StringVar(value=saved.get('geometry', str(geometry if geometry.exists() else base/'body_geometry_A.json')))
        self.geometry_by_mouse = dict(saved.get('geometry_by_mouse', {}))
        if self._selected_mouse in BLE_TARGETS:
            self.geometry_by_mouse[self._selected_mouse] = self.geometry.get()
        camera = load_camera_profile('tuning')
        self.capture_ev = tk.StringVar(value=str(camera.get('ev', -2.5)))
        self.capture_torch = tk.BooleanVar(value=camera['torch'])
        self.capture_focus = tk.BooleanVar(value=camera['focus_lock'])
        self.capture_zoom = tk.DoubleVar(value=camera['zoom_ratio'])
        self.ready = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='未接続。BLE接続を準備し、Load defaultsで実機プリセットを取得してください')
        self.detail_status = tk.StringVar(value='本体ログは400件。1 msで約0.4秒、5 msで約2秒。録画ONでは停止確認後1秒以上を撮影します')
        self.tab = ttk.Frame(notebook, padding=6)
        notebook.add(self.tab, text='ターン調整・ログ・録画')
        conditions = ttk.Frame(notebook, padding=10)
        notebook.add(conditions, text='接続・撮影条件')
        row = ttk.Frame(self.tab)
        row.pack(fill=tk.X)
        ttk.Label(row, text='機体（自己申告）').pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.mouse, values=('A','B'), state='readonly', width=3).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(row, text='録画する（Pixel）', variable=self.record_video).pack(side=tk.LEFT, padx=8)
        ttk.Label(row, text='本体ログ周期[ms]').pack(side=tk.LEFT)
        ttk.Spinbox(row, textvariable=self.log_period, from_=1, to=100, width=5).pack(side=tk.LEFT, padx=4)
        ttk.Label(row, text='今回の高さ[mm]').pack(side=tk.LEFT, padx=(8,0))
        ttk.Entry(row, textvariable=self.height, width=7).pack(side=tk.LEFT, padx=4)
        ttk.Label(row, text='録画上限[秒]').pack(side=tk.LEFT, padx=(8,0))
        ttk.Combobox(row, textvariable=self.seconds, values=(20,30), state='readonly', width=4).pack(side=tk.LEFT, padx=4)
        ttk.Label(self.tab, textvariable=self.connection_target).pack(anchor=tk.W, pady=2)
        self.workflow_text = tk.StringVar(value='配置 → BLE接続 → 設定送信 → 録画・走行 → ログ → 自動映像解析 → 結果確認')
        ttk.Label(self.tab, textvariable=self.workflow_text, wraplength=1150).pack(anchor=tk.W, pady=3)
        ttk.Checkbutton(self.tab, text='配置・向き・停止・手離しを確認（映像解析時はArUco 2個以上の画角も確認）', variable=self.ready).pack(anchor=tk.W)
        buttons = ttk.Frame(self.tab)
        buttons.pack(fill=tk.X, pady=4)
        ttk.Button(buttons, text='① BLE接続', command=self.toggle_connection).pack(side=tk.LEFT)
        self.apply_button = ttk.Button(buttons, text='② 設定送信・照合', command=self.apply_settings)
        self.apply_button.pack(side=tk.LEFT, padx=4)
        self.run_button = ttk.Button(buttons, text='③ 録画・1回走行開始', command=self.run)
        self.run_button.pack(side=tk.LEFT, padx=4)
        self.next_button = ttk.Button(buttons, text='④ 結果確認済み・次の試行', command=self.next_trial)
        self.next_button.pack(side=tk.LEFT, padx=4)
        video_options = ttk.Frame(self.tab)
        video_options.pack(fill=tk.X, pady=2)
        ttk.Label(video_options, text='動画の処理').pack(side=tk.LEFT, padx=(0, 4))
        ttk.Combobox(video_options, textvariable=self.analysis_mode, values=tuple(ANALYSIS_MODES),
            state='readonly', width=20).pack(side=tk.LEFT)
        ttk.Button(video_options, text='映像解析設定', command=self.video_setup).pack(side=tk.LEFT, padx=8)
        result_buttons = ttk.Frame(self.tab); result_buttons.pack(fill=tk.X, pady=2)
        self.result_buttons = result_buttons
        ttk.Button(result_buttons, text='結果フォルダーを開く', command=self.open_result).pack(side=tk.LEFT, padx=6)
        ttk.Button(result_buttons, text='ログ画像を開く', command=self.open_log_image).pack(side=tk.LEFT, padx=6)
        ttk.Button(result_buttons, text='正対補正動画を開く', command=self.open_rectified_video).pack(side=tk.LEFT, padx=6)
        self.preview_kind = tk.StringVar(value='本体ログ')
        choice = ttk.Combobox(result_buttons, textvariable=self.preview_kind, values=('本体ログ','動画の軌跡','正対補正画像'), state='readonly', width=12)
        choice.pack(side=tk.LEFT, padx=6)
        choice.bind('<<ComboboxSelected>>', lambda _event: self.select_result())
        ttk.Label(self.tab, textvariable=self.status, wraplength=1150).pack(anchor=tk.W, pady=2)
        ttk.Label(self.tab, textvariable=self.detail_status, wraplength=1150).pack(anchor=tk.W, pady=2)
        content = ttk.Frame(self.tab)
        content.pack(fill=tk.BOTH, expand=True)
        self.table = ttk.Treeview(content, columns=('trial','lend','forward','right','stop','full'), show='headings', height=5)
        for name, label, width in (('trial','試行',100),('lend','Lend[mm]',80),('forward','位置1[mm]',85),
                                    ('right','位置2[mm]',85),('stop','停止確認',110),('full','詳細検証',120)):
            self.table.heading(name, text=label)
            self.table.column(name, width=width, stretch=False)
        self.table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.table.bind('<<TreeviewSelect>>', lambda _event: self.select_result())
        preview_frame = ttk.Frame(content)
        preview_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8)
        self.preview = tk.Canvas(preview_frame, width=480, height=260, highlightthickness=0)
        scrollbar = ttk.Scrollbar(preview_frame, orient=tk.VERTICAL, command=self.preview.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.preview.configure(yscrollcommand=scrollbar.set)
        self.preview.pack(fill=tk.BOTH, expand=True)
        self.preview.create_text(12, 12, anchor=tk.NW, text='走行後のログ画像をここに表示します')
        ttk.Label(conditions, text='機体別BLE広告名').grid(row=0, column=0, sticky=tk.W, pady=5)
        ttk.Entry(conditions, textvariable=self.device_name, width=30, state='readonly').grid(row=0, column=1, sticky=tk.W, padx=8)
        for index, (label, variable) in enumerate((('HY0020 BLEアドレス', self.address),
                ('盤面の登録参照JSON', self.reference), ('機体・色検出プロファイルJSON', self.geometry))):
            ttk.Label(conditions, text=label).grid(row=index+1, column=0, sticky=tk.W, pady=5)
            ttk.Entry(conditions, textvariable=variable, width=95).grid(row=index+1, column=1, sticky=tk.EW, padx=8)
            if index > 0:
                ttk.Button(conditions, text='選択', command=lambda var=variable: self.choose_file(var)).grid(row=index+1, column=2)
        conditions.columnconfigure(1, weight=1)
        camera_controls = ttk.LabelFrame(conditions, text='調整用の録画設定（9×9迷路とは別に保存）', padding=6)
        camera_controls.grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=6)
        ttk.Label(camera_controls, text='撮影EV').pack(side=tk.LEFT)
        ttk.Combobox(camera_controls, textvariable=self.capture_ev,
            values=('-1.5','-2.0','-2.5','-3.0','0'), width=8).pack(side=tk.LEFT, padx=4)
        ttk.Label(camera_controls, text='ズーム倍率').pack(side=tk.LEFT)
        ttk.Entry(camera_controls, textvariable=self.capture_zoom, width=6).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(camera_controls, text='ライト', variable=self.capture_torch).pack(side=tk.LEFT, padx=8)
        ttk.Checkbutton(camera_controls, text='AF固定', variable=self.capture_focus).pack(side=tk.LEFT)
        ttk.Label(conditions, text='BLE 8766／カメラ8765。映像解析時は参照・目印高さと色条件を撮影ごとにコピーします。\n'
            '画面のターン設定を照合してから撮影します。プリセットは撮影時に読み直しません。\n'
            '登録済みArUcoを各評価フレームで2個以上参照します。ズーム変更後もマーカーが画角に入ることを確認してください。\n'
            '速報は約60fpsの暫定結果、詳細は240fps全フレーム。自動の値採用・連続走行・実行再送はしません。',
            wraplength=1100).grid(row=5, column=0, columnspan=2, sticky=tk.W, pady=10)
        self.mouse.trace_add('write', self._mouse_changed)
        self.address.trace_add('write', lambda *_args: self._show_connection_target())
        self._show_connection_target()

    def _show_connection_target(self):
        self.connection_target.set(f'BLE接続先: {self.device_name.get()} / {self.address.get()}'
            if self.mouse.get() in BLE_TARGETS else '接続する機体A/Bを選択してください')

    def _mouse_changed(self, *_args):
        mouse = self.mouse.get()
        if self.connected_mouse and getattr(self.gui, 'ser', None) and self.gui.ser.is_open and mouse != self.connected_mouse:
            self.mouse.set(self.connected_mouse)
            self.status.set('接続中の機体は変更できません。Disconnectしてから切り替えてください')
            return
        if self._selected_mouse in BLE_TARGETS:
            self.addresses[self._selected_mouse] = self.address.get().strip().upper()
        if mouse != self._selected_mouse:
            self.ready.set(False)
            self.verified_settings = None
            if self._selected_mouse in BLE_TARGETS:
                self.geometry_by_mouse[self._selected_mouse] = self.geometry.get()
            self.geometry.set(self.geometry_by_mouse.get(mouse, str(self.preferences.parent/f'body_geometry_{mouse}.json')))
            for variable in getattr(self.gui, 'values', {}).values():
                variable.set('')
        self._selected_mouse = mouse
        self.device_name.set(BLE_TARGETS.get(mouse, ('', ''))[0])
        self.address.set(self.addresses.get(mouse, ''))
        self._show_connection_target()
        if self.guided:
            self.set_workflow('placement')

    def toggle_connection(self):
        if getattr(self.gui, 'ser', None) and self.gui.ser.is_open:
            self.gui._toggle()
            if not self.gui.ser or not self.gui.ser.is_open:
                self.verified_settings = None
                if self.guided:
                    self.set_workflow('placement')
        else:
            self.prepare()

    def choose_file(self, variable):
        path = filedialog.askopenfilename(parent=self.gui, filetypes=[('設定JSON', '*.json')])
        if path:
            variable.set(path)

    def video_setup(self):
        from turn_tuning_video_setup import open_video_setup
        open_video_setup(self)

    def set_workflow(self, phase):
        self.workflow_phase = phase
        labels = dict(placement='次は配置を確認し、① BLE接続へ進みます',
            settings='接続済み。実機値を読み込み、編集して② 設定送信・照合を押してください',
            ready='設定照合済み。配置・手離しのチェック後、③ で1回だけ開始します',
            running='録画・走行中。終了後はログ取得と映像解析を自動で進めます',
            logs='ログ画像を作成中。機体とHY0020の電源をOFFにできます',
            analysis='映像を自動解析中。機体とHY0020の電源はOFFにしてください',
            review='結果を確認し、④ 次の試行へ進んでください。自動で再走行しません')
        if phase == 'running' and ANALYSIS_MODES[self.analysis_mode.get()] == 'record_only':
            labels[phase] = '録画・走行中。終了後は動画・本体ログ・ログ画像を保存します（映像解析なし）'
        self.workflow_text.set(labels[phase])
        if self.guided and not self.gui._camera_busy:
            self.run_button.configure(state='normal' if phase == 'ready' else 'disabled')
            self.next_button.configure(state='normal' if phase == 'review' else 'disabled')

    def enable_workflow(self):
        self.guided = True
        if self._saved_analysis_mode is None:
            self.analysis_mode.set('位置だけ（盤面XY）')
        self.set_workflow('placement')

    def next_trial(self):
        if self.gui._camera_busy or self.gui.lock.locked():
            self.status.set('保存・解析が終わるまでお待ちください')
            return
        self.ready.set(False)
        self.verified_settings = None
        self.verified_generation = None
        self.set_workflow('placement')
        self.status.set('パラメータは保持しています。置き直し、接続、設定送信の順に次の試行を準備してください')

    def save_preferences(self):
        try:
            saved = json.loads(self.preferences.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            saved = {}
        try:
            height = self.height.get()
        except (tk.TclError, ValueError):
            height = saved.get('height_mm', 800)
        saved.update(mouse=self.mouse.get(), height_mm=height, address=self.address.get(),
            addresses=dict(self.addresses), reference=self.reference.get(), geometry=self.geometry.get(),
            geometry_by_mouse=dict(self.geometry_by_mouse), record_video=self.record_video.get(), log_period_ms=self.log_period.get())
        saved['analysis_mode'] = ANALYSIS_MODES[self.analysis_mode.get()]
        self.preferences.parent.mkdir(parents=True, exist_ok=True)
        write_json(self.preferences, saved)

    def selected_camera_settings(self):
        from pixel_camera_profiles import validate_camera_settings
        return validate_camera_settings(dict(ev=float(self.capture_ev.get()), torch=self.capture_torch.get(),
            focus_lock=self.capture_focus.get(), zoom_ratio=self.capture_zoom.get()))

    def _busy(self, value):
        self.gui._camera_busy = value
        if value:
            def disable(parent):
                if parent is self.result_buttons:
                    return  # 取得済みの画像・ファイルは自動解析中も確認できる。
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
        if self.gui._camera_busy or self.gui.lock.locked():
            self.status.set('通信処理の完了を待ってください')
            return
        if self.gui.ser and self.gui.ser.is_open:
            self.status.set('既存の接続をDisconnectしてから準備してください')
            return
        try:
            mouse = self.mouse.get()
            name, address = mouse_ble_target(mouse, self.address.get())
        except ValueError as error:
            self.status.set(str(error))
            return
        self.address.set(address)
        self.addresses[mouse] = address
        self.save_preferences()
        record_video = self.record_video.get() and not self.guided
        video_analysis = ANALYSIS_MODES[self.analysis_mode.get()] != 'record_only'
        first_machine_connection = self.connected_mouse != mouse
        folder = self.preferences.parent/('tuning_gui_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        session = LiveSession(self.project, folder)
        self.status.set('BLE接続とログ保存を準備しています…')
        def attached(result):
            self.session = result
            self.connected_mouse = mouse
            self.gui.ser = result.connection
            self.gui._connection_generation += 1
            self.gui._shell_ready = False
            self.gui.port.set('BLE')
            self.gui.connect_button.configure(text='Disconnect')
            self.verified_settings = None
            if self.guided:
                self.set_workflow('settings')
                load_defaults = first_machine_connection
                try:
                    self.gui.values['distance'].get()
                except tk.TclError:
                    load_defaults = True
                if load_defaults:
                    self.gui.after_idle(self.gui._load_defaults)
            self.status.set('接続準備完了。Load defaultsで実機プリセットを取得し、値を編集してください')
        def prepare_session():
            check_log_environment()
            if self.session:
                self.session.close()
            return session.prepare(address, device_name=name, record_video=record_video, video_analysis=video_analysis,
                progress=lambda message:self.gui.events.put(('pixel_progress', 'connection_preparing', dict(message=message))))
        self._launch(prepare_session, attached)

    def apply_settings(self):
        if self.gui._camera_busy or self.gui.lock.locked():
            self.status.set('通信処理の完了を待ってください')
            return
        try:
            if not self.session or self.gui.ser is not self.session.connection or not self.gui.ser.is_open:
                raise ValueError('先に① BLE接続を行ってください')
            if self.gui.motion.get() not in ('turn', 'search_turn'):
                raise ValueError('Motion=turn / search_turnを選択してください')
            expected = gui_expected_settings(self.gui, self.mouse.get())
            command, delay = self.gui._set_command(), self.gui.char_delay.get()
            video_analysis = ANALYSIS_MODES[self.analysis_mode.get()] != 'record_only'
            rectify_only = ANALYSIS_MODES[self.analysis_mode.get()] == 'rectify_only'
            if self.record_video.get() and rectify_only:
                from pixel_camera_reference import load_marker_reference
                load_marker_reference(self.reference.get())
            elif self.record_video.get() and video_analysis:
                from turn_tuning_video_setup import validate_video_inputs
                validate_video_inputs(self.mouse.get(), self.reference.get(), self.geometry.get(), self.height.get())
                if ANALYSIS_MODES[self.analysis_mode.get()] == 'position_learned':
                    from pixel_camera_detection import load_detection_geometry
                    load_detection_geometry(self.geometry.get(), 'auto')
            self.save_preferences()
        except Exception as error:
            self.status.set('設定送信前の確認: '+str(error)+'。必要なら「映像解析設定」を開いてください')
            return
        generation = self.gui._connection_generation
        folder = self.session.folder/('settings_'+datetime.now().strftime('%H%M%S')+'_'+uuid.uuid4().hex[:6])
        recording = self.record_video.get()
        self.verified_settings = None
        def operation():
            self.gui._assert_connection(generation)
            if recording:
                self.session.prepare_camera(lambda message: self.gui.events.put(('pixel_progress','connection_preparing',dict(message=message))),
                    video_analysis=video_analysis)
            return apply_connected_settings(self.session, expected, command, delay, folder,
                self.gui._parse_device_settings,
                lambda name, fields: self.gui.events.put(('pixel_progress', name, fields)))
        def applied(settings):
            self.verified_settings, self.verified_generation = settings, generation
            self.settings_folder = folder
            self.set_workflow('ready')
            self.status.set('設定を送信・読み返し照合しました。配置と手離しを確認し、③ で開始してください')
        self.status.set('設定送信・実機値の照合中。この操作では走行しません')
        self._launch(operation, applied)

    def run(self):
        if self.gui._camera_busy or self.gui.lock.locked():
            self.status.set('通信・撮影処理の完了を待ってください')
            return
        try:
            if self.gui.motion.get() not in ('turn', 'search_turn'):
                raise ValueError('撮影連動はMotion=turn / search_turnの単体1回に対応します')
            if not self.session or self.gui.ser is not self.session.connection:
                raise ValueError('先に「BLE接続を準備」を押してください')
            recording = self.record_video.get()
            analysis_mode = ANALYSIS_MODES[self.analysis_mode.get()]
            video_analysis = analysis_mode != 'record_only'
            rectify_only = analysis_mode == 'rectify_only'
            expected = gui_expected_settings(self.gui, self.mouse.get())
            if self.guided and (self.verified_settings is None or self.verified_generation != self.gui._connection_generation
                    or setting_differences(self.verified_settings, expected, quantized=True)):
                self.verified_settings = None
                self.set_workflow('settings')
                raise ValueError('画面の設定を② 設定送信・照合で送ってから開始してください')
            request = TrialRequest(expected, self.gui._set_command(),
                self.gui.char_delay.get(), self.height.get() if recording and video_analysis and not rectify_only else 800, Path(self.reference.get()), Path(self.geometry.get()),
                self.seconds.get(), self.ready.get(), self.selected_camera_settings() if recording else {},
                record_video=recording, log_period_ms=self.log_period.get(), video_analysis=video_analysis,rectify_only=rectify_only)
            request.validate()
            if self.guided and recording and video_analysis and not rectify_only:
                from turn_tuning_video_setup import validate_video_inputs
                validate_video_inputs(self.mouse.get(), self.reference.get(), self.geometry.get(), self.height.get())
                if analysis_mode == 'position_learned':
                    from pixel_camera_detection import load_detection_geometry
                    load_detection_geometry(self.geometry.get(), 'auto')
            if recording:
                save_camera_profile('tuning', request.camera_settings)
                self.save_preferences()
        except Exception as error:
            self.status.set(str(error))
            return
        self.ready.set(False)  # 一回分。例外・タイムアウト後も自動で再設定しない。
        folder = self.session.folder/('run_'+datetime.now().strftime('%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.latest_folder = folder
        self.rows[str(folder)] = self.table.insert('',tk.END, values=(folder.name,request.expected['parameters']['lend'],'—','—','準備中','未実施'))
        generation = self.gui._connection_generation
        settings_folder = self.settings_folder
        def progress(name, fields):
            self.gui.events.put(('pixel_progress', name, fields))
        def operation():
            self.gui._assert_connection(generation)
            try:
                if request.record_video:
                    self.session.prepare_camera(lambda message: progress('connection_preparing', dict(message=message)),
                        video_analysis=video_analysis)
                result = capture_connected(self.session, request, folder, self.gui._parse_device_settings, progress)
            finally:
                if self.guided:
                    self.session.close()
                    self.gui.events.put(('pixel_acquisition_finished',))
            quick = None
            if result.get('saved_log', {}).get('status') == 'saved':
                progress('log_plotting', {})
                try:
                    render_trial_log(folder)
                    progress('log_image_ready', dict(folder=str(folder)))
                except Exception as error:
                    result['log_image_error'] = str(error)
                    write_json(folder/'trial.json', result)
            if result['state'] == 'CAPTURED_REVIEW_REQUIRED':
                write_json(folder/'analysis_plan.json', dict(mode=analysis_mode))
                if not video_analysis:
                    result.update(json.loads((folder/'trial.json').read_text(encoding='utf-8')))
                    result.update(analysis_mode='record_only', video_analysis_requested=False,
                        video_analysis_status='SKIPPED_RECORD_ONLY', full_analysis_status='録画のみ（解析なし）')
                    write_json(folder/'trial.json', result)
                    quick = dict(analysis_mode='record_only')
                    if not self.guided:
                        self._archive_result(folder)
                else:
                    self.gui.events.put(('pixel_progress','quick_analysis',{}))
                    try:
                        self.analyze(folder,4)
                    except Exception as error:
                        result['quick_analysis_error'] = str(error)
                    path = folder/'quick/summary.json'
                    if path.exists():
                        quick = json.loads(path.read_text(encoding='utf-8'))
                    # UIを閉じても、すでに依頼した保存データの詳細検証は完了させる。
                    if self.guided:
                        result['full_analysis_status'] = self._full(folder, archive=False)
                    else:
                        threading.Thread(target=self._full, args=(folder,), daemon=False).start()
            if self.guided:
                write_json(folder/'workflow_result.json', dict(state=result['state'], settings_stage_folder=str(settings_folder),
                    analysis_mode=analysis_mode,
                    video_analysis_status=result.get('video_analysis_status'),
                    log_image_created=(folder/'log.png').is_file(),
                    full_analysis_status=result.get('full_analysis_status'),
                    next_trial_requires_operator=True, execution_resend_count=0))
                if result['state'] == 'CAPTURED_REVIEW_REQUIRED':
                    self._archive_result(folder)
            return folder, result, quick
        self.status.set('設定を照合し、'+('録画開始後に' if recording else '')+'1回だけ走行します')
        if self.guided:
            self.set_workflow('running')
        if not self._launch(operation, self.completed):
            self.table.delete(self.rows.pop(str(folder)))
            if self.guided:
                self.set_workflow('ready')

    def analyze(self, folder, stride):
        plan = folder/'analysis_plan.json'
        mode = json.loads(plan.read_text(encoding='utf-8')).get('mode') if plan.is_file() else 'compare'
        if mode == 'record_only':
            return 0
        if mode in ('position_only', 'position_learned', 'rectify_only'):
            return analyze_trial(folder, stride, mode=mode)
        return analyze_trial(folder, stride)

    def _full(self, folder, *, archive=True):
        with self.analysis_lock:
            plan = folder/'analysis_plan.json'
            if plan.is_file() and json.loads(plan.read_text(encoding='utf-8')).get('mode') == 'record_only':
                status = '録画のみ（解析なし）'
                self.gui.events.put(('pixel_full',str(folder),status))
                if archive:
                    PixelTuningPanel._archive_result(self, folder)
                return status
            self.gui.events.put(('pixel_full',str(folder),'解析中'))
            try:
                code = self.analyze(folder,1)
                path = folder/'full/summary.json'
                result = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
                if result.get('analysis_mode') == 'rectify_only':
                    status = ('正対補正完了' if result.get('valid_frames') == result.get('frames') else '正対補正完了（欠測あり）') \
                        if code == 0 and result.get('rectification_available') else '正対補正不成立'
                elif result.get('analysis_mode') == 'position_only':
                    status = ('位置推定完了' if result.get('valid_frames', -1) == result.get('frames', 0) else '位置推定完了（欠測あり）') \
                        if code == 0 and result.get('position_available') else '位置推定不成立'
                    if code == 0 and result.get('position_available') and result.get('estimated_frames'):
                        status = f"位置推定完了（学習推定{result['estimated_frames']}枚）"
                else:
                    status = '要目視確認' if code==0 and result.get('auto_stop_confirmed') else '検証不成立'
            except Exception:
                status = '解析失敗'
            self.gui.events.put(('pixel_full',str(folder),status))
            if archive:
                PixelTuningPanel._archive_result(self, folder)
            return status

    def _archive_result(self, folder):
        try:
            archive = archive_trial(folder/'trial.json')
            write_index(archive.parent)
            self.gui.events.put(('pixel_archive',str(folder),str(archive),None))
        except Exception as error:
            self.gui.events.put(('pixel_archive',str(folder),None,str(error)))

    def completed(self, result):
        folder, trial, quick = result
        full_summary = folder/'full/summary.json'
        if self.guided and full_summary.is_file():
            quick = json.loads(full_summary.read_text(encoding='utf-8'))
        row = self.rows[str(folder)]
        values = list(self.table.item(row,'values'))
        if quick:
            self.summaries[str(folder)] = quick
            position_only = quick.get('analysis_mode') in ('position_only','rectify_only')
            keys = ('endpoint_board_x_mm','endpoint_board_y_mm') if position_only else ('endpoint_forward_mm','endpoint_right_mm')
            values[2:5] = [f"{quick[k]:.2f}" if quick.get(k) is not None else '—'
                           for k in keys]+[('ログで確認' if trial.get('log_stop_evidence', {}).get('confirmed') else '未判定')
                               if position_only else ('解析で確認' if self.guided else '速報で確認')
                               if quick.get('auto_stop_confirmed') else '要確認']
        else:
            values[4] = '要確認'
        if not trial.get('record_video', True):
            values[4] = 'ログで確認' if trial.get('log_stop_evidence', {}).get('confirmed') else '要確認'
        record_only = trial.get('analysis_mode') == 'record_only'
        if record_only:
            values[4] = 'ログで確認' if trial.get('log_stop_evidence', {}).get('confirmed') else '要確認'
        values[5] = ('録画のみ（解析なし）' if record_only else '待機中' if trial['state']=='CAPTURED_REVIEW_REQUIRED' else
                     '録画なし' if trial['state']=='LOGGED_REVIEW_REQUIRED' else trial['state'])
        self.table.item(row,values=values)
        self.table.selection_set(row)
        self.status.set('ログ・映像で停止確認（速報）。置き直し・手離しを確認して次の1回へ進めます'
            if quick and quick.get('auto_stop_confirmed') else '機体の停止・位置を確認してください。自動再送はしません')
        self.select_result()
        if (folder/'log.png').exists() and trial['state'] in ('LOGGED_REVIEW_REQUIRED', 'CAPTURED_REVIEW_REQUIRED'):
            self.status.set(f"本体{trial.get('saved_log', {}).get('period_ms', '?')} msログ・画像を保存しました: {folder}")
        elif trial['state'] not in ('LOGGED_REVIEW_REQUIRED', 'CAPTURED_REVIEW_REQUIRED'):
            self.status.set('走行結果の確認が必要です: '+trial['state']+'。保存先: '+str(folder))
        if trial.get('error') or trial.get('log_image_error'):
            self.status.set('要確認: '+trial.get('error', trial.get('log_image_error', ''))+'。保存先: '+str(folder))
        if self.guided:
            self.set_workflow('review')
            if trial.get('full_analysis_status'):
                values = list(self.table.item(row, 'values'))
                values[5] = trial['full_analysis_status']
                self.table.item(row, values=values)
                scope = '盤面XY・前後目印の中点・高さ未補正。' if quick and quick.get('analysis_mode') == 'position_only' else ''
                if quick and quick.get('analysis_mode') == 'rectify_only':
                    scope = '盤面の正対補正のみ。機体位置の推定は行っていません。'
                cause = '原因: '+quick['error']+'。' if quick and quick.get('error') else ''
                self.detail_status.set('録画のみ：映像解析を省略しました。保存した動画・ログを確認して④へ進んでください'
                    if record_only else '映像の自動解析: '+trial['full_analysis_status']+'。'+cause+scope+'ログ・動画は保存済みです。確認して④へ進んでください')

    def select_result(self):
        selected = self.table.selection()
        if not selected:
            return
        self.latest_folder = next((Path(path) for path,row in self.rows.items() if row==selected[0]),self.latest_folder)
        if self.preview_kind.get() == '本体ログ':
            path = self.latest_folder/'log_preview.png'
        elif self.preview_kind.get() == '正対補正画像':
            candidates = [self.latest_folder/subdir/'rectified_preview.png' for subdir in ('full','quick')]
            path = next((candidate for candidate in candidates if candidate.exists()),candidates[0])
        else:
            candidates = [self.latest_folder/subdir/name for subdir in ('full', 'quick')
                for name in ('position_trajectory.png', 'relative_trajectory.png')]
            path = next((candidate for candidate in candidates if candidate.exists()), candidates[0])
        summary = self.summaries.get(str(self.latest_folder), {})
        position_only = summary.get('analysis_mode') == 'position_only'
        self.table.heading('forward', text='盤面X[mm]' if position_only else '前後[mm]')
        self.table.heading('right', text='盤面Y[mm]' if position_only else '右[mm]')
        if position_only:
            self.detail_status.set('盤面左上が原点、Xは右、Yは下。前後目印の中点・高さ未補正。観測率 '
                +f"{summary.get('observation_rate', 0):.1%}"
                +(f"、学習による推定{summary['estimated_frames']}枚（橙色・観測とは別）" if summary.get('estimated_frames') else '')
                +'。車軸の絶対位置・停止判定は行いません')
        if summary.get('analysis_mode') == 'record_only':
            self.detail_status.set('録画のみ（映像解析なし）。結果フォルダーから動画を確認できます')
        if summary.get('analysis_mode') == 'rectify_only':
            self.detail_status.set('盤面の正対補正のみ。AR登録率 '+f"{summary.get('registration_rate',0):.1%}"
                +'。黒いフレームは補正不成立です。機体位置は推定していません')
        self.preview.delete('all')
        if path.exists():
            self.photo = tk.PhotoImage(file=str(path))
            scale = max(1, (self.photo.width()+479)//480)
            if scale > 1:
                self.photo = self.photo.subsample(scale)
            self.preview.create_image(0, 0, anchor=tk.NW, image=self.photo)
            self.preview.configure(scrollregion=(0, 0, self.photo.width(), self.photo.height()))
        else:
            self.preview.create_text(12, 12, anchor=tk.NW,
                text='録画のみのため動画の軌跡はありません。結果フォルダーから動画を確認してください'
                    if summary.get('analysis_mode') == 'record_only' and self.preview_kind.get() == '動画の軌跡'
                    else 'この試行の画像はありません。CSV・trial.jsonを確認してください', width=440)

    def open_log_image(self):
        if self.latest_folder and (self.latest_folder/'log.png').exists():
            os.startfile(str(self.latest_folder/'log.png'))

    def open_rectified_video(self):
        if self.latest_folder:
            folders = (archive_for_trial(self.latest_folder/'trial.json'), self.latest_folder)
            for folder in folders:
                if folder is None:
                    continue
                for subdir in ('full','quick'):
                    path = folder/subdir/'rectified.mp4'
                    if path.is_file():
                        os.startfile(str(path))
                        return
        self.status.set('正対補正動画がありません。「AR正対補正のみ」を選んで録画してください')

    def show_saved(self, trial_path):
        folder = Path(trial_path).resolve().parent
        trial = json.loads(Path(trial_path).read_text(encoding='utf-8'))
        settings = json.loads(Path(trial['type8_settings_json']).read_text(encoding='utf-8'))
        quick_path, full_path = folder/'quick/summary.json', folder/'full/summary.json'
        quick = json.loads(quick_path.read_text(encoding='utf-8')) if quick_path.exists() else {}
        full = json.loads(full_path.read_text(encoding='utf-8')) if full_path.exists() else {}
        summary = full or quick
        if summary.get('analysis_mode') == 'rectify_only':
            self.analysis_mode.set('AR正対補正のみ')
        if trial.get('analysis_mode') == 'record_only':
            summary = dict(analysis_mode='record_only')
            self.analysis_mode.set('録画のみ（解析なし）')
        self.mouse.set(trial['mouse'])
        self.height.set(trial.get('camera_height_mm') or 800)
        self.record_video.set(trial.get('record_video', True))
        self.log_period.set(trial.get('saved_log_period_ms', 1))
        if trial['motion'] in ('search_turn_right', 'search_turn_left'):
            self.gui.motion.set('search_turn')
            self.gui.direction.set(trial['motion'].removeprefix('search_turn_'))
            self.gui.search_turn_preset_speed.set(settings['preset_speed_mm_s'])
        else:
            self.gui.motion.set('turn')
            self.gui.turn_type.set(trial['motion'])
        self.gui._update_motion_fields()
        self.gui._queue_turn_params(settings['raw_response'])
        position_only = summary.get('analysis_mode') in ('position_only','rectify_only')
        keys = ('endpoint_board_x_mm','endpoint_board_y_mm') if position_only else ('endpoint_forward_mm','endpoint_right_mm')
        coordinates = [f'{summary[key]:.2f}' if summary.get(key) is not None else '—' for key in keys]
        if hasattr(self, 'summaries'):
            self.summaries[str(folder)] = summary
        self.rows[str(folder)] = self.table.insert('',tk.END, values=('保存済み',settings['parameters']['lend'],*coordinates,
            'ログで確認' if trial.get('analysis_mode') == 'record_only' and trial.get('log_stop_evidence', {}).get('confirmed')
                else '未判定' if position_only else '速報で確認' if summary.get('auto_stop_confirmed') else '要確認',
            '録画のみ（解析なし）' if trial.get('analysis_mode') == 'record_only'
                else '正対補正完了' if full.get('rectification_available')
                else '位置推定完了' if full.get('position_available') else '要目視確認' if full.get('auto_stop_confirmed') else '未検証'))
        self.table.selection_set(self.rows[str(folder)])
        self.select_result()
        self.status.set('保存済み結果を表示中。実機未接続・設定未送信。再開時は実機から設定を取得してください')

    def open_result(self):
        if self.latest_folder and self.latest_folder.exists():
            folder = archive_for_trial(self.latest_folder/'trial.json') or self.latest_folder
            os.startfile(str(folder))

    def handle(self, item):
        if item[0] == 'pixel_done':
            self._busy(False)
            _, callback, result, error = item
            if error:
                self.status.set('処理を停止しました: '+error+'。実行は再送しません')
                if self.guided:
                    self.verified_settings = None
                    self.set_workflow('review' if self.workflow_phase in ('running', 'logs', 'analysis') else 'settings')
            else:
                callback(result)
        elif item[0] == 'pixel_acquisition_finished':
            self.gui.ser = None
            self.gui._connection_generation += 1
            self.gui._shell_ready = False
            self.gui.connect_button.configure(text='Connect')
            self.verified_settings = None
            self.set_workflow('logs')
        elif item[0] == 'pixel_progress':
            name, fields = item[1:]
            if name == 'connection_preparing':
                self.status.set(fields['message'])
            labels = dict(recording_requested='停止後1秒で終了する録画を準備中…', camera_permit_confirmed='録画中を確認。走行用の一回許可を確認しました',
                gui_permit_confirmed='配置後のボタン開始を受付。1回だけ走行します',
                gui_motion_start_confirmed='1回の走行を開始。停止と本体ログを確認しています',
                gui_motion_stop_confirmed='停止を確認。本体ログを取得しています',
                log_plotting='本体ログを保存済み。ログ画像を作成しています…',
                camera_motion_start_confirmed='1回の走行を開始。CSVで停止を確認しています',
                recording_postroll='停止を200 ms確認。あと1秒以上撮影して終了します',
                recording_postroll_complete='停止後の余白を確認。録画終了・転送・照合を待っています',
                recording_stop_fallback='停止条件を確認できません。録画上限で終了します（実行再送なし）',
                quick_analysis='動画とCSVを保存済み。速報解析中…')
            if name in labels:
                self.status.set(labels[name])
            if self.guided and name == 'quick_analysis':
                self.set_workflow('analysis')
            if name == 'log_image_ready':
                self.latest_folder = Path(fields['folder'])
                row = self.rows.get(fields['folder'])
                if row:
                    self.table.selection_set(row)
                    self.select_result()
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
            self.detail_status.set(Path(path).name+'：'+state if state == '録画のみ（解析なし）'
                else Path(path).name+'：240fps全フレーム '+state+'。採用値は自動更新しません')
        elif item[0] == 'pixel_archive':
            _, path, archive, error = item
            if error:
                self.detail_status.set(Path(path).name+'：一式保存に失敗しました。元資料は保持しています: '+error)
            else:
                self.detail_status.set(Path(path).name+'：動画・CSV・設定・結果の保存先 '+archive)

    def close(self):
        if self.session:
            threading.Thread(target=self.session.close,daemon=True).start()
