"""探索条件を選び、Pixel録画とBLEログの準備後に手かざしOKを表示する。"""
import argparse
import os
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import mouse_ble_explore as explore
from mouse_ble_explore_gui import ExploreApp
from explore_camera import ExploreCameraRun, analysis_python, default_maze_reference
from pixel_camera_profiles import load_camera_profile, save_camera_profile, validate_camera_settings


class ExploreCameraApp(ExploreApp):
    def __init__(self, root, **kwargs):
        self.camera_run = None
        self.camera_close_pending = False
        self.review_path = None
        super().__init__(root, **kwargs)
        original_factory = self.session_factory
        def factory(**values):
            original_emit = values['emit']
            def emit(kind, data):
                if self.camera_run:
                    self.camera_run.observe(kind, data)
                original_emit(kind, data)
            values['emit'] = emit
            return original_factory(**values)
        self.session_factory = factory
        self.period.set(5)
        self.limit.set(180)
        self.wall_ekf_mode.set('EKF制御（control）')
        self.camera_enabled = tk.BooleanVar(value=True)
        self.camera_text = tk.StringVar(value='探索／最短の条件を選び、録画付き開始を押してください')
        self.gesture_text = tk.StringVar(value='準備前')
        first = root.winfo_children()[0]
        self.camera_panel = ttk.Frame(root, padding=(14, 8))
        self.camera_panel.pack(fill='x', before=first)
        self.gesture_sign = tk.Label(self.camera_panel, textvariable=self.gesture_text,
                                    font=('Yu Gothic UI', 20, 'bold'), bg='#dbe3ea', fg='#263445', pady=12)
        self.gesture_sign.pack(fill='x')
        ttk.Label(self.camera_panel, textvariable=self.camera_text, wraplength=1100).pack(anchor='w', pady=4)
        self.camera_choice = ttk.Checkbutton(self.camera_panel, text='Pixel動画を同時撮影（走行制限225秒まで・手かざし待ち30秒）',
                                             variable=self.camera_enabled)
        self.camera_choice.pack(side='left')
        self.review_button = ttk.Button(self.camera_panel, text='動画とログを比較', command=self.open_review, state='disabled')
        self.review_button.pack(side='right')
        self.start_button.configure(text='録画付き開始 → 手かざし待ち')
        self.analysis_enabled = tk.BooleanVar(value=True)
        self.reference_path = tk.StringVar(value=str(default_maze_reference()))
        self.cell_mm = tk.StringVar(value='90')
        self.analysis_python_path = tk.StringVar(value=str(analysis_python()))
        self.marker_detection_path = tk.StringVar(value='')
        self.wall_map_path = tk.StringVar(value='')
        self.analysis_stride = tk.IntVar(value=1)
        analysis = ttk.LabelFrame(root, text='保存後の位置解析（機体・カメラへ指令は送りません）', padding=6)
        analysis.pack(fill='x', before=first)
        ttk.Checkbutton(analysis, text='前後マーカーを自動追跡（A: 前赤・後黄 / B: 前緑・後赤）', variable=self.analysis_enabled).grid(row=0,column=0,sticky='w')
        ttk.Label(analysis,text='1マス mm（空欄＝セル単位）').grid(row=0,column=1)
        ttk.Entry(analysis,textvariable=self.cell_mm,width=8).grid(row=0,column=2)
        ttk.Label(analysis,text='9×9盤面基準JSON').grid(row=1,column=0,sticky='w')
        ttk.Entry(analysis,textvariable=self.reference_path,width=75).grid(row=1,column=1,columnspan=2,sticky='ew')
        ttk.Button(analysis,text='選択',command=lambda:self.choose_analysis_file(self.reference_path)).grid(row=1,column=3)
        ttk.Label(analysis,text='OpenCV・NumPy入りPython').grid(row=2,column=0,sticky='w')
        ttk.Entry(analysis,textvariable=self.analysis_python_path,width=75).grid(row=2,column=1,columnspan=2,sticky='ew')
        ttk.Button(analysis,text='選択',command=lambda:self.choose_analysis_file(self.analysis_python_path)).grid(row=2,column=3)
        ttk.Button(analysis,text='保存済み試行を解析',command=self.analyze_saved).grid(row=0,column=3)
        preferences = load_camera_profile('maze_9x9')
        self.capture_ev = tk.StringVar(value=str(preferences.get('ev', -2.0)))
        self.capture_torch = tk.StringVar(value='ON' if preferences['torch'] else 'OFF')
        self.capture_zoom = tk.DoubleVar(value=preferences['zoom_ratio'])
        self.capture_focus = tk.BooleanVar(value=preferences['focus_lock'])
        ttk.Label(analysis,text='撮影EV（既存動画は変更しません）').grid(row=3,column=0,sticky='w')
        camera_controls = ttk.Frame(analysis)
        camera_controls.grid(row=3,column=1,columnspan=3,sticky='w')
        self.capture_ev_choice = ttk.Combobox(camera_controls,textvariable=self.capture_ev,
            values=('0','-0.5','-1.0','-1.5','-2.0','-2.5','+0.5'),width=9)
        self.capture_ev_choice.pack(side='left')
        ttk.Label(camera_controls,text=' ライト ').pack(side='left')
        self.capture_torch_choice = ttk.Combobox(camera_controls,textvariable=self.capture_torch,
            values=('OFF','ON'),state='readonly',width=8)
        self.capture_torch_choice.pack(side='left')
        ttk.Label(camera_controls,text=' ズーム倍率 ').pack(side='left')
        self.capture_zoom_entry = ttk.Entry(camera_controls,textvariable=self.capture_zoom,width=6)
        self.capture_zoom_entry.pack(side='left')
        self.capture_focus_choice = ttk.Checkbutton(camera_controls,text='AF固定',variable=self.capture_focus)
        self.capture_focus_choice.pack(side='left',padx=8)
        ttk.Button(analysis,text='色検出・壁配置の詳細',command=self.toggle_detection_options).grid(row=4,column=0,sticky='w')
        ttk.Label(analysis,text='解析間隔（高速走行は1＝全フレーム）').grid(row=4,column=1,sticky='e')
        ttk.Combobox(analysis,textvariable=self.analysis_stride,values=(1,2,4),state='readonly',width=6).grid(row=4,column=2,sticky='w')
        advanced = self.advanced_detection = ttk.Frame(analysis)
        advanced.grid(row=5,column=0,columnspan=4,sticky='ew')
        ttk.Label(advanced,text='機体・色検出JSON（任意）').grid(row=0,column=0,sticky='w')
        ttk.Entry(advanced,textvariable=self.marker_detection_path,width=65).grid(row=0,column=1,sticky='ew')
        ttk.Button(advanced,text='選択',command=lambda:self.choose_analysis_file(self.marker_detection_path)).grid(row=0,column=2)
        ttk.Label(advanced,text='確認済み壁配置JSON（空欄＝動画から検出）').grid(row=1,column=0,sticky='w')
        ttk.Entry(advanced,textvariable=self.wall_map_path,width=65).grid(row=1,column=1,sticky='ew')
        ttk.Button(advanced,text='選択',command=lambda:self.choose_analysis_file(self.wall_map_path)).grid(row=1,column=2)
        advanced.grid_remove()
        root.title('mouse_type8 — 探索・最短動画／位置解析・BLEログ')
        root.geometry('1180x980')

    def start(self):
        if not self.camera_enabled.get():
            return super().start()
        try:
            if self.run_mode.get() == '最短走行':
                options = explore.ShortestOptions(self.goal_x.get(), self.goal_y.get(), self.goal_size.get(),
                    self.selected_profile(), self.selected_maze(), self.limit.get(), self.selected_wall_ekf_mode())
            else:
                options = explore.ExploreOptions(self.goal_x.get(), self.goal_y.get(), self.goal_size.get(),
                    self.accelerated.get(), self.roundtrip.get(), self.limit.get(), self.selected_wall_ekf_mode())
            options.command()
            if self.camera_run and self.camera_run.active:
                raise ValueError('録画の準備・走行・保存が進行中です')
            config = self.analysis_config()
            settings = self.selected_camera_settings()
            save_camera_profile('maze_9x9', settings)
            self.camera_run = ExploreCameraRun(self.session, lambda k,d:self.events.put((k,d)), analysis_config=config,
                                              camera_settings=settings)
            self.camera_run.start(options)
            self.wall_map_path.set('')  # 壁配置の明示指定は今回だけ。次の迷路へ持ち越さない。
            self.camera_choice.configure(state='disabled')
            self.start_button.configure(state='disabled')
            self.stop_button.configure(state='normal')
            self.capture_ev_choice.configure(state='disabled')
            self.capture_torch_choice.configure(state='disabled')
            self.capture_zoom_entry.configure(state='disabled')
            self.capture_focus_choice.configure(state='disabled')
        except (ValueError, AttributeError, OSError, tk.TclError) as error:
            messagebox.showerror('録画付き走行', str(error), parent=self.root)

    def choose_analysis_file(self, variable):
        path = filedialog.askopenfilename(parent=self.root)
        if path:
            variable.set(path)

    def toggle_detection_options(self):
        if self.advanced_detection.winfo_manager():
            self.advanced_detection.grid_remove()
        else:
            self.advanced_detection.grid()

    def selected_camera_settings(self):
        return validate_camera_settings(dict(ev=float(self.capture_ev.get()),
            torch=self.capture_torch.get() == 'ON', zoom_ratio=self.capture_zoom.get(),
            focus_lock=self.capture_focus.get()))

    def analysis_config(self):
        if not self.analysis_enabled.get():
            return None
        cell = float(self.cell_mm.get()) if self.cell_mm.get().strip() else None
        if cell is not None and not 10 <= cell <= 300:
            raise ValueError('1マスの寸法は10～300 mmで指定してください')
        reference = Path(self.reference_path.get())
        python = Path(self.analysis_python_path.get())
        if not reference.is_file() or not python.is_file():
            raise ValueError('位置解析の盤面基準JSONとPythonを選択してください')
        stride = self.analysis_stride.get()
        if stride not in (1,2,4):
            raise ValueError('解析フレーム間隔は1・2・4を選択してください')
        config = dict(reference=str(reference.resolve()), python=str(python.resolve()), cell_mm=cell, stride=stride,
                      marker_colours_source='run_settings.firmware.mouse', height_correction=False)
        for key, variable in (('marker_detection',self.marker_detection_path), ('wall_map',self.wall_map_path)):
            if variable.get().strip():
                path = Path(variable.get())
                if not path.is_file():
                    raise ValueError('色検出・壁配置JSONの保存先を確認してください')
                config[key] = str(path.resolve())
        return config

    def analyze_saved(self):
        import threading
        from explore_camera import analyze_trial
        if self.camera_run and self.camera_run.active:
            messagebox.showinfo('位置解析', '走行と保存の終了後に解析してください', parent=self.root)
            return
        try:
            config = self.analysis_config()
            if config is None:
                raise ValueError('マーカー自動追跡を選択してください')
            path = filedialog.askopenfilename(parent=self.root, title='対応するtrial.jsonを選択', filetypes=[('試行JSON','*.json')])
            if not path:
                return
            self.camera_text.set('保存済み動画から位置・向きを解析中です。機体の電源は不要です')
            def work():
                try:
                    review = analyze_trial(Path(path), config)
                    self.events.put(('pose_done',str(review)))
                except Exception as error:
                    self.events.put(('camera_failure',str(error)))
            threading.Thread(target=work,daemon=True,name='Saved pose analysis').start()
            self.wall_map_path.set('')
        except (ValueError, tk.TclError) as error:
            messagebox.showerror('位置解析',str(error),parent=self.root)

    def update_run_mode(self, state=None):
        super().update_run_mode(state)
        if self.camera_run and self.camera_run.active:
            self.start_button.configure(state='disabled')
            self.stop_button.configure(state='normal')
            for widget in self.run_controls:
                widget.configure(state='disabled')
            self.wall_ekf_choice.configure(state='disabled')

    def handle_event(self, kind, data):
        if kind == 'camera_ready':
            self.gesture_text.set('OK — 手かざしで走行を開始できます' if data else '録画・ログ・実機状態を確認中')
            self.gesture_sign.configure(bg='#15803d' if data else '#dbe3ea', fg='white' if data else '#263445')
        elif kind == 'camera_status':
            self.camera_text.set(data)
        elif kind == 'camera_failure':
            self.camera_text.set('中止・要確認: ' + data)
            self._message(data)
        elif kind == 'camera_done':
            self.review_path = Path(data['review'])
            self.review_button.configure(state='normal')
            self.camera_text.set(('保存・動画検査完了。' if data.get('verified') else '保存完了・動画検査は要確認。')
                                 + '動画とログを比較できます。同期は比較画面で微調整できます。')
            self._message('試行資料: ' + data['trial'])
            if data.get('position_available'):
                self.camera_text.set('保存・位置解析完了。「動画とログを比較」で軌跡・姿勢・旋回別の評価を確認できます。同期・欠測の品質表示も確認してください')
            elif data.get('position_requested'):
                self.camera_text.set('動画・ログは保存済み。位置解析は要確認です。解析ログを確認し「保存済み試行を解析」から再実行できます')
        elif kind == 'pose_done':
            self.review_path = Path(data)
            self.review_button.configure(state='normal')
            self.camera_text.set('位置解析完了。「動画とログを比較」で軌跡・向き・旋回別の評価を確認できます')
        elif kind == 'camera_finished':
            self.gesture_text.set('終了 — 次の走行条件を選択できます')
            self.camera_choice.configure(state='normal')
            self.capture_ev_choice.configure(state='normal')
            self.capture_torch_choice.configure(state='readonly')
            self.capture_zoom_entry.configure(state='normal')
            self.capture_focus_choice.configure(state='normal')
            self.update_run_mode()
            if self.camera_close_pending:
                self.close()
        else:
            super().handle_event(kind, data)
            if kind == 'state' and data == 'waiting_sensor' and self.camera_run and self.camera_run.active:
                self.readiness.set('上の緑色の「OK — 手かざしで走行を開始できます」表示を待ってください。')

    def stop(self):
        if self.camera_run and self.camera_run.active:
            self.camera_run.cancel()
        else:
            super().stop()

    def disconnect(self):
        if self.camera_run and self.camera_run.active:
            self.camera_run.cancel()
        super().disconnect()

    def close(self):
        if self.camera_run and self.camera_run.active:
            self.camera_close_pending = True
            self.camera_run.cancel()
            self.camera_text.set('停止応答・録画保存を待って終了します')
            return
        super().close()

    def open_review(self):
        if self.review_path and self.review_path.exists():
            os.startfile(self.review_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--smoke-test', action='store_true')
    args = parser.parse_args()
    root = tk.Tk(); app = ExploreCameraApp(root)
    if args.smoke_test:
        root.withdraw(); root.update()
        assert str(app.start_button['state']) == 'disabled'
        assert app.period.get() == 5 and app.camera_enabled.get()
        root.destroy(); print('Camera explore GUI smoke test OK')
    else:
        root.mainloop()


if __name__ == '__main__':
    main()
