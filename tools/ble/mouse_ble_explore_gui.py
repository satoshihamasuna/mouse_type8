"""BLE exploration GUI: start after a hand gesture and record telemetry continuously."""
import argparse
import asyncio
from collections import deque
import math
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import mouse_ble_explore as explore
from mouse_ble_log import LOG_LABELS, MIN_PERIOD_MS


STATE_LABELS = {
    'disconnected': '未接続', 'connecting': 'BLEを接続中', 'checking': '接続を確認中',
    'idle': 'BLE接続中・ログ配信停止',
    'starting': '探索開始を要求中', 'waiting_sensor': '前センサに手をかざしてください',
    'running': '探索走行中', 'stopping': '探索停止を要求中',
}
RUN_STATES = ('starting', 'waiting_sensor', 'running', 'stopping')


class ExploreApp:
    def __init__(self, root, session_factory=explore.ExploreSession):
        self.root, self.session_factory = root, session_factory
        self.session = self.worker = None
        self.events = queue.Queue()
        self.closing = False
        self.close_requested_at = None
        self.snapshot = {}
        self.history = deque(maxlen=300)
        self.name = tk.StringVar(value='HY0020-MOUSE')
        self.address = tk.StringVar()
        self.period = tk.IntVar(value=20)
        self.output = tk.StringVar(value=str(Path(__file__).resolve().parents[1] / 'logs'))
        self.goal_x, self.goal_y, self.goal_size = tk.IntVar(value=7), tk.IntVar(value=7), tk.IntVar(value=2)
        self.limit = tk.IntVar(value=360)
        self.roundtrip, self.accelerated = tk.BooleanVar(value=True), tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='未接続')
        self.summary = tk.StringVar(value='受信 0 件  /  欠落 0 件  /  CRC 0 件')
        self.battery_text = tk.StringVar(value='電池 — V')
        self.battery_limit = None
        self.readiness = tk.StringVar(value='BLE接続後、電圧・探索コマンド対応を確認します。ログ配信は探索開始時に有効になります。')
        self.file_path = tk.StringVar(value='保存先: 接続すると新しい記録フォルダを作成します')
        self.graph_field = tk.StringVar(value='ego.velo')
        root.title('mouse_type8 — BLE探索・逐次ログ')
        root.geometry('1180x820')
        root.minsize(960, 720)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self._build()
        root.after(100, self.poll)

    def _build(self):
        panel = ttk.Frame(self.root, padding=14)
        panel.pack(fill='both', expand=True)
        header = ttk.Frame(panel)
        header.pack(fill='x')
        ttk.Label(header, text='BLE探索・逐次ログ', font=('Yu Gothic UI', 18, 'bold')).pack(side='left')
        ttk.Label(header, textvariable=self.status, font=('Yu Gothic UI', 12, 'bold')).pack(side='right')
        link = ttk.LabelFrame(panel, text='BLE接続', padding=10)
        link.pack(fill='x', pady=(10, 8))
        ttk.Label(link, text='デバイス名').grid(row=0, column=0, sticky='w')
        self.name_entry = ttk.Entry(link, textvariable=self.name, width=20)
        self.name_entry.grid(row=0, column=1, padx=(6, 14))
        ttk.Label(link, text='アドレス（任意）').grid(row=0, column=2)
        self.address_entry = ttk.Entry(link, textvariable=self.address, width=22)
        self.address_entry.grid(row=0, column=3, padx=(6, 14))
        ttk.Label(link, text='ログ周期 [ms]').grid(row=0, column=4)
        self.period_entry = ttk.Spinbox(link, from_=MIN_PERIOD_MS, to=1000, increment=1, textvariable=self.period, width=6)
        self.period_entry.grid(row=0, column=5, padx=6)
        self.connect_button = ttk.Button(link, text='BLE接続', command=self.connect)
        self.connect_button.grid(row=0, column=6, padx=6)
        self.disconnect_button = ttk.Button(link, text='切断', command=self.disconnect, state='disabled')
        self.disconnect_button.grid(row=0, column=7)
        ttk.Label(link, text='保存先').grid(row=1, column=0, sticky='w', pady=(8, 0))
        self.output_entry = ttk.Entry(link, textvariable=self.output)
        self.output_entry.grid(row=1, column=1, columnspan=6, sticky='ew', padx=6, pady=(8, 0))
        ttk.Button(link, text='選択', command=self.choose_output).grid(row=1, column=7, pady=(8, 0))
        link.columnconfigure(3, weight=1)

        run = ttk.LabelFrame(panel, text='実機の探索走行', padding=10)
        run.pack(fill='x', pady=(0, 8))
        ttk.Label(run, text='開始位置 (0, 0)・北向き     ゴール X / Y / サイズ').grid(row=0, column=0, sticky='w')
        self.run_controls = []
        for column, variable in enumerate((self.goal_x, self.goal_y, self.goal_size), 1):
            widget = ttk.Spinbox(run, from_=0 if column < 3 else 1, to=31, textvariable=variable, width=5)
            widget.grid(row=0, column=column, padx=4)
            self.run_controls.append(widget)
        for column, text, variable in ((4, '往復探索', self.roundtrip), (5, '既知区間を加速', self.accelerated)):
            widget = ttk.Checkbutton(run, text=text, variable=variable)
            widget.grid(row=0, column=column, padx=8)
            self.run_controls.append(widget)
        ttk.Label(run, text='制限 [秒]').grid(row=0, column=6)
        widget = ttk.Spinbox(run, from_=1, to=3600, textvariable=self.limit, width=6)
        widget.grid(row=0, column=7, padx=4)
        self.run_controls.append(widget)
        self.start_button = ttk.Button(run, text='探索開始 → 手かざし待ち', command=self.start, state='disabled')
        self.start_button.grid(row=1, column=0, columnspan=3, sticky='ew', pady=(10, 0))
        self.stop_button = ttk.Button(run, text='探索を停止', command=self.stop, state='disabled')
        self.stop_button.grid(row=1, column=3, columnspan=2, sticky='ew', padx=8, pady=(10, 0))
        ttk.Label(run, text='手かざし待ち・探索中だけログを記録します。').grid(row=1, column=5, columnspan=3, sticky='w', pady=(10, 0))
        ttk.Label(panel, textvariable=self.readiness, wraplength=1100).pack(fill='x', pady=(0, 5))
        status_line = ttk.Frame(panel)
        status_line.pack(fill='x')
        ttk.Label(status_line, textvariable=self.summary, font=('Yu Gothic UI', 11, 'bold')).pack(side='left')
        ttk.Label(status_line, textvariable=self.battery_text, font=('Yu Gothic UI', 11, 'bold')).pack(side='right')
        ttk.Label(panel, textvariable=self.file_path, wraplength=1100).pack(fill='x', pady=(3, 8))

        tabs = ttk.Notebook(panel)
        tabs.pack(fill='both', expand=True)
        live, messages = ttk.Frame(tabs, padding=8), ttk.Frame(tabs, padding=8)
        tabs.add(live, text='ライブ値・波形')
        tabs.add(messages, text='探索イベント・応答')
        chart = ttk.Frame(live)
        chart.pack(side='left', fill='both', expand=True, padx=(0, 12))
        picker = ttk.Frame(chart)
        picker.pack(fill='x')
        ttk.Label(picker, text='表示項目').pack(side='left')
        choice = ttk.Combobox(picker, textvariable=self.graph_field, values=LOG_LABELS, state='readonly', width=28)
        choice.pack(side='left', padx=8)
        choice.bind('<<ComboboxSelected>>', lambda event: self.history.clear())
        ttk.Label(chart, text='青: 選択値   緑: 対応する目標値（ある場合）').pack(anchor='w', pady=5)
        self.canvas = tk.Canvas(chart, background='#f7f9fc', highlightthickness=0, width=540, height=350)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda event: self.draw_graph())
        table = ttk.Frame(live)
        table.pack(side='right', fill='both', expand=True)
        self.values = ttk.Treeview(table, columns=('field', 'value'), show='headings', height=16)
        self.values.heading('field', text='ログ項目')
        self.values.heading('value', text='最新値')
        self.values.column('field', width=270, minwidth=150)
        self.values.column('value', width=115, minwidth=80, anchor='e')
        scroll = ttk.Scrollbar(table, command=self.values.yview)
        self.values.configure(yscrollcommand=scroll.set)
        self.values.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        for field in LOG_LABELS:
            self.values.insert('', 'end', iid=field, values=(field, '—'))
        self.messages = tk.Text(messages, wrap='word', height=15, state='disabled', font=('Consolas', 10))
        self.messages.pack(fill='both', expand=True)

    def choose_output(self):
        directory = filedialog.askdirectory(initialdir=self.output.get())
        if directory:
            self.output.set(directory)

    def connect(self):
        if self.worker and self.worker.is_alive():
            return
        try:
            session = self.session_factory(name=self.name.get().strip(), address=self.address.get().strip() or None,
                                           period_ms=self.period.get(), output_root=self.output.get(),
                                           emit=lambda kind, data: self.events.put((kind, data)))
        except (ValueError, tk.TclError) as error:
            messagebox.showerror('接続設定', str(error), parent=self.root)
            return
        self.session = session
        self.snapshot = {}
        self.history.clear()
        self.summary.set('受信 0 件  /  欠落 0 件  /  CRC 0 件')
        self.battery_text.set('電池 — V')
        self.battery_limit = None
        self.file_path.set('保存先: 接続後に新しい記録フォルダを作成します')
        self.readiness.set('BLE接続・電圧・探索コマンド対応を確認中です。')
        for field in LOG_LABELS:
            self.values.item(field, values=(field, '—'))
        self.draw_graph()
        self.connect_button.configure(state='disabled')
        self.status.set('BLEを接続中')

        def work():
            try:
                asyncio.run(session.run())
            except Exception as error:
                self.events.put(('error', str(error)))
                self.events.put(('state', 'disconnected'))
        self.worker = threading.Thread(target=work, daemon=True, name='BLE exploration')
        self.worker.start()

    def start(self):
        try:
            options = explore.ExploreOptions(self.goal_x.get(), self.goal_y.get(), self.goal_size.get(),
                                             self.accelerated.get(), self.roundtrip.get(), self.limit.get())
            options.command()
        except (ValueError, tk.TclError) as error:
            messagebox.showerror('探索設定', str(error), parent=self.root)
            return
        if self.session:
            self.start_button.configure(state='disabled')
            self.session.action('start', options)

    def stop(self):
        if self.session:
            self.session.action('stop')

    def disconnect(self):
        if self.session:
            self.session.action('disconnect')

    def close(self):
        if self.closing:
            self.root.destroy()
            return
        self.closing = True
        self.close_requested_at = time.monotonic()
        self.disconnect()
        if not self.worker or not self.worker.is_alive():
            self.root.destroy()

    def _message(self, text):
        self.messages.configure(state='normal')
        self.messages.insert('end', str(text) + '\n')
        if int(self.messages.index('end-1c').split('.')[0]) > 500:
            self.messages.delete('1.0', '100.0')
        self.messages.see('end')
        self.messages.configure(state='disabled')

    def handle_event(self, kind, data):
        if kind == 'state':
            self.status.set(STATE_LABELS.get(data, data))
            connected = data not in ('disconnected', 'connecting')
            busy = data in RUN_STATES
            self.connect_button.configure(state='normal' if data == 'disconnected' else 'disabled')
            self.disconnect_button.configure(state='normal' if connected else 'disabled')
            self.stop_button.configure(state='normal' if busy else 'disabled')
            self.start_button.configure(state='disabled')
            for widget in (self.name_entry, self.address_entry, self.period_entry, self.output_entry):
                widget.configure(state='disabled' if connected or data == 'connecting' else 'normal')
            for widget in self.run_controls:
                widget.configure(state='disabled' if busy else 'normal')
        elif kind == 'capability':
            self.goal_x.set(int(data['goal_x']))
            self.goal_y.set(int(data['goal_y']))
            self.goal_size.set(int(data['goal_size']))
            self._message('探索対応ファームウェア: 機体 ' + data['mouse'])
        elif kind == 'output':
            self.file_path.set('保存先: ' + data)
        elif kind == 'telemetry':
            new_sample = data['time_ms'] != self.snapshot.get('time_ms')
            self.snapshot = data
            self.summary.set('受信 {:,} 件  /  欠落 {:,} 件  /  CRC {:,} 件'.format(data['rows'], data['lost'], data['crc_errors']))
            self.start_button.configure(state='normal' if data['ready'] else 'disabled')
            if data['ready']:
                self.readiness.set('BLE通信・電圧・探索コマンド対応を確認済み。ログ配信は停止中です。探索開始時に配信を開始します。')
            elif data['state'] == 'starting':
                self.readiness.set('開始コマンドを準備し、ログ受信を確認中です。まだ走行を開始していません。')
            elif data['state'] in RUN_STATES:
                self.readiness.set('ログを保存しながら探索中です。停止ボタンで探索を中断できます。')
            else:
                self.readiness.set('接続中。探索コマンド応答と電圧の確認が必要です。開始できない場合は対応ファームウェアを確認してください。')
            for field, value in data['values'].items():
                self.values.item(field, values=(field, '{:.6g}'.format(value)))
            if 'Battery' in data['values']:
                suffix = '（下限 {:.2f} V）'.format(self.battery_limit) if self.battery_limit is not None else ''
                self.battery_text.set('電池 {:.2f} V {}'.format(data['values']['Battery'], suffix))
            field = self.graph_field.get()
            target = 'ideal.' + field[4:] if field.startswith('ego.') else ''
            if new_sample and field in data['values']:
                self.history.append((data['time_ms'] / 1000, data['values'][field], data['values'].get(target, float('nan'))))
            self.draw_graph()
        elif kind == 'phase':
            self._message('往路探索' if data == 'outward' else '復路探索')
        elif kind == 'result':
            self._message('探索終了: ' + str(data))
        elif kind == 'battery':
            self.battery_limit = int(data['limit_mv']) / 1000
            self.battery_text.set('電池 {:.2f} V（下限 {:.2f} V）'.format(int(data['voltage_mv']) / 1000, self.battery_limit))
            self._message(data)
        elif kind in ('error', 'run_error', 'text'):
            self._message(data)
            if kind in ('error', 'run_error'):
                self.readiness.set(str(data))

    def poll(self):
        for _ in range(500):
            try:
                kind, data = self.events.get_nowait()
            except queue.Empty:
                break
            self.handle_event(kind, data)
        if self.closing and (not self.worker or not self.worker.is_alive() or
                             time.monotonic() - self.close_requested_at > 5):
            self.root.destroy()
            return
        self.root.after(100, self.poll)

    def draw_graph(self):
        canvas = self.canvas
        canvas.delete('all')
        width, height = max(canvas.winfo_width(), 160), max(canvas.winfo_height(), 120)
        left, top, right, bottom = 65, 22, width - 16, height - 34
        canvas.create_rectangle(left, top, right, bottom, outline='#c5ceda')
        finite = [v for row in self.history for v in row[1:] if math.isfinite(v)]
        if not finite or len(self.history) < 2:
            canvas.create_text(width / 2, height / 2, text='受信すると波形を表示します', fill='#64748b')
            return
        low, high = min(finite), max(finite)
        pad = max((high - low) * 0.1, 0.01)
        low, high = low - pad, high + pad
        start, end = self.history[0][0], self.history[-1][0]
        for i in range(5):
            y = top + i * (bottom - top) / 4
            canvas.create_line(left, y, right, y, fill='#e2e8f0')
            canvas.create_text(left - 7, y, text='{:.3g}'.format(high - i * (high - low) / 4), anchor='e', fill='#475569')
        for series, color in ((1, '#2563eb'), (2, '#059669')):
            segment = []
            for row in self.history:
                if math.isfinite(row[series]):
                    segment.extend((left + (row[0] - start) / max(end - start, 0.001) * (right - left),
                                    bottom - (row[series] - low) / (high - low) * (bottom - top)))
                else:
                    if len(segment) >= 4:
                        canvas.create_line(*segment, fill=color, width=2)
                    segment = []
            if len(segment) >= 4:
                canvas.create_line(*segment, fill=color, width=2)
        canvas.create_text(left, bottom + 18, text='0 s', anchor='w', fill='#475569')
        canvas.create_text(right, bottom + 18, text='{:.1f} s'.format(end - start), anchor='e', fill='#475569')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--smoke-test', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if sys.version_info < (3, 10):
        parser.error('Python 3.10+ with Tk is required')
    root = tk.Tk()
    app = ExploreApp(root)
    if args.smoke_test:
        root.withdraw()
        root.update()
        assert str(app.start_button['state']) == 'disabled'
        root.destroy()
        print('GUI smoke test OK')
    else:
        root.mainloop()


if __name__ == '__main__':
    main()
