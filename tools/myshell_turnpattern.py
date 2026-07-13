"""GUI editor and serial sender for the myshell ``turnpattern`` command."""

import csv
import json
import queue
import re
import struct
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


MAX_PATTERN_SIZE = 20
TURN_TYPES = (
    "long_r90", "long_l90", "long_r180", "long_l180",
    "in_r45", "in_l45", "out_r45", "out_l45",
    "in_r135", "in_l135", "out_r135", "out_l135",
    "r_v90", "l_v90",
)
PRESET_SPEEDS = (300, 500, 700, 1000, 1200, 1400, 1500, 1600, 1800, 2000)
STRAIGHT = "直線"
DIAGONAL = "斜め"
TURN_STATES = {
    "long_r90": (STRAIGHT, STRAIGHT), "long_l90": (STRAIGHT, STRAIGHT),
    "long_r180": (STRAIGHT, STRAIGHT), "long_l180": (STRAIGHT, STRAIGHT),
    "in_r45": (STRAIGHT, DIAGONAL), "in_l45": (STRAIGHT, DIAGONAL),
    "in_r135": (STRAIGHT, DIAGONAL), "in_l135": (STRAIGHT, DIAGONAL),
    "out_r45": (DIAGONAL, STRAIGHT), "out_l45": (DIAGONAL, STRAIGHT),
    "out_r135": (DIAGONAL, STRAIGHT), "out_l135": (DIAGONAL, STRAIGHT),
    "r_v90": (DIAGONAL, DIAGONAL), "l_v90": (DIAGONAL, DIAGONAL),
}

LOG_DATA_NUM = 51
LOG_MAGIC = 0xA55A
LOG_MAGIC_END = 0xFFFF
LOG_FRAME_FORMAT = "<HH" + "H" * LOG_DATA_NUM
LOG_FRAME_SIZE = struct.calcsize(LOG_FRAME_FORMAT)
LOG_DIR = Path(__file__).resolve().parent / "logs"
PATTERN_DIR = Path(__file__).resolve().parent / "turnpatterns"


class TurnPatternGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("myshell turnpattern")
        self.geometry("980x720")
        self.ser = None
        self.serial_lock = threading.Lock()
        self.events = queue.Queue()
        self.pattern = []
        self.current_file = None

        self.port = tk.StringVar(value="COM8")
        self.baud = tk.IntVar(value=115200)
        self.char_delay = tk.DoubleVar(value=0.005)
        self.timeout = tk.DoubleVar(value=60.0)
        self.new_turn = tk.StringVar(value=TURN_TYPES[0])
        self.preset_speed = tk.IntVar(value=700)
        self.turn_speed = tk.DoubleVar(value=0.7)
        self.pre_accel = tk.DoubleVar(value=6.5)
        self.post_accel = tk.DoubleVar(value=6.5)
        self.suction_enable = tk.BooleanVar(value=False)
        self.suction_duty = tk.IntVar(value=650)
        self.status = tk.StringVar(value="パターンが空です")

        self._build()
        self._refresh_ports()
        self.after(50, self._drain_events)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        conn = ttk.LabelFrame(root, text="シリアル接続", padding=8)
        conn.pack(fill=tk.X)
        ttk.Label(conn, text="Port").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(conn, textvariable=self.port, width=11)
        self.port_box.pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="更新", command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Label(conn, text="Baud").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(conn, textvariable=self.baud, width=9).pack(side=tk.LEFT, padx=4)
        ttk.Label(conn, text="文字間隔 [s]").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(conn, textvariable=self.char_delay, width=7).pack(side=tk.LEFT, padx=4)
        self.connect_button = ttk.Button(conn, text="接続", command=self._toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=8)

        body = ttk.Frame(root)
        body.pack(fill=tk.BOTH, expand=True, pady=8)

        editor = ttk.LabelFrame(body, text="走行パターン（最大20ターン）", padding=8)
        editor.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        add_row = ttk.Frame(editor)
        add_row.pack(fill=tk.X)
        ttk.Combobox(add_row, textvariable=self.new_turn, values=TURN_TYPES,
                     state="readonly", width=18).pack(side=tk.LEFT)
        ttk.Button(add_row, text="追加", command=self._add_turn).pack(side=tk.LEFT, padx=5)
        ttk.Button(add_row, text="選択位置に挿入", command=self._insert_turn).pack(side=tk.LEFT)

        list_frame = ttk.Frame(editor)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=7)
        self.pattern_list = tk.Listbox(list_frame, height=15, font=("Consolas", 11), exportselection=False)
        self.pattern_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.pattern_list.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.pattern_list.configure(yscrollcommand=scrollbar.set)

        edit_buttons = ttk.Frame(editor)
        edit_buttons.pack(fill=tk.X)
        ttk.Button(edit_buttons, text="削除", command=self._delete_turn).pack(side=tk.LEFT)
        ttk.Button(edit_buttons, text="↑", width=4, command=lambda: self._move_turn(-1)).pack(side=tk.LEFT, padx=4)
        ttk.Button(edit_buttons, text="↓", width=4, command=lambda: self._move_turn(1)).pack(side=tk.LEFT)
        ttk.Button(edit_buttons, text="全消去", command=self._clear_pattern).pack(side=tk.RIGHT)
        ttk.Label(editor, textvariable=self.status, foreground="#aa2020").pack(anchor=tk.W, pady=(7, 0))

        settings = ttk.LabelFrame(body, text="走行設定", padding=10)
        settings.pack(side=tk.LEFT, fill=tk.Y, padx=(5, 0))
        self._setting_row(settings, 0, "パラメータプリセット", self.preset_speed,
                          values=PRESET_SPEEDS, callback=self._preset_changed)
        self._setting_row(settings, 1, "ターン速度 [m/s]", self.turn_speed)
        self._setting_row(settings, 2, "前走加速度 [m/s²]", self.pre_accel)
        self._setting_row(settings, 3, "後走減速度 [m/s²]", self.post_accel)
        ttk.Separator(settings).grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=10)
        ttk.Checkbutton(settings, text="吸引を使用", variable=self.suction_enable).grid(
            row=5, column=0, columnspan=2, sticky=tk.W
        )
        self._setting_row(settings, 6, "吸引値 (0～990)", self.suction_duty)
        ttk.Label(settings, text="前走・後走距離は先頭／末尾の\nターン種別から本体が決定します。",
                  foreground="#555555").grid(row=7, column=0, columnspan=2, sticky=tk.W, pady=(12, 0))

        files = ttk.Frame(root)
        files.pack(fill=tk.X, pady=(0, 7))
        ttk.Button(files, text="新規", command=self._new_file).pack(side=tk.LEFT)
        ttk.Button(files, text="JSONを開く", command=self._load_file).pack(side=tk.LEFT, padx=5)
        ttk.Button(files, text="保存", command=self._save_file).pack(side=tk.LEFT)
        ttk.Button(files, text="名前を付けて保存", command=lambda: self._save_file(save_as=True)).pack(side=tk.LEFT, padx=5)

        actions = ttk.Frame(root)
        actions.pack(fill=tk.X)
        ttk.Button(actions, text="Apply", command=lambda: self._apply(False)).pack(side=tk.LEFT)
        ttk.Button(actions, text="Apply & exe", command=lambda: self._apply(True)).pack(side=tk.LEFT, padx=5)
        ttk.Button(actions, text="本体設定を表示", command=self._show_device).pack(side=tk.LEFT)
        ttk.Button(actions, text="ログ受信 → CSV", command=self._receive_log).pack(side=tk.LEFT, padx=(15, 5))
        ttk.Button(actions, text="ログ初期化", command=self._initialize_log).pack(side=tk.LEFT)

        self.output = tk.Text(root, height=12, wrap=tk.WORD)
        self.output.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

    @staticmethod
    def _setting_row(parent, row, label, variable, values=None, callback=None):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        if values is None:
            widget = ttk.Entry(parent, textvariable=variable, width=12)
        else:
            widget = ttk.Combobox(parent, textvariable=variable, values=values, state="readonly", width=10)
            if callback:
                widget.bind("<<ComboboxSelected>>", callback)
        widget.grid(row=row, column=1, sticky=tk.E, padx=(8, 0), pady=4)

    def _preset_changed(self, _event=None):
        self.turn_speed.set(self.preset_speed.get() / 1000.0)

    def _selected_index(self):
        selected = self.pattern_list.curselection()
        return selected[0] if selected else None

    def _add_turn(self):
        if len(self.pattern) >= MAX_PATTERN_SIZE:
            messagebox.showwarning("上限", "走行パターンは最大20ターンです。")
            return
        self.pattern.append(self.new_turn.get())
        self._refresh_pattern(len(self.pattern) - 1)

    def _insert_turn(self):
        if len(self.pattern) >= MAX_PATTERN_SIZE:
            messagebox.showwarning("上限", "走行パターンは最大20ターンです。")
            return
        index = self._selected_index()
        if index is None:
            index = len(self.pattern)
        self.pattern.insert(index, self.new_turn.get())
        self._refresh_pattern(index)

    def _delete_turn(self):
        index = self._selected_index()
        if index is not None:
            del self.pattern[index]
            self._refresh_pattern(min(index, len(self.pattern) - 1))

    def _move_turn(self, delta):
        index = self._selected_index()
        if index is None or not 0 <= index + delta < len(self.pattern):
            return
        target = index + delta
        self.pattern[index], self.pattern[target] = self.pattern[target], self.pattern[index]
        self._refresh_pattern(target)

    def _clear_pattern(self):
        self.pattern.clear()
        self._refresh_pattern()

    def _refresh_pattern(self, selection=None):
        self.pattern_list.delete(0, tk.END)
        state = TURN_STATES[self.pattern[0]][0] if self.pattern else None
        invalid_at = None
        for index, turn in enumerate(self.pattern):
            input_state, output_state = TURN_STATES[turn]
            marker = " " if input_state == state else "!"
            if marker == "!" and invalid_at is None:
                invalid_at = index
            self.pattern_list.insert(tk.END, f"{marker} {index + 1:02d}  {turn:<12}  {input_state} → {output_state}")
            state = output_state
        if selection is not None and self.pattern:
            self.pattern_list.selection_set(selection)
            self.pattern_list.see(selection)
        if not self.pattern:
            self.status.set("パターンが空です")
        elif invalid_at is not None:
            self.status.set(f"{invalid_at + 1}番目の入出力が直前のターンと接続できません")
        else:
            start = TURN_STATES[self.pattern[0]][0]
            self.status.set(f"{len(self.pattern)}/{MAX_PATTERN_SIZE} ターン  |  {start}スタート → {state}終了")

    def _validated_settings(self):
        if not self.pattern:
            raise ValueError("ターンを1つ以上追加してください。")
        state = TURN_STATES[self.pattern[0]][0]
        for index, turn in enumerate(self.pattern):
            input_state, state_after = TURN_STATES[turn]
            if input_state != state:
                raise ValueError(f"{index + 1}番目の {turn} は直前のターンと接続できません。")
            state = state_after
        try:
            preset = self.preset_speed.get()
            speed = self.turn_speed.get()
            pre_accel = self.pre_accel.get()
            post_accel = self.post_accel.get()
            duty = self.suction_duty.get()
        except tk.TclError as exc:
            raise ValueError("走行設定には数値を入力してください。") from exc
        if preset not in PRESET_SPEEDS:
            raise ValueError("プリセット速度が不正です。")
        if speed <= 0 or pre_accel <= 0 or post_accel <= 0:
            raise ValueError("速度・加速度・減速度は0より大きくしてください。")
        if not 0 <= duty <= 990:
            raise ValueError("吸引値は0～990にしてください。")
        return preset, speed, pre_accel, post_accel, duty

    def _document(self):
        preset, speed, pre_accel, post_accel, duty = self._validated_settings()
        return {
            "format": "myshell-turnpattern",
            "version": 1,
            "pattern": list(self.pattern),
            "settings": {
                "preset_speed": preset,
                "turn_speed": speed,
                "pre_accel": pre_accel,
                "post_accel": post_accel,
                "suction_enable": self.suction_enable.get(),
                "suction_duty": duty,
            },
        }

    def _new_file(self):
        self.pattern.clear()
        self.current_file = None
        self._refresh_pattern()
        self.title("myshell turnpattern")

    def _save_file(self, save_as=False):
        try:
            document = self._document()
        except ValueError as exc:
            messagebox.showerror("保存できません", str(exc))
            return
        path = self.current_file
        if save_as or path is None:
            PATTERN_DIR.mkdir(exist_ok=True)
            selected = filedialog.asksaveasfilename(
                initialdir=PATTERN_DIR, defaultextension=".json",
                filetypes=[("Turn pattern", "*.json"), ("All files", "*.*")]
            )
            if not selected:
                return
            path = Path(selected)
        path.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.current_file = path
        self.title(f"myshell turnpattern - {path.name}")
        self._append(f"保存しました: {path}\n")

    def _load_file(self):
        PATTERN_DIR.mkdir(exist_ok=True)
        selected = filedialog.askopenfilename(
            initialdir=PATTERN_DIR, filetypes=[("Turn pattern", "*.json"), ("All files", "*.*")]
        )
        if not selected:
            return
        try:
            path = Path(selected)
            document = json.loads(path.read_text(encoding="utf-8"))
            if document.get("format") != "myshell-turnpattern" or document.get("version") != 1:
                raise ValueError("未対応のファイル形式です。")
            pattern = document["pattern"]
            if not isinstance(pattern, list) or not 1 <= len(pattern) <= MAX_PATTERN_SIZE:
                raise ValueError("パターン数は1～20である必要があります。")
            if any(turn not in TURN_TYPES for turn in pattern):
                raise ValueError("未知のターン種別が含まれています。")
            settings = document["settings"]
            self.pattern = pattern
            self.preset_speed.set(settings["preset_speed"])
            self.turn_speed.set(settings["turn_speed"])
            self.pre_accel.set(settings["pre_accel"])
            self.post_accel.set(settings["post_accel"])
            self.suction_enable.set(settings.get("suction_enable", False))
            self.suction_duty.set(settings.get("suction_duty", 650))
            self._validated_settings()
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            messagebox.showerror("読み込み失敗", str(exc))
            return
        self.current_file = path
        self.title(f"myshell turnpattern - {path.name}")
        self._refresh_pattern(0)
        self._append(f"読み込みました: {path}\n")

    def _refresh_ports(self):
        ports = [item.device for item in list_ports.comports()] if list_ports else []
        self.port_box["values"] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connect_button.configure(text="接続")
            return
        if serial is None:
            messagebox.showerror("pyserialがありません", "pip install pyserial を実行してください。")
            return
        try:
            self.ser = serial.Serial(self.port.get(), self.baud.get(), timeout=0.1)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.connect_button.configure(text="切断")
            self._append(f"接続: {self.port.get()} @ {self.baud.get()}\n")
        except Exception as exc:
            messagebox.showerror("接続失敗", str(exc))

    def _ensure_connected(self):
        if not self.ser or not self.ser.is_open:
            self._toggle_connection()
        return bool(self.ser and self.ser.is_open)

    def _write_command(self, command):
        payload = (command + "\r").encode("ascii")
        delay = max(0.0, self.char_delay.get())
        for value in payload:
            self.ser.write(bytes((value,)))
            self.ser.flush()
            if delay:
                time.sleep(delay)

    def _send_and_wait(self, command, success_prefix):
        self._append(f"\n> {command}\n")
        self.ser.reset_input_buffer()
        self._write_command(command)
        deadline = time.monotonic() + self.timeout.get()
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore").replace("\x00", "").strip()
            if line:
                self._append(line + "\n")
            if line.startswith("TURNPATTERN_ERROR"):
                raise RuntimeError(line)
            if line.startswith(success_prefix):
                return
        raise TimeoutError(f"応答待ちタイムアウト: {success_prefix}")

    def _apply(self, execute):
        try:
            preset, speed, pre_accel, post_accel, duty = self._validated_settings()
        except ValueError as exc:
            messagebox.showerror("設定エラー", str(exc))
            return
        if execute and not messagebox.askyesno(
                "走行開始", "機体を安全な場所に置きましたか？\nApply後、前センサをかざすと走行します。"):
            return
        if not self._ensure_connected():
            return
        config = (
            f"turnpattern config {preset} {speed:.7g} {pre_accel:.7g} {post_accel:.7g} "
            f"{int(self.suction_enable.get())} {duty}"
        )
        threading.Thread(target=self._apply_worker, args=(config, list(self.pattern), execute), daemon=True).start()

    def _apply_worker(self, config, pattern, execute):
        try:
            with self.serial_lock:
                self._send_and_wait("turnpattern clear", "TURNPATTERN_CLEAR_DONE")
                self._send_and_wait(config, "TURNPATTERN_CONFIG_DONE")
                for turn in pattern:
                    self._send_and_wait(f"turnpattern add {turn}", "TURNPATTERN_ADD_DONE")
                self._send_and_wait("turnpattern show", "TURNPATTERN_SHOW_DONE")
                self._append("Apply完了\n")
                if execute:
                    self._append("\n> turnpattern exe\n")
                    self.ser.reset_input_buffer()
                    self._write_command("turnpattern exe")
                    self._append("exeを送信しました。前センサ待ちに入り、以降は完了応答を待ちません。\n")
        except Exception as exc:
            self._append(f"ERROR: {exc}\n")

    def _show_device(self):
        if self._ensure_connected():
            threading.Thread(target=self._simple_worker,
                             args=("turnpattern show", "TURNPATTERN_SHOW_DONE"), daemon=True).start()

    def _initialize_log(self):
        if not messagebox.askyesno("ログ初期化", "本体上のログをすべて消去しますか？"):
            return
        if self._ensure_connected():
            threading.Thread(target=self._simple_worker, args=("log init", "LOG_INIT_DONE"), daemon=True).start()

    def _simple_worker(self, command, success_prefix):
        try:
            with self.serial_lock:
                self._append(f"\n> {command}\n")
                self.ser.reset_input_buffer()
                self._write_command(command)
                deadline = time.monotonic() + self.timeout.get()
                while time.monotonic() < deadline:
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    line = raw.decode(errors="ignore").replace("\x00", "").strip()
                    if line:
                        self._append(line + "\n")
                    if line.startswith(success_prefix):
                        return
                raise TimeoutError(f"応答待ちタイムアウト: {success_prefix}")
        except Exception as exc:
            self._append(f"ERROR: {exc}\n")

    def _receive_log(self):
        if self._ensure_connected():
            threading.Thread(target=self._log_worker, daemon=True).start()

    def _read_exact(self, size, deadline):
        data = bytearray()
        while len(data) < size and time.monotonic() < deadline:
            chunk = self.ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
        if len(data) != size:
            raise TimeoutError(f"バイナリログ受信タイムアウト: {len(data)}/{size} bytes")
        return bytes(data)

    @staticmethod
    def _half_to_float(value):
        return struct.unpack("<e", struct.pack("<H", value))[0]

    def _log_worker(self):
        try:
            LOG_DIR.mkdir(exist_ok=True)
            name = self.current_file.stem if self.current_file else "unsaved"
            safe_name = re.sub(r"[^A-Za-z0-9_-]+", "_", name).strip("_") or "pattern"
            output_path = LOG_DIR / f"{datetime.now():%Y%m%d_%H%M%S}_turnpattern_{safe_name}.csv"
            frame_count = 0
            drop_count = 0
            labels = None
            with self.serial_lock:
                self._append("\n> disp log_bin\n")
                self.ser.reset_input_buffer()
                self._write_command("disp log_bin")
                deadline = time.monotonic() + self.timeout.get()
                while time.monotonic() < deadline:
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    line = raw.decode(errors="ignore").replace("\x00", "").strip()
                    self._append(line + "\n")
                    if line.startswith("HEADER,"):
                        labels = line.split(",")[1:]
                    elif line == "BINARY":
                        break
                else:
                    raise TimeoutError("BINARYマーカーを受信できませんでした。")

                with output_path.open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.writer(handle)
                    if labels:
                        writer.writerow(labels)
                    while True:
                        raw = self._read_exact(LOG_FRAME_SIZE, time.monotonic() + self.timeout.get())
                        magic, index, *half_values = struct.unpack(LOG_FRAME_FORMAT, raw)
                        if magic == LOG_MAGIC_END:
                            break
                        if magic != LOG_MAGIC:
                            drop_count += 1
                            continue
                        writer.writerow([index] + [self._half_to_float(value) for value in half_values])
                        frame_count += 1
                        if frame_count % 50 == 0:
                            self._append(f"受信済み: {frame_count} frames\n")
            self._append(f"CSV保存完了: {output_path}\n受信: {frame_count} / drop: {drop_count}\n")
        except Exception as exc:
            self._append(f"ERROR: {exc}\n")

    def _append(self, text):
        self.events.put(text)

    def _drain_events(self):
        try:
            while True:
                text = self.events.get_nowait()
                self.output.insert(tk.END, text)
                self.output.see(tk.END)
        except queue.Empty:
            pass
        self.after(50, self._drain_events)

    def _close(self):
        if self.ser:
            self.ser.close()
        self.destroy()


if __name__ == "__main__":
    TurnPatternGui().mainloop()
