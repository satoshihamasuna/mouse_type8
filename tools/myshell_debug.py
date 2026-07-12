import csv
import queue
import re
import struct
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


LOG_DATA_NUM = 51
LOG_MAGIC = 0xA55A
LOG_MAGIC_END = 0xFFFF
LOG_FRAME_FORMAT = "<HH" + "H" * LOG_DATA_NUM
LOG_FRAME_SIZE = struct.calcsize(LOG_FRAME_FORMAT)
LOG_DIR = Path(__file__).resolve().parent / "logs"
TURN_PRESET_SPEEDS = (300, 500, 700, 1000, 1200, 1400, 1500, 1600, 1800, 2000)
TURN_PARAM_RE = re.compile(
    r"velo:(-?\d+) r_min:(-?\d+) Lstart:(-?\d+) Lend:(-?\d+) degree:(-?\d+) "
    r"sp:(-?\d+),(-?\d+),(-?\d+) om:(-?\d+),(-?\d+),(-?\d+) "
    r"suction:(\d+) duty:(\d+) preset:(\d+)"
)


DEFAULTS = {
    "straight": (360.0, 6.5, 0.7, 0.0, 0.0, 4.0, 0.05, 0.0, 0.1, 0.01, 0.0),
    "diagonal": (381.78, 6.5, 0.7, 0.0, 0.0, 12.0, 0.04, 0.0, 0.6, 0.01, 0.0),
}
TURN_DEFAULTS = {
    "long_r90": (0.70, -42.5, 29.97, 38.86, -90.0), "long_l90": (0.70, 42.5, 29.97, 38.86, 90.0),
    "long_r180": (0.70, -43.5, 19.28, 30.11, -180.0), "long_l180": (0.70, 43.5, 19.28, 30.11, 180.0),
    "in_r45": (0.70, -48.5, 13.12, 40.31, -45.0), "in_l45": (0.70, 48.5, 13.12, 40.31, 45.0),
    "out_r45": (0.70, -48.5, 31.81, 24.64, -45.0), "out_l45": (0.70, 48.5, 31.81, 24.64, 45.0),
    "in_r135": (0.70, -40.5, 13.29, 14.42, -135.0), "in_l135": (0.70, 40.5, 13.29, 14.42, 135.0),
    "out_r135": (0.70, -40.5, 5.62, 22.17, -135.0), "out_l135": (0.70, 40.5, 5.62, 22.17, 135.0),
    "r_v90": (0.70, -38.5, 9.18, 18.18, -90.0), "l_v90": (0.70, 38.5, 9.18, 18.18, 90.0),
}
FIELDS = (
    ("Distance / turn velo", "distance"),
    ("Acceleration / turn r_min", "acc"),
    ("Max velocity / turn Lstart", "max_velo"),
    ("End velocity / turn Lend", "end_velo"),
    ("Degree (turn only)", "degree"),
    ("SP Kp", "sp_kp"),
    ("SP Ki", "sp_ki"),
    ("SP Kd", "sp_kd"),
    ("OM Kp", "om_kp"),
    ("OM Ki", "om_ki"),
    ("OM Kd", "om_kd"),
)


class DebugGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("myshell debug parameter tuner")
        self.geometry("1050x700")
        self.ser = None
        self.lock = threading.Lock()
        self.events = queue.Queue()
        self.port = tk.StringVar(value="COM8")
        self.baud = tk.IntVar(value=115200)
        self.char_delay = tk.DoubleVar(value=0.005)
        self.timeout = tk.DoubleVar(value=60.0)
        self.motion = tk.StringVar(value="straight")
        self.turn_type = tk.StringVar(value="long_r90")
        self.direction = tk.StringVar(value="right")
        self.turn_preset_speed = tk.IntVar(value=700)
        self.search_turn_count = tk.IntVar(value=1)
        self.suction_enable = tk.BooleanVar(value=False)
        self.suction_duty = tk.IntVar(value=650)
        self.pending_suction_override = None
        self.last_motion_name = "unknown"
        self.values = {name: tk.DoubleVar() for _, name in FIELDS}
        self._build()
        self._load_defaults()
        self.after(50, self._drain)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)
        conn = ttk.Frame(root)
        conn.pack(fill=tk.X)
        ttk.Label(conn, text="Port").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(conn, textvariable=self.port, width=11)
        self.port_box.pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Refresh", command=self._refresh).pack(side=tk.LEFT)
        ttk.Label(conn, text="Baud").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(conn, textvariable=self.baud, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Label(conn, text="Char delay").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(conn, textvariable=self.char_delay, width=6).pack(side=tk.LEFT, padx=4)
        self.connect_button = ttk.Button(conn, text="Connect", command=self._toggle)
        self.connect_button.pack(side=tk.LEFT, padx=8)

        selectors = ttk.LabelFrame(root, text="動作選択", padding=8)
        selectors.pack(fill=tk.X, pady=(10, 5))
        ttk.Label(selectors, text="Motion").grid(row=0, column=0, sticky=tk.W)
        motion_box = ttk.Combobox(selectors, textvariable=self.motion,
                                  values=("straight", "diagonal", "turn", "pivot_turn", "search_turn"),
                                  state="readonly", width=16)
        motion_box.grid(row=0, column=1, sticky=tk.W, padx=5)
        motion_box.bind("<<ComboboxSelected>>", self._on_motion_changed)

        self.turn_selector = ttk.Frame(selectors)
        ttk.Label(self.turn_selector, text="Turn type").pack(side=tk.LEFT)
        turn_box = ttk.Combobox(self.turn_selector, textvariable=self.turn_type,
                                values=tuple(TURN_DEFAULTS), state="readonly", width=16)
        turn_box.pack(side=tk.LEFT, padx=5)
        turn_box.bind("<<ComboboxSelected>>", lambda _event: self._load_defaults())

        self.direction_selector = ttk.Frame(selectors)
        ttk.Label(self.direction_selector, text="Direction").pack(side=tk.LEFT)
        direction_box = ttk.Combobox(self.direction_selector, textvariable=self.direction,
                                     values=("right", "left"), state="readonly", width=10)
        direction_box.pack(side=tk.LEFT, padx=5)
        direction_box.bind("<<ComboboxSelected>>", lambda _event: self._load_defaults())

        self.preset_selector = ttk.Frame(selectors)
        ttk.Label(self.preset_selector, text="Preset speed").pack(side=tk.LEFT)
        preset_box = ttk.Combobox(self.preset_selector, textvariable=self.turn_preset_speed,
                                  values=TURN_PRESET_SPEEDS, state="readonly", width=10)
        preset_box.pack(side=tk.LEFT, padx=5)
        preset_box.bind("<<ComboboxSelected>>", lambda _event: self._load_defaults())

        self.search_count_selector = ttk.Frame(selectors)
        ttk.Label(self.search_count_selector, text="Turn count").pack(side=tk.LEFT)
        ttk.Spinbox(self.search_count_selector, textvariable=self.search_turn_count,
                    from_=1, to=100, width=6).pack(side=tk.LEFT, padx=5)

        parameter_area = ttk.Frame(root)
        parameter_area.pack(fill=tk.X, pady=5)
        motion_params = ttk.LabelFrame(parameter_area, text="動作固有パラメータ", padding=8)
        motion_params.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        pid_params = ttk.LabelFrame(parameter_area, text="PIDゲイン", padding=8)
        pid_params.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        suction_params = ttk.LabelFrame(parameter_area, text="吸引設定", padding=8)
        suction_params.pack(side=tk.LEFT, fill=tk.BOTH, padx=(5, 0))

        self.motion_param_widgets = {}
        for row, name in enumerate(("distance", "acc", "max_velo", "end_velo", "degree")):
            label = ttk.Label(motion_params, text=name)
            entry = ttk.Entry(motion_params, textvariable=self.values[name], width=14)
            label.grid(row=row, column=0, sticky=tk.W, pady=2)
            entry.grid(row=row, column=1, sticky=tk.EW, padx=5, pady=2)
            self.motion_param_widgets[name] = (label, entry)
        motion_params.columnconfigure(1, weight=1)

        for row, (label_text, name) in enumerate(FIELDS[5:]):
            ttk.Label(pid_params, text=label_text).grid(row=row, column=0, sticky=tk.W, pady=2)
            ttk.Entry(pid_params, textvariable=self.values[name], width=10).grid(
                row=row, column=1, sticky=tk.EW, padx=5, pady=2
            )
        pid_params.columnconfigure(1, weight=1)

        ttk.Checkbutton(suction_params, text="吸引を使用する", variable=self.suction_enable).pack(anchor=tk.W)
        ttk.Label(suction_params, text="吸引値 (0～990)").pack(anchor=tk.W, pady=(8, 0))
        ttk.Entry(suction_params, textvariable=self.suction_duty, width=10).pack(anchor=tk.W, pady=3)

        self._update_motion_fields()

        actions = ttk.Frame(root)
        actions.pack(fill=tk.X)
        ttk.Button(actions, text="Load defaults", command=self._load_defaults).pack(side=tk.LEFT)
        ttk.Button(actions, text="Show device values", command=self.show_values).pack(side=tk.LEFT, padx=5)
        ttk.Button(actions, text="Reset turn", command=self.reset_turn).pack(side=tk.LEFT, padx=5)
        ttk.Button(actions, text="Apply", command=self.apply_values).pack(side=tk.LEFT, padx=5)
        ttk.Button(actions, text="Apply and execute", command=self.apply_and_execute).pack(side=tk.LEFT, padx=5)
        ttk.Button(actions, text="Get log CSV", command=self.receive_log_csv).pack(side=tk.LEFT, padx=5)
        ttk.Button(actions, text="ログ初期化", command=self.initialize_log).pack(side=tk.LEFT, padx=5)

        self.output = tk.Text(root, height=13, wrap=tk.WORD)
        self.output.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self._refresh()

    def _on_motion_changed(self, _event=None):
        self._update_motion_fields()
        self._load_defaults()

    def _update_motion_fields(self):
        motion = self.motion.get()
        self.turn_selector.grid_remove()
        self.direction_selector.grid_remove()
        self.preset_selector.grid_remove()
        self.search_count_selector.grid_remove()
        if motion == "turn":
            self.turn_selector.grid(row=0, column=2, sticky=tk.W, padx=(15, 0))
            self.preset_selector.grid(row=0, column=3, sticky=tk.W, padx=(15, 0))
        elif motion in ("pivot_turn", "search_turn"):
            self.direction_selector.grid(row=0, column=2, sticky=tk.W, padx=(15, 0))
            if motion == "search_turn":
                self.search_count_selector.grid(row=0, column=3, sticky=tk.W, padx=(15, 0))

        schemas = {
            "straight": (("distance", "Distance [mm]"), ("acc", "Acceleration"),
                         ("max_velo", "Max velocity"), ("end_velo", "End velocity")),
            "diagonal": (("distance", "Distance [mm]"), ("acc", "Acceleration"),
                         ("max_velo", "Max velocity"), ("end_velo", "End velocity")),
            "turn": (("distance", "Velocity"), ("acc", "Radius r_min [mm]"),
                     ("max_velo", "Lstart [mm]"), ("end_velo", "Lend [mm]"),
                     ("degree", "Angle [deg]")),
            "search_turn": (("distance", "Velocity"), ("acc", "Radius r_min [mm]"),
                            ("max_velo", "Lstart [mm]"), ("end_velo", "Lend [mm]"),
                            ("degree", "Angle [deg]")),
            "pivot_turn": (("distance", "Angle [deg]"), ("acc", "Angular acceleration [rad/s²]"),
                           ("max_velo", "Angular velocity [rad/s]")),
        }
        visible = {name for name, _label in schemas[motion]}
        labels = dict(schemas[motion])
        for name, (label, entry) in self.motion_param_widgets.items():
            if name in visible:
                label.configure(text=labels[name])
                label.grid()
                entry.grid()
            else:
                label.grid_remove()
                entry.grid_remove()

    def _refresh(self):
        ports = [p.device for p in list_ports.comports()] if list_ports else []
        self.port_box["values"] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])

    def _toggle(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connect_button.configure(text="Connect")
            return
        if serial is None:
            messagebox.showerror("pyserial missing", "pip install pyserial を実行してください。")
            return
        try:
            self.ser = serial.Serial(self.port.get(), self.baud.get(), timeout=0.1)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.connect_button.configure(text="Disconnect")
            self._append(f"Connected: {self.port.get()} @ {self.baud.get()}\n")
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))

    def _load_defaults(self):
        if self.motion.get() == "turn":
            if self.ser and self.ser.is_open:
                self.pending_suction_override = (self.suction_enable.get(), self.suction_duty.get())
                self._start(f"debug turn {self.turn_type.get()} preset {self.turn_preset_speed.get()}")
                return
            defaults = TURN_DEFAULTS[self.turn_type.get()] + (2.0, 0.05, 0.0, 0.1, 0.01, 0.0)
        elif self.motion.get() == "search_turn":
            sign = -1.0 if self.direction.get() == "right" else 1.0
            defaults = (0.32, sign * 26.0, 9.46, 11.16, sign * 90.0, 2.0, 0.016, 0.0, 0.1, 0.005, 0.0)
        elif self.motion.get() == "pivot_turn":
            sign = -1.0 if self.direction.get() == "right" else 1.0
            defaults = (sign * 90.0, sign * 40.0 * 3.141592653589793,
                        sign * 4.0 * 3.141592653589793, 0.0, 0.0,
                        2.0, 0.016, 0.0, 0.1, 0.005, 0.0)
        else:
            defaults = DEFAULTS[self.motion.get()]
        for (_, name), value in zip(FIELDS, defaults):
            self.values[name].set(value)

    def _set_command(self):
        try:
            values = [self.values[name].get() for _, name in FIELDS]
        except tk.TclError as exc:
            raise ValueError("全パラメータに数値を入力してください。") from exc
        if self.motion.get() == "turn":
            if values[0] <= 0 or values[1] == 0 or values[2] < 0 or values[3] < 0 or values[4] == 0:
                raise ValueError("veloは正、r_minとdegreeは0以外、Lstart/Lendは0以上にしてください。")
            right_turn = "_r" in self.turn_type.get() or self.turn_type.get().startswith("r_")
            if (right_turn and (values[1] > 0 or values[4] > 0)) or (not right_turn and (values[1] < 0 or values[4] < 0)):
                raise ValueError("右ターンのr_min/degreeは負、左ターンは正にしてください。")
            return "debug turn {} set {} {} {}".format(
                self.turn_type.get(), " ".join(f"{v:.7g}" for v in values),
                int(self.suction_enable.get()), self._suction_duty_value()
            )
        if self.motion.get() == "search_turn":
            right = self.direction.get() == "right"
            if values[0] <= 0 or values[2] < 0 or values[3] < 0:
                raise ValueError("veloは正、Lstart/Lendは0以上にしてください。")
            if (right and (values[1] >= 0 or values[4] >= 0)) or (not right and (values[1] <= 0 or values[4] <= 0)):
                raise ValueError("右はr_min/degreeを負、左は正にしてください。")
            try:
                turn_count = self.search_turn_count.get()
            except tk.TclError as exc:
                raise ValueError("ターン回数には整数を入力してください。") from exc
            if not 1 <= turn_count <= 100:
                raise ValueError("ターン回数は1～100にしてください。")
            return "debug search_turn {} set {} {} {} {}".format(
                self.direction.get(), " ".join(f"{v:.7g}" for v in values),
                int(self.suction_enable.get()), self._suction_duty_value(), turn_count
            )
        if self.motion.get() == "pivot_turn":
            pivot_values = values[:3] + values[5:]
            right = self.direction.get() == "right"
            if (right and any(value >= 0 for value in pivot_values[:3])) or (not right and any(value <= 0 for value in pivot_values[:3])):
                raise ValueError("右はdegree/rad_acc/rad_veloを負、左は正にしてください。")
            return "debug pivot_turn {} set {} {} {}".format(
                self.direction.get(), " ".join(f"{v:.7g}" for v in pivot_values),
                int(self.suction_enable.get()), self._suction_duty_value()
            )
        if values[0] <= 0 or values[1] <= 0 or values[2] <= 0 or values[3] < 0 or values[3] > values[2]:
            raise ValueError("distance/acc/max は正、end velocity は 0 以上 max 以下にしてください。")
        straight_values = values[:4] + values[5:]
        return "debug {} set {} {} {}".format(
            self.motion.get(), " ".join(f"{v:.7g}" for v in straight_values),
            int(self.suction_enable.get()), self._suction_duty_value()
        )

    def _suction_duty_value(self):
        try:
            duty = self.suction_duty.get()
        except tk.TclError as exc:
            raise ValueError("吸引値には整数を入力してください。") from exc
        if not 0 <= duty <= 990:
            raise ValueError("吸引値は0～990にしてください。")
        return duty

    def show_values(self):
        if self.motion.get() == "turn":
            self._start(f"debug turn {self.turn_type.get()} show")
        elif self.motion.get() in ("pivot_turn", "search_turn"):
            self._start(f"debug {self.motion.get()} {self.direction.get()} show")
        else:
            self._start(f"debug {self.motion.get()} show")

    def reset_turn(self):
        if self.motion.get() not in ("turn", "pivot_turn", "search_turn"):
            messagebox.showinfo("Turn only", "ターン系Motionを選択してください。")
            return
        if self.motion.get() == "turn":
            self._start(f"debug turn {self.turn_type.get()} preset {self.turn_preset_speed.get()}")
        else:
            self._load_defaults()
            self._start(f"debug {self.motion.get()} {self.direction.get()} reset")

    def apply_values(self):
        try:
            self._start(self._set_command())
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))

    def apply_and_execute(self):
        if not messagebox.askyesno("Execute motion", "機体を安全な場所に置きましたか？\nApply後、前センサをかざすと走行します。"):
            return
        try:
            set_command = self._set_command()
            if self.motion.get() == "turn":
                exe_command = f"debug turn {self.turn_type.get()} exe"
                motion_name = self.turn_type.get()
            elif self.motion.get() in ("pivot_turn", "search_turn"):
                exe_command = f"debug {self.motion.get()} {self.direction.get()} exe"
                motion_name = f"{self.motion.get()}_{self.direction.get()}"
            else:
                exe_command = f"debug {self.motion.get()} exe"
                motion_name = self.motion.get()
            commands = (set_command, exe_command)
        except ValueError as exc:
            messagebox.showerror("Invalid parameter", str(exc))
            return
        self.last_motion_name = motion_name
        self._start(*commands)

    def receive_log_csv(self):
        if not self.ser or not self.ser.is_open:
            self._toggle()
        if self.ser and self.ser.is_open:
            threading.Thread(target=self._log_binary_worker, daemon=True).start()

    def initialize_log(self):
        if not messagebox.askyesno("ログ初期化", "マイコン上のログをすべて消去しますか？"):
            return
        self._start("log init")

    @staticmethod
    def _half_to_float(value):
        return struct.unpack("<e", struct.pack("<H", value))[0]

    def _write_command(self, command):
        payload = (command + "\r").encode("ascii")
        delay = max(0.0, self.char_delay.get())
        for value in payload:
            self.ser.write(bytes((value,)))
            self.ser.flush()
            if delay > 0:
                time.sleep(delay)

    def _read_exact(self, size, deadline):
        data = bytearray()
        while len(data) < size and time.monotonic() < deadline:
            chunk = self.ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
        if len(data) != size:
            raise TimeoutError(f"バイナリログ受信タイムアウト: {len(data)}/{size} bytes")
        return bytes(data)

    def _log_binary_worker(self):
        try:
            LOG_DIR.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            motion_name = re.sub(r"[^A-Za-z0-9_-]+", "_", self.last_motion_name).strip("_") or "unknown"
            output_path = LOG_DIR / f"{timestamp}_myshell_debug_log_{motion_name}.csv"
            frame_count = 0
            drop_count = 0
            labels = None

            with self.lock:
                self._append("\n> disp log_bin\n")
                self.ser.reset_input_buffer()
                self._write_command("disp log_bin")
                deadline = time.monotonic() + self.timeout.get()

                while time.monotonic() < deadline:
                    raw_line = self.ser.readline()
                    if not raw_line:
                        continue
                    line = raw_line.decode(errors="ignore").replace("\x00", "").strip()
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
                        frame_deadline = time.monotonic() + self.timeout.get()
                        raw = self._read_exact(LOG_FRAME_SIZE, frame_deadline)
                        magic, index, *half_values = struct.unpack(LOG_FRAME_FORMAT, raw)
                        if magic == LOG_MAGIC_END:
                            break
                        if magic != LOG_MAGIC:
                            drop_count += 1
                            self._append(f"Drop frame: magic=0x{magic:04x}\n")
                            continue
                        writer.writerow([index] + [self._half_to_float(value) for value in half_values])
                        frame_count += 1
                        if frame_count % 50 == 0:
                            self._append(f"受信済みログ: {frame_count} frames\n")

            self._append(f"CSV保存完了: {output_path}\n")
            self._append(f"受信: {frame_count} frames / drop: {drop_count}\n")
        except Exception as exc:
            self._append(f"ERROR: {exc}\n")

    def _start(self, *commands):
        if not self.ser or not self.ser.is_open:
            self._toggle()
        if self.ser and self.ser.is_open:
            threading.Thread(target=self._worker, args=(commands,), daemon=True).start()

    def _worker(self, commands):
        try:
            with self.lock:
                for command in commands:
                    self._append(f"\n> {command}\n")
                    self.ser.reset_input_buffer()
                    self._write_command(command)
                    if command.endswith(" exe"):
                        self._append("exeを送信しました。以降は完了応答を待ちません。Serialを取り外せます。\n")
                        continue
                    deadline = time.monotonic() + self.timeout.get()
                    idle = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        raw = self.ser.readline()
                        if not raw:
                            if time.monotonic() > idle:
                                break
                            continue
                        line = raw.decode(errors="ignore").replace("\x00", "").rstrip("\r\n")
                        self._append(line + "\n")
                        if line.startswith("DEBUG_TURN_PARAM_X1000"):
                            self._queue_turn_params(line)
                        if line.startswith("DEBUG_SEARCH_COUNT "):
                            try:
                                self.events.put(("search_count", int(line.split()[-1])))
                            except ValueError:
                                pass
                        idle = time.monotonic() + 1.0
                        if (line.startswith("DEBUG_RUN_DONE") or line.startswith("DEBUG_PARAM_SET_DONE")
                                or line.startswith("DEBUG_TURN_SET_DONE")
                                or line.startswith("DEBUG_PIVOT_SET_DONE")
                                or line.startswith("DEBUG_SEARCH_SET_DONE")
                                or line == "LOG_INIT_DONE"):
                            if " set " in command or command == "log init":
                                break
        except Exception as exc:
            self._append(f"ERROR: {exc}\n")

    def _append(self, text):
        self.events.put(text)

    def _queue_turn_params(self, line):
        match = TURN_PARAM_RE.search(line)
        if not match:
            return
        suction_enable = bool(int(match.group(12)))
        suction_duty = int(match.group(13))
        if self.pending_suction_override is not None:
            suction_enable, suction_duty = self.pending_suction_override
            self.pending_suction_override = None
        self.events.put(("turn_params", [int(value) / 1000.0 for value in match.groups()[:11]],
                         suction_enable, suction_duty, int(match.group(14))))

    def _drain(self):
        try:
            while True:
                item = self.events.get_nowait()
                if isinstance(item, tuple) and item[0] == "turn_params":
                    _, values, suction_enable, suction_duty, preset_speed = item
                    for (_, name), value in zip(FIELDS, values):
                        self.values[name].set(value)
                    self.suction_enable.set(suction_enable)
                    self.suction_duty.set(suction_duty)
                    if preset_speed in TURN_PRESET_SPEEDS:
                        self.turn_preset_speed.set(preset_speed)
                elif isinstance(item, tuple) and item[0] == "search_count":
                    self.search_turn_count.set(item[1])
                else:
                    self.output.insert(tk.END, item)
                    self.output.see(tk.END)
        except queue.Empty:
            pass
        self.after(50, self._drain)

    def _close(self):
        if self.ser:
            self.ser.close()
        self.destroy()


if __name__ == "__main__":
    DebugGui().mainloop()
