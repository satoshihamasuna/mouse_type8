"""Tk GUI for the firmware-side myshell Search diagnostics.

The serial protocol, parsers, validation, and CSV writers are shared with
``myshell_search.py``.  This module only owns the connection and presents the
results interactively.
"""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from types import SimpleNamespace

import myshell_search as search


TOOLS_DIR = Path(__file__).resolve().parent
DEFAULT_MAZE = TOOLS_DIR / "maze_data" / "logs_maze.bin"
DEFAULT_OUTPUT_DIR = TOOLS_DIR / "logs"


class SearchGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("myshell Search verifier")
        self.geometry("1360x900")
        self.minsize(1050, 720)

        self.ser = None
        self.busy = False
        self.cancel_event = threading.Event()
        self.events: queue.Queue[tuple] = queue.Queue()
        self.payload: bytes | None = None
        self.current_result = None
        self.current_goal = (7, 7, 2)
        self.animation_positions: list[tuple[float, float]] = []
        self.animation_index = 0
        self.animation_after_id: str | None = None
        self.animation_playing = False

        self.port_var = tk.StringVar(value="COM8")
        self.baud_var = tk.IntVar(value=115200)
        self.connection_var = tk.StringVar(value="未接続")
        self.maze_var = tk.StringVar(value=str(DEFAULT_MAZE) if DEFAULT_MAZE.exists() else "")
        self.output_dir_var = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))

        self.start_x_var = tk.IntVar(value=0)
        self.start_y_var = tk.IntVar(value=0)
        self.start_dir_var = tk.StringVar(value="N")
        self.goal_x_var = tk.IntVar(value=7)
        self.goal_y_var = tk.IntVar(value=7)
        self.goal_size_var = tk.IntVar(value=2)
        self.return_x_var = tk.IntVar(value=0)
        self.return_y_var = tk.IntVar(value=0)
        self.return_size_var = tk.IntVar(value=1)

        self.out_motion_var = tk.StringVar(value="acc")
        self.out_priority_var = tk.StringVar(value="first")
        self.return_mode_var = tk.StringVar(value="full")
        self.return_motion_var = tk.StringVar(value="acc")
        self.return_priority_var = tk.StringVar(value="first")
        self.mask_var = tk.IntVar(value=1)
        self.max_steps_var = tk.IntVar(value=256)
        self.timeout_var = tk.DoubleVar(value=60.0)
        self.char_delay_var = tk.DoubleVar(value=0.02)
        self.binary_delay_var = tk.DoubleVar(value=0.002)

        self.status_var = tk.StringVar(value="COMへ接続してください")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_text_var = tk.StringVar(value="")
        self.animation_speed_var = tk.IntVar(value=10)
        self.animation_auto_var = tk.BooleanVar(value=True)
        self.animation_status_var = tk.StringVar(value="step -/-")
        self._build_ui()
        self._refresh_ports()
        self._load_initial_maze()
        self.after(50, self._drain_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.bind("<space>", self._on_animation_space)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=8)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(4, weight=1)

        connection = ttk.LabelFrame(root, text="Shell connection", padding=6)
        connection.grid(row=0, column=0, sticky="ew")
        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=(0, 4))
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=10)
        self.port_combo.grid(row=0, column=1)
        ttk.Button(connection, text="更新", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Label(connection, text="Baud").grid(row=0, column=3, padx=(12, 4))
        ttk.Entry(connection, textvariable=self.baud_var, width=9).grid(row=0, column=4)
        self.connect_button = ttk.Button(connection, text="接続", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=(12, 6))
        ttk.Label(connection, textvariable=self.connection_var).grid(row=0, column=6, sticky="w")

        files = ttk.LabelFrame(root, text="Maze and output", padding=6)
        files.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        files.columnconfigure(1, weight=1)
        ttk.Label(files, text="Maze").grid(row=0, column=0, sticky="w")
        ttk.Entry(files, textvariable=self.maze_var).grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(files, text="参照", command=self._browse_maze).grid(row=0, column=2)
        ttk.Label(files, text="Output").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(files, textvariable=self.output_dir_var).grid(
            row=1, column=1, sticky="ew", padx=5, pady=(4, 0)
        )
        ttk.Button(files, text="参照", command=self._browse_output).grid(row=1, column=2, pady=(4, 0))

        config = ttk.LabelFrame(root, text="Search configuration", padding=6)
        config.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        self._position_fields(config, 0, "Start", self.start_x_var, self.start_y_var)
        ttk.Label(config, text="Dir").grid(row=0, column=5, padx=(4, 2))
        ttk.Combobox(
            config, textvariable=self.start_dir_var, values=("N", "E", "S", "W"),
            width=4, state="readonly",
        ).grid(row=0, column=6)
        self._goal_fields(config, 0, 7, "Goal", self.goal_x_var, self.goal_y_var, self.goal_size_var)
        self._goal_fields(
            config, 0, 14, "Return", self.return_x_var, self.return_y_var, self.return_size_var
        )

        ttk.Label(config, text="Out").grid(row=1, column=0, sticky="e", pady=(6, 0))
        ttk.Combobox(
            config, textvariable=self.out_motion_var, values=("plain", "acc"), width=7,
            state="readonly",
        ).grid(row=1, column=1, pady=(6, 0))
        ttk.Combobox(
            config, textvariable=self.out_priority_var, values=("first", "second"), width=8,
            state="readonly",
        ).grid(row=1, column=2, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(config, text="Return").grid(row=1, column=5, sticky="e", pady=(6, 0))
        ttk.Combobox(
            config, textvariable=self.return_mode_var, values=("goal", "full"), width=6,
            state="readonly",
        ).grid(row=1, column=6, pady=(6, 0))
        ttk.Combobox(
            config, textvariable=self.return_motion_var, values=("plain", "acc"), width=7,
            state="readonly",
        ).grid(row=1, column=7, pady=(6, 0))
        ttk.Combobox(
            config, textvariable=self.return_priority_var, values=("first", "second"), width=8,
            state="readonly",
        ).grid(row=1, column=8, columnspan=2, sticky="w", pady=(6, 0))

        advanced = ttk.Frame(config)
        advanced.grid(row=1, column=14, columnspan=7, sticky="e", pady=(6, 0))
        self._small_field(advanced, 0, "mask", self.mask_var, 4)
        self._small_field(advanced, 2, "max", self.max_steps_var, 6)
        self._small_field(advanced, 4, "timeout", self.timeout_var, 6)
        self._small_field(advanced, 6, "char", self.char_delay_var, 6)
        self._small_field(advanced, 8, "binary", self.binary_delay_var, 6)

        actions = ttk.Frame(root)
        actions.grid(row=3, column=0, sticky="ew", pady=6)
        self.action_buttons: list[ttk.Button] = []
        for text, command in (
            ("迷路を送信", self._upload),
            ("静的Search", self._run_static),
            ("往路再生", self._run_outward),
            ("往復再生", self._run_round_trip),
            ("全32シナリオ", self._run_matrix),
        ):
            button = ttk.Button(actions, text=text, command=command, state=tk.DISABLED)
            button.pack(side=tk.LEFT, padx=(0, 5))
            self.action_buttons.append(button)
        self.stop_button = ttk.Button(actions, text="停止", command=self._stop, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=(6, 0))
        self.progress = ttk.Progressbar(
            actions, variable=self.progress_var, maximum=32, length=220, mode="determinate"
        )
        self.progress.pack(side=tk.RIGHT, padx=(6, 0))
        ttk.Label(actions, textvariable=self.progress_text_var).pack(side=tk.RIGHT)

        self.notebook = ttk.Notebook(root)
        self.notebook.grid(row=4, column=0, sticky="nsew")
        log_tab = ttk.Frame(self.notebook)
        steps_tab = ttk.Frame(self.notebook)
        self.maze_tab = ttk.Frame(self.notebook)
        scenarios_tab = ttk.Frame(self.notebook)
        self.notebook.add(log_tab, text="Shell log")
        self.notebook.add(steps_tab, text="Steps")
        self.notebook.add(self.maze_tab, text="迷路 / 経路")
        self.notebook.add(scenarios_tab, text="Scenarios")

        self.log_text = tk.Text(log_tab, wrap="none", font=("Consolas", 9))
        log_y = ttk.Scrollbar(log_tab, orient=tk.VERTICAL, command=self.log_text.yview)
        log_x = ttk.Scrollbar(log_tab, orient=tk.HORIZONTAL, command=self.log_text.xview)
        self.log_text.configure(yscrollcommand=log_y.set, xscrollcommand=log_x.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_y.grid(row=0, column=1, sticky="ns")
        log_x.grid(row=1, column=0, sticky="ew")
        log_tab.rowconfigure(0, weight=1)
        log_tab.columnconfigure(0, weight=1)

        step_columns = ("index", "pos", "next", "map", "wall", "vwall", "sense", "truth", "vw")
        self.step_tree = ttk.Treeview(steps_tab, columns=step_columns, show="headings")
        widths = (55, 100, 100, 180, 150, 150, 65, 60, 60)
        for column, width in zip(step_columns, widths):
            self.step_tree.heading(column, text=column)
            self.step_tree.column(column, width=width, anchor=tk.CENTER)
        step_y = ttk.Scrollbar(steps_tab, orient=tk.VERTICAL, command=self.step_tree.yview)
        self.step_tree.configure(yscrollcommand=step_y.set)
        self.step_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        step_y.pack(side=tk.RIGHT, fill=tk.Y)

        playback = ttk.Frame(self.maze_tab, padding=(6, 5))
        playback.pack(fill=tk.X)
        self.play_button = ttk.Button(playback, text="▶ 再生", command=self._toggle_animation)
        self.play_button.pack(side=tk.LEFT)
        ttk.Button(playback, text="|◀ 先頭", command=self._animation_reset).pack(
            side=tk.LEFT, padx=(5, 0)
        )
        ttk.Button(playback, text="◀ 1歩", command=lambda: self._animation_step(-1)).pack(
            side=tk.LEFT, padx=(5, 0)
        )
        ttk.Button(playback, text="1歩 ▶", command=lambda: self._animation_step(1)).pack(
            side=tk.LEFT, padx=(5, 0)
        )
        ttk.Label(playback, text="速度").pack(side=tk.LEFT, padx=(14, 3))
        ttk.Combobox(
            playback,
            textvariable=self.animation_speed_var,
            values=(1, 2, 5, 10, 20, 50),
            width=5,
            state="readonly",
        ).pack(side=tk.LEFT)
        ttk.Label(playback, text="steps/s").pack(side=tk.LEFT, padx=(3, 0))
        ttk.Checkbutton(
            playback, text="結果受信後に自動再生", variable=self.animation_auto_var
        ).pack(side=tk.LEFT, padx=(14, 0))
        ttk.Label(playback, textvariable=self.animation_status_var).pack(side=tk.RIGHT)

        self.maze_canvas = tk.Canvas(self.maze_tab, background="white", highlightthickness=0)
        self.maze_canvas.pack(fill=tk.BOTH, expand=True)
        self.maze_canvas.bind("<Configure>", lambda _event: self._redraw())

        scenario_columns = ("index", "scenario", "out", "return", "verification")
        self.scenario_tree = ttk.Treeview(scenarios_tab, columns=scenario_columns, show="headings")
        for column, width in zip(scenario_columns, (60, 530, 140, 140, 110)):
            self.scenario_tree.heading(column, text=column)
            self.scenario_tree.column(column, width=width, anchor=tk.CENTER)
        scenario_y = ttk.Scrollbar(
            scenarios_tab, orient=tk.VERTICAL, command=self.scenario_tree.yview
        )
        self.scenario_tree.configure(yscrollcommand=scenario_y.set)
        self.scenario_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scenario_y.pack(side=tk.RIGHT, fill=tk.Y)

        status = ttk.Frame(root)
        status.grid(row=5, column=0, sticky="ew", pady=(5, 0))
        ttk.Label(status, textvariable=self.status_var).pack(side=tk.LEFT)

    @staticmethod
    def _position_fields(parent, row, label, x_var, y_var) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e")
        ttk.Label(parent, text="x").grid(row=row, column=1)
        ttk.Entry(parent, textvariable=x_var, width=4).grid(row=row, column=2)
        ttk.Label(parent, text="y").grid(row=row, column=3)
        ttk.Entry(parent, textvariable=y_var, width=4).grid(row=row, column=4)

    @staticmethod
    def _goal_fields(parent, row, column, label, x_var, y_var, size_var) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=column, sticky="e", padx=(10, 2))
        ttk.Label(parent, text="x").grid(row=row, column=column + 1)
        ttk.Entry(parent, textvariable=x_var, width=4).grid(row=row, column=column + 2)
        ttk.Label(parent, text="y").grid(row=row, column=column + 3)
        ttk.Entry(parent, textvariable=y_var, width=4).grid(row=row, column=column + 4)
        ttk.Label(parent, text="size").grid(row=row, column=column + 5)
        ttk.Entry(parent, textvariable=size_var, width=4).grid(row=row, column=column + 6)

    @staticmethod
    def _small_field(parent, column, label, variable, width) -> None:
        ttk.Label(parent, text=label).grid(row=0, column=column, padx=(5, 2))
        ttk.Entry(parent, textvariable=variable, width=width).grid(row=0, column=column + 1)

    def _refresh_ports(self) -> None:
        try:
            from serial.tools import list_ports

            ports = [port.device for port in list_ports.comports()]
        except ImportError:
            ports = []
        self.port_combo.configure(values=ports)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _load_initial_maze(self) -> None:
        if not DEFAULT_MAZE.exists():
            return
        try:
            self.payload, _start, _goal = search.load_maze_payload(DEFAULT_MAZE)
            self.after_idle(self._redraw)
        except Exception:
            self.payload = None

    def _toggle_connection(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        if self.busy:
            return
        try:
            import serial

            self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.2)
            time.sleep(0.2)
            search.prepare_shell(self.ser, float(self.char_delay_var.get()))
        except Exception as exc:
            if self.ser is not None:
                self.ser.close()
            self.ser = None
            messagebox.showerror("Connection error", str(exc))
            return
        self.connect_button.configure(text="切断")
        self.connection_var.set(f"接続中: {self.port_var.get()}")
        self._set_action_state(tk.NORMAL)
        self.status_var.set("接続しました。迷路を送信するか、診断を開始できます。")
        self._append_log(f"Connected {self.port_var.get()} @ {self.baud_var.get()}\n")

    def _disconnect(self) -> None:
        if self.busy:
            messagebox.showwarning("Busy", "実行中です。先に停止してください。")
            return
        if self.ser is not None:
            self.ser.close()
        self.ser = None
        self.connect_button.configure(text="接続")
        self.connection_var.set("未接続")
        self._set_action_state(tk.DISABLED)
        self.status_var.set("切断しました")

    def _browse_maze(self) -> None:
        path = filedialog.askopenfilename(
            title="Maze file",
            filetypes=(("Maze files", "*.bin *.txt"), ("All files", "*.*")),
        )
        if not path:
            return
        self.maze_var.set(path)
        try:
            payload, inferred_start, inferred_goal = search.load_maze_payload(Path(path))
            self.payload = payload
            if inferred_start is not None:
                self.start_x_var.set(inferred_start[0])
                self.start_y_var.set(inferred_start[1])
                self.start_dir_var.set(inferred_start[2])
            if inferred_goal is not None:
                self.goal_x_var.set(inferred_goal[0])
                self.goal_y_var.set(inferred_goal[1])
                self.goal_size_var.set(inferred_goal[2])
            self.status_var.set(f"迷路を読み込みました: {Path(path).name}")
            self._redraw()
        except Exception as exc:
            messagebox.showerror("Maze error", str(exc))

    def _browse_output(self) -> None:
        path = filedialog.askdirectory(title="Output directory")
        if path:
            self.output_dir_var.set(path)

    def _configuration(self):
        start = (int(self.start_x_var.get()), int(self.start_y_var.get()), self.start_dir_var.get())
        goal = (int(self.goal_x_var.get()), int(self.goal_y_var.get()), int(self.goal_size_var.get()))
        return_goal = (
            int(self.return_x_var.get()), int(self.return_y_var.get()), int(self.return_size_var.get())
        )
        if not (0 <= start[0] < search.WIDTH and 0 <= start[1] < search.HEIGHT):
            raise ValueError("Start is outside the maze")
        for name, value in (("Goal", goal), ("Return goal", return_goal)):
            x, y, size = value
            if size <= 0 or x < 0 or y < 0 or x + size > search.WIDTH or y + size > search.HEIGHT:
                raise ValueError(f"{name} is outside the maze")
        mask = int(self.mask_var.get())
        if mask not in (1, 3):
            raise ValueError("mask must be 1 or 3")
        max_steps = int(self.max_steps_var.get())
        if max_steps <= 0 or max_steps > search.MAZE_SIZE * 4:
            raise ValueError("max steps is out of range")
        return SimpleNamespace(
            port=self.port_var.get(), baud=int(self.baud_var.get()), start=start, goal=goal,
            return_goal=return_goal, mask=mask, max_steps=max_steps,
            timeout=float(self.timeout_var.get()), char_delay=float(self.char_delay_var.get()),
            binary_delay=float(self.binary_delay_var.get()), verbose=False,
            out_motion=self.out_motion_var.get(), out_priority=self.out_priority_var.get(),
            return_mode=self.return_mode_var.get(), return_motion=self.return_motion_var.get(),
            return_priority=self.return_priority_var.get(),
            output_dir=Path(self.output_dir_var.get()),
        )

    def _load_payload(self) -> bytes:
        path = Path(self.maze_var.get())
        if not path.is_file():
            raise ValueError("Maze file does not exist")
        payload, _start, _goal = search.load_maze_payload(path)
        self.payload = payload
        return payload

    def _prefix(self, name: str, output_dir: Path) -> Path:
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return output_dir / f"{stamp}_{name}"

    def _start_worker(self, title: str, target) -> None:
        if self.busy:
            return
        if self.ser is None or not self.ser.is_open:
            messagebox.showwarning("Not connected", "先にCOMへ接続してください。")
            return
        try:
            args = self._configuration()
            payload = self._load_payload()
        except Exception as exc:
            messagebox.showerror("Configuration error", str(exc))
            return
        self.busy = True
        self.cancel_event.clear()
        self._set_action_state(tk.DISABLED)
        self.stop_button.configure(state=tk.NORMAL)
        self.status_var.set(title)
        self.progress_var.set(0)
        self.progress_text_var.set("")

        def worker() -> None:
            try:
                target(args, payload)
            except InterruptedError:
                self.events.put(("status", "停止しました"))
            except Exception as exc:
                self.events.put(("error", str(exc)))
            finally:
                self.events.put(("done",))

        threading.Thread(target=worker, daemon=True).start()

    def _upload(self) -> None:
        self._start_worker("迷路を送信しています", self._upload_worker)

    def _upload_worker(self, args, payload: bytes) -> None:
        search.prepare_serial_truth(self.ser, args, payload)
        self.events.put(("append", f"Maze uploaded: {len(payload)} bytes\n"))
        self.events.put(("status", "迷路とゴールを送信しました"))

    def _run_static(self) -> None:
        self._start_worker("静的Searchを実行しています", self._static_worker)

    def _static_worker(self, args, payload: bytes) -> None:
        search.prepare_serial_truth(self.ser, args, payload)
        sx, sy, heading = args.start
        mode = args.return_mode
        priority = args.out_priority
        command = f"search run {sx} {sy} {heading} {mode} {priority} {args.mask} {args.max_steps}"
        self.events.put(("append", f"> {command}\n"))
        search.write_command(self.ser, command, args.char_delay)
        _terminal, lines = search.wait_for(
            self.ser,
            lambda line: line.startswith("SEARCH_DUMP_END ") or line.startswith("SEARCH_ERROR "),
            args.timeout,
            False,
        )
        result = search.parse_search_lines(lines)
        issues = search.validate_result(result, args.start, args.goal, args.mask)
        prefix = self._prefix("search_static", args.output_dir)
        search.save_result(prefix, lines, result)
        search.plot_result(prefix.with_suffix(".png"), result, payload, args.goal)
        self.events.put(("append", "\n".join(lines) + "\n"))
        self.events.put(("result", result, payload, args.goal))
        self.events.put(("status", self._result_status("Static", result, issues, prefix)))

    def _run_outward(self) -> None:
        self._start_worker("往路を逐次再生しています", self._outward_worker)

    def _outward_worker(self, args, payload: bytes) -> None:
        search.prepare_serial_truth(self.ser, args, payload)
        prefix = self._prefix("search_replay_out", args.output_dir)
        command, lines = search.run_replay_command(
            self.ser, args, "reset", args.out_motion, args.start,
            args.goal, "goal", args.out_priority,
        )
        result = search.parse_replay_lines(lines)
        issues = search.validate_replay_result(result, args.mask)
        search.save_replay_result(prefix, command, lines, result)
        self.events.put(("append", f"> {command}\n" + "\n".join(lines) + "\n"))
        self.events.put(("result", result, payload, args.goal))
        self.events.put(("status", self._result_status("Outward", result, issues, prefix)))

    def _run_round_trip(self) -> None:
        self._start_worker("往復を逐次再生しています", self._round_trip_worker)

    def _round_trip_worker(self, args, payload: bytes) -> None:
        search.prepare_serial_truth(self.ser, args, payload)
        prefix = self._prefix("search_roundtrip", args.output_dir)
        out_command, out_lines = search.run_replay_command(
            self.ser, args, "reset", args.out_motion, args.start,
            args.goal, "goal", args.out_priority,
        )
        outward = search.parse_replay_lines(out_lines)
        out_issues = search.validate_replay_result(outward, args.mask)
        search.save_replay_result(
            prefix.with_name(prefix.name + "_out"), out_command, out_lines, outward
        )
        self.events.put(("append", f"> {out_command}\n" + "\n".join(out_lines) + "\n"))
        if out_issues:
            self.events.put(("result", outward, payload, args.goal))
            self.events.put(("status", self._result_status("Outward", outward, out_issues, prefix)))
            return
        if self.cancel_event.is_set():
            raise InterruptedError
        return_start = (outward.final_x, outward.final_y, outward.final_heading)
        ret_command, ret_lines = search.run_replay_command(
            self.ser, args, "keep", args.return_motion, return_start,
            args.return_goal, args.return_mode, args.return_priority,
        )
        returned = search.parse_replay_lines(ret_lines)
        ret_issues = search.validate_replay_result(returned, args.mask)
        search.save_replay_result(
            prefix.with_name(prefix.name + "_return"), ret_command, ret_lines, returned
        )
        self.events.put(("append", f"> {ret_command}\n" + "\n".join(ret_lines) + "\n"))
        self.events.put(("result", returned, payload, args.return_goal))
        issues = [f"outward: {issue}" for issue in out_issues]
        issues.extend(f"return: {issue}" for issue in ret_issues)
        self.events.put(("status", self._result_status("Round trip", returned, issues, prefix)))

    def _run_matrix(self) -> None:
        if not messagebox.askyesno(
            "Run matrix", "32シナリオ／64レッグを実行します。約5分かかります。開始しますか？"
        ):
            return
        for item in self.scenario_tree.get_children():
            self.scenario_tree.delete(item)
        self._start_worker("全32シナリオを実行しています", self._matrix_worker)

    def _matrix_worker(self, args, payload: bytes) -> None:
        prefix = self._prefix("search_matrix", args.output_dir)

        def progress(index, total, name, outward, returned, issues) -> None:
            if self.cancel_event.is_set():
                raise InterruptedError
            self.events.put(("scenario", index, total, name, outward, returned, issues))

        issues = search.run_replay_matrix_on_serial(
            self.ser, args, payload, prefix, progress_callback=progress
        )
        status = f"Matrix {'OK' if not issues else 'FAILED'}: {prefix}_summary.csv"
        self.events.put(("status", status))

    @staticmethod
    def _result_status(label: str, result, issues: list[str], prefix: Path) -> str:
        verification = "OK" if not issues else "FAILED"
        return (
            f"{label}: result={result.result}, steps={result.reported_steps}, "
            f"verification={verification}, saved={prefix}"
        )

    def _stop(self) -> None:
        if self.busy:
            self.cancel_event.set()
            self.status_var.set("停止要求を送信しました。現在のレッグ終了後に停止します。")

    def _set_action_state(self, state) -> None:
        for button in self.action_buttons:
            button.configure(state=state)

    def _append_log(self, text: str) -> None:
        self.log_text.insert(tk.END, text)
        self.log_text.see(tk.END)

    def _drain_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                kind = event[0]
                if kind == "append":
                    self._append_log(event[1])
                elif kind == "status":
                    self.status_var.set(event[1])
                elif kind == "error":
                    self.status_var.set(f"Error: {event[1]}")
                    self._append_log(f"ERROR: {event[1]}\n")
                    messagebox.showerror("Search error", event[1])
                elif kind == "result":
                    self._show_result(event[1], event[2], event[3])
                elif kind == "scenario":
                    self._show_scenario(*event[1:])
                elif kind == "done":
                    self.busy = False
                    self.stop_button.configure(state=tk.DISABLED)
                    if self.ser is not None and self.ser.is_open:
                        self._set_action_state(tk.NORMAL)
        except queue.Empty:
            pass
        self.after(50, self._drain_events)

    def _show_scenario(self, index, total, name, outward, returned, issues) -> None:
        self.progress_var.set(index)
        self.progress_text_var.set(f"{index}/{total}")
        verification = "OK" if not issues else "FAILED"
        self.scenario_tree.insert(
            "", tk.END,
            values=(
                index, name, f"{outward.result}/{outward.reported_steps}",
                f"{returned.result}/{returned.reported_steps}", verification,
            ),
        )
        self._append_log(
            f"[{index:02d}/{total}] {name}: out={outward.result}/{outward.reported_steps}, "
            f"return={returned.result}/{returned.reported_steps}, {verification}\n"
        )
        self._show_result(returned, self.payload, self._configuration().return_goal)

    def _show_result(self, result, payload: bytes | None, goal) -> None:
        self._animation_pause()
        self.current_result = result
        self.payload = payload
        self.current_goal = goal
        self.animation_positions = search.result_positions(result)
        self.animation_index = 0
        for item in self.step_tree.get_children():
            self.step_tree.delete(item)
        for step in result.steps:
            sensed = int(getattr(step, "sensed", False)) if hasattr(step, "sensed") else "-"
            truth = getattr(step, "truth", "-")
            self.step_tree.insert(
                "", tk.END,
                values=(
                    step.index,
                    f"{step.x},{step.y},{step.heading}",
                    f"{step.next_x},{step.next_y},{step.next_heading}",
                    ",".join(map(str, step.map_values)),
                    ",".join(map(str, step.wall_values)),
                    ",".join(map(str, step.virtual_values)),
                    sensed,
                    truth,
                    step.virtual_edges,
                ),
            )
        self.notebook.select(self.maze_tab)
        self._redraw()
        self._update_animation_status()
        if self.animation_auto_var.get() and len(self.animation_positions) > 1:
            self._animation_play()

    def _toggle_animation(self) -> None:
        if self.animation_playing:
            self._animation_pause()
        else:
            self._animation_play()

    def _on_animation_space(self, _event) -> str:
        self._toggle_animation()
        return "break"

    def _animation_play(self) -> None:
        if len(self.animation_positions) <= 1:
            return
        if self.animation_index >= len(self.animation_positions) - 1:
            self.animation_index = 0
        self.animation_playing = True
        self.play_button.configure(text="⏸ 一時停止")
        self._schedule_animation_tick()

    def _animation_pause(self) -> None:
        self.animation_playing = False
        if self.animation_after_id is not None:
            self.after_cancel(self.animation_after_id)
            self.animation_after_id = None
        if hasattr(self, "play_button"):
            self.play_button.configure(text="▶ 再生")

    def _schedule_animation_tick(self) -> None:
        if not self.animation_playing:
            return
        try:
            speed = max(1, int(self.animation_speed_var.get()))
        except (tk.TclError, ValueError):
            speed = 10
        self.animation_after_id = self.after(max(20, int(1000 / speed)), self._animation_tick)

    def _animation_tick(self) -> None:
        self.animation_after_id = None
        if not self.animation_playing:
            return
        if self.animation_index < len(self.animation_positions) - 1:
            self.animation_index += 1
            self._redraw()
            self._update_animation_status()
        if self.animation_index >= len(self.animation_positions) - 1:
            self._animation_pause()
        else:
            self._schedule_animation_tick()

    def _animation_reset(self) -> None:
        self._animation_pause()
        self.animation_index = 0
        self._redraw()
        self._update_animation_status()

    def _animation_step(self, delta: int) -> None:
        self._animation_pause()
        if not self.animation_positions:
            return
        self.animation_index = min(
            len(self.animation_positions) - 1,
            max(0, self.animation_index + delta),
        )
        self._redraw()
        self._update_animation_status()

    def _update_animation_status(self) -> None:
        if not self.animation_positions:
            self.animation_status_var.set("step -/-")
            return
        x, y = self.animation_positions[self.animation_index]
        self.animation_status_var.set(
            f"step {self.animation_index}/{len(self.animation_positions) - 1}  "
            f"({int(x - 0.5)}, {int(y - 0.5)})"
        )

    def _redraw(self) -> None:
        canvas = self.maze_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 500)
        height = max(canvas.winfo_height(), 500)
        side = min(width, height) - 30
        cell = side / search.WIDTH
        ox = (width - side) / 2
        oy = (height - side) / 2

        def point(x: float, y: float) -> tuple[float, float]:
            return ox + x * cell, oy + side - y * cell

        canvas.create_rectangle(ox, oy, ox + side, oy + side, outline="#777")
        gx, gy, goal_size = self.current_goal
        x1, y1 = point(gx, gy)
        x2, y2 = point(gx + goal_size, gy + goal_size)
        canvas.create_rectangle(
            x1,
            y2,
            x2,
            y1,
            fill="#ffe9a8",
            outline="#d39b00",
            width=2,
        )
        if self.payload is not None and len(self.payload) == search.MAZE_SIZE:
            index = 0
            for y in range(search.HEIGHT - 1, -1, -1):
                for x in range(search.WIDTH):
                    value = self.payload[index]
                    index += 1
                    north = value & 3
                    east = (value >> 2) & 3
                    south = (value >> 4) & 3
                    west = (value >> 6) & 3
                    if north in (search.WALL, search.VWALL):
                        canvas.create_line(*point(x, y + 1), *point(x + 1, y + 1), fill="#333")
                    if east in (search.WALL, search.VWALL):
                        canvas.create_line(*point(x + 1, y), *point(x + 1, y + 1), fill="#333")
                    if y == 0 and south in (search.WALL, search.VWALL):
                        canvas.create_line(*point(x, y), *point(x + 1, y), fill="#333")
                    if x == 0 and west in (search.WALL, search.VWALL):
                        canvas.create_line(*point(x, y), *point(x, y + 1), fill="#333")

        result = self.current_result
        if result is not None:
            for x, y, direction in result.virtual_edges:
                if direction == "N":
                    canvas.create_line(
                        *point(x, y + 1), *point(x + 1, y + 1),
                        fill="#d62728", width=2, dash=(4, 2)
                    )
                else:
                    canvas.create_line(
                        *point(x + 1, y), *point(x + 1, y + 1),
                        fill="#d62728", width=2, dash=(4, 2)
                    )
            full_path = self.animation_positions
            visible_path = full_path[: self.animation_index + 1]
            if len(visible_path) > 1:
                coords = [coordinate for item in visible_path for coordinate in point(*item)]
                canvas.create_line(*coords, fill="#00a6d6", width=2)
            if full_path:
                sx, sy = point(*full_path[0])
                start_radius = max(2, cell * 0.18)
                canvas.create_rectangle(
                    sx - start_radius,
                    sy - start_radius,
                    sx + start_radius,
                    sy + start_radius,
                    fill="#19a974",
                    outline="",
                )
                mx, my = point(*full_path[self.animation_index])
                mouse_radius = max(3, cell * 0.30)
                canvas.create_oval(
                    mx - mouse_radius,
                    my - mouse_radius,
                    mx + mouse_radius,
                    my + mouse_radius,
                    fill="black",
                    outline="black",
                    tags=("mouse",),
                )
        else:
            try:
                mouse_x = int(self.start_x_var.get()) + 0.5
                mouse_y = int(self.start_y_var.get()) + 0.5
                mx, my = point(mouse_x, mouse_y)
                mouse_radius = max(3, cell * 0.30)
                canvas.create_oval(
                    mx - mouse_radius,
                    my - mouse_radius,
                    mx + mouse_radius,
                    my + mouse_radius,
                    fill="black",
                    outline="black",
                    tags=("mouse",),
                )
            except (tk.TclError, ValueError):
                pass

    def _on_close(self) -> None:
        if self.busy and not messagebox.askyesno("Exit", "実行中です。終了しますか？"):
            return
        self.cancel_event.set()
        self._animation_pause()
        if self.ser is not None:
            self.ser.close()
        self.destroy()


def main() -> None:
    SearchGui().mainloop()


if __name__ == "__main__":
    main()
