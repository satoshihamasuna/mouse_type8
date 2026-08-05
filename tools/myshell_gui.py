import csv
import numpy as np
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

from maze_io import MazeFileReader, save_maze_file


DEFAULT_BAUD = 115200
DEFAULT_WIDTH = 32
DEFAULT_HEIGHT = 32
DEFAULT_GOAL = (7, 7, 2)
MAZE_DATA_DIR = Path(__file__).resolve().parent / "maze_data"
LOG_DATA_NUM = 51
LOG_MAGIC = 0xA55A
LOG_MAGIC_END = 0xFFFF
FRAME_FORMAT = "<HH" + "H" * LOG_DATA_NUM
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)
DIJKSTRA_LINE_RE = re.compile(
    r"x:\s*(-?\d+),y:\s*(-?\d+),d:\s*(-?\d+)"
    r"(?:,mdir:\s*(-?\d+))?,time:\s*\d+->"
    r"(?:count->\s*\d+)?([A-Za-z0-9_]+)"
)
DIJKSTRA_POS_RE = re.compile(r"x:\s*(-?\d+),y:\s*(-?\d+),d:\s*(-?\d+)")
DIJKSTRA_RESULT_RE = re.compile(
    r"DIJKSTRA_RESULT\s+(?P<status>GOAL|NO_PATH).*?time:(?P<time>\d+)"
)
PATH_PROFILES = {
    "Uniform 1000": "",
    "Mixed 1600/1800": "acc1600",
}
FW_NODE_CENTER = 0
FW_NODE_NORTH = 1
FW_NODE_EAST = 2
DIR_NORTH = 0
DIR_NORTHEAST = 1
DIR_EAST = 2
DIR_SOUTHEAST = 3
DIR_SOUTH = 4
DIR_SOUTHWEST = 5
DIR_WEST = 6
DIR_NORTHWEST = 7
DIR_NONE = 8

RUN_DIR_VECTORS = {
    0: ((0.0, 1.0), (1.0, 0.0)),
    1: ((1.0, 0.0), (0.0, -1.0)),
    2: ((0.0, -1.0), (-1.0, 0.0)),
    3: ((-1.0, 0.0), (0.0, 1.0)),
}
RUN_MOTION_PATHS = {
    "Turn_in_R45": [(0.0, 0.0), (0.5, 0.0), (1.0, 0.5)],
    "Turn_out_R45": [(0.0, 0.0), (0.5, 0.5), (0.5, 1.0)],
    "Turn_in_L45": [(0.0, 0.0), (0.5, 0.0), (1.0, -0.5)],
    "Turn_out_L45": [(0.0, 0.0), (0.5, -0.5), (0.5, -1.0)],
    "Turn_RV90": [(0.0, 0.0), (0.5, 0.5), (0.0, 1.0)],
    "Turn_LV90": [(0.0, 0.0), (0.5, -0.5), (0.0, -1.0)],
    "Turn_in_R135": [(0.0, 0.0), (0.5, 0.0), (1.0, 0.5), (0.5, 1.0)],
    "Turn_out_R135": [(0.0, 0.0), (0.5, 0.5), (0.0, 1.0), (-0.5, 1.0)],
    "Turn_in_L135": [(0.0, 0.0), (0.5, 0.0), (1.0, -0.5), (0.5, -1.0)],
    "Turn_out_L135": [(0.0, 0.0), (0.5, -0.5), (0.0, -1.0), (-0.5, -1.0)],
    "Long_turnR90": [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)],
    "Long_turnL90": [(0.0, 0.0), (0.5, 0.0), (0.5, -0.5)],
    "Long_turnR180": [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5)],
    "Long_turnL180": [(0.0, 0.0), (0.5, 0.0), (0.5, -0.5), (0.0, -0.5)],
}


def half_to_float(value):
    return struct.unpack("<e", struct.pack("<H", value))[0]


def cell_walls(value):
    return {
        "north": value & 0x03,
        "east": (value >> 2) & 0x03,
        "south": (value >> 4) & 0x03,
        "west": (value >> 6) & 0x03,
    }


def maze_binary_to_wall_data(data, width, height):
    expected_size = width * height
    if len(data) < expected_size:
        raise ValueError(f"Not enough maze binary data: {len(data)}/{expected_size} bytes")

    wall_data = np.zeros((width, height, 4), dtype=bool)
    index = 0
    for y in range(height - 1, -1, -1):
        for x in range(width):
            walls = cell_walls(data[index])
            index += 1
            wall_data[x, y, 0] = walls["north"] in (1, 3)
            wall_data[x, y, 1] = walls["east"] in (1, 3)
            wall_data[x, y, 2] = walls["south"] in (1, 3)
            wall_data[x, y, 3] = walls["west"] in (1, 3)
    return wall_data


def wall_data_to_maze_binary(wall_data):
    wall = np.asarray(wall_data, dtype=bool)
    if wall.ndim != 3 or wall.shape[2] != 4:
        raise ValueError(f"wall_data must have shape (x, y, 4), got {wall.shape}")

    width, height, _ = wall.shape
    payload = bytearray(width * height)
    index = 0
    for y in range(height - 1, -1, -1):
        for x in range(width):
            value = 0
            if wall[x, y, 0]:
                value |= 0x01
            if wall[x, y, 1]:
                value |= 0x04
            if wall[x, y, 2]:
                value |= 0x10
            if wall[x, y, 3]:
                value |= 0x40
            payload[index] = value
            index += 1
    return bytes(payload)


def infer_goal_area(goal_cells, default=DEFAULT_GOAL):
    if not goal_cells:
        return default

    xs = sorted({int(x) for x, _ in goal_cells})
    ys = sorted({int(y) for _, y in goal_cells})
    x0 = xs[0]
    y0 = ys[0]
    size = max(xs[-1] - x0 + 1, ys[-1] - y0 + 1)
    expected = {(x, y) for x in range(x0, x0 + size) for y in range(y0, y0 + size)}
    actual = {(int(x), int(y)) for x, y in goal_cells}
    if actual != expected:
        raise ValueError(f"Goal cells must form a square area, got {sorted(actual)}")
    return x0, y0, size


def goal_cells_from_area(goal_area):
    x0, y0, size = goal_area
    return tuple((x, y) for x in range(x0, x0 + size) for y in range(y0, y0 + size))


def dijkstra_node_point(step):
    x = float(step["x"])
    y = float(step["y"])
    node_pos = step.get("node_pos", FW_NODE_CENTER)
    if node_pos == FW_NODE_NORTH:
        return x, y + 0.5
    if node_pos == FW_NODE_EAST:
        return x + 0.5, y
    return x, y


def infer_mouse_dir(prev_step, step):
    dx = step["x"] - prev_step["x"]
    dy = step["y"] - prev_step["y"]
    if dx > 0 and dy > 0:
        return DIR_NORTHEAST
    if dx > 0 and dy < 0:
        return DIR_SOUTHEAST
    if dx < 0 and dy < 0:
        return DIR_SOUTHWEST
    if dx < 0 and dy > 0:
        return DIR_NORTHWEST
    if abs(dx) >= abs(dy) and dx > 0:
        return DIR_EAST
    if abs(dx) >= abs(dy) and dx < 0:
        return DIR_WEST
    if dy > 0:
        return DIR_NORTH
    if dy < 0:
        return DIR_SOUTH
    return DIR_NORTH


def dijkstra_segment_points(prev_step, step):
    start_point = dijkstra_node_point(prev_step)
    end_point = dijkstra_node_point(step)
    motion = step.get("motion", "")

    if motion in ("Straight", "Diagonal", "No_run", ""):
        return line_points(start_point, end_point)

    run_path = RUN_MOTION_PATHS.get(motion)
    if run_path is not None:
        fitted_path = fit_run_path_to_nodes(start_point, end_point, run_path, step.get("mouse_dir"))
        if fitted_path is not None:
            return fitted_path

    return line_points(start_point, end_point)


def line_points(start_point, end_point):
    return [start_point, end_point]


def fit_run_path_to_nodes(start_point, end_point, run_path, mouse_dir=None):
    candidates = []
    preferred_dir = firmware_dir_to_run_dir(mouse_dir)
    if preferred_dir is not None:
        candidates.append(preferred_dir)
    candidates.extend(direction for direction in range(4) if direction not in candidates)

    best_points = None
    best_error = None
    for direction in candidates:
        points = run_path_to_world(start_point, direction, run_path)
        points = scale_path_to_endpoint(start_point, end_point, points)
        if points is None:
            continue
        error = point_distance_sq(points[-1], end_point)
        if best_error is None or error < best_error:
            best_error = error
            best_points = points

    if best_points is None:
        return None
    if best_error <= 1.0e-6:
        return best_points
    return None


def scale_path_to_endpoint(start_point, end_point, points):
    vx = points[-1][0] - start_point[0]
    vy = points[-1][1] - start_point[1]
    ax = end_point[0] - start_point[0]
    ay = end_point[1] - start_point[1]
    denom = vx * vx + vy * vy
    if denom <= 1.0e-12:
        return None
    scale = (ax * vx + ay * vy) / denom
    if scale <= 0.0:
        return None
    return [
        (
            start_point[0] + (point[0] - start_point[0]) * scale,
            start_point[1] + (point[1] - start_point[1]) * scale,
        )
        for point in points
    ]


def run_path_to_world(start_point, direction, run_path):
    forward, right = RUN_DIR_VECTORS[direction]
    sx, sy = start_point
    return [
        (
            sx + forward_offset * forward[0] + right_offset * right[0],
            sy + forward_offset * forward[1] + right_offset * right[1],
        )
        for forward_offset, right_offset in run_path
    ]


def firmware_dir_to_run_dir(mouse_dir):
    if mouse_dir in (DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST):
        return mouse_dir // 2
    return None


def point_distance_sq(point_a, point_b):
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    return dx * dx + dy * dy


def make_dijkstra_step(line):
    match = DIJKSTRA_LINE_RE.search(line)
    if match:
        x = int(match.group(1))
        y = int(match.group(2))
        node_pos = int(match.group(3))
        mouse_dir = int(match.group(4)) if match.group(4) is not None else None
        motion = match.group(5)
        return {"x": x, "y": y, "node_pos": node_pos, "mouse_dir": mouse_dir, "motion": motion}

    match = DIJKSTRA_POS_RE.search(line)
    if not match:
        return None
    return {
        "x": int(match.group(1)),
        "y": int(match.group(2)),
        "node_pos": int(match.group(3)),
        "mouse_dir": None,
        "motion": "",
    }


def step_cell(step):
    return step["x"], step["y"]


def dijkstra_profile_command(profile, queue_mode=True):
    command = "path dijkstra_queue" if queue_mode else "path dijkstra"
    argument = PATH_PROFILES[profile]
    return f"{command} {argument}".rstrip()


def parse_dijkstra_result(line):
    match = DIJKSTRA_RESULT_RE.search(line)
    if match is None:
        return None
    return {"status": match.group("status"), "time": int(match.group("time"))}


class MyshellGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("myshell GUI")
        self.geometry("1100x720")
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.ui_queue = queue.Queue()

        self.port_var = tk.StringVar(value="COM8")
        self.baud_var = tk.IntVar(value=DEFAULT_BAUD)
        self.width_var = tk.IntVar(value=DEFAULT_WIDTH)
        self.height_var = tk.IntVar(value=DEFAULT_HEIGHT)
        self.timeout_var = tk.DoubleVar(value=30.0)
        self.char_delay_var = tk.DoubleVar(value=0.08)
        self.command_var = tk.StringVar(value="help")
        self.status_var = tk.StringVar(value="Disconnected")
        self.path_profile_var = tk.StringVar(value="Uniform 1000")
        self.path_result_var = tk.StringVar(value="Select a profile and run the shortest path")
        self.goal_var = tk.StringVar(value=f"Goal {DEFAULT_GOAL[0]},{DEFAULT_GOAL[1]} size {DEFAULT_GOAL[2]}")

        self._build_ui()
        self.after(50, self._drain_ui_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        root = ttk.Frame(self, padding=8)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=12)
        self.port_combo.pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)

        ttk.Label(top, text="Baud").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(top, text="Timeout").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.timeout_var, width=6).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(top, text="Char delay").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.char_delay_var, width=6).pack(side=tk.LEFT, padx=(4, 8))

        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=(12, 0))

        command_row = ttk.Frame(root)
        command_row.pack(fill=tk.X, pady=(8, 4))
        ttk.Entry(command_row, textvariable=self.command_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(command_row, text="Send", command=self.send_custom_command).pack(side=tk.LEFT, padx=(8, 0))

        quick = ttk.LabelFrame(root, text="Quick commands", padding=6)
        quick.pack(fill=tk.X, pady=(0, 8))
        for label, command in (
            ("help", lambda: self.run_text_command("help")),
            ("load save", lambda: self.run_text_command("load save")),
            ("disp maze", lambda: self.run_text_command("disp maze")),
            ("disp maze_bin", self.receive_maze_binary),
			("disp history", lambda: self.run_text_command("disp history")),
            ("path dijkstra", self.run_dijkstra_path),
            ("path queue", self.run_dijkstra_queue_path),
            ("compare profiles", self.compare_dijkstra_profiles),
            ("disp log", lambda: self.run_text_command("disp log")),
            ("disp log_bin -> CSV", self.receive_log_binary),
            ("end exe", lambda: self.run_text_command("end exe")),
        ):
            ttk.Button(quick, text=label, command=command).pack(side=tk.LEFT, padx=3, pady=2)

        profile_row = ttk.LabelFrame(root, text="Dijkstra cost profile", padding=6)
        profile_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(profile_row, text="Profile").pack(side=tk.LEFT)
        ttk.Combobox(
            profile_row,
            textvariable=self.path_profile_var,
            values=tuple(PATH_PROFILES),
            state="readonly",
            width=20,
        ).pack(side=tk.LEFT, padx=(5, 8))
        ttk.Button(
            profile_row, text="Run selected", command=self.run_dijkstra_queue_path
        ).pack(side=tk.LEFT)
        ttk.Button(
            profile_row, text="Compare / overlay", command=self.compare_dijkstra_profiles
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(profile_row, text="Blue: Uniform / Orange: Mixed").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(profile_row, textvariable=self.path_result_var).pack(side=tk.RIGHT)

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=1)

        text_frame = ttk.LabelFrame(left, text="Serial output", padding=4)
        text_frame.pack(fill=tk.BOTH, expand=True)
        self.output_text = tk.Text(text_frame, wrap=tk.NONE, height=20)
        self.output_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        text_scroll = ttk.Scrollbar(text_frame, command=self.output_text.yview)
        text_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.output_text.configure(yscrollcommand=text_scroll.set)

        bottom_left = ttk.Frame(left)
        bottom_left.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(bottom_left, text="Clear output", command=self.clear_output).pack(side=tk.LEFT)
        ttk.Button(bottom_left, text="Save output", command=self.save_output).pack(side=tk.LEFT, padx=(6, 0))

        maze_opts = ttk.Frame(right)
        maze_opts.pack(fill=tk.X)
        ttk.Label(maze_opts, text="Maze W").pack(side=tk.LEFT)
        ttk.Entry(maze_opts, textvariable=self.width_var, width=5).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(maze_opts, text="H").pack(side=tk.LEFT)
        ttk.Entry(maze_opts, textvariable=self.height_var, width=5).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(maze_opts, text="Save binary", command=self.save_last_maze_binary).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(maze_opts, text="Save maze_data", command=self.save_last_maze_data).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(maze_opts, text="Load maze_data", command=self.load_maze_data).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(maze_opts, text="Upload maze_data", command=self.upload_maze_data).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(maze_opts, textvariable=self.goal_var).pack(side=tk.LEFT, padx=(8, 0))

        canvas_frame = ttk.LabelFrame(right, text="Maze view", padding=4)
        canvas_frame.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        self.canvas = tk.Canvas(canvas_frame, background="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.last_maze_binary = None
        self.last_maze_wall_data = None
        self.last_maze_start_cells = [(0, 0)]
        self.last_maze_goal_area = DEFAULT_GOAL
        self.last_maze_goal_cells = goal_cells_from_area(DEFAULT_GOAL)
        self.last_path_cells = []
        self.last_profile_paths = {}
        self.last_draw_geometry = None

        self.refresh_ports()

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = []
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror("pyserial missing", "Install pyserial: pip install pyserial")
            return
        try:
            self.serial_port = serial.Serial(self.port_var.get(), self.baud_var.get(), timeout=0.1)
            time.sleep(0.2)
            self.serial_port.reset_input_buffer()
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            return
        self.connect_button.configure(text="Disconnect")
        self.status_var.set(f"Connected: {self.port_var.get()} @ {self.baud_var.get()}")

    def disconnect(self):
        with self.serial_lock:
            if self.serial_port:
                self.serial_port.close()
        self.connect_button.configure(text="Connect")
        self.status_var.set("Disconnected")

    def require_serial(self):
        if not self.serial_port or not self.serial_port.is_open:
            self.connect()
        return self.serial_port and self.serial_port.is_open

    def send_custom_command(self):
        self.run_text_command(self.command_var.get().strip())

    def run_text_command(self, command):
        if not command:
            return
        self._run_worker(lambda: self._text_command_worker(command))

    def receive_maze_binary(self):
        self._run_worker(self._maze_binary_worker)

    def receive_log_binary(self):
        self._run_worker(self._log_binary_worker)

    def run_dijkstra_path(self):
        command = dijkstra_profile_command(self.path_profile_var.get(), queue_mode=False)
        self._run_worker(lambda: self._dijkstra_path_worker(command))

    def run_dijkstra_queue_path(self):
        command = dijkstra_profile_command(self.path_profile_var.get(), queue_mode=True)
        self._run_worker(lambda: self._dijkstra_path_worker(command))

    def compare_dijkstra_profiles(self):
        self._run_worker(self._compare_dijkstra_profiles_worker)

    def upload_maze_data(self):
        if not self.last_maze_binary:
            messagebox.showinfo("No maze", "Load maze_data or receive disp maze_bin first.")
            return
        if self.width_var.get() != DEFAULT_WIDTH or self.height_var.get() != DEFAULT_HEIGHT:
            messagebox.showerror(
                "Unsupported maze size",
                f"load maze_bin expects {DEFAULT_WIDTH}x{DEFAULT_HEIGHT} data.",
            )
            return
        self._run_worker(self._upload_maze_binary_worker)

    def _run_worker(self, target):
        if not self.require_serial():
            return
        threading.Thread(target=self._worker_guard, args=(target,), daemon=True).start()

    def _worker_guard(self, target):
        try:
            target()
        except Exception as exc:
            self._append(f"ERROR: {exc}\n")

    def _write_command(self, command):
        for char in command + "\r":
            self.serial_port.write(char.encode("ascii"))
            self.serial_port.flush()
            delay = self.char_delay_var.get()
            if delay > 0:
                time.sleep(delay)
        self.serial_port.flush()

    def _write_binary_payload(self, payload):
        for value in payload:
            self.serial_port.write(bytes([value]))
            self.serial_port.flush()
            time.sleep(0.002)

    def _prepare_shell(self):
        self.serial_port.reset_input_buffer()
        self._write_command("")
        time.sleep(0.3)
        self.serial_port.read(self.serial_port.in_waiting or 1)

    def _readline_text(self):
        raw = self.serial_port.readline()
        if not raw:
            return None
        return raw.decode(errors="ignore").replace("\x00", "").rstrip("\r\n")

    def _text_command_worker(self, command):
        with self.serial_lock:
            self._append(f"\n> {command}\n")
            self._prepare_shell()
            self._write_command(command)
            path_steps = [self._start_path_step()] if command.startswith("path dijkstra") else None
            deadline = time.monotonic() + self.timeout_var.get()
            idle_deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                line = self._readline_text()
                if line is None:
                    if time.monotonic() > idle_deadline:
                        break
                    continue
                idle_deadline = time.monotonic() + 1.0
                self._append(line + "\n")
                if path_steps is not None:
                    self._collect_dijkstra_line(line, path_steps)
                    if line == "DIJKSTRA_END":
                        break
            if path_steps:
                self.last_path_cells = path_steps
                self.ui_queue.put(("draw_path", path_steps))

    def _dijkstra_path_worker(self, command="path dijkstra", draw=True):
        path_steps = [self._start_path_step()]
        result = None
        with self.serial_lock:
            self._append(f"\n> {command}\n")
            self._prepare_shell()
            self._write_command(command)
            started_at = time.monotonic()
            deadline = time.monotonic() + self.timeout_var.get()
            while time.monotonic() < deadline:
                line = self._readline_text()
                if line is None:
                    continue
                self._append(line + "\n")
                self._collect_dijkstra_line(line, path_steps)
                parsed = parse_dijkstra_result(line)
                if parsed is not None:
                    result = parsed
                if line == "DIJKSTRA_END":
                    elapsed_ms = (time.monotonic() - started_at) * 1000.0
                    self._append(f"ROUND_TRIP_TIME {command}: {elapsed_ms:.3f} ms\n")
                    outcome = result or {"status": "UNKNOWN", "time": None}
                    outcome.update({"round_trip_ms": elapsed_ms, "path": path_steps, "command": command})
                    if draw:
                        self.last_path_cells = path_steps
                        self.ui_queue.put(("draw_path", path_steps))
                        self.ui_queue.put(("path_result", self._format_path_result(command, outcome)))
                    return outcome
        raise TimeoutError("Timed out waiting for DIJKSTRA_END")

    def _compare_dijkstra_profiles_worker(self):
        uniform = self._dijkstra_path_worker(
            dijkstra_profile_command("Uniform 1000"), draw=False
        )
        mixed = self._dijkstra_path_worker(
            dijkstra_profile_command("Mixed 1600/1800"), draw=False
        )
        same_route = self._path_signature(uniform["path"]) == self._path_signature(mixed["path"])
        summary = (
            f"Uniform: {uniform['time']} ms | Mixed: {mixed['time']} ms | "
            f"route: {'same' if same_route else 'changed'}"
        )
        self._append(
            f"PATH_PROFILE_COMPARE uniform={uniform['time']} ms, "
            f"mixed={mixed['time']} ms, route={'same' if same_route else 'changed'}\n"
        )
        self.ui_queue.put(("draw_profile_compare", uniform["path"], mixed["path"]))
        self.ui_queue.put(("path_result", summary))

    @staticmethod
    def _path_signature(path_steps):
        return tuple(
            (step.get("x"), step.get("y"), step.get("node_pos"), step.get("motion"))
            for step in path_steps
        )

    @staticmethod
    def _format_path_result(command, outcome):
        profile = "Mixed" if command.endswith("acc1600") else "Uniform"
        cost = "-" if outcome["time"] is None else f"{outcome['time']} ms"
        return f"{profile}: {outcome['status']} / {cost}"

    def _start_path_step(self):
        start_x, start_y = self.last_maze_start_cells[0]
        return {"x": start_x, "y": start_y, "node_pos": FW_NODE_CENTER, "mouse_dir": DIR_NORTH, "motion": "Start"}

    def _collect_dijkstra_line(self, line, path_steps):
        step = make_dijkstra_step(line)
        if step is None:
            return
        if path_steps:
            previous = path_steps[-1]
            if step["mouse_dir"] is None or step["mouse_dir"] == DIR_NONE:
                step["mouse_dir"] = infer_mouse_dir(previous, step)
            if previous["x"] == step["x"] and previous["y"] == step["y"] and previous["node_pos"] == step["node_pos"]:
                path_steps[-1] = step
                return
        path_steps.append(step)

    def _maze_binary_worker(self):
        width = self.width_var.get()
        height = self.height_var.get()
        payload_size = width * height
        with self.serial_lock:
            self._append("\n> disp maze_bin\n")
            self._prepare_shell()
            self._write_command("disp maze_bin")
            self._wait_for_line("MAZE_BIN_START")
            payload = self._read_exact(payload_size)
            self.last_maze_binary = payload
            self.last_maze_wall_data = maze_binary_to_wall_data(payload, width, height)
            self.last_path_cells = []
            self.last_profile_paths = {}
            self.last_maze_start_cells = [(0, 0)]
            self.last_maze_goal_area = DEFAULT_GOAL
            self.last_maze_goal_cells = goal_cells_from_area(DEFAULT_GOAL)
            self.goal_var.set(f"Goal {DEFAULT_GOAL[0]},{DEFAULT_GOAL[1]} size {DEFAULT_GOAL[2]}")
            self._append(f"Received maze binary: {len(payload)}/{payload_size} bytes\n")
            self.ui_queue.put(("draw_maze", payload, width, height))

    def _upload_maze_binary_worker(self):
        payload = self.last_maze_binary
        payload_size = DEFAULT_WIDTH * DEFAULT_HEIGHT
        if len(payload) < payload_size:
            raise ValueError(f"Maze payload is too short: {len(payload)}/{payload_size} bytes")

        with self.serial_lock:
            self._append("\n> load maze_bin\n")
            self._prepare_shell()
            self._write_command("load maze_bin")
            self._wait_for_line_prefix("MAZE_BIN_READY")
            self._append(f"Sending maze binary: {payload_size} bytes\n")
            self._write_binary_payload(payload[:payload_size])
            self._wait_for_line("MAZE_BIN_LOAD_DONE")
            self._append("Uploaded maze_data to myshell wall_data\n")
            goal_x, goal_y, goal_size = self.last_maze_goal_area
            goal_command = f"load goal {goal_x} {goal_y} {goal_size}"
            self._append(f"\n> {goal_command}\n")
            self._write_command(goal_command)
            self._wait_for_line_prefix("GOAL_SET_DONE")

    def _log_binary_worker(self):
        out_dir = Path(__file__).resolve().parent / "logs"
        out_dir.mkdir(exist_ok=True)
        out_path = out_dir / datetime.now().strftime("%Y%m%d_%H%M%S_myshell_log.csv")
        labels = None
        rows = []

        with self.serial_lock:
            self._append("\n> disp log_bin\n")
            self._prepare_shell()
            self._write_command("disp log_bin")
            deadline = time.monotonic() + self.timeout_var.get()
            while time.monotonic() < deadline:
                line = self._readline_text()
                if line is None:
                    continue
                self._append(line + "\n")
                if line.startswith("HEADER,"):
                    labels = line.split(",")[1:]
                if line == "BINARY":
                    break
            else:
                raise TimeoutError("Timed out waiting for BINARY marker")

            while True:
                raw = self._read_exact(FRAME_SIZE)
                magic, index, *half_values = struct.unpack(FRAME_FORMAT, raw)
                if magic == LOG_MAGIC_END:
                    self._append("Binary END frame detected\n")
                    break
                if magic != LOG_MAGIC:
                    self._append(f"Drop frame: magic=0x{magic:04x}\n")
                    continue
                rows.append([index] + [half_to_float(value) for value in half_values])
                if len(rows) % 50 == 0:
                    self._append(f"Received log frames: {len(rows)}\n")

        with out_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if labels:
                writer.writerow(labels)
            writer.writerows(rows)
        self._append(f"Saved CSV: {out_path}\n")

    def _wait_for_line(self, expected):
        deadline = time.monotonic() + self.timeout_var.get()
        while time.monotonic() < deadline:
            line = self._readline_text()
            if line is None:
                continue
            self._append(line + "\n")
            if line == expected:
                return
        raise TimeoutError(f"Timed out waiting for {expected}")

    def _wait_for_line_prefix(self, expected_prefix):
        deadline = time.monotonic() + self.timeout_var.get()
        while time.monotonic() < deadline:
            line = self._readline_text()
            if line is None:
                continue
            self._append(line + "\n")
            if line.startswith(expected_prefix):
                return line
        raise TimeoutError(f"Timed out waiting for {expected_prefix}")

    def _read_exact(self, size):
        data = bytearray()
        deadline = time.monotonic() + self.timeout_var.get()
        while len(data) < size and time.monotonic() < deadline:
            chunk = self.serial_port.read(size - len(data))
            if chunk:
                data.extend(chunk)
        if len(data) != size:
            raise TimeoutError(f"Timed out reading binary payload: {len(data)}/{size} bytes")
        return bytes(data)

    def _draw_maze(self, data, width, height):
        self.canvas.delete("all")
        canvas_w = max(self.canvas.winfo_width(), 400)
        canvas_h = max(self.canvas.winfo_height(), 400)
        margin = 16
        cell = min((canvas_w - margin * 2) / width, (canvas_h - margin * 2) / height)
        ox = (canvas_w - cell * width) / 2
        oy = (canvas_h - cell * height) / 2
        self.last_draw_geometry = (width, height, cell, ox, oy)

        def point(x, y):
            return ox + x * cell, oy + (height - y) * cell

        for index, value in enumerate(data[: width * height]):
            x = index % width
            y = height - 1 - (index // width)
            walls = cell_walls(value)
            for name, state in walls.items():
                if state not in (1, 3):
                    continue
                color = "#008000" if state == 3 else "#111111"
                line_width = 3 if state == 3 else 2
                if name == "north":
                    x1, y1 = point(x, y + 1)
                    x2, y2 = point(x + 1, y + 1)
                elif name == "east":
                    x1, y1 = point(x + 1, y)
                    x2, y2 = point(x + 1, y + 1)
                elif name == "south":
                    x1, y1 = point(x, y)
                    x2, y2 = point(x + 1, y)
                else:
                    x1, y1 = point(x, y)
                    x2, y2 = point(x, y + 1)
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=line_width)

        for x, y in self.last_maze_goal_cells:
            if not (0 <= x < width and 0 <= y < height):
                continue
            x1, y1 = point(x, y + 1)
            x2, y2 = point(x + 1, y)
            rect = self.canvas.create_rectangle(x1, y1, x2, y2, fill="#ffe5e5", outline="")
            self.canvas.tag_lower(rect)

        if self.last_profile_paths:
            self._draw_profile_comparison(
                self.last_profile_paths.get("uniform", []),
                self.last_profile_paths.get("mixed", []),
            )
        elif self.last_path_cells:
            self._draw_path(self.last_path_cells)

    def _draw_path(self, path_steps, color="#0067c0", tag="path", clear=True, width_scale=0.14):
        if not self.last_draw_geometry:
            return
        width, height, cell, ox, oy = self.last_draw_geometry
        if clear:
            self.canvas.delete("path")
        if len(path_steps) < 2:
            return

        world_points = []
        for index in range(1, len(path_steps)):
            previous = path_steps[index - 1]
            current = path_steps[index]
            if not isinstance(current, dict):
                continue
            if not (0 <= current["x"] < width and 0 <= current["y"] < height):
                continue

            segment = dijkstra_segment_points(previous, current)

            for point in segment:
                if not world_points or world_points[-1] != point:
                    world_points.append(point)

        if len(world_points) < 2:
            return

        points = []
        for x, y in world_points:
            points.extend((ox + (x + 0.5) * cell, oy + (height - y - 0.5) * cell))

        self.canvas.create_line(
            *points,
            fill=color,
            width=max(2, int(cell * width_scale)),
            smooth=False,
            capstyle=tk.ROUND,
            joinstyle=tk.ROUND,
            tags=("path", tag),
        )
        radius = max(3, cell * 0.16)
        sx, sy = points[0], points[1]
        gx, gy = points[-2], points[-1]
        self.canvas.create_oval(
            sx - radius, sy - radius, sx + radius, sy + radius,
            fill=color, outline="", tags=("path", tag),
        )
        self.canvas.create_oval(
            gx - radius, gy - radius, gx + radius, gy + radius,
            fill="#111111", outline="", tags=("path", tag),
        )

    def _draw_profile_comparison(self, uniform_path, mixed_path):
        self.canvas.delete("path")
        self._draw_path(
            uniform_path, color="#0067c0", tag="path_uniform",
            clear=False, width_scale=0.16,
        )
        self._draw_path(
            mixed_path, color="#e87500", tag="path_mixed",
            clear=False, width_scale=0.09,
        )

    def _append(self, text):
        self.ui_queue.put(("append", text))

    def _drain_ui_queue(self):
        try:
            while True:
                item = self.ui_queue.get_nowait()
                if item[0] == "append":
                    self.output_text.insert(tk.END, item[1])
                    self.output_text.see(tk.END)
                elif item[0] == "draw_maze":
                    _, payload, width, height = item
                    self._draw_maze(payload, width, height)
                elif item[0] == "draw_path":
                    _, path_cells = item
                    self.last_profile_paths = {}
                    self._draw_path(path_cells)
                elif item[0] == "draw_profile_compare":
                    _, uniform_path, mixed_path = item
                    self.last_path_cells = mixed_path
                    self.last_profile_paths = {"uniform": uniform_path, "mixed": mixed_path}
                    self._draw_profile_comparison(uniform_path, mixed_path)
                elif item[0] == "path_result":
                    self.path_result_var.set(item[1])
        except queue.Empty:
            pass
        self.after(50, self._drain_ui_queue)

    def clear_output(self):
        self.output_text.delete("1.0", tk.END)

    def save_output(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text", "*.txt"), ("All", "*.*")])
        if not path:
            return
        Path(path).write_text(self.output_text.get("1.0", tk.END), encoding="utf-8")

    def save_last_maze_binary(self):
        if not self.last_maze_binary:
            messagebox.showinfo("No maze", "Receive disp maze_bin first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".bin", filetypes=[("Binary", "*.bin"), ("All", "*.*")])
        if path:
            Path(path).write_bytes(self.last_maze_binary)

    def save_last_maze_data(self):
        if self.last_maze_wall_data is None:
            messagebox.showinfo("No maze", "Receive disp maze_bin or load maze_data first.")
            return

        MAZE_DATA_DIR.mkdir(exist_ok=True)
        default_name = datetime.now().strftime("%Y%m%d_%H%M%S_myshell_maze.txt")
        path = filedialog.asksaveasfilename(
            initialdir=MAZE_DATA_DIR,
            initialfile=default_name,
            defaultextension=".txt",
            filetypes=[("Maze text", "*.txt"), ("All", "*.*")],
        )
        if not path:
            return

        try:
            saved_path = save_maze_file(
                path,
                self.last_maze_wall_data,
                start_cells=self.last_maze_start_cells,
                goal_cells=self.last_maze_goal_cells,
                backup=True,
            )
        except Exception as exc:
            messagebox.showerror("Save maze_data failed", str(exc))
            return
        self._append(f"Saved maze_data: {saved_path}\n")

    def load_maze_data(self):
        MAZE_DATA_DIR.mkdir(exist_ok=True)
        path = filedialog.askopenfilename(
            initialdir=MAZE_DATA_DIR,
            filetypes=[("Maze text", "*.txt"), ("All", "*.*")],
        )
        if not path:
            return

        try:
            data = MazeFileReader.from_file(path)
            payload = wall_data_to_maze_binary(data.maze_wall_data)
            goal_area = infer_goal_area(data.goal_cells)
        except Exception as exc:
            messagebox.showerror("Load maze_data failed", str(exc))
            return

        self.width_var.set(data.x_cnt)
        self.height_var.set(data.y_cnt)
        self.last_maze_binary = payload
        self.last_maze_wall_data = data.maze_wall_data.copy()
        self.last_path_cells = []
        self.last_profile_paths = {}
        self.last_maze_start_cells = list(data.start_cells)
        self.last_maze_goal_area = goal_area
        self.last_maze_goal_cells = goal_cells_from_area(goal_area)
        self.goal_var.set(f"Goal {goal_area[0]},{goal_area[1]} size {goal_area[2]}")
        self._draw_maze(payload, data.x_cnt, data.y_cnt)
        self._append(
            f"Loaded maze_data: {data.filename} ({data.x_cnt}x{data.y_cnt}) "
            f"goal=({goal_area[0]},{goal_area[1]}) size={goal_area[2]}\n"
        )

    def _on_close(self):
        self.disconnect()
        self.destroy()


def main():
    app = MyshellGui()
    app.mainloop()


if __name__ == "__main__":
    main()
