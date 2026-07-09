import csv
import queue
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


DEFAULT_BAUD = 115200
DEFAULT_WIDTH = 32
DEFAULT_HEIGHT = 32
LOG_DATA_NUM = 51
LOG_MAGIC = 0xA55A
LOG_MAGIC_END = 0xFFFF
FRAME_FORMAT = "<HH" + "H" * LOG_DATA_NUM
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)


def half_to_float(value):
    return struct.unpack("<e", struct.pack("<H", value))[0]


def cell_walls(value):
    return {
        "north": value & 0x03,
        "east": (value >> 2) & 0x03,
        "south": (value >> 4) & 0x03,
        "west": (value >> 6) & 0x03,
    }


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
        self.command_var = tk.StringVar(value="help")
        self.status_var = tk.StringVar(value="Disconnected")

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
            ("disp histry", lambda: self.run_text_command("disp histry")),
            ("path dijkstra", lambda: self.run_text_command("path dijkstra")),
            ("disp log", lambda: self.run_text_command("disp log")),
            ("disp log_bin -> CSV", self.receive_log_binary),
            ("end exe", lambda: self.run_text_command("end exe")),
        ):
            ttk.Button(quick, text=label, command=command).pack(side=tk.LEFT, padx=3, pady=2)

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
        ttk.Button(maze_opts, text="Save maze binary", command=self.save_last_maze_binary).pack(side=tk.LEFT, padx=(8, 0))

        canvas_frame = ttk.LabelFrame(right, text="Maze binary view", padding=4)
        canvas_frame.pack(fill=tk.BOTH, expand=True, pady=(6, 0))
        self.canvas = tk.Canvas(canvas_frame, background="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.last_maze_binary = None

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
        self.serial_port.write((command + "\r").encode("ascii"))
        self.serial_port.flush()

    def _readline_text(self):
        raw = self.serial_port.readline()
        if not raw:
            return None
        return raw.decode(errors="ignore").replace("\x00", "").rstrip("\r\n")

    def _text_command_worker(self, command):
        with self.serial_lock:
            self._append(f"\n> {command}\n")
            self.serial_port.reset_input_buffer()
            self._write_command(command)
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

    def _maze_binary_worker(self):
        width = self.width_var.get()
        height = self.height_var.get()
        payload_size = width * height
        with self.serial_lock:
            self._append("\n> disp maze_bin\n")
            self.serial_port.reset_input_buffer()
            self._write_command("disp maze_bin")
            self._wait_for_line("MAZE_BIN_START")
            payload = self._read_exact(payload_size)
            self.last_maze_binary = payload
            self._append(f"Received maze binary: {len(payload)}/{payload_size} bytes\n")
            self.ui_queue.put(("draw_maze", payload, width, height))

    def _log_binary_worker(self):
        out_dir = Path(__file__).resolve().parent / "logs"
        out_dir.mkdir(exist_ok=True)
        out_path = out_dir / datetime.now().strftime("%Y%m%d_%H%M%S_myshell_log.csv")
        labels = None
        rows = []

        with self.serial_lock:
            self._append("\n> disp log_bin\n")
            self.serial_port.reset_input_buffer()
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

    def _on_close(self):
        self.disconnect()
        self.destroy()


def main():
    app = MyshellGui()
    app.mainloop()


if __name__ == "__main__":
    main()
