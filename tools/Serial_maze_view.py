import argparse
import re
import time
import struct
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from maze import Maze


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def strip_terminal_codes(text):
    return ANSI_RE.sub("", text.replace("\x00", "")).rstrip("\r\n")


def command_line_ending(name):
    if name == "cr":
        return "\r"
    if name == "lf":
        return "\n"
    if name == "crlf":
        return "\r\n"
    raise ValueError(f"Invalid line ending: {name}")


def is_maze_line(line):
    if not line:
        return False
    return line.startswith("+") or line.startswith("|") or line.startswith(" ")


def report_progress(enabled, message):
    if enabled:
        print(message, flush=True)


def wait_for_prompt(ser, timeout, prompt, enter, debug=False, progress=False):
    deadline = time.monotonic() + timeout
    buffer = ""

    report_progress(progress, f"Waiting for prompt {prompt!r}...")
    ser.write(enter.encode("ascii"))
    while time.monotonic() < deadline:
        raw = ser.read(1)
        if not raw:
            continue

        char = raw.decode(errors="ignore")
        if debug:
            print(char, end="", flush=True)
        buffer += char
        if prompt in buffer:
            report_progress(progress, "Prompt detected.")
            return

        if len(buffer) > 256:
            buffer = buffer[-256:]

    raise TimeoutError(f"Timed out waiting for prompt {prompt!r}")


def write_slowly(ser, text, char_delay):
    for char in text:
        ser.write(char.encode("ascii"))
        if char_delay > 0:
            time.sleep(char_delay)


def receive_maze_ascii(port, baud, command, width, height, timeout, prompt, line_ending, debug, char_delay, progress):
    import serial

    expected_lines = height * 2 + 1
    maze_lines = []
    started = False
    in_marked_maze = False
    recent_lines = []
    enter = command_line_ending(line_ending)

    report_progress(progress, f"Opening {port} at {baud} baud...")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        wait_for_prompt(ser, timeout, prompt, enter, debug=debug, progress=progress)
        report_progress(progress, f"Sending command: {command!r}")
        write_slowly(ser, command + enter, char_delay)

        deadline = time.monotonic() + timeout
        last_report = time.monotonic()
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue

            raw_line = raw.decode(errors="ignore").replace("\x00", "").rstrip("\r\n")
            line = strip_terminal_codes(raw_line)
            if debug:
                print(repr(line))
            recent_lines.append(line)
            recent_lines = recent_lines[-12:]

            if line == "MAZE_START":
                started = True
                in_marked_maze = True
                maze_lines = []
                report_progress(progress, "MAZE_START detected.")
                continue

            if line == "MAZE_END":
                if len(maze_lines) >= expected_lines:
                    report_progress(progress, f"MAZE_END detected. Received {expected_lines}/{expected_lines} lines.")
                    return maze_lines[:expected_lines]
                raise ValueError(
                    f"MAZE_END received before enough lines: got {len(maze_lines)}/{expected_lines}"
                )

            if not started:
                start = line.find("+")
                if start >= 0:
                    started = True
                    maze_lines.append(line[start:])
                continue

            if is_maze_line(line):
                maze_lines.append(raw_line)
                now = time.monotonic()
                if progress and now - last_report >= 0.5:
                    report_progress(True, f"Receiving ASCII maze: {len(maze_lines)}/{expected_lines} lines")
                    last_report = now
                if len(maze_lines) >= expected_lines and not in_marked_maze:
                    report_progress(progress, f"Received {expected_lines}/{expected_lines} lines.")
                    return maze_lines[:expected_lines]

        raise TimeoutError(
            f"Timed out waiting for maze output: got {len(maze_lines)}/{expected_lines} lines. "
            f"Recent serial lines: {recent_lines}"
        )


def receive_maze_binary(port, baud, command, width, height, timeout, prompt, line_ending, debug, char_delay, progress):
    import serial

    payload_size = width * height
    marker_start = "MAZE_BIN_START"
    enter = command_line_ending(line_ending)

    report_progress(progress, f"Opening {port} at {baud} baud...")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        wait_for_prompt(ser, timeout, prompt, enter, debug=debug, progress=progress)
        report_progress(progress, f"Sending command: {command!r}")
        write_slowly(ser, command + enter, char_delay)

        deadline = time.monotonic() + timeout
        last_report = time.monotonic()
        recent_lines = []
        report_progress(progress, f"Waiting for {marker_start}...")
        while time.monotonic() < deadline:
            raw_line = ser.readline()
            if not raw_line:
                continue

            line = strip_terminal_codes(raw_line.decode(errors="ignore"))
            if debug:
                print(repr(line))
            recent_lines.append(line)
            recent_lines = recent_lines[-12:]

            if line != marker_start:
                continue

            report_progress(progress, f"{marker_start} detected. Receiving {payload_size} bytes...")
            payload = bytearray()
            while len(payload) < payload_size and time.monotonic() < deadline:
                chunk = ser.read(payload_size - len(payload))
                if not chunk:
                    continue

                payload.extend(chunk)
                now = time.monotonic()
                if progress and now - last_report >= 0.5:
                    report_progress(True, f"Receiving binary maze: {len(payload)}/{payload_size} bytes")
                    last_report = now

            if len(payload) == payload_size:
                report_progress(progress, f"Received binary maze: {payload_size}/{payload_size} bytes")
                return bytes(payload)

            raise TimeoutError(f"Timed out receiving binary payload: got {len(payload)}/{payload_size} bytes")

        raise TimeoutError(f"Timed out waiting for binary maze output: {marker_start} was not detected. Recent serial lines: {recent_lines}")


def load_maze_ascii(path):
    lines = []
    for raw_line in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines():
        line = strip_terminal_codes(raw_line)
        if is_maze_line(line):
            lines.append(line)
    return lines


def load_maze_ascii_raw(path):
    lines = []
    for raw_line in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.replace("\x00", "").rstrip("\r\n")
        if is_maze_line(strip_terminal_codes(line)):
            lines.append(line)
    return lines


def load_maze_binary(path):
    data = Path(path).read_bytes()
    marker_start = b"MAZE_BIN_START"
    start = data.find(marker_start)
    if start >= 0:
        payload_start = start + len(marker_start)
        if data[payload_start:payload_start + 2] == b"\r\n":
            payload_start += 2
        elif data[payload_start:payload_start + 1] in (b"\r", b"\n"):
            payload_start += 1
        return data[payload_start:]
    return data


def parse_maze_ascii(lines, width, height):
    expected_lines = height * 2 + 1
    if len(lines) < expected_lines:
        raise ValueError(f"Not enough maze lines: got {len(lines)}, need {expected_lines}")

    maze = Maze(width, height, verbose=False)
    maze.wall[:, :, :] = False

    line_index = 0
    for y in range(height - 1, -1, -1):
        top = strip_terminal_codes(lines[line_index])
        middle = strip_terminal_codes(lines[line_index + 1])
        line_index += 2

        for x in range(width):
            base = x * 4
            if top[base + 1 : base + 4] == "---":
                maze.set_wall(x, y, Maze.North, True)
            if base < len(middle) and middle[base] == "|":
                maze.set_wall(x, y, Maze.West, True)

        maze.set_wall(width - 1, y, Maze.East, True)

    bottom = strip_terminal_codes(lines[line_index])
    for x in range(width):
        base = x * 4
        if bottom[base + 1 : base + 4] == "---":
            maze.set_wall(x, 0, Maze.South, True)

    return maze


def virtual_segments_from_ascii(raw_lines, width, height):
    expected_lines = height * 2 + 1
    if len(raw_lines) < expected_lines:
        return []

    segments = []
    line_index = 0
    for y in range(height - 1, -1, -1):
        top_raw = raw_lines[line_index]
        middle_raw = raw_lines[line_index + 1]
        top_plain = strip_terminal_codes(top_raw)
        middle_plain = strip_terminal_codes(middle_raw)
        top_vwall_positions = _vwall_char_positions(top_raw)
        middle_vwall_positions = _vwall_char_positions(middle_raw)
        line_index += 2

        for x in range(width):
            base = x * 4
            if top_plain[base + 1 : base + 4] == "---" and any(pos in top_vwall_positions for pos in range(base + 1, base + 4)):
                segments.append(Maze._wall_segment(x, y, Maze.North))
            if base < len(middle_plain) and middle_plain[base] == "|" and base in middle_vwall_positions:
                segments.append(Maze._wall_segment(x, y, Maze.West))

    return segments


def _vwall_char_positions(raw_line):
    positions = set()
    visible_pos = 0
    index = 0
    vwall = False

    while index < len(raw_line):
        if raw_line.startswith("\x1b[31m", index):
            vwall = True
            index += len("\x1b[31m")
            continue
        if raw_line.startswith("\x1b[39m", index):
            vwall = False
            index += len("\x1b[39m")
            continue

        if raw_line[index] == "\x1b":
            match = ANSI_RE.match(raw_line, index)
            if match:
                index = match.end()
                continue

        if vwall:
            positions.add(visible_pos)
        visible_pos += 1
        index += 1

    return positions


def parse_maze_binary(data, width, height):
    expected_size = width * height
    if len(data) < expected_size:
        raise ValueError(f"Not enough binary maze data: got {len(data)}, need {expected_size}")

    maze = Maze(width, height, verbose=False)
    maze.wall[:, :, :] = False
    virtual_segments = []

    cells = struct.unpack(f"{expected_size}B", data[:expected_size])
    index = 0
    for y in range(height - 1, -1, -1):
        for x in range(width):
            value = cells[index]
            index += 1

            north = value & 0x03
            east = (value >> 2) & 0x03
            south = (value >> 4) & 0x03
            west = (value >> 6) & 0x03

            maze.wall[x, y, Maze.North] = north in (1, 3)
            maze.wall[x, y, Maze.West] = west in (1, 3)
            if x == width - 1:
                maze.wall[x, y, Maze.East] = east in (1, 3)
            if y == 0:
                maze.wall[x, y, Maze.South] = south in (1, 3)

            if north == 3:
                virtual_segments.append(Maze._wall_segment(x, y, Maze.North))
            if west == 3:
                virtual_segments.append(Maze._wall_segment(x, y, Maze.West))
            if x == width - 1 and east == 3:
                virtual_segments.append(Maze._wall_segment(x, y, Maze.East))
            if y == 0 and south == 3:
                virtual_segments.append(Maze._wall_segment(x, y, Maze.South))

    return maze, virtual_segments


def print_binary_summary(data, width, height):
    expected_size = width * height
    payload = data[:expected_size]
    head = " ".join(f"{byte:02x}" for byte in payload[:32])
    unique = len(set(payload))
    print("Binary protocol: cell")
    print(f"Binary payload: {len(payload)}/{expected_size} bytes, unique={unique}")
    print(f"First 32 bytes: {head}")

    if len(payload) >= expected_size:
        counts = {
            "north": [0, 0, 0, 0],
            "east": [0, 0, 0, 0],
            "south": [0, 0, 0, 0],
            "west": [0, 0, 0, 0],
        }
        for value in payload:
            counts["north"][value & 0x03] += 1
            counts["east"][(value >> 2) & 0x03] += 1
            counts["south"][(value >> 4) & 0x03] += 1
            counts["west"][(value >> 6) & 0x03] += 1
        print("Cell state counts [NOWALL, WALL, UNKNOWN, VWALL]:")
        for direction, values in counts.items():
            print(f"  {direction}: {values}")

        top = payload[:width]
        bottom = payload[-width:]
        left = payload[0::width]
        right = payload[width - 1::width]
        print(f"Top north WALL/VWALL count: {sum((v & 0x03) in (1, 3) for v in top)}/{width}")
        print(f"Bottom south WALL/VWALL count: {sum(((v >> 4) & 0x03) in (1, 3) for v in bottom)}/{width}")
        print(f"Left west WALL/VWALL count: {sum(((v >> 6) & 0x03) in (1, 3) for v in left)}/{height}")
        print(f"Right east WALL/VWALL count: {sum(((v >> 2) & 0x03) in (1, 3) for v in right)}/{height}")


def draw_virtual_walls(ax, virtual_segments):
    if virtual_segments:
        ax.add_collection(LineCollection(virtual_segments, colors="g", linewidths=2.5))


def main():
    parser = argparse.ArgumentParser(
        description="Receive myshell maze output and display it with maze.py."
    )
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--width", type=int, default=32)
    parser.add_argument("--height", type=int, default=32)
    parser.add_argument("--command", default=None)
    parser.add_argument("--binary", action="store_true", help="Receive/parse 'disp maze_bin' binary output.")
    parser.add_argument("--dump-binary", help="Save received binary payload to this file.")
    parser.add_argument("--print-binary-summary", action="store_true", help="Print first bytes and payload summary.")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--prompt", default=">")
    parser.add_argument("--line-ending", choices=("cr", "lf", "crlf"), default="cr")
    parser.add_argument("--char-delay", type=float, default=0.02)
    parser.add_argument("--debug", action="store_true", help="Print raw serial lines while receiving.")
    parser.add_argument("--quiet", action="store_true", help="Hide receive progress messages.")
    parser.add_argument("--file", help="Parse maze ASCII from a saved text file instead of serial.")
    parser.add_argument("--print-ascii", action="store_true", help="Print parsed maze ASCII before drawing.")
    parser.add_argument("--no-show", action="store_true", help="Do not open the matplotlib window.")
    parser.add_argument("--save", help="Save the drawn maze image to this path.")
    args = parser.parse_args()

    command = args.command or ("disp maze_bin" if args.binary else "disp maze")
    progress = not args.quiet

    if args.file:
        report_progress(progress, f"Loading maze from {args.file}...")
        if args.binary:
            binary_data = load_maze_binary(args.file)
            if args.print_binary_summary:
                print_binary_summary(binary_data, args.width, args.height)
            maze, virtual_segments = parse_maze_binary(binary_data, args.width, args.height)
        else:
            raw_lines = load_maze_ascii_raw(args.file)
            maze = parse_maze_ascii(raw_lines, args.width, args.height)
            virtual_segments = virtual_segments_from_ascii(raw_lines, args.width, args.height)
    else:
        if args.binary:
            binary_data = receive_maze_binary(
                args.port,
                args.baud,
                command,
                args.width,
                args.height,
                args.timeout,
                args.prompt,
                args.line_ending,
                args.debug,
                args.char_delay,
                progress,
            )
            if args.dump_binary:
                Path(args.dump_binary).write_bytes(binary_data)
                report_progress(progress, f"Saved binary payload: {args.dump_binary}")
            if args.print_binary_summary:
                print_binary_summary(binary_data, args.width, args.height)
            maze, virtual_segments = parse_maze_binary(
                binary_data,
                args.width,
                args.height,
            )
        else:
            lines = receive_maze_ascii(
                args.port,
                args.baud,
                command,
                args.width,
                args.height,
                args.timeout,
                args.prompt,
                args.line_ending,
                args.debug,
                args.char_delay,
                progress,
            )
            maze = parse_maze_ascii(lines, args.width, args.height)
            virtual_segments = virtual_segments_from_ascii(lines, args.width, args.height)

    report_progress(progress, "Maze parsed.")

    if args.print_ascii:
        maze.disp_map()

    if not args.no_show or args.save:
        report_progress(progress, "Drawing maze...")
        fig, ax = plt.subplots()
        maze.draw_maze(ax=ax, clear=True)
        draw_virtual_walls(ax, virtual_segments)
        ax.set_title("Received maze")
        if args.save:
            fig.savefig(args.save, dpi=150, bbox_inches="tight")
            report_progress(progress, f"Saved image: {args.save}")
        if not args.no_show:
            report_progress(progress, "Opening matplotlib window.")
            plt.show()


if __name__ == "__main__":
    main()
