import argparse
import re
import time
from pathlib import Path

import matplotlib.pyplot as plt

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


def wait_for_prompt(ser, timeout, prompt, enter, debug=False):
    deadline = time.monotonic() + timeout
    buffer = ""

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
            return

        if len(buffer) > 256:
            buffer = buffer[-256:]

    raise TimeoutError(f"Timed out waiting for prompt {prompt!r}")


def write_slowly(ser, text, char_delay):
    for char in text:
        ser.write(char.encode("ascii"))
        if char_delay > 0:
            time.sleep(char_delay)


def receive_maze_ascii(port, baud, command, width, height, timeout, prompt, line_ending, debug, char_delay):
    import serial

    expected_lines = height * 2 + 1
    maze_lines = []
    started = False
    in_marked_maze = False
    recent_lines = []
    enter = command_line_ending(line_ending)

    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        wait_for_prompt(ser, timeout, prompt, enter, debug=debug)
        write_slowly(ser, command + enter, char_delay)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue

            line = strip_terminal_codes(raw.decode(errors="ignore"))
            if debug:
                print(repr(line))
            recent_lines.append(line)
            recent_lines = recent_lines[-12:]

            if line == "MAZE_START":
                started = True
                in_marked_maze = True
                maze_lines = []
                continue

            if line == "MAZE_END":
                if len(maze_lines) >= expected_lines:
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
                maze_lines.append(line)
                if len(maze_lines) >= expected_lines and not in_marked_maze:
                    return maze_lines[:expected_lines]

        raise TimeoutError(
            f"Timed out waiting for maze output: got {len(maze_lines)}/{expected_lines} lines. "
            f"Recent serial lines: {recent_lines}"
        )


def load_maze_ascii(path):
    lines = []
    for raw_line in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines():
        line = strip_terminal_codes(raw_line)
        if is_maze_line(line):
            lines.append(line)
    return lines


def parse_maze_ascii(lines, width, height):
    expected_lines = height * 2 + 1
    if len(lines) < expected_lines:
        raise ValueError(f"Not enough maze lines: got {len(lines)}, need {expected_lines}")

    maze = Maze(width, height, verbose=False)
    maze.wall[:, :, :] = False

    line_index = 0
    for y in range(height - 1, -1, -1):
        top = lines[line_index]
        middle = lines[line_index + 1]
        line_index += 2

        for x in range(width):
            base = x * 4
            if top[base + 1 : base + 4] == "---":
                maze.set_wall(x, y, Maze.North, True)
            if base < len(middle) and middle[base] == "|":
                maze.set_wall(x, y, Maze.West, True)

        maze.set_wall(width - 1, y, Maze.East, True)

    bottom = lines[line_index]
    for x in range(width):
        base = x * 4
        if bottom[base + 1 : base + 4] == "---":
            maze.set_wall(x, 0, Maze.South, True)

    return maze


def main():
    parser = argparse.ArgumentParser(
        description="Receive myshell 'disp maze' output and display it with maze.py."
    )
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--width", type=int, default=32)
    parser.add_argument("--height", type=int, default=32)
    parser.add_argument("--command", default="disp maze")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--prompt", default=">")
    parser.add_argument("--line-ending", choices=("cr", "lf", "crlf"), default="cr")
    parser.add_argument("--char-delay", type=float, default=0.02)
    parser.add_argument("--debug", action="store_true", help="Print raw serial lines while receiving.")
    parser.add_argument("--file", help="Parse maze ASCII from a saved text file instead of serial.")
    parser.add_argument("--print-ascii", action="store_true", help="Print parsed maze ASCII before drawing.")
    parser.add_argument("--no-show", action="store_true", help="Do not open the matplotlib window.")
    parser.add_argument("--save", help="Save the drawn maze image to this path.")
    args = parser.parse_args()

    if args.file:
        lines = load_maze_ascii(args.file)
    else:
        lines = receive_maze_ascii(
            args.port,
            args.baud,
            args.command,
            args.width,
            args.height,
            args.timeout,
            args.prompt,
            args.line_ending,
            args.debug,
            args.char_delay,
        )

    maze = parse_maze_ascii(lines, args.width, args.height)

    if args.print_ascii:
        maze.disp_map()

    if not args.no_show or args.save:
        fig, ax = plt.subplots()
        maze.draw_maze(ax=ax, clear=True)
        ax.set_title("Received maze")
        if args.save:
            fig.savefig(args.save, dpi=150, bbox_inches="tight")
        if not args.no_show:
            plt.show()


if __name__ == "__main__":
    main()
