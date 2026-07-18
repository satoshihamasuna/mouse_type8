"""Replay and verify the firmware Search logic through myshell.

The tool can upload a packed maze, a saved ``wall_histry`` text log, or a
standard ``maze_data`` text maze.  It then runs the firmware-side ``search run``
diagnostic and checks map selection, protected cells, and virtual-wall safety.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Iterable


WIDTH = 32
HEIGHT = 32
MAZE_SIZE = WIDTH * HEIGHT
UNKNOWN = 2
NOWALL = 0
WALL = 1
VWALL = 3
DIRECTION_INDEX = {"N": 0, "E": 1, "S": 2, "W": 3}
DIRECTION_DELTA = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
OPPOSITE = {"N": "S", "E": "W", "S": "N", "W": "E"}

HISTORY_RE = re.compile(
    r"\(x,y\)->\(\s*(\d+)\s*,\s*(\d+)\s*\),"
    r"\(n,e,s,w\)->\(\s*([0-3])\s*,\s*([0-3])\s*,\s*([0-3])\s*,\s*([0-3])\s*\)"
)


@dataclass
class Step:
    index: int
    x: int
    y: int
    heading: str
    self_value: int
    map_values: tuple[int, int, int, int]
    wall_values: tuple[int, int, int, int]
    virtual_values: tuple[int, int, int, int]
    next_x: int
    next_y: int
    next_heading: str
    local: int
    found: bool
    selected_vwall: bool
    virtual_edges: int


@dataclass
class SearchResult:
    config: dict[str, str] = field(default_factory=dict)
    steps: list[Step] = field(default_factory=list)
    result: str = "missing"
    final_x: int = 0
    final_y: int = 0
    final_heading: str = "N"
    reported_steps: int = 0
    reported_virtual_edges: int = 0
    virtual_edges: list[tuple[int, int, str]] = field(default_factory=list)
    map_rows: dict[int, list[int]] = field(default_factory=dict)
    firmware_errors: list[str] = field(default_factory=list)


@dataclass
class ReplayStep:
    index: int
    x: int
    y: int
    heading: str
    self_value: int
    map_values: tuple[int, int, int, int]
    wall_values: tuple[int, int, int, int]
    virtual_values: tuple[int, int, int, int]
    sensed: bool
    next_x: int
    next_y: int
    next_heading: str
    local: int
    truth: int
    selected_vwall: bool
    next_acc: int
    virtual_edges: int


@dataclass
class ReplayResult:
    config: dict[str, str] = field(default_factory=dict)
    steps: list[ReplayStep] = field(default_factory=list)
    result: str = "missing"
    final_x: int = 0
    final_y: int = 0
    final_heading: str = "N"
    reported_steps: int = 0
    sensed: int = 0
    history: int = 0
    reported_virtual_edges: int = 0
    virtual_edges: list[tuple[int, int, str]] = field(default_factory=list)
    map_rows: dict[int, list[int]] = field(default_factory=dict)
    firmware_errors: list[str] = field(default_factory=list)


def parse_fields(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in line.split()[1:]:
        if ":" not in token:
            continue
        key, value = token.split(":", 1)
        fields[key] = value
    return fields


def parse_ints(text: str, count: int) -> tuple[int, ...]:
    values = tuple(int(value) for value in text.split(","))
    if len(values) != count:
        raise ValueError(f"expected {count} comma-separated integers, got {text!r}")
    return values


def parse_search_lines(lines: Iterable[str]) -> SearchResult:
    result = SearchResult()
    for raw in lines:
        line = raw.replace("\x00", "").strip()
        if line.startswith("SEARCH_RUN_START "):
            result.config = parse_fields(line)
        elif line.startswith("SEARCH_STEP "):
            fields = parse_fields(line)
            pos = fields["pos"].split(",")
            nxt = fields["next"].split(",")
            result.steps.append(
                Step(
                    index=int(fields["index"]),
                    x=int(pos[0]),
                    y=int(pos[1]),
                    heading=pos[2],
                    self_value=int(fields["self"]),
                    map_values=parse_ints(fields["map"], 4),
                    wall_values=parse_ints(fields["wall"], 4),
                    virtual_values=parse_ints(fields["vwall"], 4),
                    next_x=int(nxt[0]),
                    next_y=int(nxt[1]),
                    next_heading=nxt[2],
                    local=int(fields["local"]),
                    found=fields["found"] == "1",
                    selected_vwall=fields["selected_vwall"] == "1",
                    virtual_edges=int(fields["virtual_edges"]),
                )
            )
        elif line.startswith("SEARCH_RUN_END "):
            fields = parse_fields(line)
            final = fields["final"].split(",")
            result.result = fields["result"]
            result.reported_steps = int(fields["steps"])
            result.final_x = int(final[0])
            result.final_y = int(final[1])
            result.final_heading = final[2]
            result.reported_virtual_edges = int(fields["virtual_edges"])
        elif line.startswith("SEARCH_VIRTUAL "):
            fields = parse_fields(line)
            result.virtual_edges.append((int(fields["x"]), int(fields["y"]), fields["dir"]))
        elif line.startswith("SEARCH_MAP "):
            fields = parse_fields(line)
            result.map_rows[int(fields["y"])] = list(parse_ints(fields["values"], WIDTH))
        elif line.startswith("SEARCH_ERROR "):
            result.firmware_errors.append(line)
    return result


def parse_replay_lines(lines: Iterable[str]) -> ReplayResult:
    result = ReplayResult()
    for raw in lines:
        line = raw.replace("\x00", "").strip()
        if line.startswith("REPLAY_START "):
            result.config = parse_fields(line)
        elif line.startswith("REPLAY_STEP "):
            fields = parse_fields(line)
            pos = fields["pos"].split(",")
            nxt = fields["next"].split(",")
            result.steps.append(
                ReplayStep(
                    index=int(fields["index"]),
                    x=int(pos[0]),
                    y=int(pos[1]),
                    heading=pos[2],
                    self_value=int(fields["self"]),
                    map_values=parse_ints(fields["map"], 4),
                    wall_values=parse_ints(fields["wall"], 4),
                    virtual_values=parse_ints(fields["vwall"], 4),
                    sensed=fields["sensed"] == "1",
                    next_x=int(nxt[0]),
                    next_y=int(nxt[1]),
                    next_heading=nxt[2],
                    local=int(fields["local"]),
                    truth=int(fields["truth"]),
                    selected_vwall=fields["selected_vwall"] == "1",
                    next_acc=int(fields["next_acc"]),
                    virtual_edges=int(fields["virtual_edges"]),
                )
            )
        elif line.startswith("REPLAY_END "):
            fields = parse_fields(line)
            final = fields["final"].split(",")
            result.result = fields["result"]
            result.reported_steps = int(fields["steps"])
            result.final_x = int(final[0])
            result.final_y = int(final[1])
            result.final_heading = final[2]
            result.sensed = int(fields["sensed"])
            result.history = int(fields["history"])
            result.reported_virtual_edges = int(fields["virtual_edges"])
        elif line.startswith("REPLAY_VIRTUAL "):
            fields = parse_fields(line)
            result.virtual_edges.append((int(fields["x"]), int(fields["y"]), fields["dir"]))
        elif line.startswith("REPLAY_MAP "):
            fields = parse_fields(line)
            result.map_rows[int(fields["y"])] = list(parse_ints(fields["values"], WIDTH))
        elif line.startswith("REPLAY_ERROR "):
            result.firmware_errors.append(line)
    return result


def edge_endpoints(x: int, y: int, direction: str) -> tuple[tuple[int, int], tuple[int, int]]:
    dx, dy = DIRECTION_DELTA[direction]
    return (x, y), (x + dx, y + dy)


def parse_goal(text: str) -> tuple[int, int, int]:
    values = parse_ints(text, 3)
    x, y, size = values
    if size <= 0 or x < 0 or y < 0 or x + size > WIDTH or y + size > HEIGHT:
        raise argparse.ArgumentTypeError("goal must be in range as X,Y,SIZE")
    return x, y, size


def parse_start(text: str) -> tuple[int, int, str]:
    parts = text.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("start must be X,Y,N|E|S|W")
    x, y = int(parts[0]), int(parts[1])
    heading = parts[2].upper()
    if not (0 <= x < WIDTH and 0 <= y < HEIGHT) or heading not in DIRECTION_INDEX:
        raise argparse.ArgumentTypeError("start must be X,Y,N|E|S|W in maze range")
    return x, y, heading


def validate_result(
    result: SearchResult,
    start: tuple[int, int, str],
    goal: tuple[int, int, int],
    mask: int,
) -> list[str]:
    issues = list(result.firmware_errors)
    if not result.config:
        issues.append("SEARCH_RUN_START is missing")
    if result.result == "missing":
        issues.append("SEARCH_RUN_END is missing")
    if result.result != "goal":
        issues.append(f"firmware result is {result.result!r}, expected 'goal'")
    if result.reported_steps != sum(1 for step in result.steps if step.found and not step.selected_vwall):
        issues.append(
            f"step count mismatch: end={result.reported_steps}, records={len(result.steps)}"
        )

    for step in result.steps:
        if step.selected_vwall:
            issues.append(f"step {step.index}: selected edge was closed by the following update")
        if not step.found:
            issues.append(f"step {step.index}: no candidate")
            continue
        if step.next_heading not in DIRECTION_INDEX:
            issues.append(f"step {step.index}: invalid next heading {step.next_heading!r}")
            continue
        direction_index = DIRECTION_INDEX[step.next_heading]
        dx, dy = DIRECTION_DELTA[step.next_heading]
        if (step.next_x, step.next_y) != (step.x + dx, step.y + dy):
            issues.append(f"step {step.index}: next coordinate does not match heading")
        if (step.wall_values[direction_index] & mask) != NOWALL:
            issues.append(f"step {step.index}: selected a physically closed edge")
        if step.virtual_values[direction_index] != 0:
            issues.append(f"step {step.index}: selected an existing virtual wall")

        candidates = [
            step.map_values[index]
            for index in range(4)
            if (step.wall_values[index] & mask) == NOWALL
            and step.virtual_values[index] == 0
            and step.map_values[index] < MAZE_SIZE
        ]
        if not candidates:
            issues.append(f"step {step.index}: selected without a finite map candidate")
        elif step.map_values[direction_index] != min(candidates):
            issues.append(
                f"step {step.index}: selected map={step.map_values[direction_index]}, min={min(candidates)}"
            )

    if len(result.map_rows) != HEIGHT:
        issues.append(f"map dump rows={len(result.map_rows)}, expected {HEIGHT}")
    if len(result.virtual_edges) != result.reported_virtual_edges:
        issues.append(
            f"virtual edge count mismatch: dump={len(result.virtual_edges)}, "
            f"end={result.reported_virtual_edges}"
        )

    gx, gy, goal_size = goal
    protected = {(start[0], start[1]), (result.final_x, result.final_y)}
    protected.update(
        (x, y)
        for x in range(gx, gx + goal_size)
        for y in range(gy, gy + goal_size)
    )
    for x, y, direction in result.virtual_edges:
        a, b = edge_endpoints(x, y, direction)
        if a in protected or b in protected:
            issues.append(f"virtual edge {(x, y, direction)} touches protected cell")
    return issues


def validate_replay_result(result: ReplayResult, mask: int) -> list[str]:
    issues = list(result.firmware_errors)
    if not result.config:
        issues.append("REPLAY_START is missing")
    if result.result == "missing":
        issues.append("REPLAY_END is missing")
    if result.result != "goal":
        issues.append(f"firmware result is {result.result!r}, expected 'goal'")
    if result.reported_steps != len(result.steps):
        issues.append(
            f"step count mismatch: end={result.reported_steps}, records={len(result.steps)}"
        )

    for step in result.steps:
        if step.selected_vwall:
            issues.append(f"step {step.index}: selected edge was closed by pre-motion update")
        if step.next_heading not in DIRECTION_INDEX:
            issues.append(f"step {step.index}: invalid next heading {step.next_heading!r}")
            continue
        direction_index = DIRECTION_INDEX[step.next_heading]
        dx, dy = DIRECTION_DELTA[step.next_heading]
        if (step.next_x, step.next_y) != (step.x + dx, step.y + dy):
            issues.append(f"step {step.index}: next coordinate does not match heading")
        if (step.wall_values[direction_index] & mask) != NOWALL:
            issues.append(f"step {step.index}: selected a closed observed edge")
        if step.virtual_values[direction_index] != 0:
            issues.append(f"step {step.index}: selected an existing virtual wall")
        if step.truth != NOWALL:
            issues.append(
                f"step {step.index}: selected edge truth={step.truth}; physical collision or missing truth"
            )

        candidates = [
            step.map_values[index]
            for index in range(4)
            if (step.wall_values[index] & mask) == NOWALL
            and step.virtual_values[index] == 0
            and step.map_values[index] < MAZE_SIZE
        ]
        if not candidates:
            issues.append(f"step {step.index}: selected without a finite map candidate")
        elif step.map_values[direction_index] != min(candidates):
            issues.append(
                f"step {step.index}: selected map={step.map_values[direction_index]}, "
                f"min={min(candidates)}"
            )

    if len(result.map_rows) != HEIGHT:
        issues.append(f"map dump rows={len(result.map_rows)}, expected {HEIGHT}")
    if len(result.virtual_edges) != result.reported_virtual_edges:
        issues.append(
            f"virtual edge count mismatch: dump={len(result.virtual_edges)}, "
            f"end={result.reported_virtual_edges}"
        )

    if result.config:
        start_parts = result.config.get("maze_start", result.config["start"]).split(",")
        goal_parts = result.config.get("maze_goal", result.config["goal"]).split(",")
        start_cell = (int(start_parts[0]), int(start_parts[1]))
        gx, gy, goal_size = map(int, goal_parts)
        protected = {start_cell, (result.final_x, result.final_y)}
        protected.update(
            (x, y)
            for x in range(gx, gx + goal_size)
            for y in range(gy, gy + goal_size)
        )
        for x, y, direction in result.virtual_edges:
            a, b = edge_endpoints(x, y, direction)
            if a in protected or b in protected:
                issues.append(f"virtual edge {(x, y, direction)} touches protected cell")
    return issues


def empty_known_maze() -> list[list[list[int]]]:
    walls = [[[UNKNOWN for _ in range(4)] for _ in range(HEIGHT)] for _ in range(WIDTH)]
    for x in range(WIDTH):
        set_edge(walls, x, 0, "S", WALL)
        set_edge(walls, x, HEIGHT - 1, "N", WALL)
    for y in range(HEIGHT):
        set_edge(walls, 0, y, "W", WALL)
        set_edge(walls, WIDTH - 1, y, "E", WALL)
    set_edge(walls, 0, 0, "E", WALL)
    return walls


def set_edge(walls: list[list[list[int]]], x: int, y: int, direction: str, state: int) -> None:
    walls[x][y][DIRECTION_INDEX[direction]] = state
    dx, dy = DIRECTION_DELTA[direction]
    nx, ny = x + dx, y + dy
    if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
        walls[nx][ny][DIRECTION_INDEX[OPPOSITE[direction]]] = state


def pack_walls(walls: list[list[list[int]]]) -> bytes:
    payload = bytearray()
    for y in range(HEIGHT - 1, -1, -1):
        for x in range(WIDTH):
            north, east, south, west = walls[x][y]
            payload.append(north | (east << 2) | (south << 4) | (west << 6))
    return bytes(payload)


def load_history(path: Path) -> bytes | None:
    walls = empty_known_maze()
    matches = 0
    for line in path.read_text(encoding="utf-8-sig", errors="ignore").splitlines():
        match = HISTORY_RE.search(line)
        if match is None:
            continue
        x, y, north, east, south, west = (int(value) for value in match.groups())
        if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
            raise ValueError(f"history coordinate out of range: {(x, y)}")
        for direction, state in zip(("N", "E", "S", "W"), (north, east, south, west)):
            set_edge(walls, x, y, direction, state)
        matches += 1
    return pack_walls(walls) if matches else None


def derive_square_goal(goal_cells: Iterable[tuple[int, int]]) -> tuple[int, int, int] | None:
    cells = set(goal_cells)
    if not cells:
        return None
    xs = sorted({x for x, _ in cells})
    ys = sorted({y for _, y in cells})
    if len(xs) != len(ys) or cells != {(x, y) for x in xs for y in ys}:
        raise ValueError("maze_data goal cells are not a square")
    return min(xs), min(ys), len(xs)


def load_maze_payload(
    path: Path,
) -> tuple[bytes, tuple[int, int, str] | None, tuple[int, int, int] | None]:
    raw = path.read_bytes()
    if path.suffix.lower() == ".bin":
        if len(raw) != WIDTH * HEIGHT:
            raise ValueError(f"binary maze must be {WIDTH * HEIGHT} bytes, got {len(raw)}")
        return raw, None, None

    history = load_history(path)
    if history is not None:
        return history, None, None

    # Some captures use a .txt extension even though they contain the raw
    # 1024-byte packed maze payload.
    if len(raw) == WIDTH * HEIGHT and any(value < 0x09 or value > 0x7E for value in raw):
        return raw, None, None

    from maze_io import MazeFileReader

    data = MazeFileReader.from_file(path)
    if data.x_cnt != WIDTH or data.y_cnt != HEIGHT:
        raise ValueError(f"maze_data must be {WIDTH}x{HEIGHT}")
    walls = [
        [
            [WALL if bool(data.maze_wall_data[x, y, index]) else NOWALL for index in range(4)]
            for y in range(HEIGHT)
        ]
        for x in range(WIDTH)
    ]
    start_cell = data.start_cells[0] if data.start_cells else (0, 0)
    return pack_walls(walls), (start_cell[0], start_cell[1], "N"), derive_square_goal(data.goal_cells)


def write_command(ser, command: str, char_delay: float) -> None:
    for char in command + "\r":
        ser.write(char.encode("ascii"))
        ser.flush()
        if char_delay > 0:
            time.sleep(char_delay)


def read_line(ser) -> str | None:
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode(errors="ignore").replace("\x00", "").strip()


def wait_for(ser, predicate, timeout: float, verbose: bool) -> tuple[str, list[str]]:
    deadline = time.monotonic() + timeout
    lines: list[str] = []
    while time.monotonic() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        lines.append(line)
        if verbose:
            print(line)
        if predicate(line):
            return line, lines
    raise TimeoutError(f"serial timeout; recent lines={lines[-12:]}")


def prepare_shell(ser, char_delay: float) -> None:
    ser.reset_input_buffer()
    write_command(ser, "", char_delay)
    time.sleep(0.3)
    ser.read(ser.in_waiting or 1)


def upload_payload(
    ser,
    payload: bytes,
    goal: tuple[int, int, int],
    timeout: float,
    char_delay: float,
    binary_delay: float,
    verbose: bool,
) -> None:
    write_command(ser, "load maze_bin", char_delay)
    wait_for(ser, lambda line: line.startswith("MAZE_BIN_READY"), timeout, verbose)
    for value in payload:
        ser.write(bytes((value,)))
        ser.flush()
        if binary_delay > 0:
            time.sleep(binary_delay)
    wait_for(ser, lambda line: line == "MAZE_BIN_LOAD_DONE", timeout, verbose)
    gx, gy, size = goal
    write_command(ser, f"load goal {gx} {gy} {size}", char_delay)
    wait_for(ser, lambda line: line.startswith("GOAL_SET_DONE"), timeout, verbose)


def run_serial(args, payload: bytes | None) -> list[str]:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required: pip install pyserial") from exc

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(0.2)
        prepare_shell(ser, args.char_delay)
        if payload is not None:
            upload_payload(
                ser,
                payload,
                args.goal,
                args.timeout,
                args.char_delay,
                args.binary_delay,
                args.verbose,
            )
        else:
            write_command(ser, "load save", args.char_delay)
            wait_for(ser, lambda line: line.startswith("read_save_data done"), args.timeout, args.verbose)
            gx, gy, size = args.goal
            write_command(ser, f"load goal {gx} {gy} {size}", args.char_delay)
            wait_for(ser, lambda line: line.startswith("GOAL_SET_DONE"), args.timeout, args.verbose)

        sx, sy, heading = args.start
        command = (
            f"search run {sx} {sy} {heading} {args.mode} {args.priority} "
            f"{args.mask} {args.max_steps}"
        )
        print(f"> {command}")
        write_command(ser, command, args.char_delay)
        terminal_line, lines = wait_for(
            ser,
            lambda line: line.startswith("SEARCH_DUMP_END ") or line.startswith("SEARCH_ERROR "),
            args.timeout,
            args.verbose,
        )
        if terminal_line.startswith("SEARCH_ERROR "):
            raise RuntimeError(terminal_line)
        return lines


def run_replay_command(
    ser,
    args,
    state: str,
    motion: str,
    start: tuple[int, int, str],
    goal: tuple[int, int, int],
    mode: str,
    priority: str,
) -> tuple[str, list[str]]:
    sx, sy, heading = start
    gx, gy, goal_size = goal
    command = (
        f"search replay {state} {motion} {sx} {sy} {heading} "
        f"{gx} {gy} {goal_size} {mode} {priority} {args.mask} {args.max_steps}"
    )
    print(f"> {command}")
    write_command(ser, command, args.char_delay)
    _, lines = wait_for(
        ser,
        lambda line: line.startswith("REPLAY_DUMP_END ")
        or line in ("REPLAY_ERROR invalid_argument", "REPLAY_ERROR keep_without_reset"),
        args.timeout,
        args.verbose,
    )
    return command, lines


def prepare_serial_truth(ser, args, payload: bytes | None) -> None:
    time.sleep(0.2)
    prepare_shell(ser, args.char_delay)
    if payload is not None:
        upload_payload(
            ser,
            payload,
            args.goal,
            args.timeout,
            args.char_delay,
            args.binary_delay,
            args.verbose,
        )
    else:
        write_command(ser, "load save", args.char_delay)
        wait_for(
            ser,
            lambda line: line.startswith("read_save_data done"),
            args.timeout,
            args.verbose,
        )


def save_replay_result(prefix: Path, command: str, lines: list[str], result: ReplayResult) -> None:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    prefix.with_suffix(".txt").write_text(
        f"> {command}\n" + "\n".join(lines) + "\n", encoding="utf-8"
    )
    with prefix.with_name(prefix.name + "_steps.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "index", "x", "y", "heading", "self", "map_n", "map_e", "map_s", "map_w",
                "wall_n", "wall_e", "wall_s", "wall_w", "vwall_n", "vwall_e", "vwall_s",
                "vwall_w", "sensed", "next_x", "next_y", "next_heading", "local", "truth",
                "selected_vwall", "next_acc", "virtual_edges",
            ]
        )
        for step in result.steps:
            writer.writerow(
                [
                    step.index, step.x, step.y, step.heading, step.self_value,
                    *step.map_values, *step.wall_values, *step.virtual_values,
                    int(step.sensed), step.next_x, step.next_y, step.next_heading,
                    step.local, step.truth, int(step.selected_vwall), step.next_acc,
                    step.virtual_edges,
                ]
            )


def run_replay_matrix_on_serial(
    ser,
    args,
    payload: bytes | None,
    prefix: Path,
    progress_callback: Callable[[int, int, str, ReplayResult, ReplayResult, list[str]], None] | None = None,
) -> list[str]:
    outward_variants = [(motion, priority) for motion in ("plain", "acc") for priority in ("first", "second")]
    return_variants = [
        (mode, motion, priority)
        for mode in ("goal", "full")
        for motion in ("plain", "acc")
        for priority in ("first", "second")
    ]
    summary: list[list[object]] = []
    all_issues: list[str] = []
    prefix.parent.mkdir(parents=True, exist_ok=True)

    prepare_serial_truth(ser, args, payload)
    scenario_index = 0
    for out_motion, out_priority in outward_variants:
        for ret_mode, ret_motion, ret_priority in return_variants:
            scenario_name = (
                f"{scenario_index:02d}_out_goal_{out_motion}_{out_priority}"
                f"__ret_{ret_mode}_{ret_motion}_{ret_priority}"
            )
            scenario_prefix = prefix.with_name(prefix.name + "_" + scenario_name)

            out_command, out_lines = run_replay_command(
                ser, args, "reset", out_motion, args.start, args.goal, "goal", out_priority
            )
            outward = parse_replay_lines(out_lines)
            out_issues = validate_replay_result(outward, args.mask)
            save_replay_result(
                scenario_prefix.with_name(scenario_prefix.name + "_out"),
                out_command,
                out_lines,
                outward,
            )

            return_result = ReplayResult(result="skipped")
            ret_issues: list[str] = ["return skipped because outward failed"] if out_issues else []
            if not out_issues:
                return_start = (outward.final_x, outward.final_y, outward.final_heading)
                ret_command, ret_lines = run_replay_command(
                    ser,
                    args,
                    "keep",
                    ret_motion,
                    return_start,
                    args.return_goal,
                    ret_mode,
                    ret_priority,
                )
                return_result = parse_replay_lines(ret_lines)
                ret_issues = validate_replay_result(return_result, args.mask)
                save_replay_result(
                    scenario_prefix.with_name(scenario_prefix.name + "_return"),
                    ret_command,
                    ret_lines,
                    return_result,
                )

            issues = [f"outward: {issue}" for issue in out_issues]
            issues.extend(f"return: {issue}" for issue in ret_issues)
            all_issues.extend(f"{scenario_name}: {issue}" for issue in issues)
            summary.append(
                [
                    scenario_name,
                    outward.result,
                    outward.reported_steps,
                    f"{outward.final_x},{outward.final_y},{outward.final_heading}",
                    outward.reported_virtual_edges,
                    return_result.result,
                    return_result.reported_steps,
                    f"{return_result.final_x},{return_result.final_y},{return_result.final_heading}",
                    return_result.reported_virtual_edges,
                    "OK" if not issues else "FAILED",
                    " | ".join(issues),
                ]
            )
            print(
                f"[{scenario_index + 1:02d}/32] {scenario_name}: "
                f"out={outward.result} return={return_result.result} "
                f"verification={'OK' if not issues else 'FAILED'}"
            )
            scenario_index += 1
            if progress_callback is not None:
                progress_callback(
                    scenario_index, 32, scenario_name, outward, return_result, issues
                )

    summary_path = prefix.with_name(prefix.name + "_summary.csv")
    with summary_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "scenario", "out_result", "out_steps", "out_final", "out_virtual_edges",
                "return_result", "return_steps", "return_final", "return_virtual_edges",
                "verification", "issues",
            ]
        )
        writer.writerows(summary)
    print(f"summary={summary_path}")
    return all_issues


def run_replay_matrix(args, payload: bytes | None, prefix: Path) -> list[str]:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required: pip install pyserial") from exc

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        return run_replay_matrix_on_serial(ser, args, payload, prefix)


def save_result(prefix: Path, lines: list[str], result: SearchResult) -> None:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    prefix.with_suffix(".txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    with prefix.with_name(prefix.name + "_steps.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "index", "x", "y", "heading", "self", "map_n", "map_e", "map_s", "map_w",
                "wall_n", "wall_e", "wall_s", "wall_w", "vwall_n", "vwall_e", "vwall_s",
                "vwall_w", "next_x", "next_y", "next_heading", "local", "found",
                "selected_vwall", "virtual_edges",
            ]
        )
        for step in result.steps:
            writer.writerow(
                [
                    step.index, step.x, step.y, step.heading, step.self_value,
                    *step.map_values, *step.wall_values, *step.virtual_values,
                    step.next_x, step.next_y, step.next_heading, step.local,
                    int(step.found), int(step.selected_vwall), step.virtual_edges,
                ]
            )
    with prefix.with_name(prefix.name + "_map.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        for y in range(HEIGHT - 1, -1, -1):
            writer.writerow([y, *result.map_rows.get(y, [])])
    with prefix.with_name(prefix.name + "_virtual.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["x", "y", "direction"])
        writer.writerows(result.virtual_edges)


def plot_result(
    output: Path,
    result: SearchResult,
    payload: bytes | None,
    goal: tuple[int, int, int],
) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.patches import Rectangle
    except ImportError:
        print("plot skipped: install matplotlib and numpy")
        return

    grid = np.full((HEIGHT, WIDTH), np.nan)
    for y, values in result.map_rows.items():
        grid[y, :] = [np.nan if value >= MAZE_SIZE else value for value in values]
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(grid, origin="lower", extent=(0, WIDTH, 0, HEIGHT), cmap="viridis_r", alpha=0.65)

    if payload is not None:
        index = 0
        states = [[[UNKNOWN] * 4 for _ in range(HEIGHT)] for _ in range(WIDTH)]
        for y in range(HEIGHT - 1, -1, -1):
            for x in range(WIDTH):
                value = payload[index]
                index += 1
                states[x][y] = [value & 3, (value >> 2) & 3, (value >> 4) & 3, (value >> 6) & 3]
        for x in range(WIDTH):
            for y in range(HEIGHT):
                if states[x][y][0] in (WALL, VWALL):
                    ax.plot([x, x + 1], [y + 1, y + 1], color="black", linewidth=0.7)
                if states[x][y][1] in (WALL, VWALL):
                    ax.plot([x + 1, x + 1], [y, y + 1], color="black", linewidth=0.7)
                if y == 0 and states[x][y][2] in (WALL, VWALL):
                    ax.plot([x, x + 1], [y, y], color="black", linewidth=0.7)
                if x == 0 and states[x][y][3] in (WALL, VWALL):
                    ax.plot([x, x], [y, y + 1], color="black", linewidth=0.7)

    for x, y, direction in result.virtual_edges:
        if direction == "N":
            ax.plot([x, x + 1], [y + 1, y + 1], color="red", linewidth=2.0)
        else:
            ax.plot([x + 1, x + 1], [y, y + 1], color="red", linewidth=2.0)
    path = [(step.x + 0.5, step.y + 0.5) for step in result.steps]
    path.append((result.final_x + 0.5, result.final_y + 0.5))
    if path:
        ax.plot([p[0] for p in path], [p[1] for p in path], color="cyan", linewidth=1.5, marker="o", markersize=2)
    gx, gy, size = goal
    ax.add_patch(Rectangle((gx, gy), size, size, fill=False, edgecolor="lime", linewidth=2.0))
    ax.set_xlim(0, WIDTH)
    ax.set_ylim(0, HEIGHT)
    ax.set_aspect("equal")
    ax.set_title(f"myshell Search: {result.result}, steps={result.reported_steps}, virtual={len(result.virtual_edges)}")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--maze", type=Path, help=".bin, wall_histry .txt, or standard maze_data .txt")
    parser.add_argument("--transcript", type=Path, help="parse an existing SEARCH_* transcript without serial")
    parser.add_argument("--goal", type=parse_goal, default=(7, 7, 2))
    parser.add_argument("--return-goal", type=parse_goal, default=(0, 0, 1))
    parser.add_argument("--start", type=parse_start, default=(0, 0, "N"))
    parser.add_argument("--mode", choices=("goal", "full"), default="goal")
    parser.add_argument("--priority", choices=("first", "second"), default="first")
    parser.add_argument("--mask", type=int, choices=(1, 3), default=1)
    parser.add_argument("--max-steps", type=int, default=256)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--char-delay", type=float, default=0.02)
    parser.add_argument("--binary-delay", type=float, default=0.002)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument(
        "--replay-matrix",
        action="store_true",
        help="run all goal outward x (goal/full) return x plain/acc x first/second combinations",
    )
    parser.add_argument("--output", type=Path, help="output prefix; defaults under tools/logs")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    payload = None
    if args.maze is not None:
        payload, inferred_start, inferred_goal = load_maze_payload(args.maze)
        if args.start == (0, 0, "N") and inferred_start is not None:
            args.start = inferred_start
        if args.goal == (7, 7, 2) and inferred_goal is not None:
            args.goal = inferred_goal

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = args.output or Path(__file__).resolve().parent / "logs" / f"{stamp}_myshell_search"

    if args.replay_matrix:
        if args.transcript is not None:
            raise ValueError("--replay-matrix cannot be combined with --transcript")
        issues = run_replay_matrix(args, payload, prefix)
        if issues:
            print("matrix_verification=FAILED")
            for issue in issues:
                print(f"  - {issue}")
            return 1
        print("matrix_verification=OK scenarios=32 legs=64")
        return 0

    if args.transcript is not None:
        lines = args.transcript.read_text(encoding="utf-8", errors="ignore").splitlines()
    else:
        lines = run_serial(args, payload)

    result = parse_search_lines(lines)
    issues = validate_result(result, args.start, args.goal, args.mask)
    save_result(prefix, lines, result)
    if not args.no_plot:
        plot_result(prefix.with_suffix(".png"), result, payload, args.goal)

    print(
        f"result={result.result} steps={result.reported_steps} "
        f"final=({result.final_x},{result.final_y},{result.final_heading}) "
        f"virtual_edges={len(result.virtual_edges)}"
    )
    print(f"saved={prefix.parent / prefix.name}")
    if issues:
        print("verification=FAILED")
        for issue in issues:
            print(f"  - {issue}")
        return 1
    print("verification=OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
