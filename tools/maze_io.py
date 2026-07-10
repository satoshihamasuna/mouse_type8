"""Maze text-file I/O for the micromouse simulator.

This module handles the original text maze format used by the simulator.
Maze wall storage and drawing are handled by :mod:`maze`.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Iterable, List, Sequence, Tuple

import numpy as np


_TRUE_VALUES = {"1", "true", "t", "yes", "y"}
_FALSE_VALUES = {"0", "false", "f", "no", "n"}
DEFAULT_MAZE_DIR = "maze_data"
DEFAULT_EXTENSION = ".txt"


Cell = Tuple[int, int]


def _to_bool(text: str) -> bool:
    value = text.strip().lower()
    if value in _TRUE_VALUES:
        return True
    if value in _FALSE_VALUES:
        return False
    raise ValueError(f"Invalid boolean token in maze file: {text!r}")


def _split_csv(line: str) -> List[str]:
    """Split a CSV-like line and remove empty trailing fields."""
    parts = [part.strip() for part in line.strip().split(",")]
    while parts and parts[-1] == "":
        parts.pop()
    return parts


def resolve_maze_path(
    name: str | Path,
    maze_dir: str | Path = DEFAULT_MAZE_DIR,
    extension: str = DEFAULT_EXTENSION,
    for_save: bool = False,
) -> Path:
    """Resolve a maze name to a file path.

    Examples
    --------
    ``39_AllJapan_Final_Half`` -> ``./maze_data/39_AllJapan_Final_Half.txt``
    ``foo.txt`` -> ``./foo.txt``
    ``C:/.../foo.txt`` -> absolute path unchanged
    """
    path = Path(name)
    if path.suffix:
        resolved = path
    else:
        resolved = Path.cwd() / maze_dir / f"{name}{extension}"

    if for_save:
        resolved.parent.mkdir(parents=True, exist_ok=True)

    return resolved


@dataclass(frozen=True)
class MazeFileData:
    """Parsed maze file data."""

    filename: Path
    x_cnt: int
    y_cnt: int
    maze_wall_data: np.ndarray
    sx: np.ndarray
    sy: np.ndarray
    gx: np.ndarray
    gy: np.ndarray
    start_cells: tuple[Cell, ...]
    goal_cells: tuple[Cell, ...]

    def to_maze(self):
        """Create a :class:`maze.Maze` instance using the parsed wall data."""
        from maze import Maze

        maze = Maze(self.x_cnt, self.y_cnt)
        if hasattr(maze, "load_edit_maze"):
            maze.load_edit_maze(self.maze_wall_data)
        else:
            maze.wall = self.maze_wall_data.copy()
        return maze


class MazeFileReader:
    """Read the original micromouse text maze format.

    The original code exposed attributes such as ``x_cnt``, ``maze_wall_data``,
    ``sx`` and ``gx`` directly from ``read_maze``. Those attributes are kept
    here for compatibility. Exact start/goal cell lists are also exposed as
    ``start_cells`` and ``goal_cells``.
    """

    MARKER = "Maze_Start_Goal_Data"

    def __init__(self, name: str | Path, maze_dir: str | Path = DEFAULT_MAZE_DIR, extension: str = DEFAULT_EXTENSION):
        filename = resolve_maze_path(name, maze_dir=maze_dir, extension=extension)
        data = self.from_file(filename)
        self.data = data

        self.filename = str(data.filename)
        self.x_cnt = data.x_cnt
        self.y_cnt = data.y_cnt
        self.maze_wall_data = data.maze_wall_data
        self.sx = data.sx
        self.sy = data.sy
        self.gx = data.gx
        self.gy = data.gy
        self.start_cells = list(data.start_cells)
        self.goal_cells = list(data.goal_cells)

        print(f"file -> {self.filename}")
        print(self.sx, self.sy, self.gx, self.gy)

    @classmethod
    def from_file(cls, filename: str | Path) -> MazeFileData:
        filename = Path(filename)
        lines = filename.read_text(encoding="utf-8-sig").splitlines()

        try:
            marker_index = lines.index(cls.MARKER)
        except ValueError as exc:
            raise ValueError(f"{cls.MARKER!r} was not found in {filename}") from exc

        wall_lines = cls._extract_wall_lines(lines[:marker_index])
        if not wall_lines:
            raise ValueError(f"No wall data was found in {filename}")

        x_cnt = len(wall_lines[0]) // 4
        y_cnt = len(wall_lines)
        wall_data = np.zeros((x_cnt, y_cnt, 4), dtype=bool)

        for yy, row in enumerate(wall_lines):
            if len(row) != x_cnt * 4:
                raise ValueError(
                    f"Inconsistent wall row length at y={yy}: "
                    f"expected {x_cnt * 4}, got {len(row)}"
                )
            for xx in range(x_cnt):
                for direction in range(4):
                    wall_data[xx, yy, direction] = _to_bool(row[xx * 4 + direction])

        start_cells, goal_cells = cls._extract_start_goal_cells(lines[marker_index + 1 :])
        sx, sy = _cells_to_axis_arrays(start_cells, default=(0, 0))
        gx, gy = _cells_to_axis_arrays(goal_cells, default=None)

        return MazeFileData(
            filename=filename,
            x_cnt=x_cnt,
            y_cnt=y_cnt,
            maze_wall_data=wall_data,
            sx=sx,
            sy=sy,
            gx=gx,
            gy=gy,
            start_cells=tuple(start_cells),
            goal_cells=tuple(goal_cells),
        )

    @staticmethod
    def _extract_wall_lines(lines: Sequence[str]) -> List[List[str]]:
        wall_rows: List[List[str]] = []
        for line in lines:
            parts = _split_csv(line)
            if not parts:
                continue
            if len(parts) % 4 != 0:
                continue
            try:
                for token in parts:
                    _to_bool(token)
            except ValueError:
                continue
            wall_rows.append(parts)
        return wall_rows

    @staticmethod
    def _extract_start_goal_cells(lines: Iterable[str]) -> tuple[list[Cell], list[Cell]]:
        start_cells: set[Cell] = set()
        goal_cells: set[Cell] = set()

        for line in lines:
            parts = _split_csv(line)
            if len(parts) < 3:
                continue
            try:
                x = int(parts[0])
                y = int(parts[1])
            except ValueError:
                continue

            mark = parts[2].strip().upper()
            if mark == "S":
                start_cells.add((x, y))
            elif mark == "G":
                goal_cells.add((x, y))

        if not start_cells:
            start_cells.add((0, 0))

        return sorted(start_cells), sorted(goal_cells)

    @staticmethod
    def _extract_start_goal(lines: Iterable[str]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Compatibility helper returning old-style sx, sy, gx, gy arrays."""
        start_cells, goal_cells = MazeFileReader._extract_start_goal_cells(lines)
        sx, sy = _cells_to_axis_arrays(start_cells, default=(0, 0))
        gx, gy = _cells_to_axis_arrays(goal_cells, default=None)
        return sx, sy, gx, gy


def _cells_to_axis_arrays(cells: Iterable[Cell], default: Cell | None) -> tuple[np.ndarray, np.ndarray]:
    normalized = sorted({(int(x), int(y)) for x, y in cells})
    if not normalized and default is not None:
        normalized = [default]
    xs = np.array(sorted({x for x, _ in normalized}), dtype=int)
    ys = np.array(sorted({y for _, y in normalized}), dtype=int)
    return xs, ys


def _normalize_cells(cells: Iterable[Cell] | None) -> list[Cell]:
    if cells is None:
        return []
    return sorted({(int(x), int(y)) for x, y in cells})


def _validate_cells(cells: Iterable[Cell], width: int, height: int, label: str) -> None:
    for x, y in cells:
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError(f"{label} cell ({x}, {y}) is outside maze size {width}x{height}")


def _make_backup(path: Path) -> Path | None:
    if not path.exists():
        return None
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_name(f"{path.stem}_{timestamp}.bak{path.suffix}")
    backup.write_bytes(path.read_bytes())
    return backup


def save_maze_file(
    name: str | Path,
    wall_data: np.ndarray,
    start_cells: Iterable[Cell] | None = None,
    goal_cells: Iterable[Cell] | None = None,
    maze_dir: str | Path = DEFAULT_MAZE_DIR,
    extension: str = DEFAULT_EXTENSION,
    backup: bool = True,
) -> Path:
    """Save a maze file in the original text format.

    The write is atomic: data is written to a temporary file first and then
    moved into place. If ``backup`` is True and the target file already exists,
    a timestamped backup is created before replacing it.
    """
    wall = np.asarray(wall_data, dtype=bool)
    if wall.ndim != 3 or wall.shape[2] != 4:
        raise ValueError(f"wall_data must have shape (x, y, 4), got {wall.shape}")

    width, height, _ = wall.shape
    start = _normalize_cells(start_cells) or [(0, 0)]
    goal = _normalize_cells(goal_cells)
    _validate_cells(start, width, height, "Start")
    _validate_cells(goal, width, height, "Goal")

    filename = resolve_maze_path(name, maze_dir=maze_dir, extension=extension, for_save=True)

    if backup:
        _make_backup(filename)

    lines: list[str] = ["Maze_Wall_Data"]
    for yy in range(height):
        row: list[str] = []
        for xx in range(width):
            for direction in range(4):
                row.append("True" if wall[xx, yy, direction] else "False")
        lines.append(",".join(row) + ",")

    lines.append("")
    lines.append("Maze_Start_Goal_Data")
    for x, y in start:
        lines.append(f"{x},{y},S")
    for x, y in goal:
        lines.append(f"{x},{y},G")
    lines.append("")

    text = "\n".join(lines)

    with NamedTemporaryFile("w", encoding="utf-8", dir=filename.parent, delete=False, newline="\n") as tmp:
        tmp.write(text)
        tmp_path = Path(tmp.name)

    tmp_path.replace(filename)
    return filename


# Backward-compatible name used by the original search.py.
class read_maze(MazeFileReader):
    pass
