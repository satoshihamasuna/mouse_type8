"""Host-side simulator for the Subsys micromouse search code.

The simulator keeps three wall layers separate:

* truth: walls from a maze_data text file
* known: the firmware-style UNKNOWN/NOWALL/WALL sensor map
* virtual: walls inferred by virtual_wall_class

It intentionally supports both the legacy virtual-wall guard and the current
guard so regressions can be reproduced without running the robot.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import deque
from dataclasses import asdict, dataclass
from enum import IntEnum
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from maze_io import MazeFileReader


UNKNOWN = 2
NOWALL = 0
WALL = 1
VWALL = 3


class Direction(IntEnum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6


DIRECTIONS = (Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST)
DIR_INDEX = {
    Direction.NORTH: 0,
    Direction.EAST: 1,
    Direction.SOUTH: 2,
    Direction.WEST: 3,
}
DELTA = {
    Direction.NORTH: (0, 1),
    Direction.EAST: (1, 0),
    Direction.SOUTH: (0, -1),
    Direction.WEST: (-1, 0),
}
DIR_NAME = {
    Direction.NORTH: "N",
    Direction.EAST: "E",
    Direction.SOUTH: "S",
    Direction.WEST: "W",
}
LOCAL_NAME = {0: "Front", 2: "Right", 4: "Rear", 6: "Left"}


def opposite(direction: Direction) -> Direction:
    return Direction((int(direction) + 4) % 8)


@dataclass(frozen=True)
class Position:
    x: int
    y: int
    direction: Direction = Direction.NORTH


@dataclass(frozen=True)
class VirtualContext:
    start: Position
    mouse: Position
    goal_x: int
    goal_y: int
    goal_size: int


@dataclass
class Selection:
    found: bool
    relative_direction: int
    next_position: Position
    map_value: int


@dataclass
class StepRecord:
    step: int
    x: int
    y: int
    heading: str
    selected: str
    next_x: int
    next_y: int
    found: bool
    known: str
    virtual: str
    neighbor_map: str
    virtual_count: int
    event: str


@dataclass
class SimulationResult:
    maze: str
    virtual_enabled: bool
    guard: str
    timing: str
    map_mode: str
    goal_x: int
    goal_y: int
    goal_size: int
    features: str
    success: bool
    reason: str
    steps: int
    final_position: Position
    records: list[StepRecord]
    wall_model: "WallModel | None" = None


class WallModel:
    def __init__(self, truth: np.ndarray):
        if truth.ndim != 3 or truth.shape[2] != 4:
            raise ValueError(f"truth wall shape must be (x, y, 4), got {truth.shape}")
        self.truth = np.asarray(truth, dtype=bool)
        self.width, self.height, _ = self.truth.shape
        self.known = np.full(self.truth.shape, UNKNOWN, dtype=np.uint8)
        self.virtual = np.zeros(self.truth.shape, dtype=bool)
        self.virtual_reason = np.full(self.truth.shape, "", dtype=object)
        self._initialize_known_walls()

    @classmethod
    def from_packed_snapshot(cls, data: bytes, width: int = 32, height: int = 32) -> "WallModel":
        expected = width * height
        if len(data) < expected:
            raise ValueError(f"binary snapshot has {len(data)} bytes; expected {expected}")
        known = np.full((width, height, 4), UNKNOWN, dtype=np.uint8)
        index = 0
        for y in range(height - 1, -1, -1):
            for x in range(width):
                value = data[index]
                index += 1
                known[x, y, 0] = value & 0x03
                known[x, y, 1] = (value >> 2) & 0x03
                known[x, y, 2] = (value >> 4) & 0x03
                known[x, y, 3] = (value >> 6) & 0x03

        # A binary log is a belief snapshot rather than a complete truth maze.
        # Known WALL/VWALL edges are definite walls; UNKNOWN is treated as open
        # only for the purpose of exercising the firmware's mask=0x01 planner.
        truth = np.logical_or(known == WALL, known == VWALL)
        model = cls(truth)
        model.known[:, :, :] = known
        return model

    def _initialize_known_walls(self) -> None:
        for x in range(self.width):
            self._set_known(x, 0, Direction.SOUTH, WALL)
            self._set_known(x, self.height - 1, Direction.NORTH, WALL)
        for y in range(self.height):
            self._set_known(0, y, Direction.WEST, WALL)
            self._set_known(self.width - 1, y, Direction.EAST, WALL)
        if self.width >= 2:
            self._set_known(0, 0, Direction.EAST, WALL)

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def neighbor(self, x: int, y: int, direction: Direction) -> tuple[int, int] | None:
        dx, dy = DELTA[direction]
        nx, ny = x + dx, y + dy
        return (nx, ny) if self.in_bounds(nx, ny) else None

    def _set_known(self, x: int, y: int, direction: Direction, state: int) -> None:
        self.known[x, y, DIR_INDEX[direction]] = state
        adjacent = self.neighbor(x, y, direction)
        if adjacent is not None:
            nx, ny = adjacent
            self.known[nx, ny, DIR_INDEX[opposite(direction)]] = state

    def sense(self, position: Position) -> None:
        """Reveal all four physical walls at one logical mouse position."""
        for direction in DIRECTIONS:
            state = WALL if self.truth[position.x, position.y, DIR_INDEX[direction]] else NOWALL
            self._set_known(position.x, position.y, direction, state)

    def physical_state(self, x: int, y: int, direction: Direction) -> int:
        if not self.in_bounds(x, y):
            return WALL
        return int(self.known[x, y, DIR_INDEX[direction]])

    def truth_has_wall(self, x: int, y: int, direction: Direction) -> bool:
        if not self.in_bounds(x, y):
            return True
        return bool(self.truth[x, y, DIR_INDEX[direction]])

    def clear_virtual_wall(self) -> None:
        self.virtual.fill(False)
        self.virtual_reason.fill("")

    def clear_virtual_edge(self, x: int, y: int, direction: Direction) -> None:
        if not self.in_bounds(x, y):
            return
        index = DIR_INDEX[direction]
        self.virtual[x, y, index] = False
        self.virtual_reason[x, y, index] = ""
        adjacent = self.neighbor(x, y, direction)
        if adjacent is not None:
            nx, ny = adjacent
            reverse = DIR_INDEX[opposite(direction)]
            self.virtual[nx, ny, reverse] = False
            self.virtual_reason[nx, ny, reverse] = ""

    def get_virtual_wall(self, x: int, y: int, direction: Direction) -> bool:
        if not self.in_bounds(x, y):
            return True
        return bool(self.virtual[x, y, DIR_INDEX[direction]])

    def set_virtual_wall(self, x: int, y: int, direction: Direction, reason: str) -> bool:
        if not self.in_bounds(x, y) or self.get_virtual_wall(x, y, direction):
            return False
        self.virtual[x, y, DIR_INDEX[direction]] = True
        self.virtual_reason[x, y, DIR_INDEX[direction]] = reason
        adjacent = self.neighbor(x, y, direction)
        if adjacent is not None:
            nx, ny = adjacent
            reverse = DIR_INDEX[opposite(direction)]
            self.virtual[nx, ny, reverse] = True
            self.virtual_reason[nx, ny, reverse] = reason
        return True

    def is_open(self, x: int, y: int, direction: Direction, mask: int) -> bool:
        if self.neighbor(x, y, direction) is None:
            return False
        if self.get_virtual_wall(x, y, direction):
            return False
        return (self.physical_state(x, y, direction) & mask) == NOWALL

    def is_unknown(self, x: int, y: int) -> bool:
        return bool(np.any(self.known[x, y, :] == UNKNOWN))

    def virtual_edge_count(self) -> int:
        count = 0
        for x in range(self.width):
            for y in range(self.height):
                if y + 1 < self.height and self.get_virtual_wall(x, y, Direction.NORTH):
                    count += 1
                if x + 1 < self.width and self.get_virtual_wall(x, y, Direction.EAST):
                    count += 1
        return count


class VirtualWallEngine:
    def __init__(
        self,
        walls: WallModel,
        enabled: bool,
        guard: str,
        features: set[str],
    ):
        self.walls = walls
        self.enabled = enabled
        self.guard = guard
        self.features = features

    def is_goal(self, x: int, y: int, context: VirtualContext) -> bool:
        return (
            context.goal_x <= x < context.goal_x + context.goal_size
            and context.goal_y <= y < context.goal_y + context.goal_size
        )

    def is_protected(self, x: int, y: int, context: VirtualContext) -> bool:
        protected = {
            (context.start.x, context.start.y),
            (context.mouse.x, context.mouse.y),
        }
        return (x, y) in protected or self.is_goal(x, y, context)

    def edge_touches_protected(
        self, x: int, y: int, direction: Direction, context: VirtualContext
    ) -> bool:
        if self.guard == "legacy":
            if self.is_goal(x, y, context):
                return True
            adjacent = self.walls.neighbor(x, y, direction)
            return adjacent is not None and self.is_goal(*adjacent, context)

        if self.is_protected(x, y, context):
            return True
        adjacent = self.walls.neighbor(x, y, direction)
        return adjacent is not None and self.is_protected(*adjacent, context)

    def set_wall(
        self,
        x: int,
        y: int,
        direction: Direction,
        context: VirtualContext,
        reason: str,
    ) -> bool:
        if self.edge_touches_protected(x, y, direction, context):
            return False
        return self.walls.set_virtual_wall(x, y, direction, reason)

    def clear_cell_walls(self, x: int, y: int) -> None:
        for direction in DIRECTIONS:
            self.walls.clear_virtual_edge(x, y, direction)

    def clear_protected_walls(self, context: VirtualContext) -> None:
        self.clear_cell_walls(context.start.x, context.start.y)
        for x in range(context.goal_x, context.goal_x + context.goal_size):
            for y in range(context.goal_y, context.goal_y + context.goal_size):
                self.clear_cell_walls(x, y)

    def add_pillar_walls(self, context: VirtualContext) -> bool:
        changed = False
        for px in range(1, self.walls.width):
            for py in range(1, self.walls.height):
                edges = (
                    (px, py, Direction.WEST),
                    (px, py, Direction.SOUTH),
                    (px - 1, py - 1, Direction.EAST),
                    (px - 1, py - 1, Direction.NORTH),
                )
                open_count = 0
                unknown_edge: tuple[int, int, Direction] | None = None
                for x, y, direction in edges:
                    state = self.walls.physical_state(x, y, direction)
                    if state == NOWALL:
                        open_count += 1
                    elif state == UNKNOWN:
                        unknown_edge = (x, y, direction)
                if open_count == 3 and unknown_edge is not None:
                    x, y, direction = unknown_edge
                    if self.set_wall(x, y, direction, context, "pillar"):
                        changed = True
        return changed

    def add_dead_end_walls(self, context: VirtualContext) -> bool:
        changed = False
        for x in range(self.walls.width):
            for y in range(self.walls.height):
                if self.is_protected(x, y, context):
                    continue
                blocked = 0
                remaining: Direction | None = None
                for direction in DIRECTIONS:
                    state = self.walls.physical_state(x, y, direction)
                    if state in (WALL, VWALL) or self.walls.get_virtual_wall(x, y, direction):
                        blocked += 1
                    else:
                        remaining = direction
                if blocked == 3 and remaining is not None:
                    if self.set_wall(x, y, remaining, context, "dead_end"):
                        changed = True
        return changed

    def add_explored_branch_walls(
        self, context: VirtualContext, unknown_open: bool = False
    ) -> bool:
        """Close a single-entry branch.

        In the normal mode UNKNOWN edges are not traversable, so the branch
        must be fully observed.  ``unknown_open`` mirrors the firmware's
        aggressive full-exploration mode: UNKNOWN is treated as an open edge
        while looking for bridges, and only VWALL/physical WALL blocks a
        branch.  This intentionally trades the all-cell guarantee for fewer
        return steps.
        """
        width = self.walls.width
        height = self.walls.height
        count = width * height
        discover = [0] * count
        low = [0] * count
        parent = [-1] * count
        order = [0] * count
        next_dir = [0] * count
        flags = [0] * count
        protected_flag = 0x01
        incomplete_flag = 0x02
        bridge_flag = 0x04
        detached_flag = 0x08

        def index(x: int, y: int) -> int:
            return y * width + x

        def position(value: int) -> tuple[int, int]:
            return value % width, value // width

        def initial_flags(x: int, y: int) -> int:
            result = protected_flag if self.is_protected(x, y, context) else 0
            states = self.walls.known[x, y, :]
            incomplete_states = states == VWALL
            if not unknown_open:
                incomplete_states = np.logical_or(incomplete_states, states == UNKNOWN)
            if np.any(incomplete_states):
                result |= incomplete_flag
            return result

        discovered = 0
        for root_x in range(width):
            for root_y in range(height):
                if not self.is_protected(root_x, root_y, context):
                    continue
                root = index(root_x, root_y)
                if discover[root] != 0:
                    continue
                discovered += 1
                discover[root] = low[root] = discovered
                order[discovered - 1] = root
                flags[root] = initial_flags(root_x, root_y)
                stack = [root]

                while stack:
                    vertex = stack[-1]
                    x, y = position(vertex)
                    if next_dir[vertex] < len(DIRECTIONS):
                        direction = DIRECTIONS[next_dir[vertex]]
                        next_dir[vertex] += 1
                        adjacent = self.walls.neighbor(x, y, direction)
                        if (
                            adjacent is None
                            or (
                                self.walls.physical_state(x, y, direction) != NOWALL
                                and not (
                                    unknown_open
                                    and self.walls.physical_state(x, y, direction)
                                    == UNKNOWN
                                )
                            )
                        ):
                            continue
                        nx, ny = adjacent
                        next_vertex = index(nx, ny)
                        if discover[next_vertex] == 0:
                            parent[next_vertex] = vertex
                            discovered += 1
                            discover[next_vertex] = low[next_vertex] = discovered
                            order[discovered - 1] = next_vertex
                            flags[next_vertex] = initial_flags(nx, ny)
                            stack.append(next_vertex)
                        elif parent[vertex] != next_vertex:
                            low[vertex] = min(low[vertex], discover[next_vertex])
                    else:
                        stack.pop()
                        parent_vertex = parent[vertex]
                        if parent_vertex >= 0:
                            if low[vertex] > discover[parent_vertex]:
                                flags[vertex] |= bridge_flag
                            low[parent_vertex] = min(low[parent_vertex], low[vertex])
                            flags[parent_vertex] |= flags[vertex] & (
                                protected_flag | incomplete_flag
                            )

        changed = False
        for order_index in range(discovered):
            vertex = order[order_index]
            parent_vertex = parent[vertex]
            if parent_vertex < 0:
                continue
            if flags[parent_vertex] & detached_flag:
                flags[vertex] |= detached_flag
                continue
            if not flags[vertex] & bridge_flag or flags[vertex] & (
                protected_flag | incomplete_flag
            ):
                continue
            x, y = position(vertex)
            px, py = position(parent_vertex)
            dx, dy = x - px, y - py
            direction = next(
                direction
                for direction in DIRECTIONS
                if DELTA[direction] == (dx, dy)
            )
            if self.walls.get_virtual_wall(px, py, direction):
                flags[vertex] |= detached_flag
            elif self.set_wall(
                px, py, direction, context, "explored_branch"
            ):
                flags[vertex] |= detached_flag
                changed = True
        return changed

    def update(self, context: VirtualContext, unknown_open: bool = False) -> None:
        if not self.enabled:
            self.walls.clear_virtual_wall()
            return
        self.clear_protected_walls(context)
        if "pillar" in self.features:
            self.add_pillar_walls(context)
        if "dead_end" in self.features:
            self.add_dead_end_walls(context)
        if "branch" in self.features:
            self.add_explored_branch_walls(context, unknown_open=unknown_open)


def make_step_map(
    walls: WallModel,
    goal_x: int,
    goal_y: int,
    goal_size: int,
    mask: int,
) -> np.ndarray:
    maximum = walls.width * walls.height
    step_map = np.full((walls.width, walls.height), maximum, dtype=np.int32)
    queue: deque[tuple[int, int]] = deque()
    for x in range(goal_x, goal_x + goal_size):
        for y in range(goal_y, goal_y + goal_size):
            if walls.in_bounds(x, y):
                step_map[x, y] = 0
                queue.append((x, y))
    while queue:
        x, y = queue.popleft()
        for direction in DIRECTIONS:
            if not walls.is_open(x, y, direction, mask):
                continue
            adjacent = walls.neighbor(x, y, direction)
            if adjacent is None:
                continue
            nx, ny = adjacent
            if step_map[nx, ny] == maximum:
                step_map[nx, ny] = step_map[x, y] + 1
                queue.append((nx, ny))
    return step_map


def make_full_search_map(
    walls: WallModel,
    target: Position,
    goal_x: int,
    goal_y: int,
    goal_size: int,
    mask: int,
) -> tuple[np.ndarray, bool]:
    """Reproduce make_map_queue_zenmen; bool indicates goal-map fallback."""
    maximum = walls.width * walls.height
    step_map = np.full((walls.width, walls.height), maximum, dtype=np.int32)
    queue: deque[tuple[int, int]] = deque()
    for x in range(walls.width):
        for y in range(walls.height):
            if walls.is_unknown(x, y):
                step_map[x, y] = 0
                queue.append((x, y))

    while queue:
        x, y = queue.popleft()
        for direction in DIRECTIONS:
            if not walls.is_open(x, y, direction, mask):
                continue
            adjacent = walls.neighbor(x, y, direction)
            if adjacent is None:
                continue
            nx, ny = adjacent
            if step_map[nx, ny] == maximum:
                step_map[nx, ny] = step_map[x, y] + 1
                queue.append((nx, ny))
        if x == target.x and y == target.y:
            break

    if step_map[target.x, target.y] == maximum:
        return make_step_map(walls, goal_x, goal_y, goal_size, mask), True
    return step_map, False


def direction_priority(walls: WallModel, current: Position, candidate: Position) -> int:
    if current.direction == candidate.direction:
        priority = 2
    elif (8 + int(current.direction) - int(candidate.direction)) % 8 == 4:
        priority = 0
    else:
        priority = 1
    if walls.is_unknown(candidate.x, candidate.y):
        priority += 4
    return priority


def select_next(
    walls: WallModel,
    step_map: np.ndarray,
    current: Position,
    stale_next: Position,
    mask: int,
) -> Selection:
    """Reproduce adachi::select_next_direction, including no-candidate behavior."""
    little = walls.width * walls.height
    priority = 0
    selected: Position | None = None
    for direction in DIRECTIONS:
        if not walls.is_open(current.x, current.y, direction, mask):
            continue
        adjacent = walls.neighbor(current.x, current.y, direction)
        if adjacent is None:
            continue
        nx, ny = adjacent
        candidate = Position(nx, ny, direction)
        candidate_priority = direction_priority(walls, current, candidate)
        value = int(step_map[nx, ny])
        if value < little:
            little = value
            selected = candidate
            priority = candidate_priority
        elif value == little and priority < candidate_priority:
            selected = candidate
            priority = candidate_priority

    # This is deliberately faithful to the firmware: when nothing was chosen,
    # glob_next_pos still contains its value from the preceding call.
    next_position = selected if selected is not None else stale_next
    relative = (8 + int(next_position.direction) - int(current.direction)) % 8
    return Selection(selected is not None, relative, next_position, little)


def derive_goal(goal_cells: Sequence[tuple[int, int]]) -> tuple[int, int, int]:
    if not goal_cells:
        raise ValueError("maze file has no goal cells; pass --goal X,Y,SIZE")
    xs = sorted({x for x, _ in goal_cells})
    ys = sorted({y for _, y in goal_cells})
    expected = {(x, y) for x in xs for y in ys}
    if len(xs) != len(ys) or expected != set(goal_cells):
        raise ValueError("goal cells are not a square; pass --goal X,Y,SIZE")
    return min(xs), min(ys), len(xs)


def compact_edge_values(
    walls: WallModel, step_map: np.ndarray, position: Position
) -> tuple[str, str, str]:
    known_parts: list[str] = []
    virtual_parts: list[str] = []
    map_parts: list[str] = []
    maximum = walls.width * walls.height
    for direction in DIRECTIONS:
        name = DIR_NAME[direction]
        known_parts.append(f"{name}:{walls.physical_state(position.x, position.y, direction)}")
        virtual_parts.append(
            f"{name}:{int(walls.get_virtual_wall(position.x, position.y, direction))}"
        )
        adjacent = walls.neighbor(position.x, position.y, direction)
        value = maximum if adjacent is None else int(step_map[adjacent[0], adjacent[1]])
        map_parts.append(f"{name}:{value}")
    return " ".join(known_parts), " ".join(virtual_parts), " ".join(map_parts)


def simulate(
    maze_path: Path,
    virtual_enabled: bool,
    guard: str,
    timing: str,
    map_mode: str,
    features: set[str],
    max_steps: int,
    mask: int = 0x01,
    goal_override: tuple[int, int, int] | None = None,
    start_override: Position | None = None,
    maze_start_override: Position | None = None,
    maze_goal_override: tuple[int, int, int] | None = None,
    snapshot_sensing: str = "frozen",
    branch_mode: str = "observed",
    initial_walls: "WallModel | None" = None,
) -> SimulationResult:
    # Firmware dumps are raw 1024-byte wall snapshots.  Some captured files
    # retain a .txt suffix, so detect the packed format by size as well.
    snapshot_mode = maze_path.suffix.lower() == ".bin" or maze_path.stat().st_size == 32 * 32
    if initial_walls is not None:
        walls = initial_walls
        snapshot_mode = False
        data = MazeFileReader.from_file(maze_path)
        intrinsic_goal = maze_goal_override or derive_goal(data.goal_cells)
        data_start_x, data_start_y = data.start_cells[0]
        intrinsic_start = maze_start_override or Position(
            data_start_x, data_start_y, Direction.NORTH
        )
    elif snapshot_mode:
        snapshot_walls = WallModel.from_packed_snapshot(maze_path.read_bytes())
        walls = (
            WallModel(snapshot_walls.truth)
            if snapshot_sensing == "replay-known"
            else snapshot_walls
        )
        intrinsic_goal = maze_goal_override or (7, 7, 2)
        intrinsic_start = maze_start_override or Position(0, 0, Direction.NORTH)
    else:
        data = MazeFileReader.from_file(maze_path)
        intrinsic_goal = maze_goal_override or derive_goal(data.goal_cells)
        data_start_x, data_start_y = data.start_cells[0]
        intrinsic_start = maze_start_override or Position(
            data_start_x, data_start_y, Direction.NORTH
        )
        walls = WallModel(data.maze_wall_data)
    goal_x, goal_y, goal_size = goal_override or intrinsic_goal
    maze_goal_x, maze_goal_y, maze_goal_size = intrinsic_goal
    start = start_override or intrinsic_start
    current = start
    tmp_next = start
    if snapshot_mode and snapshot_sensing == "replay-known":
        walls.sense(current)
    engine = VirtualWallEngine(walls, virtual_enabled, guard, features)
    context = VirtualContext(
        intrinsic_start, current, maze_goal_x, maze_goal_y, maze_goal_size
    )
    unknown_open = branch_mode == "unknown_open"
    engine.update(context, unknown_open=unknown_open)
    fallback_used = False

    def rebuild_map(target: Position) -> np.ndarray:
        nonlocal fallback_used
        if map_mode == "full":
            result, fallback = make_full_search_map(
                walls, target, goal_x, goal_y, goal_size, mask
            )
            fallback_used = fallback_used or fallback
            return result
        return make_step_map(walls, goal_x, goal_y, goal_size, mask)

    step_map = rebuild_map(tmp_next)
    records: list[StepRecord] = []

    reason = "max_steps"
    success = False
    for step in range(max_steps):
        if goal_x <= current.x < goal_x + goal_size and goal_y <= current.y < goal_y + goal_size:
            reason = "goal"
            success = True
            break

        if timing == "fresh" and step > 0:
            context = VirtualContext(
                intrinsic_start, current, maze_goal_x, maze_goal_y, maze_goal_size
            )
            engine.update(context, unknown_open=unknown_open)
            step_map = rebuild_map(current)

        known_text, virtual_text, map_text = compact_edge_values(walls, step_map, current)
        selection = select_next(walls, step_map, current, tmp_next, mask)
        event_parts: list[str] = []
        if not selection.found:
            event_parts.append("no_candidate_stale_next")

        selected_name = LOCAL_NAME.get(selection.relative_direction, f"Invalid({selection.relative_direction})")
        if selection.relative_direction not in LOCAL_NAME:
            event_parts.append("invalid_relative_direction")

        # Firmware timing rebuilds the map after choosing a direction and
        # before executing the motion.  The protected mouse position is still
        # the pre-motion current cell; expand_end is used only to stop the map
        # expansion at the selected destination.
        if timing == "firmware":
            context = VirtualContext(
                intrinsic_start, current, maze_goal_x, maze_goal_y, maze_goal_size
            )
            engine.update(context, unknown_open=unknown_open)
            step_map = rebuild_map(selection.next_position)

        selected_absolute = selection.next_position.direction
        if walls.get_virtual_wall(current.x, current.y, selected_absolute):
            reason_text = walls.virtual_reason[
                current.x, current.y, DIR_INDEX[selected_absolute]
            ]
            event_parts.append(f"selected_edge_closed_after_update:{reason_text}")

        collision = walls.truth_has_wall(current.x, current.y, selected_absolute)
        if collision:
            event_parts.append("collision_with_truth_wall")

        records.append(
            StepRecord(
                step=step,
                x=current.x,
                y=current.y,
                heading=DIR_NAME[current.direction],
                selected=selected_name,
                next_x=selection.next_position.x,
                next_y=selection.next_position.y,
                found=selection.found,
                known=known_text,
                virtual=virtual_text,
                neighbor_map=map_text,
                virtual_count=walls.virtual_edge_count(),
                event="|".join(event_parts),
            )
        )

        if collision:
            reason = "collision"
            break
        if not selection.found:
            reason = "no_candidate"
            break
        if selection.relative_direction not in LOCAL_NAME:
            reason = "invalid_direction"
            break

        current = selection.next_position
        tmp_next = selection.next_position
        if not snapshot_mode or snapshot_sensing in ("assume-open", "replay-known"):
            walls.sense(current)
    else:
        step = max_steps

    if goal_x <= current.x < goal_x + goal_size and goal_y <= current.y < goal_y + goal_size:
        reason = "goal"
        success = True

    return SimulationResult(
        maze=str(maze_path),
        virtual_enabled=virtual_enabled,
        guard=guard,
        timing=timing,
        map_mode=map_mode,
        goal_x=goal_x,
        goal_y=goal_y,
        goal_size=goal_size,
        features=",".join(sorted(features)) or "none",
        success=success,
        reason=f"{reason}+fallback" if fallback_used else reason,
        steps=len(records),
        final_position=current,
        records=records,
        wall_model=walls,
    )


def write_csv(path: Path, result: SimulationResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(result.records[0]).keys()))
        writer.writeheader()
        for record in result.records:
            writer.writerow(asdict(record))


def audit_snapshot_contexts(
    maze_path: Path,
    guard: str,
    features: set[str],
    goal: tuple[int, int, int],
) -> tuple[int, list[str]]:
    """Audit every plausible observed current->next context in a .bin snapshot."""
    walls = WallModel.from_packed_snapshot(maze_path.read_bytes())
    engine = VirtualWallEngine(walls, True, guard, features)
    start = Position(0, 0, Direction.NORTH)
    goal_x, goal_y, goal_size = goal
    contexts: list[tuple[Position, Position, Direction]] = []
    for x in range(walls.width):
        for y in range(walls.height):
            # A real search calls set_wall at the current cell before choosing,
            # so restrict the audit to cells whose four sides were observed.
            if walls.is_unknown(x, y):
                continue
            for direction in DIRECTIONS:
                if (walls.physical_state(x, y, direction) & 0x01) != NOWALL:
                    continue
                adjacent = walls.neighbor(x, y, direction)
                if adjacent is None:
                    continue
                nx, ny = adjacent
                contexts.append(
                    (
                        Position(x, y, direction),
                        Position(nx, ny, direction),
                        direction,
                    )
                )

    issues: list[str] = []
    for current, next_position, selected_direction in contexts:
        # Each audit case is independent rather than a continuous walk.
        walls.clear_virtual_wall()
        context = VirtualContext(start, current, goal_x, goal_y, goal_size)
        engine.update(context)
        if walls.get_virtual_wall(current.x, current.y, selected_direction):
            reason = walls.virtual_reason[
                current.x, current.y, DIR_INDEX[selected_direction]
            ]
            issues.append(
                f"current=({current.x},{current.y}) next=({next_position.x},{next_position.y}) "
                f"edge={DIR_NAME[selected_direction]} reason={reason}"
            )
    return len(contexts), issues


def print_result(result: SimulationResult, trace: bool) -> None:
    mode = "on" if result.virtual_enabled else "off"
    print(
        f"virtual={mode:<3} guard={result.guard:<6} timing={result.timing:<8} map={result.map_mode:<4} "
        f"features={result.features:<15} result={result.reason:<12} steps={result.steps:<4} "
        f"final=({result.final_position.x},{result.final_position.y},{DIR_NAME[result.final_position.direction]})"
    )
    if trace:
        for record in result.records:
            print(
                f"{record.step:03d} pos=({record.x:02d},{record.y:02d},{record.heading}) "
                f"select={record.selected:<6} next=({record.next_x:02d},{record.next_y:02d}) "
                f"found={int(record.found)} vw={record.virtual_count:03d} "
                f"[{record.virtual}] map[{record.neighbor_map}] {record.event}"
            )


def animation_positions(result: SimulationResult) -> list[tuple[float, float]]:
    """Return cell-center coordinates for every displayed animation frame."""
    positions = [(record.x + 0.5, record.y + 0.5) for record in result.records]
    final = (result.final_position.x + 0.5, result.final_position.y + 0.5)
    if not positions or positions[-1] != final:
        positions.append(final)
    return positions


def show_animation(results: Sequence[SimulationResult], interval_ms: int = 250) -> None:
    """Animate one or more simulation results on their physical mazes."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.collections import LineCollection
        from matplotlib.patches import Rectangle
    except ImportError as exc:
        raise RuntimeError(
            "animation requires matplotlib; install it with 'pip install matplotlib'"
        ) from exc

    if not results:
        return

    figure, axes = plt.subplots(
        1,
        len(results),
        figsize=(max(6, 5 * len(results)), 6),
        squeeze=False,
    )
    paths = [animation_positions(result) for result in results]
    artists: list[tuple[object, object, object]] = []

    for axis, result, positions in zip(axes[0], results, paths):
        walls = result.wall_model
        if walls is None:
            raise ValueError("simulation result has no wall model to draw")

        axis.add_patch(
            Rectangle(
                (result.goal_x, result.goal_y),
                result.goal_size,
                result.goal_size,
                facecolor="#ffe9a8",
                edgecolor="#d39b00",
                linewidth=1.2,
                zorder=0,
            )
        )

        physical_segments: list[list[tuple[float, float]]] = []
        virtual_segments: list[list[tuple[float, float]]] = []
        for x in range(walls.width):
            for y in range(walls.height):
                edges = (
                    (Direction.NORTH, [(x, y + 1), (x + 1, y + 1)]),
                    (Direction.EAST, [(x + 1, y), (x + 1, y + 1)]),
                )
                if y == 0:
                    edges += ((Direction.SOUTH, [(x, y), (x + 1, y)]),)
                if x == 0:
                    edges += ((Direction.WEST, [(x, y), (x, y + 1)]),)
                for direction, segment in edges:
                    if walls.truth_has_wall(x, y, direction):
                        physical_segments.append(segment)
                    elif walls.get_virtual_wall(x, y, direction):
                        virtual_segments.append(segment)

        if physical_segments:
            axis.add_collection(
                LineCollection(physical_segments, colors="#303030", linewidths=1.6)
            )
        if virtual_segments:
            axis.add_collection(
                LineCollection(
                    virtual_segments,
                    colors="#d62728",
                    linewidths=1.2,
                    linestyles="dashed",
                )
            )

        start_x, start_y = positions[0]
        axis.plot(start_x, start_y, marker="s", color="#19a974", markersize=5, zorder=3)
        trail, = axis.plot([], [], color="#00a6d6", linewidth=1.8, zorder=2)
        mouse, = axis.plot(
            [],
            [],
            marker="o",
            color="black",
            markeredgecolor="black",
            markersize=max(5, min(12, 100 / max(walls.width, walls.height))),
            linestyle="None",
            zorder=4,
        )
        status = axis.text(
            0.01,
            0.99,
            "",
            transform=axis.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8, "pad": 2},
            zorder=5,
        )
        axis.set_xlim(0, walls.width)
        axis.set_ylim(0, walls.height)
        axis.set_aspect("equal", adjustable="box")
        tick_step = 1 if max(walls.width, walls.height) <= 16 else 4
        axis.set_xticks(range(0, walls.width + 1, tick_step))
        axis.set_yticks(range(0, walls.height + 1, tick_step))
        axis.grid(color="#dddddd", linewidth=0.35, zorder=-1)
        mode = "off" if not result.virtual_enabled else result.guard
        axis.set_title(f"virtual={mode}  {result.reason}")
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        artists.append((trail, mouse, status))

    frame_count = max(len(positions) for positions in paths)

    def update(frame: int):
        changed = []
        for positions, (trail, mouse, status) in zip(paths, artists):
            index = min(frame, len(positions) - 1)
            visible = positions[: index + 1]
            trail.set_data(
                [position[0] for position in visible],
                [position[1] for position in visible],
            )
            x, y = positions[index]
            mouse.set_data([x], [y])
            status.set_text(
                f"step {index}/{len(positions) - 1}   cell ({int(x - 0.5)}, {int(y - 0.5)})"
            )
            changed.extend((trail, mouse, status))
        return changed

    animation = FuncAnimation(
        figure,
        update,
        frames=frame_count,
        interval=interval_ms,
        repeat=False,
        blit=False,
    )
    update(0)
    playback = {"paused": False}

    def toggle_pause(event) -> None:
        if event.key != " ":
            return
        if playback["paused"]:
            animation.event_source.start()
        else:
            animation.event_source.stop()
        playback["paused"] = not playback["paused"]

    figure.canvas.mpl_connect("key_press_event", toggle_pause)
    figure.suptitle("Search simulation (Space: pause / resume)")
    figure.tight_layout(rect=(0, 0, 1, 0.95))
    plt.show()


def parse_goal(text: str) -> tuple[int, int, int]:
    try:
        x_text, y_text, size_text = text.split(",")
        return int(x_text), int(y_text), int(size_text)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("goal must be X,Y,SIZE") from exc


def parse_start(text: str) -> Position:
    try:
        x_text, y_text, direction_text = text.split(",")
        direction = {
            "N": Direction.NORTH,
            "E": Direction.EAST,
            "S": Direction.SOUTH,
            "W": Direction.WEST,
        }[direction_text.strip().upper()]
        return Position(int(x_text), int(y_text), direction)
    except (ValueError, KeyError, TypeError) as exc:
        raise argparse.ArgumentTypeError("start must be X,Y,N|E|S|W") from exc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("maze", type=Path, help="maze_data text file or packed 32x32 .bin snapshot")
    parser.add_argument("--virtual", choices=("on", "off"), default="on")
    parser.add_argument("--guard", choices=("legacy", "fixed"), default="fixed")
    parser.add_argument("--timing", choices=("firmware", "fresh"), default="firmware")
    parser.add_argument("--map-mode", choices=("goal", "full"), default="goal")
    parser.add_argument(
        "--branch-mode",
        choices=("observed", "unknown_open"),
        default="observed",
        help="branch closure requires observed cells, or treats UNKNOWN as open",
    )
    parser.add_argument(
        "--features",
        default="pillar,dead_end,branch",
        help="comma separated: pillar,dead_end,branch (default: all)",
    )
    parser.add_argument("--goal", type=parse_goal, help="override goal as X,Y,SIZE")
    parser.add_argument("--start", type=parse_start, help="override start as X,Y,N|E|S|W")
    parser.add_argument(
        "--snapshot-sensing",
        choices=("frozen", "assume-open", "replay-known"),
        default="frozen",
        help="keep a snapshot frozen, sense UNKNOWN as open, or reset and replay its known truth",
    )
    parser.add_argument("--max-steps", type=int, default=4096)
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--csv", type=Path, help="write per-step diagnostics")
    parser.add_argument(
        "--animate",
        action="store_true",
        help="show the route as an animation; the mouse is a black circle",
    )
    parser.add_argument(
        "--animation-interval",
        type=int,
        default=250,
        metavar="MS",
        help="milliseconds per movement step (default: 250)",
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="compare virtual off, legacy guard, and fixed guard",
    )
    parser.add_argument(
        "--audit-contexts",
        action="store_true",
        help="for a .bin snapshot, audit every fully-observed current->next context",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.animation_interval <= 0:
        raise SystemExit("--animation-interval must be greater than zero")
    features = {item.strip() for item in args.features.split(",") if item.strip()}
    invalid = features - {"pillar", "dead_end", "branch"}
    if invalid:
        raise SystemExit(f"unknown features: {', '.join(sorted(invalid))}")

    if args.audit_contexts:
        if args.maze.suffix.lower() != ".bin":
            raise SystemExit("--audit-contexts requires a packed .bin snapshot")
        goal = args.goal or (7, 7, 2)
        guards = ("legacy", "fixed") if args.compare else (args.guard,)
        for guard in guards:
            context_count, issues = audit_snapshot_contexts(
                args.maze, guard, features, goal
            )
            print(
                f"audit guard={guard:<6} contexts={context_count} "
                f"selected_edge_closed={len(issues)}"
            )
            for issue in issues[:20]:
                print(f"  {issue}")
        return 0

    configurations = [(args.virtual == "on", args.guard)]
    if args.compare:
        configurations = [(False, "fixed"), (True, "legacy"), (True, "fixed")]

    results: list[SimulationResult] = []
    for enabled, guard in configurations:
        result = simulate(
            maze_path=args.maze,
            virtual_enabled=enabled,
            guard=guard,
            timing=args.timing,
            map_mode=args.map_mode,
            features=features,
            max_steps=args.max_steps,
            goal_override=args.goal,
            start_override=args.start,
            snapshot_sensing=args.snapshot_sensing,
            branch_mode=args.branch_mode,
        )
        results.append(result)
        print_result(result, args.trace)

    if args.csv:
        if len(results) == 1:
            write_csv(args.csv, results[0])
        else:
            for result in results:
                mode = "off" if not result.virtual_enabled else result.guard
                output = args.csv.with_name(f"{args.csv.stem}_{mode}{args.csv.suffix or '.csv'}")
                write_csv(output, result)

    if args.animate:
        show_animation(results, interval_ms=args.animation_interval)

    return 0 if all(result.success for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
