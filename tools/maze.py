import math
from copy import deepcopy

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


class Maze:
    """Micromouse maze wall data and simple matplotlib viewer/editor.

    Wall order is kept compatible with the original implementation:
    North=0, East=1, South=2, West=3.
    """

    North, East, South, West = range(4)

    def __init__(self, x, y, verbose=True):
        self.maze_size_x = int(x)
        self.maze_size_y = int(y)
        self.verbose = verbose

        self.wall = np.zeros(
            (self.maze_size_x, self.maze_size_y, 4),
            dtype=bool,
        )
        self._set_outer_walls()

        # Original start wall.
        if self.maze_size_x >= 2 and self.maze_size_y >= 1:
            self.wall[0, 0, Maze.East] = True
            self.wall[1, 0, Maze.West] = True

    # ---------------------------------------------------------------------
    # Wall data
    # ---------------------------------------------------------------------
    def _set_outer_walls(self):
        """Set the maze boundary walls."""
        self.wall[:, self.maze_size_y - 1, Maze.North] = True
        self.wall[:, 0, Maze.South] = True
        self.wall[self.maze_size_x - 1, :, Maze.East] = True
        self.wall[0, :, Maze.West] = True

    def load_edit_maze(self, wall_data):
        """Load wall data copied from another maze/editor."""
        self.wall = deepcopy(wall_data)

    def has_wall(self, x, y, direction):
        """Return True if a wall exists at (x, y, direction)."""
        return bool(self.wall[x, y, direction])

    def set_wall(self, x, y, direction, exists=True, mirror=True):
        """Set one wall and, when possible, the neighbor's opposite wall."""
        self.wall[x, y, direction] = exists

        if not mirror:
            return

        neighbor = self._neighbor_cell(x, y, direction)
        if neighbor is None:
            return

        nx, ny, opposite = neighbor
        self.wall[nx, ny, opposite] = exists

    def _neighbor_cell(self, x, y, direction):
        """Return (nx, ny, opposite_dir), or None for outer boundary."""
        if direction == Maze.North and y + 1 < self.maze_size_y:
            return x, y + 1, Maze.South
        if direction == Maze.East and x + 1 < self.maze_size_x:
            return x + 1, y, Maze.West
        if direction == Maze.South and y - 1 >= 0:
            return x, y - 1, Maze.North
        if direction == Maze.West and x - 1 >= 0:
            return x - 1, y, Maze.East
        return None

    def toggle_wall(self, x, y, direction):
        """Toggle a wall and return the new value."""
        new_value = not self.has_wall(x, y, direction)
        self.set_wall(x, y, direction, new_value)
        return new_value

    # ---------------------------------------------------------------------
    # Text display
    # ---------------------------------------------------------------------
    def disp_map(self):
        """Print the maze in the same ASCII style as the original code."""
        lines = []

        for yy in range(self.maze_size_y - 1, -1, -1):
            top = []
            middle = []

            for xx in range(self.maze_size_x):
                top.append("+---" if self.wall[xx, yy, Maze.North] else "+   ")
                middle.append("|   " if self.wall[xx, yy, Maze.West] else "    ")

                if xx == self.maze_size_x - 1:
                    top.append("+")
                    middle.append("|" if self.wall[xx, yy, Maze.East] else " ")

            lines.append("".join(top))
            lines.append("".join(middle))

        lines.append("+---" * self.maze_size_x + "+")
        print("\n".join(lines))

    # ---------------------------------------------------------------------
    # Drawing
    # ---------------------------------------------------------------------
    def draw_maze(self, ax=None, clear=False, show_grid_points=True):
        """Draw the maze.

        The original code called plt.plot for every wall segment. This version
        batches wall segments with LineCollection, which is much faster for
        large mazes.
        """
        ax = ax or plt.gca()

        if clear:
            ax.clear()

        wall_segments = self._collect_wall_segments()
        if wall_segments:
            ax.add_collection(LineCollection(wall_segments, colors="r", linewidths=1.5))

        if show_grid_points:
            xs, ys = self._grid_points()
            ax.plot(xs, ys, "r+", linestyle="None", markersize=4)

        self._format_axes(ax)
        plt.draw()

    def _collect_wall_segments(self):
        """Collect visible wall segments without drawing duplicates."""
        segments = []

        for yy in range(self.maze_size_y):
            for xx in range(self.maze_size_x):
                if self.wall[xx, yy, Maze.North]:
                    segments.append(self._wall_segment(xx, yy, Maze.North))

                if self.wall[xx, yy, Maze.West]:
                    segments.append(self._wall_segment(xx, yy, Maze.West))

                if xx == self.maze_size_x - 1 and self.wall[xx, yy, Maze.East]:
                    segments.append(self._wall_segment(xx, yy, Maze.East))

                if yy == 0 and self.wall[xx, yy, Maze.South]:
                    segments.append(self._wall_segment(xx, yy, Maze.South))

        return segments

    @staticmethod
    def _wall_segment(x, y, direction):
        """Return a line segment for a wall."""
        if direction == Maze.North:
            return [(x - 0.5, y + 0.5), (x + 0.5, y + 0.5)]
        if direction == Maze.East:
            return [(x + 0.5, y - 0.5), (x + 0.5, y + 0.5)]
        if direction == Maze.South:
            return [(x - 0.5, y - 0.5), (x + 0.5, y - 0.5)]
        if direction == Maze.West:
            return [(x - 0.5, y - 0.5), (x - 0.5, y + 0.5)]

        raise ValueError(f"Invalid direction: {direction}")

    def _grid_points(self):
        """Return grid intersection points for red '+' markers."""
        xs = np.arange(-0.5, self.maze_size_x + 0.5, 1.0)
        ys = np.arange(-0.5, self.maze_size_y + 0.5, 1.0)
        xx, yy = np.meshgrid(xs, ys)
        return xx.ravel(), yy.ravel()

    def _format_axes(self, ax):
        ax.set_aspect("equal", adjustable="box")
        ax.set_xticks(range(0, self.maze_size_x + 1, 1))
        ax.set_yticks(range(0, self.maze_size_y + 1, 1))
        ax.set_xlim([-0.75, self.maze_size_x - 0.25])
        ax.set_ylim([-0.75, self.maze_size_y - 0.25])

    # ---------------------------------------------------------------------
    # Interactive wall editor
    # ---------------------------------------------------------------------
    def attach_wall_toggle(self, fig=None):
        """Enable click-to-toggle for internal East/North walls."""
        fig = fig or plt.gcf()
        return fig.canvas.mpl_connect("button_press_event", self.button_press_event)

    def button_press_event(self, event):
        """Toggle the wall closest to the clicked grid line."""
        if event.xdata is None or event.ydata is None:
            return

        x, y = event.xdata, event.ydata
        wall_info = self._clicked_internal_wall(x, y)

        if wall_info is None:
            return

        xx, yy, direction = wall_info
        exists = self.toggle_wall(xx, yy, direction)
        self._draw_toggled_wall(xx, yy, direction, exists)

        if self.verbose:
            xi = math.modf(x)[1]
            yi = math.modf(y)[1]
            print(xx, yy, xi, yi)
            self.disp_map()

        plt.draw()

    def _clicked_internal_wall(self, x, y):
        """Return (x, y, direction) for the nearest editable internal wall."""
        xf, _ = math.modf(x)
        yf, _ = math.modf(y)

        # Close to a vertical grid line: edit East wall of the left cell.
        if abs(xf - 0.5) < abs(yf - 0.5):
            xx = int(round(x - 0.5))
            yy = int(round(y))

            if 0 <= xx < self.maze_size_x - 1 and 0 <= yy < self.maze_size_y:
                return xx, yy, Maze.East

        # Close to a horizontal grid line: edit North wall of the lower cell.
        else:
            xx = int(round(x))
            yy = int(round(y - 0.5))

            if 0 <= xx < self.maze_size_x and 0 <= yy < self.maze_size_y - 1:
                return xx, yy, Maze.North

        return None

    def _draw_toggled_wall(self, x, y, direction, exists):
        """Overlay the changed wall segment after a click."""
        (x0, y0), (x1, y1) = self._wall_segment(x, y, direction)

        if exists:
            plt.plot([x0, x1], [y0, y1], "r", lw=1.5)
        else:
            plt.plot([x0, x1], [y0, y1], "w", lw=3)

        plt.plot([x0, x1], [y0, y1], "r+", linestyle="None", markersize=4)


if __name__ == "__main__":
    test = Maze(16, 16)
    test.draw_maze()
    test.attach_wall_toggle()
    plt.show()
