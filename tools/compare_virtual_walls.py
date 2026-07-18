"""Compare virtual-wall feature sets for the bundled 2011-2025 mazes."""
from pathlib import Path
import argparse
import copy

from search_simulator import Direction, MazeFileReader, Position, derive_goal, simulate


def compare(path: Path, max_steps: int, return_map_mode: str = "full") -> list[str]:
    data = MazeFileReader.from_file(path)
    goal = derive_goal(data.goal_cells)
    sx, sy = data.start_cells[0]
    start = Position(sx, sy, Direction.NORTH)
    rows = []
    features = {"pillar", "dead_end", "branch"}
    out = simulate(path, True, "fixed", "firmware", "goal", features,
                   max_steps, start_override=start, goal_override=goal)
    for label, branch_mode in (("full_observed", "observed"), ("full_prune", "unknown_open")):
        ret = simulate(path, True, "fixed", "firmware", return_map_mode, features,
                        max_steps, start_override=out.final_position,
                        goal_override=(sx, sy, 1), maze_goal_override=goal,
                        branch_mode=branch_mode,
                        initial_walls=copy.deepcopy(out.wall_model))
        rows.append(
            f"{path.stem[2:6]},{label},{out.reason},{out.steps},"
            f"{ret.reason},{ret.steps}"
        )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-steps", type=int, default=4096)
    parser.add_argument("--return-map-mode", choices=("goal", "full"), default="goal")
    parser.add_argument("mazes", nargs="*", type=Path)
    args = parser.parse_args()
    mazes = args.mazes or sorted(Path(__file__).with_name("maze_data").glob("MM20??MM.txt"))
    print("year,features,outward_result,outward_steps,return_result,return_steps")
    for path in mazes:
        print("\n".join(compare(path, args.max_steps, args.return_map_mode)))


if __name__ == "__main__":
    main()
