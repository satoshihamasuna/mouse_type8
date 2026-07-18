# Subsys maze search simulator

`search_simulator.py` reproduces the relevant behavior of:

- `wall_class` (`UNKNOWN`, `NOWALL`, `WALL`, separate virtual walls)
- `virtual_wall_class` pillar and dead-end inference
- `make_map_queue` and `make_map_queue_zenmen`
- `adachi::get_next_dir`
- the current firmware ordering: initialize motion, rebuild virtual walls/map,
  execute motion

Virtual walls persist between updates.  A search start clears the whole virtual
layer; subsequent updates clear only edges incident to the start, current
`expand_end`, or goal area, then run one pillar pass and one dead-end pass.

It accepts both complete text mazes and the packed 1024-byte maze snapshot
emitted by the firmware.

## Check the captured maze

From the repository root:

```powershell
python tools/search_simulator.py tools/maze_data/logs_maze.bin --compare
```

Compare full-search return behavior:

```powershell
python tools/search_simulator.py tools/maze_data/logs_maze.bin `
  --compare --map-mode full --start 7,8,E --goal 0,0,1
```

Audit every fully-observed current-to-next context in the snapshot:

```powershell
python tools/search_simulator.py tools/maze_data/logs_maze.bin `
  --audit-contexts --compare
```

Isolate one inference rule:

```powershell
python tools/search_simulator.py tools/maze_data/logs_maze.bin `
  --audit-contexts --compare --features dead_end

python tools/search_simulator.py tools/maze_data/logs_maze.bin `
  --audit-contexts --compare --features pillar
```

Write a per-step CSV trace:

```powershell
python tools/search_simulator.py tools/maze_data/logs_maze.bin `
  --compare --csv tools/logs/virtual_wall_sim.csv
```

## Interpretation of binary snapshots

A `.bin` file contains the mouse's known wall states, not the complete physical
maze. Known `WALL` edges are treated as definite walls. `UNKNOWN` edges remain
unknown and are considered traversable with the firmware's normal `mask=0x01`.
By default the snapshot is frozen during a simulated walk.

`--snapshot-sensing assume-open` is available for synthetic experiments, but
it assumes unknown physical space is open and therefore must not be interpreted
as a collision-proof reproduction of the real maze.

## Current logs_maze.bin findings

- Goal-map walk: virtual OFF, legacy guard, and fixed guard all reach `(7,8)` in
  33 simulated steps.
- Pillar-only inference does not close a selected movement edge in the 136
  audited contexts.
- With `dead_end` enabled, the legacy guard closes the already selected edge
  in 19 of those contexts.
- Pillar-only inference produces zero selected-edge closures.
- The fixed edge guard produces zero selected-edge closures.
- The position protected during the pre-motion update is the physical current
  cell. `expand_end` is kept separate and is used only as the map-expansion
  destination; protecting it would erase virtual walls before arrival.
- Full-search return from `(7,8,E)` to `(0,0)` falls back to the normal goal map
  even with virtual walls OFF; all three configurations then return in 33 steps.
