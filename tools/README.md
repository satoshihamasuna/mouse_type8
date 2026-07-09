# Tools

Python tools for serial logging, maze display, and log analysis.

Run commands from the repository root.

## Setup

Install the Python packages used by the tools:

```powershell
pip install pyserial pandas numpy matplotlib
```

## myshell GUI

Open the GUI for common myshell operations:

```powershell
python tools\myshell_gui.py
```

It can send commands, receive `disp maze_bin` into a maze canvas, and save `disp log_bin` as CSV.

Detailed command/protocol notes:

```text
tools/myshell_README.md
```

## Serial Binary Log

Receive binary log data from STM32 and save it as CSV under `tools/logs/`.

```powershell
python tools\Serial_binary.py
```

Output example:

```text
tools/logs/20260708_2340_type8a_MPQ.csv
```

`tools/logs/` is ignored by git, so generated CSV files are not pushed.

## Analyze Logs

Open this notebook:

```text
tools/Serial_indicate.ipynb
```

The notebook automatically reads the latest CSV in `tools/logs/`.
It also displays PID gains if the CSV includes these columns:

```text
speed_ctrl.Kp, speed_ctrl.Ki, speed_ctrl.Kd
omega_ctrl.Kp, omega_ctrl.Ki, omega_ctrl.Kd
```

## Maze Viewer

Display maze data from myshell.

STM32 side command used internally:

```text
disp maze
```

Python command:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200
```

If serial input is unstable, send characters more slowly:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --char-delay 0.05
```

For debugging received serial text:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --debug
```

Save the maze image:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --save tools\logs\maze.png
```

Parse a saved ASCII maze text instead of serial:

```powershell
python tools\Serial_maze_view.py --file tools\logs\maze_sample.txt --width 32 --height 32
```

## Maze Editor / Viewer Core

`tools/maze.py` defines the `Maze` class used by `Serial_maze_view.py`.
It can also be run directly to open a simple interactive maze viewer/editor:

```powershell
python tools\maze.py
```
