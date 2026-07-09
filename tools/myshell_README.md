# myshell serial guide

This document describes the STM32 `myshell` commands and the matching Python GUI.

## Setup

Install Python dependencies from the repository root:

```powershell
pip install pyserial
```

The GUI itself uses Tkinter, which is included with the normal Windows Python installer.

## Start myshell on the mouse

Select the shell mode on the mouse firmware, then connect the serial port from the PC.
The default GUI settings are:

```text
Port: COM8
Baud: 115200
Line ending: CR
```

Change the port in the GUI if Windows assigned a different COM number.

## myshell commands

| Command | Description |
| --- | --- |
| `help` | Show the command list. |
| `info sys` | Print the system name. |
| `info ver` | Print the firmware version string. |
| `load save` | Load saved wall data from flash into the shell-side `wall_data`. |
| `disp maze` | Load saved data and print the maze as ASCII. |
| `disp maze_bin` | Load saved data and output one binary byte per maze cell. |
| `disp histry` | Load saved data and print wall history. |
| `disp log` | Print log data as CSV-like text. |
| `disp log_bin` | Output log data as binary frames for fast CSV capture. |
| `path dijkstra` | Load saved data and print `check_run_Dijkstra()` result. |
| `log mode0` | Enable log header output. |
| `log mode1` | Disable log header output. |
| `end exe` | Exit myshell. |

## Binary maze format

`disp maze_bin` prints:

```text
MAZE_BIN_START
<width * height raw bytes>
MAZE_BIN_END
```

The default maze size is 32 x 32, so the payload is 1024 bytes.

Each byte describes one cell:

```text
bit 0-1: north
bit 2-3: east
bit 4-5: south
bit 6-7: west
```

Wall states:

```text
0: NOWALL
1: WALL
2: UNKNOWN
3: VWALL
```

The payload order is row-major from the top row (`y = height - 1`) to the bottom row.

## Binary log format

`disp log_bin` prints ASCII metadata first, then binary frames:

```text
START
HEADER,cnt,...
BINARY
<binary log_frame_t frames>
<LOG_MAGIC_END frame>
END
```

Each normal frame is:

```c
uint16_t magic;       // 0xA55A
uint16_t index;
half_float data[51];
```

The end frame uses `magic = 0xFFFF`.

## GUI

Run:

```powershell
python tools\myshell_gui.py
```

Main functions:

| Button | Action |
| --- | --- |
| `Connect` | Open the selected serial port. |
| `Send` | Send the command in the text box. |
| `load save` | Load flash save data. |
| `disp maze_bin` | Receive binary maze data and draw it on the canvas. |
| `disp log_bin -> CSV` | Receive binary log frames and save a CSV under `tools/logs/`. |
| `path dijkstra` | Print the Dijkstra path check output. |
| `Save output` | Save the serial text window to a file. |
| `Save maze binary` | Save the latest received maze payload to a `.bin` file. |

If the GUI shows a timeout, increase the Timeout field or press `Connect` again after resetting the mouse-side shell.

## Existing command-line tools

The older command-line tools are still useful:

```powershell
python tools\Serial_maze_view.py --port COM8 --baud 115200 --binary
python tools\Serial_binary.py
```

Use the GUI for quick manual operation, and the command-line tools when you want scripted captures.
