import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from myshell_gui import (
    dijkstra_run_config_command,
    make_dijkstra_step,
    parse_dijkstra_result,
    parse_run_config,
)


def test_dijkstra_run_config_commands():
    assert dijkstra_run_config_command("uniform1000") == "path dijkstra_queue uniform1000"
    assert dijkstra_run_config_command("acc1600_v1", queue_mode=False) == "path dijkstra acc1600_v1"


def test_parse_run_config():
    assert parse_run_config(
        "RUN_CONFIG key:acc1600_v1 type:2 suction:700 name:Variable turn 1600 V1"
    ) == {
        "key": "acc1600_v1",
        "name": "Variable turn 1600 V1",
        "type": 2,
        "suction": 700,
    }
    assert parse_run_config("RUN_CONFIG_END") is None


def test_parse_dijkstra_goal_result():
    assert parse_dijkstra_result(
        "DIJKSTRA_RESULT GOAL last=( 7, 8, 0) time:1301"
    ) == {"status": "GOAL", "time": 1301}


def test_parse_dijkstra_no_path_result():
    assert parse_dijkstra_result(
        "DIJKSTRA_RESULT NO_PATH last=( 0, 0, 0) time:0"
    ) == {"status": "NO_PATH", "time": 0}
    assert parse_dijkstra_result("DIJKSTRA_END") is None


def test_parse_last_expand_path_step():
    assert make_dijkstra_step(
        "DIJKSTRA_LAST_EXPAND x:25,y:27,d:0,mdir:0,time:6400->count-> 1Straight"
    ) == {
        "x": 25,
        "y": 27,
        "node_pos": 0,
        "mouse_dir": 0,
        "motion": "Straight",
        "last_expand": True,
    }
