import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from myshell_gui import dijkstra_profile_command, parse_dijkstra_result


def test_dijkstra_profile_commands():
    assert dijkstra_profile_command("Uniform 1000") == "path dijkstra_queue"
    assert dijkstra_profile_command("Mixed 1600/1800") == "path dijkstra_queue acc1600"
    assert dijkstra_profile_command("Mixed 1600/1800", queue_mode=False) == "path dijkstra acc1600"


def test_parse_dijkstra_goal_result():
    assert parse_dijkstra_result(
        "DIJKSTRA_RESULT GOAL last=( 7, 8, 0) time:1301"
    ) == {"status": "GOAL", "time": 1301}


def test_parse_dijkstra_no_path_result():
    assert parse_dijkstra_result(
        "DIJKSTRA_RESULT NO_PATH last=( 0, 0, 0) time:0"
    ) == {"status": "NO_PATH", "time": 0}
    assert parse_dijkstra_result("DIJKSTRA_END") is None
