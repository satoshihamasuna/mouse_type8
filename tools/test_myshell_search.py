import tempfile
import unittest
from pathlib import Path

from myshell_search import (
    HEIGHT,
    ReplayResult,
    WIDTH,
    load_history,
    load_maze_payload,
    parse_replay_lines,
    parse_search_lines,
    validate_replay_result,
    validate_result,
)


class MyshellSearchTest(unittest.TestCase):
    @staticmethod
    def valid_lines(selected_vwall=0):
        lines = [
            "SEARCH_RUN_START start:0,0,N goal:0,1,1 mode:goal priority:first mask:1 max_steps:8",
            "SEARCH_STEP index:0 pos:0,0,N self:33 map:32,1024,1024,1024 "
            "wall:0,1,1,1 vwall:0,0,0,0 next:0,1,N local:0 found:1 "
            f"selected_vwall:{selected_vwall} virtual_edges:1",
            "SEARCH_RUN_END result:goal steps:1 final:0,1,N virtual_edges:1",
            "SEARCH_DUMP_START",
            "SEARCH_VIRTUAL x:2 y:2 dir:N",
        ]
        for y in range(HEIGHT - 1, -1, -1):
            lines.append(f"SEARCH_MAP y:{y} values:" + ",".join(["1024"] * WIDTH))
        lines.append("SEARCH_DUMP_END virtual_edges:1 map_rows:32")
        return lines

    def test_valid_transcript(self):
        result = parse_search_lines(self.valid_lines())
        self.assertEqual(validate_result(result, (0, 0, "N"), (0, 1, 1), 1), [])

    def test_selected_edge_closure_is_reported(self):
        result = parse_search_lines(self.valid_lines(selected_vwall=1))
        issues = validate_result(result, (0, 0, "N"), (0, 1, 1), 1)
        self.assertTrue(any("selected edge" in issue for issue in issues))

    def test_wall_history_text_is_packed(self):
        text = "0:(x,y)->( 0, 1),(n,e,s,w)->( 0, 1, 0, 1)\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "history.txt"
            path.write_text(text, encoding="utf-8")
            payload = load_history(path)
        self.assertIsNotNone(payload)
        index = (HEIGHT - 1 - 1) * WIDTH
        self.assertEqual(payload[index] & 0xFF, 0 | (1 << 2) | (0 << 4) | (1 << 6))

    def test_raw_payload_with_txt_extension_is_accepted(self):
        payload = bytes([0xAA] * (WIDTH * HEIGHT))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "maze.txt"
            path.write_bytes(payload)
            loaded, start, goal = load_maze_payload(path)
        self.assertEqual(loaded, payload)
        self.assertIsNone(start)
        self.assertIsNone(goal)

    def test_valid_incremental_replay_transcript(self):
        lines = [
            "REPLAY_START state:reset motion:acc start:0,0,N goal:0,1,1 "
            "mode:goal priority:first mask:1 max_steps:8",
            "REPLAY_STEP index:0 pos:0,0,N self:1 map:0,1024,1024,1024 "
            "wall:0,1,1,1 vwall:0,0,0,0 sensed:0 next:0,1,N local:0 "
            "truth:0 selected_vwall:0 next_acc:8 virtual_edges:1",
            "REPLAY_END result:goal steps:1 final:0,1,N sensed:1 history:1 virtual_edges:1",
            "REPLAY_DUMP_START",
            "REPLAY_VIRTUAL x:2 y:2 dir:N",
        ]
        for y in range(HEIGHT - 1, -1, -1):
            lines.append(f"REPLAY_MAP y:{y} values:" + ",".join(["1024"] * WIDTH))
        lines.append("REPLAY_DUMP_END virtual_edges:1 map_rows:32 history:1")
        result = parse_replay_lines(lines)
        self.assertEqual(validate_replay_result(result, 1), [])

    def test_replay_protects_intrinsic_maze_goal_on_return_leg(self):
        result = ReplayResult(
            config={
                "start": "7,8,E",
                "goal": "0,0,1",
                "maze_start": "0,0",
                "maze_goal": "7,7,2",
            },
            result="goal",
            final_x=0,
            final_y=0,
            final_heading="S",
            reported_steps=0,
            reported_virtual_edges=1,
            virtual_edges=[(6, 7, "E")],
            map_rows={y: [1024] * WIDTH for y in range(HEIGHT)},
        )
        issues = validate_replay_result(result, 1)
        self.assertTrue(any("touches protected cell" in issue for issue in issues))


if __name__ == "__main__":
    unittest.main()
