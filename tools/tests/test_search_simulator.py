import sys
import unittest
from pathlib import Path

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from search_simulator import (  # noqa: E402
    Direction,
    Position,
    VirtualContext,
    VirtualWallEngine,
    WallModel,
    audit_snapshot_contexts,
    simulate,
)


SNAPSHOT = TOOLS_DIR / "maze_data" / "logs_maze.bin"


class SearchSimulatorTest(unittest.TestCase):
    @staticmethod
    def make_open_model(size=5):
        return WallModel(np.zeros((size, size, 4), dtype=bool))

    @staticmethod
    def make_known_graph(size=6, second_entrance=False, incomplete=False):
        walls = WallModel(np.ones((size, size, 4), dtype=bool))
        walls.known.fill(1)
        edges = [
            (1, 1, Direction.EAST),
            (2, 1, Direction.EAST),
            (2, 1, Direction.NORTH),
            (2, 2, Direction.NORTH),
            (2, 3, Direction.EAST),
            (3, 3, Direction.SOUTH),
            (3, 2, Direction.WEST),
        ]
        if second_entrance:
            edges.append((3, 1, Direction.NORTH))
        for x, y, direction in edges:
            walls._set_known(x, y, direction, 0)
        if incomplete:
            walls._set_known(2, 3, Direction.NORTH, 2)
        return walls

    def test_snapshot_decoder_preserves_wall_states(self):
        walls = WallModel.from_packed_snapshot(SNAPSHOT.read_bytes())
        counts = {
            state: int((walls.known == state).sum())
            for state in range(4)
        }
        self.assertEqual(counts, {0: 136, 1: 272, 2: 3688, 3: 0})

    def test_dead_end_legacy_closes_selected_edges_but_fixed_does_not(self):
        goal = (7, 7, 2)
        contexts, legacy = audit_snapshot_contexts(
            SNAPSHOT, "legacy", {"dead_end"}, goal
        )
        _, fixed = audit_snapshot_contexts(
            SNAPSHOT, "fixed", {"dead_end"}, goal
        )
        self.assertEqual(contexts, 136)
        self.assertGreater(len(legacy), 0)
        self.assertEqual(fixed, [])

    def test_pillar_does_not_close_selected_edges(self):
        _, legacy = audit_snapshot_contexts(
            SNAPSHOT, "legacy", {"pillar"}, (7, 7, 2)
        )
        self.assertEqual(legacy, [])

    def test_pillar_closes_fourth_edge_after_three_confirmed_open_edges(self):
        walls = self.make_open_model()
        edges = (
            (2, 2, Direction.WEST),
            (2, 2, Direction.SOUTH),
            (1, 1, Direction.EAST),
            (1, 1, Direction.NORTH),
        )
        for x, y, direction in edges[:3]:
            walls._set_known(x, y, direction, 0)

        engine = VirtualWallEngine(walls, True, "fixed", {"pillar"})
        context = VirtualContext(
            Position(0, 0), Position(4, 4), 3, 4, 1
        )
        engine.update(context)

        x, y, direction = edges[3]
        self.assertTrue(walls.get_virtual_wall(x, y, direction))
        self.assertEqual(
            walls.virtual_reason[x, y, int(direction) // 2], "pillar"
        )

    def test_virtual_dead_end_closure_propagates_on_the_next_update(self):
        walls = self.make_open_model()
        # (3,1) is the physical dead end.  Because the scan runs from low x
        # to high x, (2,1) sees that closure only on the following update.
        walls._set_known(2, 1, Direction.NORTH, 1)
        walls._set_known(2, 1, Direction.SOUTH, 1)
        walls._set_known(2, 1, Direction.WEST, 0)
        walls._set_known(2, 1, Direction.EAST, 0)
        walls._set_known(3, 1, Direction.NORTH, 1)
        walls._set_known(3, 1, Direction.SOUTH, 1)
        walls._set_known(3, 1, Direction.EAST, 1)

        engine = VirtualWallEngine(walls, True, "fixed", {"dead_end"})
        context = VirtualContext(
            Position(0, 0), Position(4, 4), 3, 4, 1
        )
        engine.update(context)

        self.assertTrue(walls.get_virtual_wall(3, 1, Direction.WEST))
        self.assertFalse(walls.get_virtual_wall(2, 1, Direction.WEST))

        engine.update(context)
        self.assertTrue(walls.get_virtual_wall(2, 1, Direction.WEST))
        self.assertEqual(
            walls.virtual_reason[2, 1, int(Direction.WEST) // 2], "dead_end"
        )

    def test_start_and_goal_edges_are_exempt_and_rebuilt(self):
        walls = self.make_open_model()
        for direction in (Direction.NORTH, Direction.SOUTH, Direction.WEST):
            walls._set_known(1, 1, direction, 1)
        walls._set_known(1, 1, Direction.EAST, 0)
        engine = VirtualWallEngine(walls, True, "fixed", {"dead_end"})

        protected_contexts = (
            VirtualContext(Position(1, 1), Position(4, 4), 3, 4, 1),
            VirtualContext(Position(0, 0), Position(4, 4), 1, 1, 1),
        )
        for context in protected_contexts:
            engine.update(context)
            self.assertFalse(walls.get_virtual_wall(1, 1, Direction.EAST))

        engine.update(VirtualContext(Position(0, 0), Position(4, 4), 3, 4, 1))
        self.assertTrue(walls.get_virtual_wall(1, 1, Direction.EAST))


    def test_mouse_blocks_new_inference_without_erasing_existing_wall(self):
        walls = self.make_open_model()
        for direction in (Direction.NORTH, Direction.SOUTH, Direction.WEST):
            walls._set_known(1, 1, direction, 1)
        walls._set_known(1, 1, Direction.EAST, 0)
        engine = VirtualWallEngine(walls, True, "fixed", {"dead_end"})

        mouse_here = VirtualContext(Position(0, 0), Position(1, 1), 3, 4, 1)
        engine.update(mouse_here)
        self.assertFalse(walls.get_virtual_wall(1, 1, Direction.EAST))

        engine.update(VirtualContext(Position(0, 0), Position(4, 4), 3, 4, 1))
        self.assertTrue(walls.get_virtual_wall(1, 1, Direction.EAST))

        # Returning next to an already closed dead end must not reopen it.
        engine.update(mouse_here)
        self.assertTrue(walls.get_virtual_wall(1, 1, Direction.EAST))

    def test_pre_motion_update_does_not_clear_virtual_wall_at_destination(self):
        walls = self.make_open_model(size=8)
        # (5,1) is a known dead end whose south entrance is virtual-closed.
        for direction in (Direction.NORTH, Direction.EAST, Direction.WEST):
            walls._set_known(5, 1, direction, 1)
        walls._set_known(5, 1, Direction.SOUTH, 0)
        engine = VirtualWallEngine(walls, True, "fixed", {"dead_end"})
        away = VirtualContext(Position(0, 0), Position(7, 7), 6, 6, 1)
        engine.update(away)
        self.assertTrue(walls.get_virtual_wall(5, 0, Direction.NORTH))

        # While moving from (6,0) to (5,0), (6,0) is the mouse exception.
        # Protecting the destination (5,0) here would erase the north wall and
        # make the following step enter the dead end at (5,1).
        pre_motion = VirtualContext(Position(0, 0), Position(6, 0), 6, 6, 1)
        engine.update(pre_motion)
        self.assertTrue(walls.get_virtual_wall(5, 0, Direction.NORTH))

    def test_explored_multi_cell_branch_closes_only_its_single_entrance(self):
        walls = self.make_known_graph()
        engine = VirtualWallEngine(walls, True, "fixed", {"branch"})
        context = VirtualContext(Position(1, 1), Position(3, 1), 5, 5, 1)

        engine.update(context)

        self.assertTrue(walls.get_virtual_wall(2, 1, Direction.NORTH))
        self.assertEqual(
            walls.virtual_reason[2, 1, int(Direction.NORTH) // 2],
            "explored_branch",
        )
        self.assertEqual(walls.virtual_edge_count(), 1)

    def test_explored_branch_with_unknown_wall_is_not_closed(self):
        walls = self.make_known_graph(incomplete=True)
        engine = VirtualWallEngine(walls, True, "fixed", {"branch"})
        context = VirtualContext(Position(1, 1), Position(3, 1), 5, 5, 1)

        engine.update(context)

        self.assertFalse(walls.get_virtual_wall(2, 1, Direction.NORTH))

    def test_unknown_open_branch_mode_closes_single_entry_branch(self):
        walls = self.make_known_graph(incomplete=True)
        engine = VirtualWallEngine(walls, True, "fixed", {"branch"})
        context = VirtualContext(Position(1, 1), Position(3, 1), 5, 5, 1)

        engine.update(context, unknown_open=True)

        self.assertTrue(walls.get_virtual_wall(2, 1, Direction.NORTH))

    def test_explored_branch_with_two_entrances_is_not_closed(self):
        walls = self.make_known_graph(second_entrance=True)
        engine = VirtualWallEngine(walls, True, "fixed", {"branch"})
        context = VirtualContext(Position(1, 1), Position(3, 1), 5, 5, 1)

        engine.update(context)

        self.assertFalse(walls.get_virtual_wall(2, 1, Direction.NORTH))
        self.assertFalse(walls.get_virtual_wall(3, 1, Direction.NORTH))

    def test_explored_branch_containing_mouse_is_not_closed(self):
        walls = self.make_known_graph()
        engine = VirtualWallEngine(walls, True, "fixed", {"branch"})
        context = VirtualContext(Position(1, 1), Position(2, 3), 5, 5, 1)

        engine.update(context)

        self.assertFalse(walls.get_virtual_wall(2, 1, Direction.NORTH))

    def test_full_search_snapshot_return_is_stable(self):
        common = dict(
            maze_path=SNAPSHOT,
            timing="firmware",
            map_mode="full",
            features={"pillar", "dead_end"},
            max_steps=100,
            goal_override=(0, 0, 1),
            start_override=Position(7, 8, Direction.EAST),
        )
        off = simulate(virtual_enabled=False, guard="fixed", **common)
        fixed = simulate(virtual_enabled=True, guard="fixed", **common)
        self.assertTrue(off.success)
        self.assertTrue(fixed.success)
        self.assertEqual(off.steps, fixed.steps)
        self.assertIn("fallback", off.reason)
        self.assertIn("fallback", fixed.reason)


if __name__ == "__main__":
    unittest.main()
