import unittest
import numpy as np
from demo import metrics, simulate
from model import advance, MAX_ACCEL, MAX_SPEED
from planner import circle_to_grid, formation
from demo import run_show, SAFE
from scipy.spatial.distance import pdist
from contextlib import redirect_stdout
from io import StringIO


class BaselineChecks(unittest.TestCase):
    def test_intersample_crossing(self):
        h = np.array([[[-1., 0], [1., 0]], [[1., 0], [-1., 0]]])
        gap, episodes, distance, _ = metrics(h)
        self.assertEqual((gap, episodes, distance), (0., 1, 4.))

    def test_persistent_violation_counts_once(self):
        h = np.array([[[0., 0], [.5, 0]]] * 5)
        self.assertEqual(metrics(h)[1], 1)

    def test_equilibrium_and_bounds(self):
        pos = np.zeros((1, 2))
        vel = pos.copy()
        rest, _ = advance(pos, vel, pos, .05)
        np.testing.assert_array_equal(pos, rest)
        for _ in range(200):
            pos, new = advance(pos, vel, np.array([[100., 100.]]), .05)
            self.assertLessEqual(np.linalg.norm(new), MAX_SPEED+1e-12)
            self.assertLessEqual(np.linalg.norm(new-vel)/.05, MAX_ACCEL+1e-12)
            vel = new

    def test_assignments_and_arrival(self):
        start, index = circle_to_grid("index")
        _, near = circle_to_grid("minimum")
        self.assertEqual(set(map(tuple, near)), set(map(tuple, index)))
        self.assertEqual(len(set(map(tuple, near))), 20)
        for goal in (index, near):
            h, time = simulate(start, goal)
            self.assertIsNotNone(time)
            self.assertLessEqual(np.linalg.norm(h[-1]-goal, axis=1).max(), .10)

    
    def test_minimum_show_has_safe_formations_and_transition_violation(self):
        for name in ("circle", "grid", "heart"):
            self.assertGreater(pdist(formation(name)).min(), SAFE)
        for dt in (.05, .025):
            with redirect_stdout(StringIO()):
                h, goals, labels = run_show("minimum", dt, "heart")
            self.assertLess(metrics(h)[0], SAFE)
            self.assertEqual(metrics(h)[1], 1)
            self.assertEqual(labels[-1], "Holding heart")
            self.assertLessEqual(np.linalg.norm(h[-1]-goals[-1], axis=1).max(), .10)
            # No jump at a formation boundary.
            self.assertLessEqual(np.linalg.norm(np.diff(h, axis=0), axis=2).max(),
                                 MAX_SPEED*dt+1e-12)

    
    def test_x_has_clear_endpoints_and_more_transition_violations(self):
        self.assertEqual(len(set(map(tuple, formation("x")))), 20)
        self.assertGreater(pdist(formation("x")).min(), SAFE)
        for dt in (.05, .025):
            with redirect_stdout(StringIO()):
                h, goals, labels = run_show("minimum", dt, "x")
            gap, count, _, _ = metrics(h)
            self.assertGreater(count, 1)
            self.assertLess(gap, .70)
            self.assertEqual(labels[-1], "Holding x")
            self.assertLessEqual(np.linalg.norm(h[-1]-goals[-1], axis=1).max(), .10)
            self.assertLessEqual(np.linalg.norm(np.diff(h, axis=0), axis=2).max(),
                                 MAX_SPEED*dt+1e-12)
