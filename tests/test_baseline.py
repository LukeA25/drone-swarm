import unittest
import numpy as np
from drone_swarm.simulation import metrics, simulate
from drone_swarm.model import advance, MAX_ACCEL, MAX_SPEED
from drone_swarm.planner import assign_targets, formation3d
from drone_swarm.simulation import run_show, SAFE
from scipy.spatial.distance import pdist
from contextlib import redirect_stdout
from io import StringIO


class BaselineChecks(unittest.TestCase):
    def test_intersample_crossing(self):
        h = np.array([[[-1., 0, 0], [1., 0, 0]], [[1., 0, 0], [-1., 0, 0]]])
        gap, episodes, distance, _ = metrics(h)
        self.assertEqual((gap, episodes, distance), (0., 1, 4.))

    def test_persistent_violation_counts_once(self):
        h = np.array([[[0., 0, 0], [.5, 0, 0]]] * 5)
        self.assertEqual(metrics(h)[1], 1)

    def test_equilibrium_and_bounds(self):
        pos = np.zeros((1, 3))
        vel = pos.copy()
        rest, _ = advance(pos, vel, pos, .05)
        np.testing.assert_array_equal(pos, rest)
        for _ in range(200):
            pos, new = advance(pos, vel, np.array([[100., 100., 100.]]), .05)
            self.assertLessEqual(np.linalg.norm(new), MAX_SPEED+1e-12)
            self.assertLessEqual(np.linalg.norm(new-vel)/.05, MAX_ACCEL+1e-12)
            vel = new

    def test_assignments_and_arrival(self):
        start = formation3d("sphere")
        index = assign_targets(start, formation3d("grid"), "index")
        near = assign_targets(start, formation3d("grid"), "minimum")
        self.assertEqual(set(map(tuple, near)), set(map(tuple, index)))
        self.assertEqual(len(set(map(tuple, near))), 60)
        for goal in (index, near):
            h, time = simulate(start, goal)
            self.assertIsNotNone(time)
            self.assertLessEqual(np.linalg.norm(h[-1]-goal, axis=1).max(), .10)

    
    def test_formations_have_safe_unique_slots(self):
        for name in ("sphere", "grid", "helix", "heart", "x"):
            slots = formation3d(name)
            self.assertEqual(slots.shape, (60, 3))
            self.assertEqual(len(set(map(tuple, slots))), 60)
            self.assertGreater(pdist(slots).min(), SAFE)

    def test_helix_transition_regression(self):
        for dt in (.05, .025):
            with redirect_stdout(StringIO()):
                h, goals, labels = run_show("minimum", dt)
            self.assertEqual(metrics(h)[1], 7)
            self.assertAlmostEqual(metrics(h)[0], .635, places=3)
            self.assertEqual(labels[-1], "Holding helix")
            self.assertLessEqual(np.linalg.norm(h[-1]-goals[-1], axis=1).max(), .10)
            self.assertLessEqual(np.linalg.norm(np.diff(h, axis=0), axis=2).max(),
                                 MAX_SPEED*dt+1e-12)
