"""Generate formation slots and assign targets before each transition."""
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist


def formation(name):
    if name == "circle":
        angles = 2 * np.pi * np.arange(20) / 20
        return 5 * np.column_stack((np.cos(angles), np.sin(angles)))
    if name == "grid":
        return np.array([(2*x, 2*y) for y in (-1.5, -.5, .5, 1.5)
                         for x in (-2, -1, 0, 1, 2)])
    if name == "x":
        # Five distinct slots on each diagonal arm; no duplicate center slot.
        return np.array([(sx*r, sy*r)
                         for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))
                         for r in range(1, 6)], dtype=float)
    if name == "heart":
        # Sample uniformly in arc length, avoiding crowded slots near the tip.
        t = np.linspace(0, 2*np.pi, 20001)
        curve = np.column_stack((16*np.sin(t)**3,
                  13*np.cos(t)-5*np.cos(2*t)-2*np.cos(3*t)-np.cos(4*t)))
        arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))]
        samples = np.arange(20) * arc[-1] / 20
        return .35 * np.column_stack([np.interp(samples, arc, curve[:,i])
                                       for i in range(2)])
    raise ValueError("Choose circle, grid, heart, or x.")


def assign_targets(start, slots, assignment):
    if assignment == "minimum":
        _, columns = linear_sum_assignment(cdist(start, slots))
        return slots[columns]
    if assignment == "index":
        return slots.copy()
    raise ValueError("Choose index or minimum.")


def circle_to_grid(assignment):
    start = formation("circle")
    return start, assign_targets(start, formation("grid"), assignment)
