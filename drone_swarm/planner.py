"""Generate formation slots and assign targets before each transition."""
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist


def _outline(name):
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
    raise ValueError("Choose grid, heart, or x.")


def formation3d(name):
    if name == "sphere":
        # Spiral points evenly in height using the golden angle in longitude.
        i = np.arange(60)
        z = 1 - 2*(i + 0.5)/60
        angle = i*np.pi*(3-np.sqrt(5))
        radius = np.sqrt(1-z*z)
        return 5*np.column_stack((radius*np.cos(angle), radius*np.sin(angle), z)) + [0, 0, 7]
    if name == "helix":
        # Thirty heights, with two opposite slots at each height.
        t = np.linspace(0, 1, 30)
        angle = 4*np.pi*t + np.pi/2
        x, y = 2*np.cos(angle), 2*np.sin(angle)
        z = 7 + 5*(t - 0.5)
        return np.vstack((np.column_stack((x, y, z)),
                          np.column_stack((-x, -y, z))))
    # Three vertical cross-sections give the grid, X and heart real depth.
    # Taper the outer X and heart outlines to emphasize their central section.
    outline = _outline(name)
    layers = []
    for depth in (-2, 0, 2):
        scale = 0.8 if depth and name != "grid" else 1
        layers.append(np.column_stack((scale*outline[:, 0],
                                       np.full(len(outline), depth),
                                       7 + scale*outline[:, 1])))
    return np.vstack(layers)


def assign_targets(start, slots, assignment):
    if assignment == "minimum":
        _, columns = linear_sum_assignment(cdist(start, slots))
        return slots[columns]
    if assignment == "index":
        return slots.copy()
    raise ValueError("Choose index or minimum.")
