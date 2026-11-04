"""Generate 3D reference motion and measure swarm spacing."""
from itertools import combinations
import numpy as np
from .model import advance
from .planner import formation3d, assign_targets

SAFE = 0.75


def simulate(start, target, dt=0.05, horizon=30, initial_velocity=None,
             return_velocity=False):
    pos = start.copy()
    vel = np.zeros_like(pos) if initial_velocity is None else initial_velocity.copy()
    history = [pos]
    settled = 0
    for step in range(1, int(horizon/dt) + 1):
        pos, vel = advance(pos, vel, target, dt)
        history.append(pos)
        within = (np.linalg.norm(pos-target, axis=1).max() <= 0.10
                  and np.linalg.norm(vel, axis=1).max() <= 0.10)
        settled = settled + 1 if within else 0
        # K consecutive intervals require K+1 qualifying samples.
        if (settled - 1) * dt >= 0.50:
            return (np.array(history), step * dt, vel) if return_velocity else (np.array(history), step * dt)
    return (np.array(history), None, vel) if return_velocity else (np.array(history), None)


def metrics(history):
    """Closest approach along each piecewise-linear numerical step.
    Count each pair once per contiguous open interval below SAFE.
    """
    i, j = np.array(list(combinations(range(history.shape[1]), 2))).T
    relative = history[:, i] - history[:, j]
    first = relative[:-1]
    change = np.diff(relative, axis=0)
    squared = np.sum(change**2, axis=2)
    fraction = np.divide(-np.sum(first*change, axis=2), squared,
                         out=np.zeros_like(squared), where=squared > 0)
    closest = np.linalg.norm(first + np.clip(fraction, 0, 1)[:, :, None]*change, axis=2)
    unsafe = closest < SAFE
    entries = unsafe & (np.linalg.norm(first, axis=2) >= SAFE)
    episodes = entries.sum() + (np.linalg.norm(relative[0], axis=1) < SAFE).sum()
    distance = np.linalg.norm(np.diff(history, axis=0), axis=2).sum()
    return closest.min(), int(episodes), distance, closest.min(axis=1)


def run_show(assignment, dt, final_shape="helix", make_formation=formation3d, first_shape="sphere"):
    """Preserve positions and velocities across both legs and the grid hold."""
    pos = make_formation(first_shape)
    vel = np.zeros_like(pos)
    history = [pos.copy()]
    goals = [pos.copy()]
    labels = [first_shape.title()]
    for source, destination in ((first_shape, "grid"), ("grid", final_shape)):
        goal = assign_targets(pos, make_formation(destination), assignment)
        h, time, vel = simulate(pos, goal, dt, initial_velocity=vel, return_velocity=True)
        gap, count, distance, _ = metrics(h)
        print(f"{assignment} {source}->{destination}: min={gap:.3f} m, episodes={count}, "
              f"time={time if time is None else format(time, '.3f')} s, distance={distance:.3f} m")
        history.extend(h[1:])
        goals.extend([goal] * (len(h)-1))
        labels.extend([source.title()+" to "+destination.title()] * (len(h)-1))
        pos = h[-1].copy()
        if time is None:
            print("Transition timed out; sequence stopped.")
            break
        # Keep applying the controller while holding; never teleport or reset velocity.
        for _ in range(round(1.0/dt)):
            pos, vel = advance(pos, vel, goal, dt)
            history.append(pos.copy())
            goals.append(goal)
            labels.append("Holding "+destination)
    return np.array(history), np.array(goals), labels
