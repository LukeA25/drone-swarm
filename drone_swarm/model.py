"""One sampled local PD controller per drone; rows are independent."""
import numpy as np

MAX_SPEED = 2.0
MAX_ACCEL = 2.0


def limit(rows, maximum):
    return rows * (maximum / np.maximum(maximum, np.linalg.norm(rows, axis=1, keepdims=True)))


def advance(position, velocity, target, dt):
    acceleration = limit(1.0 * (target - position) - 2.0 * velocity, MAX_ACCEL)
    next_velocity = limit(velocity + dt * acceleration, MAX_SPEED)
    return position + dt * next_velocity, next_velocity
