"""Run two deterministic assignments with the same baseline controller."""
import argparse
from itertools import combinations
import numpy as np
from model import advance
from planner import formation, assign_targets

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


def run_show(assignment, dt, final_shape="x", make_formation=formation, first_shape="circle"):
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


def main():
    parser = argparse.ArgumentParser(description="Circle to grid to a selected shape with local target tracking.")
    parser.add_argument("--shape", choices=["x", "heart"], default="x",
                        help="Final formation after grid (default: x).")
    parser.add_argument("--plot", metavar="PNG")
    parser.add_argument("--animate", choices=["index", "minimum"])
    parser.add_argument("--dt", type=float, choices=[0.05, 0.025], default=0.05)
    args = parser.parse_args()
    runs = {name: run_show(name, args.dt, args.shape) for name in ("index", "minimum")}
    if args.plot or args.animate:
        import matplotlib.pyplot as plt
    if args.plot:
        fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
        for ax, (name, (h, goals, labels)) in zip(axes, runs.items()):
            ax.plot(h[:, :, 0], h[:, :, 1], lw=1)
            ax.scatter(*formation("grid").T, marker="+", color="gray")
            ax.scatter(*formation(args.shape).T, marker="x", color="black", s=16)
            ax.set(aspect="equal", title=name.capitalize()+" assignment", xlabel="x (m)", ylabel="y (m)")
            trace = metrics(h)[3]
            axes[2].plot(np.arange(len(trace))*args.dt, trace, label=name)
        axes[2].axhline(SAFE, color="black", ls="--", label="spacing threshold")
        axes[2].set(xlabel="Step start (s)", ylabel="Closest pair (m)", title="Separation throughout show")
        axes[2].legend()
        fig.savefig(args.plot, dpi=150)
        plt.close(fig)
    if args.animate:
        from matplotlib.animation import FuncAnimation
        from matplotlib.patches import Circle
        h, goals, labels = runs[args.animate]
        fig, ax = plt.subplots()
        ax.set(xlim=(-7, 7), ylim=(-7, 7), aspect="equal", xlabel="x (m)", ylabel="y (m)")
        targets = ax.scatter(*goals[0].T, marker="x", color="gray", label="Assigned targets")
        dots = ax.scatter(*h[0].T, s=30, zorder=3, label="Drones (red when too close)")
        rings = [Circle(p, SAFE/2, fill=False, alpha=.45, color="tab:blue") for p in h[0]]
        for ring in rings:
            ax.add_patch(ring)
        status = ax.text(.02, .02, "", transform=ax.transAxes, fontsize=9)
        ax.legend(loc="upper right", fontsize=8)
        def frame(k):
            separation = np.linalg.norm(h[k,:,None,:]-h[k,None,:,:], axis=2)
            np.fill_diagonal(separation, np.inf)
            unsafe = (separation < SAFE).any(axis=1)
            colors = np.where(unsafe, "tab:red", "tab:blue")
            dots.set_offsets(h[k])
            dots.set_color(colors)
            targets.set_offsets(goals[k])
            for ring, p, color in zip(rings, h[k], colors):
                ring.center = p
                ring.set_color(color)
            ax.set_title(f"{args.animate} | {labels[k]} | {k*args.dt:.3f} s")
            status.set_text(f"Closest pair now {separation.min():.3f} m | required {SAFE:.3f} m\n"
                            "Rings show half the required separation; overlap = violation")
        animation = FuncAnimation(fig, frame, frames=len(h), interval=1000*args.dt, repeat=True)
        plt.show()


if __name__ == "__main__":
    main()
