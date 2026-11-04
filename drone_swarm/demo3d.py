"""60-drone 3D show. Run python -m drone_swarm.demo3d; drag the live view to rotate."""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.distance import cdist
from .simulation import SAFE, run_show
from .planner import formation3d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shape", choices=["helix", "x", "heart"], default="helix")
    parser.add_argument("--assignment", choices=["index", "minimum"], default="minimum")
    parser.add_argument("--gif", metavar="FILE", help="Save an animated GIF instead of opening the window")
    args = parser.parse_args()
    dt = 0.05
    h, goals, labels = run_show(args.assignment, dt, args.shape, formation3d, "sphere")
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set(xlim=(-6,6), ylim=(-6,6), zlim=(0,14), xlabel="x (m)", ylabel="y (m)", zlabel="Altitude (m)")
    ax.set_box_aspect((12,12,14))
    ax.view_init(elev=18, azim=-74)
    targets, = ax.plot([], [], [], linestyle="", marker="x", ms=4, color="gray", alpha=.45)
    normal, = ax.plot([], [], [], linestyle="", marker="o", ms=4, color="tab:blue")
    unsafe, = ax.plot([], [], [], linestyle="", marker="o", ms=6, color="crimson")
    caption = fig.text(.5, .06, "", ha="center")
    fig.text(.5, .02, "Gray × = target     Red = spacing violation", ha="center")
    def frame(k):
        pos = h[k]
        gap = cdist(pos, pos)
        np.fill_diagonal(gap, np.inf)
        flagged = gap.min(axis=1) < SAFE
        normal.set_data_3d(*pos[~flagged].T)
        unsafe.set_data_3d(*pos[flagged].T)
        targets.set_data_3d(*goals[k].T)
        ax.set_title(f"60 drones   |   {labels[k]}   |   {args.assignment} assignment")
        caption.set_text(f"Time {k*dt:.3f} s     Closest pair {gap.min():.3f} m     Required {SAFE:.3f} m")
    frames = list(range(0, len(h), 2))
    if frames[-1] != len(h)-1:
        frames.append(len(h)-1)
    animation = FuncAnimation(fig, frame, frames=frames, interval=100, repeat=True)
    if args.gif:
        animation.save(args.gif, writer="pillow", fps=10, dpi=80)
    else:
        plt.show()


if __name__ == "__main__":
    main()
