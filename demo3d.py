"""60-drone 3D show. Run python demo3d.py or drag the live view to rotate."""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.distance import cdist
from demo import SAFE, run_show
from planner import formation


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
    outline = formation(name)
    layers = []
    for depth in (-2, 0, 2):
        scale = 0.8 if depth and name != "grid" else 1
        layers.append(np.column_stack((scale*outline[:, 0],
                                       np.full(len(outline), depth),
                                       7 + scale*outline[:, 1])))
    return np.vstack(layers)


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
