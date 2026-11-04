# Drone swarm baseline
20 drones transition circle → grid → heart with independent local PD tracking.
Run from this directory using Python with numpy, scipy, and matplotlib installed.

    python demo.py --animate minimum
    python demo.py --animate index
    python demo.py --plot baseline.png
    python -m unittest -v

The simulator completes both runs first, prints per-transition metrics, then
plays the chosen animation. A one-second controlled hold follows each arrival.
The minimum-distance assignment is recomputed at the start of each leg from the
actual current positions. Velocities carry through the entire sequence.

Blue rings have radius 0.375 m. Overlap means the 0.750 m center-spacing rule
is violated. Red highlights are evaluated at displayed frames. Printed metrics
also check closest approach along each numerical step. Dot sizes are graphical.

Heart geometry uses x = 16 sin³(t) and
y = 13 cos(t) − 5 cos(2t) − 2 cos(3t) − cos(4t), scaled by 0.350 m per
coordinate unit. Twenty slots are approximately equally spaced along arc length,
rather than equally spaced in t. The heart scale was deliberately selected as
a stress case after checking several scales. Its minimum target spacing is
1.556 m. At both tested steps (0.050 and 0.025 s), the minimum-distance
grid-to-heart transition yields one violation episode and about 0.746 m minimum
separation. This is a small spacing breach, not evidence of physical contact.
All endpoints are safely spaced. Assignment minimizes summed straight-line
distance without enforcing trajectory clearance.

The controller uses p_dot = v, v_dot = u. Each update computes
u = norm_limit((target − p) − 2v, 2 m/s²), updates velocity with a speed cap
of 2 m/s, and uses that velocity to update position (semi-implicit Euler).
Arrival requires all position errors ≤ 0.100 m and speeds ≤ 0.100 m/s over
sample times spanning 0.500 s. No avoidance controller is implemented.
Earlier Wednesday report numbers describe only the original circle-to-grid
experiment; this extension was added afterward.

## X transition extension
The default sequence is now circle → grid → X. Use --shape heart to restore
the previous sequence. Run python demo.py --animate minimum for the X show.
Each of four diagonal arms has five slots, at (±r, ±r) for r = 1,...,5.
There is no center slot. Minimum target spacing is 1.414 m.
This X layout was selected after comparing several scales and rotations to
produce a stronger transition stress case with safe endpoints.
At h = 0.050 s, minimum assignment produces four grid-to-X violation episodes
and a minimum separation of 0.644 m. Both are simulation observations.

## 3D show
Run python demo3d.py for 60 drones moving from a sphere to a 5 × 3 × 4 grid
and then a double helix. Drag the plot to rotate it. Use --shape x for the old X,
--shape heart for the layered heart, --assignment index for index matching,
or --gif show.gif to export. The default matching minimizes total 3D
distance. All axes are in metres. The same vector PD and acceleration limits
apply in 3D, assuming an ideal lower-level autopilot.

The helix has 30 evenly spaced heights over 5 m. Two drones occupy opposite
points on a circle of radius 2 m at each height. The angle increases from
π/2 to π/2 + 4π, giving each strand two full turns. These slots are destinations.
Each drone flies directly toward its assigned slot using the same controller.

## Single-drone physics demo (optional)

Run a separate Crazyflie CF2X simulation using the open-source
[gym-pybullet-drones](https://github.com/learnsyslab/gym-pybullet-drones) environment
and its built-in PID controller. On macOS, in a terminal with Conda initialized
and Apple's command-line build tools available:

    conda create -n swarm-physics python=3.12 -y
    conda activate swarm-physics
    CFLAGS="-Dfdopen=fdopen" python -m pip install pybullet --no-cache-dir
    python -m pip install git+https://github.com/learnsyslab/gym-pybullet-drones.git
    cd ~/drone-swarm
    python physics_demo.py --gui

Omit --gui to run headlessly. The initial setpoint is a hover at 1 m altitude.
After two simulated seconds the position setpoint shifts 1 m horizontally.
The controller commands four motor speeds. PyBullet computes the resulting
translation and rotation. The terminal reports position and tilt during flight.

In the same swarm-physics environment, run `python physics_swarm.py --gui`
for the 60-Crazyflie sphere → grid → double helix experiment. Omit `--gui`
for a headless run. Each vehicle tracks the positions and segment velocities
from the original point-model show. The final output measures spacing and
formation errors from the PyBullet positions sampled at 48 Hz.
