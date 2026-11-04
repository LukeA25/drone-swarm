"""Fly one simulated Crazyflie using the simulator's built-in controller."""
import argparse
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet 3D view")
    args = parser.parse_args()
    try:
        from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
        from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
        from gym_pybullet_drones.utils.enums import DroneModel
    except ModuleNotFoundError as exc:
        parser.exit(1, f"Missing dependency ({exc.name}). Follow the physics setup in README.md.\n")

    hz = 48
    model = DroneModel.CF2X
    home = np.array([0., 0., 1.])
    goal = home + [1., 0., 0.]
    env = CtrlAviary(drone_model=model, initial_xyzs=home[None, :],
                     pyb_freq=240, ctrl_freq=hz, gui=args.gui)
    try:
        controller = DSLPIDControl(drone_model=model)
        obs, _ = env.reset()
        peak_tilt = 0.
        for tick in range(8 * hz):
            target = home if tick < 2 * hz else goal
            rpm, _, _ = controller.computeControlFromState(
                control_timestep=1 / hz, state=obs[0], target_pos=target)
            obs, _, terminated, truncated, _ = env.step(rpm.reshape(1, 4))
            # Quaternion entries 3:7 are x, y, z, w. Rzz gives vertical thrust alignment.
            qx, qy = obs[0, 3:5]
            tilt = np.degrees(np.arccos(np.clip(1 - 2*(qx*qx + qy*qy), -1, 1)))
            peak_tilt = max(peak_tilt, tilt)
            if (tick + 1) % hz == 0:
                x, y, z = obs[0, :3]
                print(f"t={(tick+1)/hz:.3f} s   xyz=({x:.3f}, {y:.3f}, {z:.3f}) m   tilt={tilt:.3f} deg")
            if args.gui:
                import time
                time.sleep(1 / hz)
            if terminated or truncated:
                break
        print(f"Peak tilt {peak_tilt:.3f} deg; final distance from goal "
              f"{np.linalg.norm(obs[0, :3] - goal):.3f} m")
    finally:
        env.close()


if __name__ == "__main__":
    main()
