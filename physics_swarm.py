"""Replay the 60-drone baseline reference using simulated Crazyflies."""
import argparse
import time
import numpy as np
import pybullet as pb
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel
from demo import metrics, run_show
from demo3d import formation3d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    dt = 0.05
    hz = 48
    print("Point-model reference:")
    path, goals, labels = run_show("minimum", dt, "helix", formation3d, "sphere")
    model = DroneModel.CF2X
    print("Loading 60 quadrotors…", flush=True)
    env = CtrlAviary(drone_model=model, num_drones=len(path[0]), initial_xyzs=path[0],
                     ctrl_freq=hz, pyb_freq=240, gui=args.gui, user_debug_gui=False)
    try:
        controllers = [DSLPIDControl(drone_model=model) for _ in path[0]]
        # CtrlAviary already loaded the bodies and sampled their initial states.
        obs = env._computeObs()
        if args.gui:
            pb.resetDebugVisualizerCamera(18, 35, -15, [0, 0, 7], physicsClientId=env.CLIENT)
            for body in env.DRONE_IDS:
                pb.addUserDebugText("●", [0, 0, 0], [0, .6, 1], 1.3,
                                    parentObjectUniqueId=int(body), physicsClientId=env.CLIENT)
        history = [obs[:, :3].copy()]
        phases = ["sphere"]
        start = time.monotonic()
        for tick in range(int(np.ceil((len(path)-1)*dt*hz))):
            frame = tick / (hz*dt)
            i = min(int(frame), len(path)-2)
            if labels[i+1] != phases[-1]:
                print(f"Flying: {labels[i+1]} ({tick/hz:.3f} s)", flush=True)
            delta = path[i+1] - path[i]
            target = path[i] + (frame-i)*delta
            rpm = [controller.computeControlFromState(
                control_timestep=1/hz, state=state, target_pos=pos, target_vel=vel)[0]
                for controller, state, pos, vel in zip(controllers, obs, target, delta/dt)]
            obs, _, done, truncated, _ = env.step(np.asarray(rpm))
            history.append(obs[:, :3].copy())
            phases.append(labels[i+1])
            if done or truncated:
                raise RuntimeError("Physics simulation stopped before completing the show")
            if args.gui:
                time.sleep(max(0, start+(tick+1)/hz-time.monotonic()))
        actual = np.asarray(history)
        print("PyBullet measurements (including the hold after each transition):")
        split = phases.index("Grid to Helix") - 1
        for name, positions, goal in (("grid", actual[:split+1], goals[labels.index("Holding grid")]),
                                      ("helix", actual[split:], goals[-1])):
            gap, count, _, _ = metrics(positions)
            error = np.linalg.norm(positions[-1]-goal, axis=1).max()
            print(f"{name}: closest={gap:.3f} m, episodes={count}, worst final error={error:.3f} m")
    finally:
        if pb.isConnected(env.CLIENT):
            env.close()


if __name__ == "__main__":
    main()
