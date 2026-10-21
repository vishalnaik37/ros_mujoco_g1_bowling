#!/usr/bin/env python3
"""Drive the G1 + Dex3-1 with the walking policy in the MuJoCo viewer.

Keys (click the viewer window first):
  Up / Down           forward speed  +/- 0.1 m/s
  Left / Right        turn left / right  +/- 0.1 rad/s
  Page Up / Page Down sideways (left / right)  +/- 0.1 m/s
  Home                stop (all commands to zero)
"""
import argparse
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

from g1_bowling_control.sim import load_model, scene_path
from g1_bowling_control.walk import WalkController, load_walk_config, yaw_of

SCENES = {"plain": "scene.xml", "bowling": "bowling_scene.xml"}

KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT = 265, 264, 263, 262
KEY_PAGE_UP, KEY_PAGE_DOWN, KEY_HOME = 266, 267, 268

CMD_STEP = np.array([0.1, 0.1, 0.1])
CMD_MIN = np.array([-0.5, -0.3, -0.8])
CMD_MAX = np.array([1.0, 0.3, 0.8])


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", default="plain", choices=SCENES)
    parser.add_argument("--config", default=None, help="walk config yaml (default: package config)")
    parser.add_argument("--cmd", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("VX", "VY", "WZ"), help="initial velocity command")
    args, _ = parser.parse_known_args()  # ignore --ros-args from ros2 launch

    cfg = load_walk_config(args.config)
    model = load_model(scene_path(SCENES[args.scene]), fixed_base=False,
                       hand_mass_scale=cfg["hand_mass_scale"])
    data = mujoco.MjData(model)
    walker = WalkController(model, cfg)
    walker.reset(data)

    if cfg["hand_mass_scale"] != 1.0:
        print(f"[walk] Dex3 hand mass scaled by {cfg['hand_mass_scale']} (temporary, see g1_walk.yaml)")

    command = np.array(args.cmd, dtype=float)
    lock = threading.Lock()

    def on_key(key):
        with lock:
            if key == KEY_UP:
                command[0] += CMD_STEP[0]
            elif key == KEY_DOWN:
                command[0] -= CMD_STEP[0]
            elif key == KEY_PAGE_UP:
                command[1] += CMD_STEP[1]
            elif key == KEY_PAGE_DOWN:
                command[1] -= CMD_STEP[1]
            elif key == KEY_LEFT:
                command[2] += CMD_STEP[2]
            elif key == KEY_RIGHT:
                command[2] -= CMD_STEP[2]
            elif key == KEY_HOME:
                command[:] = 0.0
            else:
                return
            np.clip(command, CMD_MIN, CMD_MAX, out=command)
            command[np.abs(command) < 1e-9] = 0.0
            print(f"[walk] command vx={command[0]:+.1f} vy={command[1]:+.1f} wz={command[2]:+.1f}")

    print(__doc__)
    pelvis = model.body("pelvis").id
    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = pelvis
        viewer.cam.distance = 3.0
        next_print = 0.0
        while viewer.is_running():
            step_start = time.time()
            with lock:
                walker.set_command(*command)
            walker.step(data)
            mujoco.mj_step(model, data)
            viewer.sync()

            if data.time >= next_print:
                next_print += 1.0
                rot = data.xmat[pelvis].reshape(3, 3)
                v = rot.T @ data.qvel[0:3]
                print(f"[walk] t={data.time:5.1f}s  pos=({data.qpos[0]:+.2f}, {data.qpos[1]:+.2f})"
                      f"  yaw={np.degrees(yaw_of(data.qpos[3:7])):+5.0f} deg"
                      f"  speed=({v[0]:+.2f}, {v[1]:+.2f}) m/s  height={data.qpos[2]:.2f}")
                if data.qpos[2] < 0.5:
                    print("[walk] robot fell; press Backspace in the viewer to reset")

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
