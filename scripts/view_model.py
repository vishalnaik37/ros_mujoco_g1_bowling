#!/usr/bin/env python3
"""Load a MuJoCo XML and show it in the passive viewer."""
import sys
import time

import mujoco
import mujoco.viewer


def main():
    if len(sys.argv) < 2:
        print("usage: view_model.py <scene.xml>")
        sys.exit(1)
    xml_path = sys.argv[1]

    model = mujoco.MjModel.from_xml_path(xml_path)   # static description
    data = mujoco.MjData(model)                      # changing state

    print(f"nq={model.nq}  nv={model.nv}  nu={model.nu}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)   # advance physics by one step
            viewer.sync()                 # push the new state to the viewer

            # keep roughly real time
            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
