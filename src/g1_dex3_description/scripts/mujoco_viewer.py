#!/usr/bin/env python3
"""Open a G1 + Dex3-1 MJCF scene in the interactive MuJoCo viewer.

The MJCF references meshes in g1_description and dex3_description by paths
relative to the source tree; this script re-points them at the installed
share directories so it works from any colcon install layout.
"""
import argparse
from pathlib import Path

import mujoco
import mujoco.viewer
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)

import numpy as np

MESH_PACKAGES = ("g1_description", "dex3_description")
# Package root when run straight from the source tree (src/<pkg>/scripts/).
SOURCE_PKG_DIR = Path(__file__).resolve().parents[1]

# PD gains (kp [Nm/rad], kd [Nm s/rad]) by joint-name fragment; first match wins.
# Leg values follow unitree_rl_gym's G1 config.
PD_GAINS = [
    ("_hand_", 1.0, 0.01),
    ("hip", 100.0, 2.0),
    ("knee", 150.0, 4.0),
    ("ankle", 40.0, 2.0),
    ("waist", 200.0, 5.0),
    ("wrist", 20.0, 0.5),
    ("shoulder", 40.0, 1.0),
    ("elbow", 40.0, 1.0),
]


def share_dir(pkg):
    """Installed share directory of pkg, or None if it is not on the ament path."""
    try:
        return Path(get_package_share_directory(pkg))
    except PackageNotFoundError:
        return None


def load_model(scene, fixed_base):
    spec = mujoco.MjSpec.from_file(str(scene))

    # Included files arrive with paths already joined to the scene folder,
    # e.g. ".../share/g1_dex3_description/mjcf/../../g1_description/meshes/x",
    # so match on the "/<package>/" part and re-root it at that package.
    # Packages not on the ament path keep their source-tree relative path.
    for mesh in spec.meshes:
        path = "/" + mesh.file
        for pkg in MESH_PACKAGES:
            idx = path.rfind(f"/{pkg}/")
            share = share_dir(pkg)
            if idx >= 0 and share is not None:
                mesh.file = str(share / path[idx + len(pkg) + 2:])
                break

    if fixed_base:
        spec.delete(spec.joint("floating_base_joint"))

    return spec.compile()


def make_pd_hold(model, gravity_comp):
    """Control callback holding every actuated joint at its qpos0 angle.

    With gravity_comp, adds the gravity/Coriolis feed-forward (qfrc_bias) so
    joints do not sag. Only valid with a fixed base: on a floating base the
    bias includes forces on the pelvis that no joint motor can produce.
    """
    joints = model.actuator_trnid[:, 0]
    qadr = model.jnt_qposadr[joints]
    dadr = model.jnt_dofadr[joints]
    target = model.qpos0[qadr].copy()
    kp = np.zeros(model.nu)
    kd = np.zeros(model.nu)
    for i, j in enumerate(joints):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        for fragment, p, d in PD_GAINS:
            if fragment in name:
                kp[i], kd[i] = p, d
                break

    def control(model, data):
        data.ctrl[:] = kp * (target - data.qpos[qadr]) - kd * data.qvel[dadr]
        if gravity_comp:
            data.ctrl[:] += data.qfrc_bias[dadr]

    return control


def main():
    default_scene = (share_dir("g1_dex3_description") or SOURCE_PKG_DIR) / "mjcf" / "scene.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default=str(default_scene))
    parser.add_argument("--fixed-base", action="store_true",
                        help="weld the pelvis in place so the robot does not fall")
    parser.add_argument("--no-hold", action="store_true",
                        help="disable the PD pose hold (joints go limp, Control sliders work)")
    args, _ = parser.parse_known_args()  # ignore --ros-args from ros2 launch

    model = load_model(args.scene, args.fixed_base)
    print(f"nq={model.nq}  nv={model.nv}  nu={model.nu}")
    if not args.no_hold:
        mujoco.set_mjcb_control(make_pd_hold(model, gravity_comp=args.fixed_base))
    mujoco.viewer.launch(model)


if __name__ == "__main__":
    main()
