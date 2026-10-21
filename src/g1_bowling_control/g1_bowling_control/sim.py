"""Loading the G1 + Dex3-1 MuJoCo scenes and holding joints with PD."""
from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)

MESH_PACKAGES = ("g1_description", "dex3_description")

# PD gains (kp [Nm/rad], kd [Nm s/rad]) by joint-name fragment; first match wins.
HOLD_GAINS = [
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


def scene_path(name):
    """Path of a scene in g1_dex3_description/mjcf ("bowling_scene.xml", ...)."""
    root = share_dir("g1_dex3_description")
    if root is None:  # running from the source tree without the workspace sourced
        root = Path(__file__).resolve().parents[2] / "g1_dex3_description"
    return root / "mjcf" / name


def load_model(scene, fixed_base=False, hand_mass_scale=1.0):
    """Compile a scene, re-pointing mesh paths at the installed packages."""
    spec = mujoco.MjSpec.from_file(str(scene))

    # Included files arrive with paths already joined to the scene folder, so
    # match on the "/<package>/" part and re-root it at that package.
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

    model = spec.compile()

    if hand_mass_scale != 1.0:
        for b in range(model.nbody):
            if "_hand_" in model.body(b).name:
                model.body_mass[b] *= hand_mass_scale
                model.body_inertia[b] *= hand_mass_scale
    return model


class JointHold:
    """PD hold of a set of actuated joints at fixed target angles."""

    def __init__(self, model, actuators=None, targets=None, gravity_comp=False):
        if actuators is None:
            actuators = range(model.nu)
        self.act = np.array(list(actuators), dtype=int)
        joints = model.actuator_trnid[self.act, 0]
        self.qadr = model.jnt_qposadr[joints]
        self.dadr = model.jnt_dofadr[joints]
        self.target = (model.qpos0[self.qadr].copy() if targets is None
                       else np.asarray(targets, dtype=float))
        self.gravity_comp = gravity_comp

        self.kp = np.zeros(len(self.act))
        self.kd = np.zeros(len(self.act))
        for i, j in enumerate(joints):
            name = model.joint(j).name
            for fragment, kp, kd in HOLD_GAINS:
                if fragment in name:
                    self.kp[i], self.kd[i] = kp, kd
                    break

    def apply(self, data):
        tau = self.kp * (self.target - data.qpos[self.qadr]) - self.kd * data.qvel[self.dadr]
        if self.gravity_comp:  # only valid with a fixed base
            tau += data.qfrc_bias[self.dadr]
        data.ctrl[self.act] = tau
