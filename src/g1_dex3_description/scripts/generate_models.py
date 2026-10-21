#!/usr/bin/env python3
"""Build the G1 29-DoF + Dex3-1 model from g1_description and dex3_description.

Replaces the G1 rubber hands with Dex3-1 hands and writes:
  urdf/g1_29dof_dex3.urdf  for robot_state_publisher / RViz
  mjcf/g1_29dof_dex3.xml   robot only, for MuJoCo (include it from a scene)

No meshes are copied: both files reference the STLs in g1_description and
dex3_description.

Run from anywhere, with the MuJoCo venv active:
  python src/g1_dex3_description/scripts/generate_models.py
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

PKG_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PKG_DIR.parent
G1_DIR = SRC_DIR / "g1_description"
DEX3_DIR = SRC_DIR / "dex3_description" / "dex3_1"

SIDES = {"left": "l", "right": "r"}
# Palm mount on *_wrist_yaw_link, same offset as the rubber hand it replaces.
PALM_MOUNT = {"left": [0.0415, 0.003, 0.0], "right": [0.0415, -0.003, 0.0]}
# Mesh folders as seen from <src>/g1_dex3_description/mjcf/.
MJCF_G1_MESHES = "../../g1_description/meshes"
MJCF_DEX3_MESHES = "../../dex3_description/dex3_1/meshes"

# Reflected rotor inertia [kg m^2] of the G1 joint motors (motor type in
# brackets; ankle and waist roll/pitch are driven by two 5020 motors). The
# source MJCF has none, which leaves light links like the ankle roll with
# ~2e-7 kg m^2 and makes any PD damping unstable at a 2 ms timestep.
# First matching name fragment wins.
G1_ARMATURE = [
    ("hip_roll", 0.025101),      # 7520-22
    ("knee", 0.025101),          # 7520-22
    ("hip", 0.010177),           # 7520-14 (hip pitch, hip yaw)
    ("waist_yaw", 0.010177),     # 7520-14
    ("ankle", 0.00721),          # 2 x 5020
    ("waist", 0.00721),          # 2 x 5020 (waist roll, waist pitch)
    ("wrist_pitch", 0.00425),    # 4010
    ("wrist_yaw", 0.00425),      # 4010
    ("shoulder", 0.003609),      # 5020
    ("elbow", 0.003609),         # 5020
    ("wrist_roll", 0.003609),    # 5020
]


def build_urdf():
    g1 = ET.parse(G1_DIR / "urdf" / "g1_29dof.urdf").getroot()
    g1.set("name", "g1_29dof_dex3")

    for el in g1.findall("mujoco"):
        g1.remove(el)

    for side, s in SIDES.items():
        rubber = g1.find(f"link[@name='{side}_rubber_hand']")
        if rubber is not None:
            g1.remove(rubber)
        palm_joint = g1.find(f"joint[@name='{side}_hand_palm_joint']")
        palm_joint.find("child").set("link", f"{side}_hand_palm_link")

        hand = ET.parse(DEX3_DIR / f"dex3_1_{s}.urdf").getroot()
        for el in hand:
            if el.tag in ("link", "joint"):
                g1.append(el)

    ET.indent(g1, space="  ")
    out = PKG_DIR / "urdf" / "g1_29dof_dex3.urdf"
    out.parent.mkdir(exist_ok=True)
    ET.ElementTree(g1).write(out, encoding="utf-8", xml_declaration=True)
    return out


def build_mjcf():
    spec = mujoco.MjSpec.from_file(str(G1_DIR / "mjcf" / "g1_29dof.xml"))
    spec.modelname = "g1_29dof_dex3"

    # Drop the rubber hands and the built-in floor/sky/light: the robot file
    # holds only the robot, scenes add their own environment.
    for geom in list(spec.geoms):
        if "rubber_hand" in geom.meshname or geom.name == "floor":
            spec.delete(geom)
    for mesh in list(spec.meshes):
        if "rubber_hand" in mesh.name:
            spec.delete(mesh)
    for el in list(spec.materials) + list(spec.textures):
        spec.delete(el)
    for light in list(spec.worldbody.lights):
        spec.delete(light)

    for side, s in SIDES.items():
        hand = mujoco.MjSpec.from_file(str(DEX3_DIR / f"dex3_1_{s}.urdf"))
        for geom in hand.geoms:
            geom.rgba = [0.2, 0.2, 0.2, 1]
        frame = spec.body(f"{side}_wrist_yaw_link").add_frame(pos=PALM_MOUNT[side])
        frame.attach_body(hand.body(f"{side}_hand_palm_link"), "", "")
        # The thumb's middle link rests against the palm mesh; without this
        # exclusion that contact fights every grasp.
        spec.add_exclude(bodyname1=f"{side}_hand_palm_link",
                         bodyname2=f"{side}_hand_thumb_1_link")

    for joint in spec.joints:
        for fragment, armature in G1_ARMATURE:
            if "_hand_" not in joint.name and fragment in joint.name:
                joint.armature = armature
                break

    # One torque motor per hand joint, like the G1 body joints.
    for joint in spec.joints:
        if "_hand_" not in joint.name:
            continue
        effort = joint.actfrcrange[1]
        joint.actfrclimited = mujoco.mjtLimited.mjLIMITED_TRUE
        joint.damping[0] = 0.01
        joint.armature = 1e-4
        spec.add_actuator(
            name=joint.name,
            target=joint.name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
            ctrlrange=[-effort, effort],
        )

    # Point every mesh at its source package. Compile with absolute paths
    # (attached specs resolve relative paths differently), then write them
    # relative to <src>/g1_dex3_description/mjcf/.
    g1_meshes = G1_DIR / "meshes"
    dex3_meshes = DEX3_DIR / "meshes"
    for mesh in spec.meshes:
        name = Path(mesh.file).name
        mesh.file = str((dex3_meshes if "_hand_" in mesh.name else g1_meshes) / name)
    spec.meshdir = ""
    spec.compile()

    xml = spec.to_xml()
    xml = xml.replace(f'file="{dex3_meshes}/', f'file="{MJCF_DEX3_MESHES}/')
    xml = xml.replace(f'file="{g1_meshes}/', f'file="{MJCF_G1_MESHES}/')
    # Attaching with an empty prefix leaves the hands' unnamed default
    # classes behind as empty <default/> elements; nothing uses them.
    xml = "\n".join(l for l in xml.splitlines() if l.strip() != "<default/>") + "\n"
    out = PKG_DIR / "mjcf" / "g1_29dof_dex3.xml"
    out.parent.mkdir(exist_ok=True)
    out.write_text(xml)
    return out


def main():
    print("urdf ->", build_urdf())
    mjcf = build_mjcf()
    print("mjcf ->", mjcf)

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    print(f"check: nq={model.nq} nv={model.nv} nu={model.nu}")


if __name__ == "__main__":
    main()
