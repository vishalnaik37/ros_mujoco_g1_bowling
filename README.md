# ros_mujoco_g1_bowling

Unitree **G1** humanoid with **Dex3-1** hands learning to bowl a cricket ball in
**MuJoCo**, with ROS 2 Jazzy for visualization and tooling.

Goal: the G1 picks a ball from a tray on a table, runs up along the pitch, and
bowls it at the stumps.

| Stage | Status |
|---|---|
| G1 29-DoF + Dex3-1 model (URDF + MJCF) | ✅ done |
| Bowling scene (table, tray, ball, pitch, stumps) | ✅ done |
| Joint PD hold (fixed base) | ✅ done |
| Walking from velocity commands (pretrained policy) | ✅ works at 0.5–0.6 m/s, see [Known limitations](#known-limitations) |
| Grasping the ball with Dex3 | ⬜ next |
| Run-up + delivery + release timing | ⬜ planned |
| Walking/running policy retrained on G1 + Dex3 | ⬜ planned (needs GPU) |

**New to this?** Start with the [Reading Guide](docs/reading_guide.md): the
papers and books behind each part of the project, with a week-by-week plan
tied to exercises in this repo. The detailed step-by-step plan is in
[Reading Plan (PDF)](docs/reading_plan.pdf). To work through the project
hands-on, follow the [Training Plan](docs/training_plan.md): eight weeks of
daily tasks, each with a "done when" check.

## Packages

| Package | Contents |
|---|---|
| [`g1_description`](src/g1_description) | G1 23/29-DoF URDF + MJCF. Vendored from [isri-aist/g1_description](https://github.com/isri-aist/g1_description) with fixes (MJCF mesh path, ROS 2 RViz config, `display.launch.py`). |
| [`dex3_description`](src/dex3_description) | Dex3-1 left/right hand URDFs and meshes. |
| [`g1_dex3_description`](src/g1_dex3_description) | Combined G1 29-DoF + Dex3-1 robot: URDF for RViz, MJCF for MuJoCo, the bowling scene, and a display launch for both. |
| [`g1_bowling_control`](src/g1_bowling_control) | Controllers: `WalkController` (Unitree pretrained walking policy + heading hold, speed correction, command ramping), `JointHold` (PD), keyboard-driven MuJoCo viewer. |

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- Python 3.12 virtual environment with `mujoco` (tested 3.11), `torch`, `numpy`, `pyyaml`, `mink`

ROS 2 Jazzy uses the system Python 3.12, so the venv must be Python 3.12 and
able to see the apt-installed ROS Python packages:

```bash
python3 -m venv --system-site-packages ~/mujoco_env310
source ~/mujoco_env310/bin/activate
pip install mujoco torch mink
```

If you already have a venv without system packages, set
`include-system-site-packages = true` in its `pyvenv.cfg`.

## Build

```bash
git clone https://github.com/vishalnaik37/ros_mujoco_g1_bowling.git
cd ros_mujoco_g1_bowling
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

In every new terminal, in this order:

```bash
source ~/mujoco_env310/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Run

**Robot in RViz** (joint sliders):

```bash
ros2 launch g1_dex3_description display.launch.py
```

**Robot in the MuJoCo bowling scene** (pelvis held in place, joints PD-held):

```bash
ros2 launch g1_dex3_description display.launch.py viewer:=mujoco
ros2 launch g1_dex3_description display.launch.py viewer:=mujoco scene:=plain fixed_base:=false
```

**Walk with the keyboard** (click the viewer window first):

```bash
ros2 launch g1_bowling_control walk.launch.py                  # plain floor
ros2 launch g1_bowling_control walk.launch.py scene:=bowling   # bowling scene
```

| Key | Action |
|---|---|
| ↑ / ↓ | forward speed ± 0.1 m/s |
| ← / → | turn left / right ± 0.1 rad/s |
| Page Up / Page Down | sideways ± 0.1 m/s |
| Home | stop |

**Dex3-1 hand only** (RViz): `ros2 launch dex3_description display.launch.py hand:=right`

## Bowling scene layout

World frame, metres, z up. Cricket dimensions from the Laws of Cricket.

```
            +y  (run-up and pitch direction)
            ^
  y = 26.12 |   target stumps + bails (free bodies, can be knocked over)
            |       ... 20.12 m pitch, centred on x = 0.6 ...
  y =  7.22 |   popping crease: release before this line
  y =  6.00 |   bowler's stumps (fixed)
            |
  y =  0    G1 (facing +x) ----> table + tray + ball (front edge x = 0.7)
            +--------------------------------------------------> +x
```

Useful named elements: bodies `ball`, `tray`, `table`, `stump_far_*`,
`bail_far_*`; sites `tray_center`, `release_line`, `target`.

## Regenerating the combined model

`g1_dex3_description/urdf/g1_29dof_dex3.urdf` and
`g1_dex3_description/mjcf/g1_29dof_dex3.xml` are **generated**: the script
replaces the G1 rubber hands with Dex3-1 hands, adds hand motors, motor
armature for every G1 joint, and contact exclusions. Do not edit them by hand;
change `g1_description` / `dex3_description` or the script and run:

```bash
python src/g1_dex3_description/scripts/generate_models.py
```

Meshes are not copied: both files reference the STLs in `g1_description` and
`dex3_description`.

## Model notes

- Sizes: `nq = 50`, `nv = 49`, `nu = 43` (29 body + 14 hand torque motors).
  Joint angles start at `qpos[7:]`, joint velocities at `qvel[6:]`.
- The original G1 MJCF has no motor armature, which makes light links (ankle
  roll ≈ 2e-7 kg·m²) unstable under any PD control. The generator adds the real
  G1 motor armature (0.0036–0.025 kg·m²).
- In the G1 zero pose the forearms point forward, so the hands sit about
  0.37 m in front of the pelvis at table height.

## Known limitations

- **Walking policy vs Dex3 mass.** The pretrained policy
  (`unitree_rl_gym`, `deploy/pre_train/g1/motion.pt`) was trained with rubber
  hands (0.34 kg total). The Dex3 hands weigh 1.39 kg, which makes the policy
  drift and fall. Until the policy is retrained on this model,
  `g1_bowling_control/config/g1_walk.yaml` scales the hand mass by
  `hand_mass_scale: 0.25`. Set it back to `1.0` with a retrained policy.
- Reliable walking speed is about 0.5–0.6 m/s; 0.8 m/s falls in roughly 1 of 8
  run-ups. Running needs a retrained policy.

## Credits and licenses

- G1 and Dex3-1 models and meshes: [Unitree Robotics](https://github.com/unitreerobotics).
- `g1_description` is vendored from
  [isri-aist/g1_description](https://github.com/isri-aist/g1_description)
  (no license declared upstream).
- Walking policy from
  [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym),
  BSD 3-Clause, see [`policies/g1_walk/LICENSE`](src/g1_bowling_control/policies/g1_walk/LICENSE).
- Packages written in this repository: BSD 3-Clause.
