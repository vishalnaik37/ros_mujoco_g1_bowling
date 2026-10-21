"""Velocity-commanded walking for the G1 using Unitree's pretrained policy.

Layers, from the top:
  command (vx, vy, wz)          from the user / task state machine
  -> outer loops                heading hold + body-speed correction
  -> policy (50 Hz)             target leg joint angles
  -> leg PD (every sim step)    leg torques
The upper body and hands are held still by a JointHold.
"""
from pathlib import Path

import mujoco
import numpy as np
import torch
import yaml

from g1_bowling_control.sim import JointHold, share_dir


def package_dir():
    root = share_dir("g1_bowling_control")
    return root if root is not None else Path(__file__).resolve().parents[1]


def load_walk_config(path=None):
    path = Path(path) if path else package_dir() / "config" / "g1_walk.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def gravity_in_body(quat):
    """Gravity direction in the pelvis frame (as in unitree_rl_gym)."""
    w, x, y, z = quat
    return np.array([2 * (-z * x + w * y),
                     -2 * (z * y + w * x),
                     1 - 2 * (w * w + z * z)])


def yaw_of(quat):
    w, x, y, z = quat
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class WalkController:
    """Call step(data) once per physics step; set the command with set_command()."""

    def __init__(self, model, cfg):
        self.model = model
        self.cfg = cfg
        self.policy = torch.jit.load(str(package_dir() / cfg["policy"]))

        leg_joints = [model.joint(n).id for n in cfg["leg_joints"]]
        self.leg_qadr = model.jnt_qposadr[leg_joints]
        self.leg_dadr = model.jnt_dofadr[leg_joints]
        self.leg_act = np.array([model.actuator(n).id for n in cfg["leg_joints"]])
        for dof in self.leg_dadr:
            model.dof_armature[dof] = cfg["leg_armature"]
            model.dof_damping[dof] = cfg["leg_damping"]
            model.dof_frictionloss[dof] = cfg["leg_frictionloss"]

        upper = [a for a in range(model.nu) if a not in set(self.leg_act)]
        self.upper_hold = JointHold(model, upper)

        self.kps = np.array(cfg["kps"], dtype=float)
        self.kds = np.array(cfg["kds"], dtype=float)
        self.default = np.array(cfg["default_angles"], dtype=np.float32)
        self.cmd_scale = np.array(cfg["cmd_scale"], dtype=np.float32)
        self.cmd_accel = np.array(cfg["cmd_accel_limit"])
        self.cmd_min = np.array(cfg["policy_cmd_min"])
        self.cmd_max = np.array(cfg["policy_cmd_max"])
        self.decimation = int(round(cfg["control_dt"] / model.opt.timestep))
        self.pelvis = model.body("pelvis").id

        self.desired = np.zeros(3)   # what the user asked for
        self.command = np.zeros(3)   # ramped toward desired at cmd_accel_limit
        self.reset_state()

    def reset_state(self, heading=0.0):
        self.target = self.default.copy()
        self.action = np.zeros(len(self.default), dtype=np.float32)
        self.obs = np.zeros(9 + 3 * len(self.default) + 2, dtype=np.float32)
        self.heading = heading
        self.command[:] = 0.0
        self.vel_filtered = np.zeros(2)
        self.vel_integral = np.zeros(2)
        self.steps = 0

    def reset(self, data):
        """Put the legs in the policy's default pose and clear controller state."""
        data.qpos[self.leg_qadr] = self.default
        data.qvel[self.leg_dadr] = 0.0
        mujoco.mj_forward(self.model, data)
        self.reset_state(heading=yaw_of(data.qpos[3:7]))

    def set_command(self, vx, vy, wz):
        """Desired body velocity: forward [m/s], left [m/s], yaw rate [rad/s].

        The robot ramps to it at cmd_accel_limit; sudden stops make it fall.
        """
        self.desired[:] = (vx, vy, wz)

    def set_heading(self, yaw):
        """Turn to a world yaw [rad] (0 = +x, pi/2 = +y) using the heading hold.

        The turn rate is limited by policy_cmd_max[2]. Use with wz = 0.
        """
        self.heading = wrap_angle(yaw)

    def heading_error(self, data):
        return wrap_angle(self.heading - yaw_of(data.qpos[3:7]))

    def step(self, data):
        self.upper_hold.apply(data)
        data.ctrl[self.leg_act] = (self.kps * (self.target - data.qpos[self.leg_qadr])
                                   - self.kds * data.qvel[self.leg_dadr])
        self.steps += 1
        if self.steps % self.decimation == 0:
            self._policy_update(data)

    def _corrected_command(self, data):
        cfg, dt = self.cfg, self.cfg["control_dt"]
        max_change = self.cmd_accel * dt
        self.command += np.clip(self.desired - self.command, -max_change, max_change)
        cmd = self.command.copy()

        # Heading hold: integrate the commanded yaw rate into a desired heading
        # and correct the yaw-rate command toward it (stops slow turning drift).
        self.heading = wrap_angle(self.heading + self.command[2] * dt)
        heading_err = wrap_angle(self.heading - yaw_of(data.qpos[3:7]))
        cmd[2] += cfg["heading_kp"] * heading_err

        # Speed correction on body velocity, low-passed over a gait cycle
        # because the pelvis speeds up and slows down within every step.
        rot = data.xmat[self.pelvis].reshape(3, 3)
        vel_body = (rot.T @ data.qvel[0:3])[:2]
        self.vel_filtered += (vel_body - self.vel_filtered) * dt / cfg["speed_filter_tau"]
        err = self.command[:2] - self.vel_filtered
        lim = cfg["speed_integral_limit"]
        self.vel_integral = np.clip(self.vel_integral + err * dt, -lim, lim)
        cmd[:2] += cfg["speed_kp"] * err + cfg["speed_ki"] * self.vel_integral

        return np.clip(cmd, self.cmd_min, self.cmd_max)

    def _policy_update(self, data):
        cfg, n = self.cfg, len(self.default)
        t = self.steps * self.model.opt.timestep
        phase = (t % cfg["gait_period"]) / cfg["gait_period"]

        obs = self.obs
        obs[0:3] = data.qvel[3:6] * cfg["ang_vel_scale"]
        obs[3:6] = gravity_in_body(data.qpos[3:7])
        obs[6:9] = self._corrected_command(data) * self.cmd_scale
        obs[9:9 + n] = (data.qpos[self.leg_qadr] - self.default) * cfg["dof_pos_scale"]
        obs[9 + n:9 + 2 * n] = data.qvel[self.leg_dadr] * cfg["dof_vel_scale"]
        obs[9 + 2 * n:9 + 3 * n] = self.action
        obs[9 + 3 * n:] = (np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase))

        with torch.no_grad():
            self.action = self.policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
        self.target = self.action * cfg["action_scale"] + self.default
