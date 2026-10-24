# G1 Bowling Training Plan

Eight weeks (Phases 0–4 of the roadmap) that take you from reading MuJoCo
models to a fixed-base G1 that grasps a cricket ball and bowls it at the
stumps. Interactive version with progress tracking:
[G1 Bowling Training Plan on Claude](https://claude.ai/artifact/4BmQ846Pr4ig8xMFV3ikRi).
Spreadsheet tracker (status, dates, hours): [training_plan.xlsx](training_plan.xlsx).

Each week has a **robotics track** (the main work, in this repo) and a smaller
**ML track** that builds up to PPO. Move on when the "done when" check passes,
not when the days run out. Papers referenced here are described in the
[Reading Guide](reading_guide.md).

**Time budget:** 12–15 h per week: five 1.5 h weekday sessions plus one 4–5 h
weekend block.

**Weekly rhythm:**

- Mon–Fri: one robotics task per day, about 1.5 h
- ML track: spread over the week, about 3 h in total
- Weekend: finish the deliverable, write notes, commit
- Sunday: answer the checkpoint questions without notes

**End state after week 8:** grasp, swing and release from the crease, a
measured hit rate over 50 throws, and a written comparison of CMA-ES vs PPO.

## Overview

| Week | Phase | Focus |
| --- | --- | --- |
| [1](#week-1-know-your-simulator) | Phase 0 | Know your simulator |
| [2](#week-2-joint-control) | Phase 1 | Joint control |
| [3](#week-3-kinematics-and-reaching) | Phase 1 | Kinematics and reaching |
| [4](#week-4-take-apart-the-walking-policy) | Phase 2 | Take apart the walking policy |
| [5](#week-5-grasping-the-ball) | Phase 3 | Grasping the ball |
| [6](#week-6-reliable-grasp-and-the-first-task-chain) | Phase 3 | Reliable grasp and the first task chain |
| [7](#week-7-the-physics-of-the-delivery) | Phase 4 | The physics of the delivery |
| [8](#week-8-release-compare-report) | Phase 4 + review | Release, compare, report |

## Week 1: Know your simulator

*Phase 0.* Read any MuJoCo model like an engineer: sizes, masses, contacts, and what makes a simulation stable.

**Read this week:**

- MuJoCo docs: Overview and Computation (Simulation pipeline, Contact)
- MuJoCo Python tutorial notebook, first half

### Robotics track

- [ ] **Mon: Build and run everything.** Build the workspace and run every launch file in the README (RViz, MuJoCo hold, walking on both scenes). Write down every warning you see.
  - *Done when:* All four launch commands run and you can walk the robot to the table with the keyboard.
- [ ] **Tue: Write scripts/inspect_model.py.** Load `bowling_scene.xml`. Print `nq/nv/nu`, every body with its mass, the total robot mass, and each joint's name, `qposadr`, `dofadr` and range.
  - *Done when:* The printed sizes match the README (50 / 49 / 43) and you can say where any joint lives in `qpos`.
- [ ] **Wed: Centre of mass and contacts.** Let the free-base robot stand under JointHold for 5 s. Log `subtree_com` of the pelvis and foot contact forces with `mj_contactForce`. Plot both.
  - *Done when:* The sum of normal foot forces equals m·g within 2% once the robot settles.
- [ ] **Thu: Timestep and integrator sweep.** Run standing with timestep 0.001, 0.002, 0.005, 0.01 and integrators Euler, implicitfast, RK4. Record which combinations fall or blow up.
  - *Done when:* A table of 12 runs with the result (stable / falls / NaN) and the wall-clock time for each.
- [ ] **Fri: Armature experiment.** Set ankle-roll armature to 0, rerun standing, then restore it. Compute the ankle-roll natural frequency √(kp/I) with and without armature.
  - *Done when:* You can explain with numbers why the ankle went unstable at the old timestep.

### ML track

- [ ] **PyTorch refresher.** Tensors, autograd, nn.Module, an optimizer loop. Fit an MLP to y = sin(x) + noise with a train/validation split.
  - *Done when:* You can plot training and validation loss and explain overfitting from the plot.

**Deliverable (commit by Sunday):** `scripts/inspect_model.py`, the contact-force plot, the sweep table, and `notes/week1.md`.

**Checkpoint (answer without notes):**

- Why does qpos have one more entry than qvel?
- What does solref change in a contact, physically?
- Why can a light link make PD control unstable at a fixed timestep?

## Week 2: Joint control

*Phase 1.* Make one arm follow a trajectory precisely with PD, gravity compensation and computed torque.

**Read this week:**

- Modern Robotics ch. 8.1–8.3 (dynamics) and ch. 11.1–11.4 (motion control)

### Robotics track

- [ ] **Mon: PD step response.** Fixed base. Command a 0.5 rad step on the right shoulder pitch and elbow. Plot the response for kp ∈ {5, 20, 40, 100} with critical damping kd = 2√(kp·I).
  - *Done when:* Four plots, with rise time, overshoot and steady-state error written on each.
- [ ] **Tue: Add gravity compensation.** Add `qfrc_bias` as a feedforward torque. Repeat Monday's kp = 20 test.
  - *Done when:* Steady-state error drops by at least 5× compared with plain PD.
- [ ] **Wed: Computed torque.** Use `mj_inverse` to compute the torque for a desired acceleration, then add PD on the error. Track a 0.5 Hz sinusoid on the elbow.
  - *Done when:* An RMS-error table for PD, PD + gravity, and computed torque on the same trajectory.
- [ ] **Thu: Smooth trajectories.** Write a quintic polynomial between two arm poses (zero start/end velocity and acceleration). Track it with computed torque.
  - *Done when:* Joint velocity and acceleration plots are smooth, with no spikes at the start or end.
- [ ] **Fri: Torque limits.** Read `actuator_ctrlrange` for the arm. Speed the trajectory up until torques saturate and watch what tracking does.
  - *Done when:* You know the fastest arm swing the motors can follow. This number matters for bowling in week 7.

### ML track

- [ ] **Learn forward kinematics.** Sample 100k random right-arm poses, run `mj_kinematics`, record palm position. Train an MLP from joint angles to palm position.
  - *Done when:* Validation error under 1 cm, and you can explain why more data helped (or didn't).

**Deliverable (commit by Sunday):** `scripts/arm_control.py` with the three controllers and one comparison plot.

**Checkpoint (answer without notes):**

- What does qfrc_bias contain besides gravity?
- Why does computed torque need an accurate model?
- What happens to PD tracking when the torque saturates?

## Week 3: Kinematics and reaching

*Phase 1.* Move the right palm to the tray with your own differential IK, then with mink.

**Read this week:**

- Modern Robotics ch. 4–6 (forward kinematics, Jacobian, inverse kinematics)
- Robotic Manipulation (Tedrake): Basic pick and place, differential IK

### Robotics track

- [ ] **Mon: Check the Jacobian.** Compute the palm site Jacobian with `mj_jacSite`. Verify it against finite differences of the site position.
  - *Done when:* The two Jacobians agree to 1e-6.
- [ ] **Tue: Differential IK from scratch.** Damped least squares: Δq = Jᵀ(JJᵀ + λ²I)⁻¹·e. Move the palm to `tray_center` with the base fixed.
  - *Done when:* The palm reaches within 1 cm of the tray from the zero pose.
- [ ] **Wed: Limits and posture.** Clamp joint limits and add a null-space task that pulls the arm toward a comfortable posture.
  - *Done when:* No joint hits its limit, and the elbow stays below the shoulder.
- [ ] **Thu: Same task with mink.** Rebuild the reach with mink (position plus palm-down orientation). Compare iterations and final error with your solver.
  - *Done when:* A short table comparing your IK with mink.
- [ ] **Fri: Robustness test.** Run 20 random start poses through both solvers. Record success rate and iterations to converge.
  - *Done when:* At least 19 of 20 starts reach within 1 cm.

### ML track

- [ ] **Learned IK, and why it fails.** Train an MLP from palm position to joint angles using week 2's data. Find poses with two IK solutions and look at what the network predicts.
  - *Done when:* You can show a case where the network averages two solutions into a wrong pose. This is why multimodal targets need other methods.

**Deliverable (commit by Sunday):** `scripts/reach_tray.py`, the 20-start table, and a short video of the reach.

**Checkpoint (answer without notes):**

- What does the damping λ trade off?
- What is the null space of J and why is it useful on a 7-DoF arm?
- Why does regression average over multiple solutions?

## Week 4: Take apart the walking policy

*Phase 2.* Treat motion.pt as a system under test: what it sees, where it breaks, and why.

**Read this week:**

- Rudin et al. 2021 (paper 5 in the reading guide)
- Spinning Up: Intro to RL parts 1–3

### Robotics track

- [ ] **Mon: Label the observation.** Write a table of all 47 observation entries in `walk.py`: index range, meaning, scale.
  - *Done when:* Every index from 0 to 46 is labelled and you can rebuild the vector by hand.
- [ ] **Tue: Headless evaluation script.** Write `scripts/eval_walk.py`: N episodes of 15 s, a fall when pelvis height drops below 0.4 m, and lateral drift measured at the end.
  - *Done when:* One command prints fall rate and mean drift for a given speed.
- [ ] **Wed: Speed × hand-mass sweep.** Sweep speed 0.3–1.0 m/s in 0.1 steps and `hand_mass_scale` ∈ {0.25, 0.5, 1.0}, with 50 episodes per cell.
  - *Done when:* A heatmap of fall rate. This is your baseline for retraining after month two.
- [ ] **Thu: Pushes and noise.** Apply lateral pushes with `xfrc_applied` (20–100 N for 0.1 s) and add noise to the observation.
  - *Done when:* A curve of fall rate against push force.
- [ ] **Fri: Ablate your outer loops.** Set `heading_kp` and `speed_kp` to 0 one at a time and rerun the 0.5 m/s case.
  - *Done when:* You can say in numbers how much each loop contributes.

### ML track

- [ ] **REINFORCE on CartPole.** Implement the vanilla policy gradient from Spinning Up on Gymnasium CartPole-v1. Run 5 seeds.
  - *Done when:* A learning curve with mean ± std over 5 seeds that reaches 500 reward.

**Deliverable (commit by Sunday):** `scripts/eval_walk.py`, the heatmap, and a one-page evaluation report.

**Checkpoint (answer without notes):**

- Why is the gait phase part of the observation?
- Why does the policy output joint targets rather than torques?
- Why do you need 5 seeds before believing a learning curve?

## Week 5: Grasping the ball

*Phase 3.* Pick the ball from the tray with the Dex3 hand, base fixed, using a scripted controller.

**Read this week:**

- Robotic Manipulation (Tedrake): Grasping chapter (contact, friction cones, force closure)

### Robotics track

- [ ] **Mon: Know the hand.** List the Dex3 joints, actuators and ranges. Write open and close commands for the right hand and test them in the air.
  - *Done when:* The fingers open and close smoothly with no jitter.
- [ ] **Tue: Pre-grasp and approach.** Use your IK to place the palm 8 cm above the ball, then descend to the grasp pose.
  - *Done when:* The palm reaches the grasp pose without touching the tray.
- [ ] **Wed: Close, lift and tune contacts.** Close the fingers, lift 15 cm, log fingertip contact forces. Tune ball `friction`, `condim` and `solref` if it slips.
  - *Done when:* The ball stays in the hand for a 3 s hold while the arm moves.
- [ ] **Thu: Failure log.** Run 20 grasps and classify every failure: slip, knocked off, penetration, missed.
  - *Done when:* A failure table with a likely cause for each type.
- [ ] **Fri: Grasp state machine.** Wrap it as states: PREGRASP → APPROACH → CLOSE → LIFT → HOLD, with a timeout and a success check in each state.
  - *Done when:* One function call runs the full grasp and reports success or the failing state.

### ML track

- [ ] **PPO from scratch.** Implement PPO with GAE and the clipped objective (about 300 lines of PyTorch). Solve Pendulum-v1.
  - *Done when:* Average return above −200 on 3 seeds.

**Deliverable (commit by Sunday):** `g1_bowling_control/grasp.py`, the contact-force plot, and a grasp video.

**Checkpoint (answer without notes):**

- What is a friction cone and how does condim change it?
- Why does a grasp that works at rest fail when the arm accelerates?
- What does GAE's λ trade off?

## Week 6: Reliable grasp and the first task chain

*Phase 3.* Grasp with the ball position randomized, then chain walking and grasping in one run.

**Read this week:**

- MuJoCo docs: XML reference for contact pairs and exclusions (as needed)
- PPO paper (Schulman 2017), sections 3–5

### Robotics track

- [ ] **Mon: Randomize the ball.** Shift the ball ±3 cm in x and y on each reset. Run 100 grasps.
  - *Done when:* A success rate measured on 100 trials.
- [ ] **Tue: Use the ball position.** Read the ball pose from `data` and plan the pre-grasp from it instead of from `tray_center`.
  - *Done when:* Success rate above 90% on 100 randomized trials.
- [ ] **Wed: Stand while grasping.** Free base. Walk to the table, command zero velocity, then grasp while the walking policy keeps balance.
  - *Done when:* The robot stays upright while the arm moves in 10 of 10 runs.
- [ ] **Thu: Task state machine.** Write `task.py`: WALK_TO_TABLE → STOP → REACH → GRASP → LIFT → TURN, using your walk controller and grasp code.
  - *Done when:* The full chain runs in one command.
- [ ] **Fri: Measure the chain.** Run the chain 20 times and record which stage fails.
  - *Done when:* A per-stage success table. Fix the worst stage first.

### ML track

- [ ] **Your PPO against SB3.** Run your PPO and Stable-Baselines3 PPO on Hopper-v4 with 3 seeds each. Then vary learning rate and clip range.
  - *Done when:* A comparison plot, and a list of which hyperparameters matter most.

**Deliverable (commit by Sunday):** `g1_bowling_control/task.py` and the per-stage table.

**Checkpoint (answer without notes):**

- Why did feedback from the ball pose raise success so much?
- How does arm motion disturb a walking policy that was trained with a rigid upper body?
- Which PPO hyperparameter hurt most when wrong?

## Week 7: The physics of the delivery

*Phase 4.* Compute the release that hits the stumps, then find an arm swing that produces it.

**Read this week:**

- TossingBot (Zeng 2019), the throwing and residual-physics sections
- MCC Laws of Cricket, Law 21.3 (fair delivery: the arm)

### Robotics track

- [ ] **Mon: Projectile solver.** From a release point 2.0 m high at the popping crease, solve for speed and angle that hit `target`. Include a bounce: real deliveries pitch before the stumps.
  - *Done when:* A function release_for(target, pitch_length) that returns speed and angle.
- [ ] **Tue: Check it in simulation.** Place the ball at the release point and set its `qvel` directly. Compare the simulated landing with your prediction. Add drag if needed.
  - *Done when:* Predicted and simulated landing points agree within 5 cm.
- [ ] **Wed: Error budget.** Perturb speed (±0.2 m/s), angle (±1°) and release time and record the miss distance at the stumps.
  - *Done when:* You know how precise the arm must be to hit stumps 22.9 cm wide.
- [ ] **Thu: Parameterize the swing.** Fixed base at the crease. Describe the bowling arm swing with 4–6 spline via-points. Keep the elbow near-straight: bowling allows at most 15° of elbow extension.
  - *Done when:* Forward simulation returns the hand velocity at release for any set of via-points.
- [ ] **Fri: Optimize with CMA-ES.** Optimize the via-points (pip `cma`) so the hand velocity at release matches Monday's solution, within torque limits from week 2.
  - *Done when:* The optimized swing gives a release velocity within 5% of the target.

### ML track

- [ ] **Build a Gymnasium environment.** Wrap the fixed-base throw as an env: observation (arm state, ball state), action (arm joint targets plus a release signal), reward (minus miss distance at the stumps).
  - *Done when:* `gymnasium.utils.env_checker` passes and a random policy runs for 100 episodes.

**Deliverable (commit by Sunday):** `scripts/projectile.py`, `scripts/throw_opt.py`, and the error-budget plot.

**Checkpoint (answer without notes):**

- Why is a throw with an extending elbow illegal in cricket, and how does it change your search space?
- Which release error dominates the miss distance?
- Why would a learned residual help on top of the physics model?

## Week 8: Release, compare, report

*Phase 4 + review.* Bowl a real grasped ball from the hand, compare the optimizer with RL, and write up the two months.

**Read this week:**

- Domain randomization (Tobin 2017), to prepare for month three
- Your own notes from weeks 1–7

### Robotics track

- [ ] **Mon: Release timing.** Open the fingers at time t during the swing. Add ±10 ms jitter and measure the hit rate.
  - *Done when:* A plot of hit rate against release-time error.
- [ ] **Tue: Grasped ball, full swing.** Start with the ball in the hand, run the optimized swing and release. 50 throws.
  - *Done when:* A measured hit rate on 50 throws with the ball released from the fingers.
- [ ] **Wed: Train PPO on the throw.** Train PPO (yours or SB3) on the week 7 environment. Log returns and hit rate.
  - *Done when:* A learning curve, plus the number of simulated throws it took.
- [ ] **Thu: Optimizer versus RL.** Compare CMA-ES and PPO on hit rate, simulated throws needed and robustness to ball-position noise.
  - *Done when:* A comparison table with a one-paragraph explanation of why the winner won.
- [ ] **Fri: Two-month report.** Write `docs/month2_report.md`: what works, measured numbers, failure analysis, next steps (GPU retraining of walking, imitation of a bowling action).
  - *Done when:* The report links every result to a script that reproduces it.

### ML track

- [ ] **Plan month three.** List what retraining walking with domain randomization needs: compute (your GTX 1650 has 4 GB), environment count, randomized parameters.
  - *Done when:* A one-page plan for Phase 6 with a rough compute estimate.

**Deliverable (commit by Sunday):** A demo video of a grasp-and-bowl, the comparison table, and `docs/month2_report.md`.

**Checkpoint (answer without notes):**

- Where did most of your hit-rate loss come from: grasp, swing or release?
- When is a model-based optimizer better than RL?
- What would change if the base were free and the robot had just run up?
