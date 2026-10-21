# Reading Guide: Papers for the G1 Bowling Project

Links checked on 2026-10-21. Online version with comments:
[Reading Guide on Claude](https://claude.ai/artifact/YAyswo5wLT2VuYCkn9cirq).

## How to use this guide

Read papers in the order the robot needs them: walking first, then grasping,
then the throw. Each entry says what problem the paper solves, its key idea,
what to focus on, and which part of this repo it explains.

The project has four layers. A paper usually explains one layer, so knowing the
layer tells you where its idea lands in the code:

| Layer | What it decides | Where it lives in the repo |
| --- | --- | --- |
| Task logic | Walk to table, grasp, turn, run, bowl | Not written yet (state machine) |
| Skills | How to walk, reach, grasp, throw | [`walk.py`](../src/g1_bowling_control/g1_bowling_control/walk.py) (walking); grasp and throw to come |
| Joint control | Target angle to motor torque (PD) | `JointHold` in [`sim.py`](../src/g1_bowling_control/g1_bowling_control/sim.py), leg PD in `walk.py` |
| Physics | Torque to motion and contacts | MuJoCo, scenes in [`g1_dex3_description/mjcf/`](../src/g1_dex3_description/mjcf) |

All papers are free on arXiv except Kajita 2003 (IEEE).

## Reinforcement learning foundations

These two papers explain how the walking policy was trained and why training
randomizes the robot. Read them before retraining the policy on the G1 + Dex3
model.

### 1. Proximal Policy Optimization Algorithms

Schulman, Wolski, Dhariwal, Radford, Klimov (OpenAI), 2017 ·
[arXiv:1707.06347](https://arxiv.org/abs/1707.06347)

- **Problem:** policy-gradient RL is unstable. One large update can wreck a policy that was working.
- **Key idea:** limit how far each update moves the policy by clipping the probability ratio between the new and old policy. Simple to implement, stable, and works with many parallel environments.
- **Focus on:** the clipped objective (section 3), and the advantage estimate (GAE) used to score actions.
- **In your project:** PPO trained `policies/g1_walk/motion.pt`, and you will use it again to retrain walking and running. Read it together with the [Spinning Up PPO page](https://spinningup.openai.com).

### 2. Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World

Tobin, Fong, Ray, Schneider, Zaremba, Abbeel, 2017 ·
[arXiv:1703.06907](https://arxiv.org/abs/1703.06907)

- **Problem:** a policy trained in one exact simulation fails when reality (or another simulator) differs a little.
- **Key idea:** randomize the simulation during training so the real world looks like just one more variation.
- **Focus on:** the idea itself; the paper's experiments are on vision, but the same principle is applied to mass, friction and motor gains in legged robots.
- **In your project:** this is exactly why the pretrained policy struggles with the Dex3 hands. It never saw 1.39 kg at the wrists. When you retrain, randomize hand mass and payload (the ball) so the policy stays robust.

## Walking and humanoid locomotion

Rudin 2021 (paper 5) is the most important one here: it is the method behind
your walking policy. Kajita 2003 gives the classic model-based view, so you
understand what the learned policy replaces.

### 3. Biped walking pattern generation by using preview control of zero-moment point

Kajita, Kanehiro, Kaneko, Fujiwara, Harada, Yokoi, Hirukawa, ICRA 2003 ·
[DOI 10.1109/ROBOT.2003.1241826](https://doi.org/10.1109/ROBOT.2003.1241826)
(IEEE, may need university access)

- **Problem:** how to plan a humanoid's centre-of-mass motion so it does not tip over while stepping.
- **Key idea:** model the robot as a point mass on a massless leg at constant height (the linear inverted pendulum). Keep the zero-moment point (ZMP) inside the foot, and use preview control over upcoming footsteps to plan the centre of mass.
- **Focus on:** the cart-table model and why the ZMP must stay in the support polygon. Skim the control derivation.
- **In your project:** background, not code. It explains why the robot fell when we stopped abruptly from 0.8 m/s, and why `cmd_accel_limit` in `g1_walk.yaml` helps.

### 4. Learning agile and dynamic motor skills for legged robots

Hwangbo, Lee, Dosovitskiy, Bellicoso, Tsounis, Koltun, Hutter (ETH Zurich),
Science Robotics 2019 · [arXiv:1901.08652](https://arxiv.org/abs/1901.08652)

- **Problem:** RL policies trained in simulation did not work on real legged robots.
- **Key idea:** learn a neural model of the real actuators from measured data and put it in the simulator, then train with RL. First strong sim-to-real result on the ANYmal quadruped.
- **Focus on:** the actuator network and the overall training pipeline figure.
- **In your project:** shows why actuator details matter. Our missing motor armature in the G1 MJCF caused the ankle instability for the same reason.

### 5. Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning

Rudin, Hoeller, Reist, Hutter (ETH Zurich, NVIDIA), CoRL 2021 ·
[arXiv:2109.11978](https://arxiv.org/abs/2109.11978)

- **Problem:** training a walking policy took days.
- **Key idea:** run thousands of simulated robots in parallel on one GPU with PPO, plus a terrain curriculum. Walking is learned in minutes. This is the `legged_gym` framework that `unitree_rl_gym` is built on.
- **Focus on:** the observation list, the reward terms (velocity tracking plus penalties on torque, joint acceleration, foot slip), and the command-velocity setup.
- **In your project:** read it beside `walk.py`. The 47-number observation (angular velocity, gravity direction, command, joint positions and velocities, last action, gait phase) and the 0.25 action scale come from this line of work. Retraining for running means widening the command range and adding your Dex3 model.

### 6. Real-World Humanoid Locomotion with Reinforcement Learning

Radosavovic, Xiao, Zhang, Darrell, Malik, Sreenath (UC Berkeley),
Science Robotics 2024 · [arXiv:2303.03381](https://arxiv.org/abs/2303.03381)

- **Problem:** humanoids are less stable than quadrupeds, and RL walking for full-size humanoids outdoors was not shown.
- **Key idea:** a transformer policy trained with RL on a history of observations and actions, with heavy randomization, walks a real Digit humanoid zero-shot.
- **Focus on:** why a history of observations helps the policy adapt (it can infer mass and terrain from how its body responds).
- **In your project:** an option when retraining. A policy with observation history could adapt to carrying the ball versus not.

### 7. MuJoCo Playground

Zakka et al. (Google DeepMind and collaborators), 2025 ·
[arXiv:2502.08844](https://arxiv.org/abs/2502.08844)

- **Problem:** fast GPU RL training usually meant leaving MuJoCo for another simulator.
- **Key idea:** a library of GPU-accelerated MuJoCo (MJX) environments, including humanoid locomotion, with training recipes that transfer to real robots.
- **Focus on:** the locomotion environments and how they define commands and rewards.
- **In your project:** a candidate for retraining walking on your exact MJCF, since you already work in MuJoCo. Note that a GTX 1650 has 4 GB of memory, so you will run fewer parallel environments.

## Whole-body skills and throwing

The bowling delivery is a fast whole-body motion, and these papers show how to
learn one from human motion and how to time a throw. TossingBot (paper 11) is
the closest to your release-timing problem.

### 8. DeepMimic: Example-Guided Deep Reinforcement Learning of Physics-Based Character Skills

Peng, Abbeel, Levine, van de Panne, SIGGRAPH 2018 ·
[arXiv:1804.02717](https://arxiv.org/abs/1804.02717)

- **Problem:** hand-designing rewards for flips, kicks or throws gives unnatural motion.
- **Key idea:** reward the simulated character for tracking a motion-capture clip pose by pose, while still obeying physics. Reference state initialization (starting episodes at random points of the clip) and early termination make it learn quickly.
- **Focus on:** the imitation reward terms and reference state initialization.
- **In your project:** the main recipe for the delivery. Record or find a bowling motion clip, retarget it to the G1, and train a tracking policy.

### 9. AMP: Adversarial Motion Priors for Stylized Physics-Based Character Control

Peng, Ma, Abbeel, Levine, Kanazawa, SIGGRAPH 2021 ·
[arXiv:2104.02180](https://arxiv.org/abs/2104.02180)

- **Problem:** DeepMimic tracks one clip exactly, which is rigid when the task (hit the stumps) needs variation.
- **Key idea:** a discriminator learns what natural motion looks like from a motion dataset and rewards the policy for moving in that style, while a task reward says what to achieve.
- **Focus on:** how the style reward and the task reward are combined.
- **In your project:** an option once plain imitation works: "bowl like a human" (style) plus "ball lands on the stumps" (task).

### 10. ASAP: Aligning Simulation and Real-World Physics for Learning Agile Humanoid Whole-Body Skills

He et al. (Carnegie Mellon University, NVIDIA), 2025 ·
[arXiv:2502.01143](https://arxiv.org/abs/2502.01143)

- **Problem:** agile whole-body skills learned in simulation break on a real humanoid because the physics does not match.
- **Key idea:** first train motion-tracking policies from retargeted human videos in simulation. Then deploy, record real data, and learn a correction that aligns simulated dynamics with reality before fine-tuning.
- **Focus on:** the motion-tracking stage (human video, retargeting, tracking policy). The real-robot alignment stage matters only if you move to hardware.
- **In your project:** it was demonstrated on the Unitree G1, so its tracking setup is a close template for learning the bowling action.

### 11. TossingBot: Learning to Throw Arbitrary Objects with Residual Physics

Zeng, Song, Lee, Rodriguez, Funkhouser, RSS 2019 ·
[arXiv:1903.11239](https://arxiv.org/abs/1903.11239)

- **Problem:** where a thrown object lands depends on how it was grasped and released, which a physics model alone predicts poorly.
- **Key idea:** a simple projectile model computes the release velocity needed to hit a target, and a learned residual corrects it from experience. Grasping and throwing are learned together.
- **Focus on:** the residual-physics idea (section on throwing) and how the grasp affects the throw.
- **In your project:** start with the pure physics part. From the crease, compute the release speed and angle that hit the stumps. Later learn a correction for release timing and finger opening of the Dex3.

## Free books and courses

The papers assume background these books give. Read only the parts listed for
the stage you are working on.

| Resource | Read these parts | Project stage |
| --- | --- | --- |
| [Modern Robotics](https://modernrobotics.org) (Lynch, Park) | Ch. 2–4 configuration and kinematics, ch. 6 inverse kinematics, ch. 8 dynamics, ch. 11 robot control | Joint control, PD and gravity compensation |
| [MuJoCo documentation](https://mujoco.readthedocs.io) | Overview, Computation, XML Reference (as needed), the Python tutorial notebook | Physics and scenes |
| [Underactuated Robotics](https://underactuated.mit.edu) (Tedrake) | Simple models of walking, humanoids (ZMP, linear inverted pendulum) | Walking |
| [Spinning Up in Deep RL](https://spinningup.openai.com) (OpenAI) | Introduction to RL parts 1–3, the PPO page | Retraining the walking policy |
| [Reinforcement Learning: An Introduction](http://incompleteideas.net/book/the-book-2nd.html) (Sutton, Barto) | Ch. 3 MDPs, ch. 13 policy gradients | RL theory, optional |
| [Robotic Manipulation](https://manipulation.mit.edu) (Tedrake) | Basic pick and place (differential IK), grasping (contact, friction cones) | Reaching and grasping the ball |

The Sutton and Barto site timed out when checked; if it does not open, search
for the title, since the authors offer the PDF free.

## Reading plan

Five stages, each pairing reading with one exercise in the repo. Move on when
the exercise works, not when the reading is finished.

1. **Week 1: physics and joint control.** Read the MuJoCo Overview and Modern Robotics ch. 11.
    - Exercise: in [`mujoco_viewer.py`](../src/g1_dex3_description/scripts/mujoco_viewer.py), lower the elbow `kp` from 40 to 5 and watch it sag; toggle gravity compensation.
2. **Week 2: how the walking policy works.** Read paper 5 (Rudin), then paper 1 (PPO) with the Spinning Up PPO page.
    - Exercise: label every part of the 47-number observation in `walk.py`; set `heading_kp: 0` in [`g1_walk.yaml`](../src/g1_bowling_control/config/g1_walk.yaml) and watch the drift return.
3. **Week 3: reaching and grasping.** Read Robotic Manipulation (differential IK, grasping) and the mink examples.
    - Exercise: with the base fixed, move the right palm to the `tray_center` site and close the Dex3 fingers.
4. **Week 4: the throw.** Read paper 11 (TossingBot).
    - Exercise: place the ball at the crease, compute the release speed and angle that hit the `target` site, and check it by giving the ball that velocity in the bowling scene.
5. **Week 5 onward: learning the motion.** Read paper 2 (domain randomization), paper 8 (DeepMimic), then paper 10 (ASAP).
    - Exercise: retrain walking on the G1 + Dex3 model with hand mass randomized, then set `hand_mass_scale` back to 1.0.

Progress:

- [ ] Week 1: physics and joint control
- [ ] Week 2: walking policy
- [ ] Week 3: reaching and grasping
- [ ] Week 4: the throw
- [ ] Week 5: learning the motion

After each paper, fill in this template in a notes file next to your code:

```markdown
## <Paper title> (<year>)
- Problem it solves (1 line):
- Key idea (1 line):
- Where it applies in my repo (file or stage):
- One thing to try:
```
