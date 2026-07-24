# AIML ZG528 – Assignment 2
## Part A – Research Paper Study

**Paper:** *Deep Reinforcement Learning with Enhanced PPO for Safe Mobile Robot Navigation* — H. Taheri, S. R. Hosseini, M. A. Nekoui (KN Toosi University of Technology).
**Chosen Domain (carried from Assignment 1):** Hospitality Bots – Restaurant Food Delivery Robot.

| Group Member | BITS ID |
|--------------|---------|
| Ayushi Gupta | 2024AC05720 |

---

### I. Objective
To train a wheeled mobile robot to perform **mapless, collision-free navigation** toward arbitrary goal positions using only sparse LiDAR readings and pose information, by enhancing the Proximal Policy Optimization (PPO) algorithm with **Residual Blocks** in both the actor and critic networks, together with a shaped reward function that balances goal-seeking, obstacle avoidance and smooth motion.

### II. Domain
Indoor autonomous ground-robot navigation (TurtleBot3) in **static and cluttered indoor environments**, benchmarked in the ROS + Gazebo simulator. The methodology directly transfers to service robots operating in structured indoor spaces – e.g. warehouses, hospitals, and **restaurants** (our chosen domain).

### III. Scope of the Work
- Formulate mapless navigation as a continuous-control MDP with a 16-dim observation (10 batched LiDAR minima + previous v/ω + polar target + yaw + heading deviation) and a 2-dim action (linear v via sigmoid, angular ω via tanh).
- Design a ResBlock-augmented actor–critic network for PPO and compare against vanilla PPO and DDPG.
- Design **two reward functions**: a basic distance-progress reward and an advanced reward that penalizes proximity to walls and rewards approach to the goal exponentially.
- Evaluate in obstacle-free and cluttered 10×10 m Gazebo worlds on avg. reward, success %, and steps/episode.

### IV. Key Areas Open for Future Enhancement / Optimization
1. **Partial observability** – lift the 30-beam / 10-batch LiDAR bottleneck using recurrent (LSTM/GRU) or transformer policies, or richer sensors (RGB-D).
2. **Sample efficiency & Sim-to-Real** – off-policy PPO variants, domain randomization, and curriculum learning to bridge Gazebo → real restaurant floors.
3. **Safety guarantees** – shielded/constrained RL (CMDP, control-barrier functions) rather than soft penalties, especially critical when the robot carries hot food near guests.
4. **Dynamic obstacles & multi-agent settings** – walking waiters/guests are not modelled; social-navigation rewards and multi-agent PPO would extend the approach.
5. **Standard benchmarks** – lack of common evaluation environments makes fair comparison hard; open reproducible benchmarks are needed.

---

### V. Key Design Aspects of the Mobile-Robotics Phases

**A. Deployment Platform.**
The paper deploys a **TurtleBot3** – a non-holonomic differential-drive wheeled robot – inside a 10 × 10 m walled indoor arena in Gazebo. Max linear speed is capped at 0.25 m/s and max angular speed at 1 rad/s, matching realistic small-service-robot dynamics. The same class of platform is directly reusable for our restaurant delivery bot.

**B. Sensors & Types.**
The primary exteroceptive sensor is a **2-D LiDAR** producing 30 range readings across the front 180° (–90° to +90°), normalized to [0,1]. Proprioceptive state comes from **wheel-odometry-derived velocities** (previous v, ω) and the robot’s **yaw**; the relative target position is provided externally in polar form (as ground truth in simulation). Notably, the paper reports the target cylinder is intentionally *invisible* to the LiDAR, so the goal must be reached via odometric/heading cues.

**C. Perception – Sensor & Motion Models.**
*Sensor model:* raw LiDAR is compressed by taking the **minimum of every 3 consecutive beams**, yielding 10 “worst-case obstacle” features – a hand-crafted, safety-biased observation encoder (Fig. 2). *Motion model:* the policy directly outputs continuous (v, ω); the underlying differential-drive kinematics are handled by ROS/Gazebo, so the network implicitly learns an inverse motion model from experience rather than using an explicit analytic form.

**D. Localization & Mapping.**
The approach is **mapless**: no SLAM, no global map, no explicit localization module. The agent’s “state” consists only of local LiDAR features and a relative goal vector supplied by the simulator’s ground-truth transform tree. This shifts the localization/mapping burden onto the learned policy, which is a key strength (no map maintenance) but also a limitation for large or changing spaces.

**E. Trajectory Planning.**
There is no separate global/local planner. Trajectory generation is **end-to-end reactive**: the actor network emits (v, ω) every control step given the current 16-dim observation, so the trajectory emerges implicitly from the policy. ResBlock skip-connections keep gradients healthy and let the policy learn smoother, more strategic paths, particularly under the advanced reward.

**F. Control Technique.**
Low-level control is **direct continuous velocity command** – sigmoid-bounded linear velocity (0–v_max) and tanh-bounded angular velocity (±ω_max) – fed straight to the differential-drive controller. Policy updates use PPO’s clipped surrogate objective (Eq. 1–2) with learning rate 3 × 10⁻⁴, giving stable, monotonic-improvement behaviour without an explicit PID/MPC layer.

**G. Communication Infrastructure.**
Training and execution rely on **ROS** as middleware (topic-based pub/sub between the policy node, Gazebo, and the TurtleBot drivers) with **Gazebo** as the high-fidelity simulator; no cloud, IoT, or Wi-Fi off-boarding is used. For real deployment (e.g. our restaurant bot), the same ROS 2 stack can be extended with Wi-Fi/MQTT links to a kitchen-side order-management service.

---

**Comparative note vs. traditional techniques.** Classical restaurant-delivery stacks use AMCL localization + occupancy-grid SLAM + a global planner (A*/Dijkstra) + a local planner (DWA/TEB) + PID. The ResBlock-PPO approach collapses the *last three* of those into a single learned policy, trading interpretability for adaptability and hand-tuning effort. Our Part B experiments below re-instantiate the traditional pipeline (MCL + Grid SLAM + D*) for the restaurant domain and then show where RL (Part D) and deep learning (Part E) can complement or replace individual stages – exactly the trade-off this paper analyses.
