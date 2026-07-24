# ROS 2 VLab Run Guide for `ros2_dstar_node.py`

This guide helps you run the planner node in BITS Virtual Cloud Labs.

## What This Node Does

- Subscribes to:
  - `/map` (`nav_msgs/OccupancyGrid`)
  - `/odom` (`nav_msgs/Odometry`)
  - `/goal_pose` (`geometry_msgs/PoseStamped`)
- Publishes:
  - `/plan` (`nav_msgs/Path`)

The file is: `Ass-2/ros2_dstar_node.py`

---

## Option A (Fastest): Run as a Python Script

Use this when you just need a quick demo and your ROS environment is already active.

1. Open terminal in VLab.
2. Source ROS 2.

```bash
source /opt/ros/humble/setup.bash
```

If VLab uses Jazzy, use:

```bash
source /opt/ros/jazzy/setup.bash
```

3. Install Python deps if needed:

```bash
python3 -m pip install numpy
```

4. Run the node directly:

```bash
python3 ros2_dstar_node.py
```

5. In another terminal, publish a goal (example):

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 7.0, y: 4.0, z: 0.0}, orientation: {w: 1.0}}}"
```

6. Check plan output:

```bash
ros2 topic echo /plan
```

---

## Option B (Recommended): Run as a Proper ROS 2 Package

Use this for cleaner submission and reproducible grading.

### 1) Create workspace

```bash
mkdir -p ~/vlab_ws/src
cd ~/vlab_ws/src
```

### 2) Create package

```bash
ros2 pkg create --build-type ament_python dstar_planner --dependencies rclpy nav_msgs geometry_msgs
```

### 3) Copy node file

```bash
cp /path/to/Ass-2/ros2_dstar_node.py ~/vlab_ws/src/dstar_planner/dstar_planner/ros2_dstar_node.py
chmod +x ~/vlab_ws/src/dstar_planner/dstar_planner/ros2_dstar_node.py
```

### 4) Update `setup.py`

Open `~/vlab_ws/src/dstar_planner/setup.py` and ensure:

```python
entry_points={
    'console_scripts': [
        'ros2_dstar_node = dstar_planner.ros2_dstar_node:main',
    ],
},
```

### 5) Build

```bash
cd ~/vlab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dstar_planner
```

### 6) Run

```bash
source /opt/ros/humble/setup.bash
source ~/vlab_ws/install/setup.bash
ros2 run dstar_planner ros2_dstar_node
```

---

## Parameterized Run (if topic names differ)

```bash
ros2 run dstar_planner ros2_dstar_node --ros-args \
  -p map_topic:=/map \
  -p odom_topic:=/odom \
  -p goal_topic:=/goal_pose \
  -p plan_topic:=/plan \
  -p occupied_threshold:=50 \
  -p max_iterations:=200000
```

---

## RViz Check (Recommended for Screenshot Evidence)

1. Open RViz2:

```bash
rviz2
```

2. Add displays:
- `Map` topic: `/map`
- `Path` topic: `/plan`
- `TF` if available

3. Publish goal and capture screenshot of planned path.

---

## Common Issues and Fixes

1. `ModuleNotFoundError: rclpy`
- ROS not sourced. Run `source /opt/ros/humble/setup.bash` first.

2. Node runs but no `/plan`
- Ensure `/map`, `/odom`, `/goal_pose` topics are active.
- Check if start/goal are inside map and free.

3. `ros2 run` cannot find executable
- Rebuild with `colcon build` and source `~/vlab_ws/install/setup.bash`.
- Verify `console_scripts` entry in `setup.py`.

4. Path not visible in RViz
- Frame mismatch. Use `map` frame for goal and RViz fixed frame.

---

## Suggested Submission Evidence

- Terminal showing node startup log.
- `ros2 topic list` showing `/plan`.
- RViz screenshot with generated path.
- One paragraph in report: "Part C (D* planner) executed in ROS 2 using `ros2_dstar_node.py`."