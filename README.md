# UR5 Control — ROS2 Custom Motion Controller

A ROS2 package for controlling a UR5e robot in RViz2 **without MoveIt**. All kinematics (Forward Kinematics, Inverse Kinematics, Jacobian) are computed from scratch using **KDL** (Kinematics and Dynamics Library) at a strict **500 Hz** control frequency.

---

## Features

| Task | Capability | Description |
|------|-----------|-------------|
| **Task 1** | Basic Motion Interface | Smooth point-to-point motion via Joint Space or Cartesian Space interpolation |
| **Task 2** | Keyboard Velocity Control | Real-time keyboard-driven end-effector velocity control using Jacobian pseudo-inverse |

**Key highlights:**
- Cosine S-Curve velocity profile for jerk-free, smooth motion
- Numerically stable IK via seed-based Newton-Raphson solver
- 500 Hz wall-timer control loop matching real robot timing

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ur5_control.launch.py                        │
│                                                                 │
│  ┌──────────────────────┐    ┌───────────────────────────────┐  │
│  │  robot_state_pub     │    │  rviz2                        │  │
│  │  (reads URDF xacro)  │───▶│  (3D visualization)           │  │
│  └──────────┬───────────┘    └───────────────▲───────────────┘  │
│             │                                │                  │
│             │ TF transforms                  │ /joint_states    │
│             │                                │                  │
│  ┌──────────▼───────────┐    ┌───────────────┴───────────────┐  │
│  │  motion_task1        │    │  keyboard_task2               │  │
│  │  (Task 1: 500 Hz)    │    │  (Task 2: 500 Hz)             │  │
│  │  - S-Curve interp    │    │  - Keyboard input (termios)   │  │
│  │  - FK / IK (KDL)     │    │  - Jacobian pseudo-inverse    │  │
│  └──────────────────────┘    └───────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

- **ROS2** (Humble / Jazzy)
- **ur_description** — UR5e URDF/xacro files
- **KDL** — `orocos_kdl`, `kdl_parser`
- **robot_state_publisher**, **rviz2**

Install system dependencies if not already present:
```bash
sudo apt install ros-$ROS_DISTRO-kdl-parser ros-$ROS_DISTRO-orocos-kdl \
                 ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-rviz2 \
                 ros-$ROS_DISTRO-ur-description
```

---

## Build

```bash
# From the workspace root
colcon build --packages-select ur5_control
source install/setup.bash
```

---

## Usage

### Launch

```bash
ros2 launch ur5_control ur5_control.launch.py
```

This starts:
- `robot_state_publisher` — broadcasts TF frames from URDF
- `rviz2` — opens the 3D visualization window
- `keyboard_task2` node in a separate `xterm` terminal (default)

> **Note:** The launch file currently defaults to Task 2. To run Task 1 instead, edit `ur5_control.launch.py` — uncomment the `motion_task1_node` block and comment out the `keyboard_task2_node`.

---

### Task 1 — Basic Motion Interface

Run the node directly:

```bash
ros2 run ur5_control motion_task1
```

On startup, the node prompts:
```
Do you want to move in joint space(Press 0) or Cartesian space(Press 1)?
```

| Mode | Behaviour |
|------|-----------|
| **0 — Joint Space** | Smoothly interpolates each of the 6 joint angles from start to goal using cosine S-Curve. The end-effector follows a curved arc through space. |
| **1 — Cartesian Space** | Interpolates the end-effector pose (position + RPY orientation) in a straight line. At every 2ms tick, KDL IK solves for the required joint angles. |

**Motion parameters:**
- Duration: **5 seconds**
- Start pose (home): `{0, -1.57, 1.57, -1.57, -1.57, 0}` radians
- Goal pose: home + offset `{+0.2m X, +0.1m Z}`

---

### Task 2 — Keyboard Velocity Control

```bash
ros2 run ur5_control keyboard_task2
```

Control the UR5e end-effector in real-time using keyboard input. Cartesian velocities are converted to joint velocities via the **Jacobian pseudo-inverse** and integrated at 500 Hz.

#### Key Mapping

| Key | Axis | Direction |
|-----|------|-----------|
| `w` / `s` | **vx** | Forward / Backward |
| `a` / `d` | **vy** | Left / Right |
| `q` / `e` | **vz** | Up / Down |
| `i` / `k` | **wx** | Roll + / − |
| `j` / `l` | **wy** | Pitch + / − |
| `u` / `o` | **wz** | Yaw + / − |
| `Space` | — | Stop (zero velocity) |

Velocity step size: **0.1 m/s** per key press.

---

## Project Structure

```
src/ur5_control/
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # ROS2 package manifest & dependencies
├── launch/
│   └── ur5_control.launch.py      # Launch file (RSP + RViz2 + control node)
├── include/ur5_control/
│   └── ik_controller.hpp          # IKController class declaration
├── src/
│   ├── ik_controller.cpp          # KDL kinematics wrapper (FK, IK, Jacobian)
│   ├── motion_task1.cpp           # Task 1: 500 Hz motion controller
│   ├── keyboard_task2.cpp         # Task 2: Keyboard velocity control
│   ├── main.cpp                   # Basic joint state publisher (demo)
│   ├── pub.cpp                    # Beginner pub node
│   └── sub.cpp                    # Beginner sub node
└── urdf/
    └── ur5e_gazebo.urdf.xacro     # UR5e Gazebo-ready URDF
```

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **No MoveIt** | Kinematics built from first principles using KDL — full control and understanding of the math |
| **Cosine S-Curve interpolation** | `s = 0.5 × (1 − cos(πt/T))` ensures zero velocity at start and end → no mechanical jerk |
| **Seed-based IK** | Current joint state passed as initial guess to Newton-Raphson solver → fast convergence, avoids singularities, guarantees 500 Hz timing |
| **500 Hz wall timer** | Matches real UR robot control loop frequency |
| **Joint state publishing** | Minimal interface — publishes `sensor_msgs/JointState` to `/joint_states`, letting `robot_state_publisher` handle all TF geometry |

---

## Assumptions

1. **URDF source** — UR5e model is provided by the `ur_description` package (xacro-generated), not bundled in this repo
2. **Kinematic chain** — `base_link` → `tool0`, containing exactly 6 revolute joints
3. **Task 1 duration** — Fixed at 5 seconds per motion
4. **Task 2 velocity step** — 0.1 m/s per key press for all 6 Cartesian DOF
5. **Safe home pose** — `{0, −1.57, 1.57, −1.57, −1.57, 0}` radians (all joints within safe limits)
6. **Terminal** — Task 2 requires a raw terminal (uses `termios`); launched via `xterm -e` by default
7. **No collision checking** — This package handles kinematics only; obstacle avoidance is out of scope

---

## Screenshots

> *Add screenshots here after running the launch file:*

| Task 1 — Joint Space | Task 1 — Cartesian Space | Task 2 — Keyboard Control |
|:---:|:---:|:---:|
| ![Task 1 Joint Space](assets/task1_joint_space.png) | ![Task 1 Cartesian](assets/task1_cartesian.png) | ![Task 2 Keyboard](assets/task2_keyboard.png) |

---

## Submission

- **Package:** `ur5_control` (ROS2 ament_cmake)
- **Build:** `colcon build --packages-select ur5_control`
- **Run:** `ros2 launch ur5_control ur5_control.launch.py`