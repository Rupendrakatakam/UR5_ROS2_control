# Task 1: UR5e Custom Motion Controller (Without MoveIt)

## 1. Objective

Take a standard UR5e robot, load it into RViz for visualization, and make it move smoothly from Point A to Point B **without using MoveIt**. The motion controller was built from scratch using KDL (Kinematics and Dynamics Library) and runs at a strict **500 Hz** frequency.

---

## 2. Architecture Overview

The system is composed of two main components:

| Component | File | Role |
|-----------|------|------|
| **IKController** | `ik_controller.hpp` / `ik_controller.cpp` | Mathematical brain — handles all KDL kinematics |
| **MotionTask1Node** | `motion_task1.cpp` | 500 Hz control loop — orchestrates motion and publishes to RViz |

---

## 3. The Mathematical Brain: IKController

### 3.1 Purpose

The `IKController` class (`src/ur5_control/src/ik_controller.cpp`) is a C++ wrapper around KDL that translates between:

- **Joint Space** — the 6 joint angles of the UR5e
- **Cartesian Space** — the X, Y, Z position and orientation (Roll, Pitch, Yaw) of the robot's tool

### 3.2 Initialization Pipeline

The `init()` method performs three critical steps:

#### Step 1: URDF Parsing
```cpp
kdl_parser::treeFromString(urdf_string, my_tree);
```
The raw URDF text file describing the UR5e's physical structure (link lengths, joint types, etc.) is parsed into a KDL `Tree` — a hierarchical mathematical representation of all links and joints.

#### Step 2: Chain Extraction
```cpp
my_tree.getChain("base_link", "tool0", chain_);
```
From the full tree, the specific kinematic chain from `base_link` (robot base) to `tool0` (end-effector) is extracted. This chain contains exactly 6 revolute joints.

#### Step 3: Solver Initialization
Four KDL solvers are instantiated:

| Solver | Type | Purpose |
|--------|------|---------|
| `ChainFkSolverPos_recursive` | Forward Kinematics (FK) | Given 6 joint angles → compute end-effector pose |
| `ChainIkSolverVel_pinv` | Inverse Velocity Kinematics | Given Cartesian velocity → compute joint velocities (pseudo-inverse Jacobian) |
| `ChainIkSolverPos_NR` | Inverse Position Kinematics (IK) | Given target pose → compute joint angles (Newton-Raphson method, max 100 iterations, 1e-6 tolerance) |
| `ChainJntToJacSolver` | Jacobian Solver | Computes the Jacobian matrix mapping joint velocities to Cartesian velocities |

### 3.3 Key Methods

- **`computeFK()`** — Converts a vector of 6 joint angles into a `KDL::Frame` (position + orientation)
- **`computeIK()`** — Converts a target `KDL::Frame` into 6 joint angles, using the current joint state as a seed for numerical stability
- **`computeJointVelocities()`** — Converts a 6-element Cartesian velocity vector `[vx, vy, vz, wx, wy, wz]` into joint velocities via the pseudo-inverse Jacobian

---

## 4. Achieving Smooth Motion: The S-Curve Profile

### 4.1 The Problem

Linearly interpolating joint angles from start to goal means the robot would instantly jump from 0 velocity to full speed — causing mechanical jerk, potential motor damage, and non-smooth motion.

### 4.2 The Solution: Cosine Interpolation

A cosine-based S-curve velocity profile ensures **zero velocity at both start and end**, with smooth acceleration and deceleration:

```
s = 0.5 × (1.0 - cos(π × t / duration))
```

| Time | Interpolation Factor `s` | Velocity |
|------|--------------------------|----------|
| t = 0 | s = 0.0 | Zero (stationary) |
| t = duration/2 | s ≈ 0.5 | Maximum (mid-speed) |
| t = duration | s = 1.0 | Zero (stationary) |

This produces a bell-shaped velocity curve — the hallmark of smooth, jerk-free motion.

---

## 5. The 500 Hz Control Loop: MotionTask1Node

### 5.1 Node Setup

The `MotionTask1Node` class (`src/ur5_control/src/motion_task1.cpp`) is a ROS 2 node that:

1. **Receives the URDF** via the `robot_description` parameter (passed from the launch file)
2. **Initializes the IKController** with base_link → tool0 chain
3. **Creates a publisher** on `/joint_states` topic
4. **Starts a wall timer** at **2ms intervals** (500 Hz):
   ```cpp
   timer_ = this->create_wall_timer(2ms, std::bind(&MotionTask1Node::controlLoop, this));
   ```

### 5.2 Start and Goal Configuration

```cpp
start_joints_ = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};   // Safe home pose
goal_joints_  = {0.5, -1.0, 1.0, -1.0, -1.57, 0.5};       // Arbitrary safe goal
```

The start pose's Cartesian frame is computed via FK. The goal pose is derived by offsetting the start pose (+0.2m in X, +0.1m in Z).

### 5.3 The Control Loop (executed every 2ms)

Each timer tick executes the following logic:

#### Step 1: Compute Interpolation Factor
```cpp
double t = (now - start_time_).seconds();
double s = 0.5 * (1.0 - std::cos(M_PI * (t / duration_)));
```

#### Step 2: Path Selection Based on Mode

---

**Mode 0: Joint Space Interpolation**

Each of the 6 joints is independently interpolated:
```cpp
for (size_t i = 0; i < 6; i++) {
    target_joints[i] = start_joints_[i] + s * (goal_joints_[i] - start_joints_[i]);
}
```
- Simple and computationally cheap
- The end-effector path through space will be a curved arc (not a straight line)
- No IK solving required — purely algebraic

---

**Mode 1: Cartesian Space Interpolation**

This is the more complex path:

1. **Interpolate Position** — Linear interpolation of X, Y, Z using the S-curve factor:
   ```cpp
   target_pose.p(i) = start_pose_.p(i) + s * (goal_pose_.p(i) - start_pose_.p(i));
   ```

2. **Interpolate Orientation** — Extract Roll-Pitch-Yaw from start and goal frames, then interpolate each:
   ```cpp
   start_pose_.M.GetRPY(r1, p1, y1);
   goal_pose_.M.GetRPY(r2, p2, y2);
   target_pose.M = KDL::Rotation::RPY(r1 + s*(r2-r1), p1 + s*(p2-p1), y1 + s*(y2-y1));
   ```

3. **Solve IK** — Feed the interpolated target pose into the KDL IK solver:
   ```cpp
   ik_controller_.computeIK(target_pose, current_joints_, target_joints);
   ```

4. **Numerical Stability Secret** — The current joint state (`current_joints_`) is passed as the **seed** (initial guess) to the Newton-Raphson solver. Since the robot only moves a fraction of a millimeter every 2ms, the previous joint angles are nearly identical to the next ones. This means:
   - The solver converges in very few iterations
   - Singularities are avoided
   - The 500 Hz timing constraint is never violated

---

#### Step 3: Publish Joint States
```cpp
sensor_msgs::msg::JointState msg;
msg.header.stamp = this->now();
msg.name = joint_names_;
msg.position = current_joints_;
joint_pub_->publish(msg);
```

The joint names correspond to the UR5e's 6 joints:
1. `shoulder_pan_joint`
2. `shoulder_lift_joint`
3. `elbow_joint`
4. `wrist_1_joint`
5. `wrist_2_joint`
6. `wrist_3_joint`

---

## 6. Visualization Pipeline: Publishing to RViz

The visualization chain works as follows:

```
MotionTask1Node ──[sensor_msgs/JointState]──→ /joint_states topic
                                                      │
                                                      ▼
                                         robot_state_publisher
                                                      │
                                         (reads URDF geometry)
                                                      │
                                                      ▼
                                              RViz2 (3D display)
```

1. **MotionTask1Node** publishes `sensor_msgs::msg::JointState` messages to `/joint_states` at 500 Hz
2. **robot_state_publisher** (a separate node running in the background) subscribes to `/joint_states`, reads the URDF to compute the 3D transforms of every link, and broadcasts TF frames
3. **RViz2** subscribes to the TF frames and the RobotModel display, rendering the UR5e's 3D geometry in real-time

---

## 7. Dependencies

From `package.xml`:

| Dependency | Purpose |
|------------|---------|
| `kdl_parser` | Parse URDF into KDL tree |
| `orocos_kdl` | Core KDL kinematics library |
| `urdf` | URDF parsing utilities |
| `rclcpp` | ROS 2 C++ client library |
| `sensor_msgs` | JointState message type |
| `robot_state_publisher` | Converts joint states → TF transforms |
| `rviz2` | 3D visualization |

---

## 8. Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **No MoveIt** | Build kinematics understanding from first principles |
| **Newton-Raphson IK with seed** | Fast convergence, avoids singularities, guarantees 500 Hz timing |
| **Cosine S-curve interpolation** | Zero velocity at endpoints → no mechanical jerk |
| **2ms wall timer** | Matches real robot control frequency (500 Hz) |
| **Joint state publishing** | Minimal interface — lets robot_state_publisher handle all TF geometry |

---

## 9. File Locations

```
src/ur5_control/
├── include/ur5_control/
│   └── ik_controller.hpp          # IKController class declaration
├── src/
│   ├── ik_controller.cpp          # KDL solver implementation
│   └── motion_task1.cpp           # 500Hz control loop node
└── package.xml                    # Package dependencies
```
