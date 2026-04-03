# Task 2: UR5e Keyboard-Based Velocity Control

## 1. Objective

Real-time interactive control of the UR5e end-effector using keyboard input. Cartesian velocities (vx, vy, vz, wx, wy, wz) are converted to joint velocities via the **Jacobian pseudo-inverse** and integrated at **500 Hz**.

---

## 2. The Core Concept: From Keyboard to Robot Motion

### 2.1 The Big Picture (Non-Technical)

When you press **'w'** on your keyboard, you're telling the robot "move forward." But the robot doesn't understand "forward" — it only understands "turn motor 1 by X degrees, motor 2 by Y degrees..." The **Jacobian** is the translator between these two languages.

### 2.2 The Jacobian Matrix — The Mathematical Translator

The Jacobian is a **6×6 matrix** that maps joint velocities to end-effector velocities:

```
v_cartesian = J(q) × q_dot
```

Where:
- `J(q)` = Jacobian matrix (depends on current joint positions)
- `q_dot` = 6 joint velocities (radians/second)
- `v_cartesian` = 6 end-effector velocities (linear + angular)

To go backwards (from desired Cartesian velocity to joint velocities):

```
q_dot = J⁺(q) × v_cartesian
```

Where `J⁺` is the **pseudo-inverse** of the Jacobian.

### 2.3 The 6×6 Jacobian Explained

Each row of the Jacobian tells you how one Cartesian coordinate changes with each joint:

```
J = [ ∂x/∂θ₁  ∂x/∂θ₂  ∂x/∂θ₃  ∂x/∂θ₄  ∂x/∂θ₅  ∂x/∂θ₆ ]  ← Linear velocity in X
    [ ∂y/∂θ₁  ∂y/∂θ₂  ∂y/∂θ₃  ∂y/∂θ₄  ∂y/∂θ₅  ∂y/∂θ₆ ]  ← Linear velocity in Y
    [ ∂z/∂θ₁  ∂z/∂θ₂  ∂z/∂θ₃  ∂z/∂θ₄  ∂z/∂θ₅  ∂z/∂θ₆ ]  ← Linear velocity in Z
    [∂rx/∂θ₁ ∂rx/∂θ₂ ∂rx/∂θ₃ ∂rx/∂θ₄ ∂rx/∂θ₅ ∂rx/∂θ₆ ]  ← Angular velocity (roll)
    [∂ry/∂θ₁ ∂ry/∂θ₂ ∂ry/∂θ₃ ∂ry/∂θ₄ ∂ry/∂θ₅ ∂ry/∂θ₆ ]  ← Angular velocity (pitch)
    [∂rz/∂θ₁ ∂rz/∂θ₂ ∂rz/∂θ₃ ∂rz/∂θ₄ ∂rz/∂θ₅ ∂rz/∂θ₆ ]  ← Angular velocity (yaw)
```

Each column tells you: **"If I move joint i by 1 rad/s, how does the end-effector move?"**

### 2.4 Why Pseudo-Inverse (Not Regular Inverse)?

At certain robot configurations (called **singularities**), the Jacobian becomes non-invertible:

- **Singularity example:** The arm is fully stretched out — small changes in joint angles produce huge changes in position, and the math breaks down
- **Regular inverse:** `J⁻¹` doesn't exist at singularities (determinant = 0)
- **Pseudo-inverse:** Uses SVD (Singular Value Decomposition) to find the "best approximate" solution that minimizes error

KDL's `ChainIkSolverVel_pinv` implements this automatically.

---

## 3. The Keyboard Interface (termios)

### 3.1 The Problem with Normal Terminal Input

Normally, when you type in a terminal, the system waits for you to press **Enter** before sending the input to the program. For real-time robot control, we need each keypress to register **instantly** the moment you press it.

### 3.2 Raw Terminal Mode

The code in `keyboard_task2.cpp` configures the terminal for raw, non-blocking input:

```cpp
struct termios oldt_, newt_;

void setTerminalMode(bool raw) {
    if (raw) {
        tcgetattr(STDIN_FILENO, &oldt_);          // Save current terminal settings
        newt_ = oldt_;                             // Copy to new settings
        newt_.c_lflag &= ~(ICANON | ECHO);         // Disable line buffering & echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);  // Apply new settings immediately
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);  // Make read() non-blocking
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);  // Restore original terminal settings
        fcntl(STDIN_FILENO, F_SETFL, 0);           // Restore blocking mode
    }
}
```

| Flag | What it does |
|------|--------------|
| `~ICANON` | Disables line buffering — program receives each keypress immediately |
| `~ECHO` | Stops echoing typed characters to the screen (no "w" appears when you press w) |
| `O_NONBLOCK` | `read()` returns immediately — doesn't wait if no key is pressed |

The terminal is restored to normal when the node shuts down (in the destructor).

### 3.3 Key Mapping

The keyboard controls 6 Cartesian degrees of freedom:

| Key | Cartesian Axis | Motion Direction | Code |
|-----|---------------|------------------|------|
| `w` | **vx** | Forward (+X) | `cartesian_velocities_[0] = 0.1` |
| `s` | **vx** | Backward (−X) | `cartesian_velocities_[0] = -0.1` |
| `a` | **vy** | Left (+Y) | `cartesian_velocities_[1] = 0.1` |
| `d` | **vy** | Right (−Y) | `cartesian_velocities_[1] = -0.1` |
| `q` | **vz** | Up (+Z) | `cartesian_velocities_[2] = 0.1` |
| `e` | **vz** | Down (−Z) | `cartesian_velocities_[2] = -0.1` |
| `i` | **wx** | Roll + (rotate around X) | `cartesian_velocities_[3] = 0.1` |
| `k` | **wx** | Roll − | `cartesian_velocities_[3] = -0.1` |
| `j` | **wy** | Pitch + (rotate around Y) | `cartesian_velocities_[4] = 0.1` |
| `l` | **wy** | Pitch − | `cartesian_velocities_[4] = -0.1` |
| `u` | **wz** | Yaw + (rotate around Z) | `cartesian_velocities_[5] = 0.1` |
| `o` | **wz** | Yaw − | `cartesian_velocities_[5] = -0.1` |
| `Space` | — | **Stop** (all velocities = 0) | `fill(cartesian_velocities_.begin(), ..., 0.0)` |

Velocity step: **0.1 m/s** (or rad/s for angular) per key press.

---

## 4. The 500 Hz Control Loop

### 4.1 Node Initialization

```cpp
KeyboardControlNode() : Node("keyboard_control") {
    // Load URDF from parameter
    this->declare_parameter<std::string>("robot_description", "");
    this->get_parameter("robot_description", urdf_string);

    // Initialize KDL kinematics (FK, IK, Jacobian solvers)
    if (!ik_controller_.init(urdf_string, "base_link", "tool0")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load URDF or init IK controller");
        return;
    }

    // Create publisher and 500 Hz timer
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = this->create_wall_timer(2ms, std::bind(&KeyboardControlNode::controlLoop, this));

    // Initialize joint state
    joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    current_joints_ = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    cartesian_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Setup terminal for keyboard input
    setTerminalMode(true);
    printInstructions();
}
```

### 4.2 The Control Loop (Executed Every 2ms)

This is the heart of Task 2 — it runs 500 times per second:

```cpp
void controlLoop() {
    // Step 1: Check for keyboard input → update Cartesian velocity vector
    processKeyboardInput();

    // Step 2: Convert Cartesian velocity → Joint velocity via Jacobian
    std::vector<double> joint_velocities(6, 0.0);
    bool success = ik_controller_.computeJointVelocities(
        current_joints_,           // Current joint positions
        cartesian_velocities_,     // Desired Cartesian velocities [vx, vy, vz, wx, wy, wz]
        joint_velocities           // Output: required joint velocities
    );

    if (success) {
        // Step 3: Euler integration — q(t+dt) = q(t) + q_dot × dt
        double dt = 0.002;  // 500 Hz = 0.002 seconds per tick
        for (size_t i = 0; i < 6; i++) {
            current_joints_[i] += joint_velocities[i] * dt;
        }
    } else {
        // Step 4: Singularity reached — warn but don't crash
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Jacobian Singularity reached! Cannot move.");
    }

    // Step 5: Publish joint states to RViz
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position = current_joints_;
    joint_pub_->publish(msg);
}
```

### 4.3 Euler Integration Explained

In Task 1, we moved the robot to a **specific target pose**. In Task 2, we move the robot at a **specific velocity**. To convert velocity to position:

```
new_position = old_position + (velocity × time_step)
```

At 500 Hz, `dt = 0.002` seconds.

**Example:**
- Desired Cartesian velocity: `vx = 0.1 m/s`
- Jacobian converts to joint velocity: `q_dot = [0.05, -0.02, 0.08, ...] rad/s`
- In one tick: `Δq = 0.05 × 0.002 = 0.0001 radians`
- In 1 second (500 ticks): `0.0001 × 500 = 0.05 radians` of total movement

Tiny steps every 2ms → smooth, continuous motion.

---

## 5. Singularity Handling

### 5.1 What is a Singularity?

A **singularity** is a robot configuration where the Jacobian matrix loses rank — the robot temporarily loses a degree of freedom.

**Common singularity examples:**
- **Wrist singularity:** All three wrist joints align → loss of orientation control
- **Reach singularity:** Arm fully stretched out → loss of linear motion in one direction

At these points, the Jacobian determinant approaches zero, and `J⁻¹` doesn't exist.

### 5.2 How the Code Handles It

```cpp
bool success = ik_controller_.computeJointVelocities(current_joints_, cartesian_velocities_, joint_velocities);

if (!success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Jacobian Singularity reached! Cannot move.");
    // joint_velocities stays at [0,0,0,0,0,0] → robot stops
    return;
}
```

- The KDL pseudo-inverse solver detects when it cannot find a valid solution
- Returns `false` → joint velocities set to zero
- **Robot stops moving but doesn't crash**
- User can press different keys to move away from the singularity

---

## 6. Visualization Pipeline (Same as Task 1)

```
keyboard_task2 ──[sensor_msgs/JointState]──→ /joint_states topic
                                                      │
                                                      ▼
                                         robot_state_publisher
                                                      │
                                         (reads URDF → computes TF)
                                                      │
                                                      ▼
                                              RViz2 (3D display)
```

1. **keyboard_task2** publishes `sensor_msgs::msg::JointState` at 500 Hz
2. **robot_state_publisher** converts joint angles → 3D link transforms (TF)
3. **RViz2** renders the robot in real-time

---

## 7. Task 1 vs Task 2: Key Differences

| Aspect | Task 1 | Task 2 |
|--------|--------|--------|
| **Input** | Pre-defined start/goal poses | Real-time keyboard keypresses |
| **Math** | Cosine S-Curve interpolation + IK solver | Jacobian pseudo-inverse + Euler integration |
| **Motion type** | Point-to-point trajectory | Continuous velocity control |
| **Control mode** | Open-loop (pre-planned) | Interactive (user-driven) |
| **IK usage** | Solves for pose at each step | Not used — velocity mapping only |
| **Jacobian usage** | Not used | Core component (CartToJnt) |
| **Singularity handling** | Seed-based IK (current joints as initial guess) | Pseudo-inverse solver returns false |

---

## 8. File Locations

```
src/ur5_control/
├── include/ur5_control/
│   └── ik_controller.hpp                 # computeJointVelocities() declaration
├── src/
│   ├── ik_controller.cpp                  # KDL Jacobian solver implementation
│   │                                       # ChainIkSolverVel_pinv (lines 81-108)
│   └── keyboard_task2.cpp                 # Task 2: Keyboard velocity control
│       ├── setTerminalMode()              # Raw terminal setup (lines 92-104)
│       ├── processKeyboardInput()         # Key mapping switch-case (lines 64-90)
│       └── controlLoop()                  # 500 Hz control loop (lines 44-63)
└── launch/
    └── ur5_control.launch.py              # Launch file (defaults to Task 2)
```

---

## 9. Code Flow Summary (Non-Technical)

```
User presses 'w'
        ↓
processKeyboardInput() sets vx = 0.1
        ↓
controlLoop() calls computeJointVelocities()
        ↓
IKController uses KDL ChainIkSolverVel_pinv
        ↓
Jacobian pseudo-inverse calculates joint velocities
        ↓
Euler integration: new_joints = old_joints + (joint_vel × 0.002)
        ↓
Publish to /joint_states → RViz updates display
        ↓
Repeat 500 times per second while key is held
```

---

## 10. Summary

Task 2 demonstrates:

1. **Real-time input** — Raw terminal (termios) for instant keypress detection without Enter
2. **Velocity control** — Jacobian pseudo-inverse maps Cartesian velocities → joint velocities
3. **Numerical integration** — Euler integration converts velocities to positions over time
4. **Graceful failure** — Singularity handling prevents crashes, robot simply stops
5. **500 Hz loop** — Same high-frequency control as Task 1 for smooth motion