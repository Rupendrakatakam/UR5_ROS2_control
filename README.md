Objective :
A small C++ ROS package that lets you:
Visualize a robot in RViz (use UR5e)
Move it either in joint space or Cartesian space
Control it in real-time using keyboard velocity inputs

Note: Do not use MOVE IT or any other similar packages.

Task 1: Basic Motion Interface
Start with a node that:
Takes a URDF as input and loads the robot into RViz
Uses KDL for all kinematics (FK, IK, Jacobian)
When you run the node, it should ask:
"Do you want to move in joint space(Press 0) or Cartesian space(Press 1)?"
If Joint Space is selected:
Move between configurations using interpolation
Keep the motion smooth and continuous
If Cartesian Space is selected:
Interpolate the end-effector pose
Use IK to compute corresponding joint values
Important:
Run everything at 500 Hz
Focus on smoothness and numerical stability (no jerky motion)

Task 2: Keyboard-Based Velocity Control
Now make it interactive.
Using keyboard input, control:
Linear velocities → vx, vy, vz
Angular velocities → wx, wy, wz
Both positive and negative directions
What should happen:
Convert Cartesian velocity → joint velocity using Jacobian
(Use inverse or pseudo-inverse as needed)
Integrate joint velocities over time
Update robot motion continuously
Again, this should run at 500 Hz

General instructions & Expected output :
Keep things simple and readable
Don’t overcomplicate the architecture
Make sure you understand what you’re coding
You should be able to explain:
How your IK works
How are you using the Jacobian
How interpolation is done
