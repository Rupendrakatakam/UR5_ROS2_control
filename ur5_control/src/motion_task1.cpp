#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ur5_control/ik_controller.hpp"
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

class MotionTask1Node : public rclcpp::Node {
public:
    MotionTask1Node(int user_chosen_mode) : Node("motion_task1") {
        mode = user_chosen_mode;

        // Get the robot description - URDF
        this->declare_parameter<std::string>("robot_description", "");
        std::string urdf_string;
        this->get_parameter("robot_description", urdf_string);

        // Turn on IK Controller
        ik_controller.init(urdf_string, "base_link", "tool0");

        // Setup the robot joint Publisher 
        joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        
        // 500 Hz means running our loop every 2 milliseconds
        timer = this->create_wall_timer(2ms, std::bind(&MotionTask1Node::controlLoop, this));

        // joint names
        joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        current_joints = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0}; 
        start_joints = current_joints;
        goal_joints  = {0.5, -1.0, 1.0, -1.0, -1.57, 0.5};

        ik_controller.computeFK(start_joints, start_pose);

        // goal 20cm forward (x) and 10cm up (z)
        goal_pose = start_pose;
        goal_pose.p.x(start_pose.p.x() + 0.2);
        goal_pose.p.z(start_pose.p.z() + 0.1);

        start_time = this->now();
    }

private:
    void controlLoop() {
        auto current_time = this->now();
        double time_elapsed = (current_time - start_time).seconds();
        if (time_elapsed >= 5.0) {
            return; 
        }
        // We use a cosine curve so the robot speeds up slowly and slows down smoothly (No jerking!) - interpolation
        double progress = 0.5 * (1.0 - std::cos(M_PI * (time_elapsed / 5.0)));

        std::vector<double> target_joints(6, 0.0);

        // JOINT SPACE MODE (Mode 0)
        if (mode == 0) {
            for (int i = 0; i < 6; i++) {
                // Formula: Current = Start + Progress * (Difference between Goal and Start)
                target_joints[i] = start_joints[i] + progress * (goal_joints[i] - start_joints[i]);
            }
            current_joints = target_joints;
        } 
        
        // CARTESIAN SPACE MODE (Mode 1)
        else if (mode == 1) {
            KDL::Frame target_pose;
            
            // Move XYZ position smoothly
            for (int i = 0; i < 3; i++) {
                target_pose.p(i) = start_pose.p(i) + progress * (goal_pose.p(i) - start_pose.p(i));
            }

            // Move Rotation smoothly
            double start_roll, start_pitch, start_yaw;
            double goal_roll, goal_pitch, goal_yaw;
            
            start_pose.M.GetRPY(start_roll, start_pitch, start_yaw);
            goal_pose.M.GetRPY(goal_roll, goal_pitch, goal_yaw);
            
            double current_roll = start_roll + progress * (goal_roll - start_roll);
            double current_pitch = start_pitch + progress * (goal_pitch - start_pitch);
            double current_yaw = start_yaw + progress * (goal_yaw - start_yaw);
            
            target_pose.M = KDL::Rotation::RPY(current_roll, current_pitch, current_yaw);

            // 3. Use Inverse Kinematics to find the joint angles for this specific exact pose
            ik_controller.computeIK(target_pose, current_joints, target_joints);
            current_joints = target_joints;
        }

        // Tell the robot to move to these new joints
        publishJoints(current_joints);
    }

    void publishJoints(const std::vector<double>& joints) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names;
        msg.position = joints;
        joint_publisher->publish(msg);
    }
    int mode;
    rclcpp::Time start_time;
    IKController ik_controller;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<std::string> joint_names;
    std::vector<double> start_joints;
    std::vector<double> goal_joints;
    std::vector<double> current_joints;

    KDL::Frame start_pose;
    KDL::Frame goal_pose;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // Start ROS 2

    // Ask the user what they want to do
    int mode = -1;
    std::cout << "Type 0 for Joint Space or 1 for Cartesian Space: ";
    std::cin >> mode;

    // Create the node and keep it running
    auto node = std::make_shared<MotionTask1Node>(mode);
    rclcpp::spin(node); 
    
    // Shut down when done
    rclcpp::shutdown();
    return 0;
}