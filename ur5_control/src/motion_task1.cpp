#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ur5_control/ik_controller.hpp"
#include <iostream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class MotionTask1Node : public rclcpp::Node {
public:
    MotionTask1Node(int mode) : Node("motion_task1"), mode_(mode), initialized_(false) {
        // 1. Declare and get the URDF parameter (passed from launch file)
        this->declare_parameter<std::string>("robot_description", "");
        std::string urdf_string;
        this->get_parameter("robot_description", urdf_string);

        if (urdf_string.empty()) {
            RCLCPP_ERROR(this->get_logger(), "URDF string is empty! Ensure robot_description is passed.");
            return;
        }

        // 2. Initialize IK Controller
        // For UR5e, the kinematic chain usually goes from "base_link" to "tool0"
        if (!ik_controller_.init(urdf_string, "base_link", "tool0")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize IK Controller.");
            return;
        }

        // 3. Setup Publisher and Timer (500 Hz = 2 milliseconds)
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(2ms, std::bind(&MotionTask1Node::controlLoop, this));

        // 4. Define Start and Goal Configurations
        joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        // Typical safe starting pose for UR5e
        current_joints_ = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0}; 
        start_joints_ = current_joints_;
        goal_joints_  = {0.5, -1.0, 1.0, -1.0, -1.57, 0.5}; // Arbitrary safe goal

        // Compute FK to get Start Pose
        ik_controller_.computeFK(start_joints_, start_pose_);

        // Define Goal Pose (Shift X by 0.2m, Z by 0.1m, keep orientation)
        goal_pose_ = start_pose_;
        goal_pose_.p.x(start_pose_.p.x() + 0.2);
        goal_pose_.p.z(start_pose_.p.z() + 0.1);

        start_time_ = this->now();
        duration_ = 5.0; // Complete the motion in 5 seconds
        initialized_ = true;

        RCLCPP_INFO(this->get_logger(), "Starting motion at 500Hz in %s Space.", (mode_ == 0 ? "Joint" : "Cartesian"));
    }

private:
    void controlLoop() {
        if (!initialized_) return;

        auto now = this->now();
        double t = (now - start_time_).seconds();

        // 1. Calculate smooth interpolation factor 's' using Cosine (S-Curve)
        // This ensures zero velocity at start and end -> NO JERK
        double s = 0.0;
        if (t >= duration_) {
            s = 1.0;
        } else {
            s = 0.5 * (1.0 - std::cos(M_PI * (t / duration_)));
        }

        std::vector<double> target_joints(6, 0.0);

        if (mode_ == 0) {
            // Task 1A: JOINT SPACE INTERPOLATION
            for (size_t i = 0; i < 6; i++) {
                target_joints[i] = start_joints_[i] + s * (goal_joints_[i] - start_joints_[i]);
            }
            current_joints_ = target_joints;

        } else if (mode_ == 1) {
            // Task 1B: CARTESIAN SPACE INTERPOLATION
            KDL::Frame target_pose;
            
            // Interpolate Position linearly using S-curve
            for (int i = 0; i < 3; i++) {
                target_pose.p(i) = start_pose_.p(i) + s * (goal_pose_.p(i) - start_pose_.p(i));
            }

            // Interpolate Orientation (Euler ZYX)
            double r1, p1, y1, r2, p2, y2;
            start_pose_.M.GetRPY(r1, p1, y1);
            goal_pose_.M.GetRPY(r2, p2, y2);
            
            double r_t = r1 + s * (r2 - r1);
            double p_t = p1 + s * (p2 - p1);
            double y_t = y1 + s * (y2 - y1);
            target_pose.M = KDL::Rotation::RPY(r_t, p_t, y_t);

            // Compute IK to get corresponding joint values
            // We pass 'current_joints_' as the seed to guarantee numerical stability!
            if (!ik_controller_.computeIK(target_pose, current_joints_, target_joints)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "IK Failed! Singularity or Unreachable Pose.");
                return; 
            }
            current_joints_ = target_joints;
        }

        // Publish Joint States
        publishJoints(current_joints_);

        if (t >= duration_) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Motion Complete.");
        }
    }

    void publishJoints(const std::vector<double>& joints) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;
        msg.position = joints;
        joint_pub_->publish(msg);
    }

    int mode_;
    bool initialized_;
    double duration_;
    rclcpp::Time start_time_;

    IKController ik_controller_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<double> start_joints_;
    std::vector<double> goal_joints_;
    std::vector<double> current_joints_;

    KDL::Frame start_pose_;
    KDL::Frame goal_pose_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    int mode = -1;
    while (mode != 0 && mode != 1) {
        std::cout << "\n============================================\n";
        std::cout << "Do you want to move in joint space(Press 0) or Cartesian space(Press 1)?\n";
        std::cout << "Selection: ";
        std::cin >> mode;
        if (std::cin.fail() || (mode != 0 && mode != 1)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Please press 0 or 1.\n";
        }
    }

    auto node = std::make_shared<MotionTask1Node>(mode);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}