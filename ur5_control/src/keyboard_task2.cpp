#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ur5_control/ik_controller.hpp"
#include "termios.h"
#include "vector"
#include "unistd.h"
#include "fcntl.h"
#include "iostream"
#include <algorithm>
#include <cstddef>
#include <fcntl.h>

using namespace std::chrono_literals;

class KeyboardControlNode : public rclcpp::Node{
public:
    KeyboardControlNode() : Node("keyboard_control"){
        this->declare_parameter<std::string>("robot_description", "");
        std::string urdf_string;
        this->get_parameter("robot_description", urdf_string);

        if (urdf_string.empty() || !ik_controller_.init(urdf_string, "base_link", "tool0")){
            RCLCPP_ERROR(this->get_logger(),"falied to load urdf or init ik controller");
            return;
        }

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
        timer_ = this->create_wall_timer(2ms, std::bind(&KeyboardControlNode::controlLoop,this));

        joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        current_joints_ = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
        cartesian_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        setTerminalMode(true);
        printInstructions();
        
    }

    ~KeyboardControlNode(){
        setTerminalMode(false);
    }
private:
    void controlLoop(){
        processKeyboardInput();
        std::vector<double> joint_velocities(6,0.0);
        bool success = ik_controller_.computeJointVelocities(current_joints_, cartesian_velocities_, joint_velocities);

        if(success){
            // new position = old position +(Velocity * time)
            double dt = 0.002;
            for (size_t i=0; i<6; i++){
                current_joints_[i] += joint_velocities[i]*dt;
            }
        }else{
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Jacobian Singularity reached! Cannot move.");
        }
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;
        msg.position = current_joints_;
        joint_pub_->publish(msg);
    }
    void processKeyboardInput(){
        char c = getKeyPress();
        double speed = 0.1;

        if(c != -1){
            std::fill(cartesian_velocities_.begin(), cartesian_velocities_.end(), 0.0);

            switch (c) {
                case 'w': cartesian_velocities_[0] = speed; break;  // +vx
                case 's': cartesian_velocities_[0] = -speed; break; // -vx
                case 'a': cartesian_velocities_[1] = speed; break;  // +vy
                case 'd': cartesian_velocities_[1] = -speed; break; // -vy
                case 'q': cartesian_velocities_[2] = speed; break;  // +vz
                case 'e': cartesian_velocities_[2] = -speed; break; // -vz
                
                // Angular Velocities
                case 'i': cartesian_velocities_[3] = speed; break;  // +wx
                case 'k': cartesian_velocities_[3] = -speed; break; // -wx
                case 'j': cartesian_velocities_[4] = speed; break;  // +wy
                case 'l': cartesian_velocities_[4] = -speed; break; // -wy
                case 'u': cartesian_velocities_[5] = speed; break;  // +wz
                case 'o': cartesian_velocities_[5] = -speed; break; // -wz

                case ' ': break;
            }
        }
    }

    struct termios oldt_, newt_;
    void setTerminalMode(bool raw){
        if(raw){
            tcgetattr(STDIN_FILENO, &oldt_);
            newt_ = oldt_;
            newt_.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
            fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        }else{
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
            fcntl(STDIN_FILENO, F_SETFL, 0);
        }
    }

    char getKeyPress(){
        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0){
            return ch;
        }
        return -1;
    }

    void printInstructions(){
        std::cout << "\n============================================\n";
        std::cout << "  Task 2: Real-time Velocity Control        \n";
        std::cout << "============================================\n";
        std::cout << "Linear Velocity:\n";
        std::cout << "  w / s : +vx / -vx (Forward/Back)\n";
        std::cout << "  a / d : +vy / -vy (Left/Right)\n";
        std::cout << "  q / e : +vz / -vz (Up/Down)\n";
        std::cout << "Angular Velocity:\n";
        std::cout << "  i / k : +wx / -wx (Roll)\n";
        std::cout << "  j / l : +wy / -wy (Pitch)\n";
        std::cout << "  u / o : +wz / -wz (Yaw)\n";
        std::cout << "  [SPACE] : Stop moving\n";
        std::cout << "============================================\n";
    }

    IKController ik_controller_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<double> current_joints_;
    std::vector<double> cartesian_velocities_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}