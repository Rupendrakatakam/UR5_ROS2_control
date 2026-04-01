#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include <rclcpp/rate.hpp>

int main(int argc, char* argv[]){
    // intialize the ros2
    rclcpp::init(argc,argv);

    // creating the ur5 node
    auto node = rclcpp::Node::make_shared("ur5_control_node");

    // create pub node
    auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);

    // create joint state message
    sensor_msgs::msg::JointState msg;

        // UR5e has exactly these 6 joint names — must match the URDF exactly
    msg.name = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

        // Set all joints to 0.0 (home position)
    // 6 joints, all at zero radians
    msg.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rclcpp::Rate rate(10);

    RCLCPP_INFO(node->get_logger(),"Publishing joint states");

    while(rclcpp::ok()){
        msg.header.stamp = node->now();
        joint_pub->publish(msg);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}