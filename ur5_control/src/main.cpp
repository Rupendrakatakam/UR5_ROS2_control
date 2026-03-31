#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStatePublisher : public rclcpp::Node{
    public:
        JointStatePublisher() : Node("ur5_joint_state_publisher"){
           pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);

           //50 hz
           timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&JointStatePublisher::publish, this)
           );

           Joint_names_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
           };

           position_ ={0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};

           RCLCPP_INFO(this->get_logger(),"UR5 joint stet pub started");
        }
    private:
        void publish(){
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = this->get_clock()->now();
            msg.name = Joint_names_;
            msg.position = position_;
            pub_->publish(msg);
        }
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::string> Joint_names_;
        std::vector<double> position_;
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}