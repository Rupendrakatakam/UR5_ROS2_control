#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "master_ros2/msg/custom_msg.hpp"

class PublishNode : public rclcpp::Node{
    public:
        PublishNode() : Node("ur5_joint_state_publisher"){
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
            
            publisher_ = this->create_publisher<std_msgs::msg::String>("std_string_topic",qos_profile);

            custom_publisher_ = this->create_publisher<master_ros2::msg::CustomMsg>("custom_topic", qos_profile);

            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PublishNode::publish_messages, this));
        }

    private:
        void publish_messages(){
            auto string_msg = std_msgs::msg::String();
            string_msg.data = "Hello from ROS2";
            RCLCPP_INFO(this->get_logger(), "Publishing: %s", string_msg.data.c_str());
            publisher_->publish(string_msg);

            auto custom_msg = master_ros2::msg::CustomMsg();
            custom_msg.data = "Custom Hello";
            custom_msg.number = 42;
            RCLCPP_INFO(this->get_logger(), "Publishing Custom: %s, %d", custom_msg.data.c_str(), custom_msg.number);
            custom_publisher_->publish(custom_msg);
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Publisher<master_ros2::msg::CustomMsg>::SharedPtr custom_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishNode>());
    rclcpp::shutdown();
    return 0;
}
