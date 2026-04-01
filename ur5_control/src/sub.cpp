#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "master_ros2/msg/custom_msg.hpp"

class SubscribeNode : public rclcpp::Node{
    public:
        SubscribeNode() : Node("ur5_joint_state_subscriber"){
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
            
            subscription_ = this->create_subscription<std_msgs::msg::String>("std_string_topic",qos_profile,std::bind(&SubscribeNode::string_callback, this, std::placeholders::_1));

            custom_subscription_ = this->create_subscription<master_ros2::msg::CustomMsg>("custom_topic", qos_profile,std::bind(&SubscribeNode::custom_callback, this, std::placeholders::_1));
        }

    private:
        void string_callback(const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
        }

        void custom_callback(const master_ros2::msg::CustomMsg::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received Custom: %s, %d", msg->data.c_str(), msg->number);
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Subscription<master_ros2::msg::CustomMsg>::SharedPtr custom_subscription_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscribeNode>());
    rclcpp::shutdown();
    return 0;
}
