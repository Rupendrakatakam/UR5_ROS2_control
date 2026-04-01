#include "rclcpp/rclcpp.hpp"

class MainNode : public rclcpp::Node{
    public:
        MainNode() : Node("main_node"){
            RCLCPP_INFO(this->get_logger(),"main node started");
        }
}
    
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}
