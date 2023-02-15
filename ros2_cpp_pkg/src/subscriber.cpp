#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

class HelloWorldSubNode : public rclcpp::Node 
{
public:
    HelloWorldSubNode() : Node("hello_world_sub_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "hello_world", 10, std::bind(&HelloWorldSubNode::sub_callback,
            this, std::placeholders::_1)
        );
    }

private:
    void sub_callback(const std_msgs::msg::String & msg) const
    { 
        RCLCPP_INFO(this->get_logger(), msg.data.c_str());
      // std::cout << msg.data << std::endl;
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorldSubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}