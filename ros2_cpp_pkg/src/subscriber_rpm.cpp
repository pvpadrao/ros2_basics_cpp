#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <iostream>
#include <math.h>

const float WHEEL_RADIUS_DEFAULT = 10 / 100; // cm to m

class SpeedCalcSubNode : public rclcpp::Node 
{
public:
    SpeedCalcSubNode() : Node("rpm_sub_node")
    {
        this->declare_parameter<double>("wheel_radius_value", WHEEL_RADIUS_DEFAULT);
        rpm_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "robot_rpm", 10, std::bind(&SpeedCalcSubNode::calc_and_pub_speed_callback,
            this, std::placeholders::_1)
        );

        speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "robot_speed", 10);

        std::cout << "Calculating Speed..." << std::endl;
    }   

private:
    void calc_and_pub_speed_callback(const std_msgs::msg::Float64 & rpm_msg) const
    { 
      auto speed_msg = std_msgs::msg::Float64();
      // speed [m/s]
      rclcpp::Parameter wheel_radius_param_ = this->get_parameter("wheel_radius_value");
      speed_msg.data = rpm_msg.data * (2 * wheel_radius_param_.as_double() * M_PI) / 60;
      speed_publisher_->publish(speed_msg);

    }
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedCalcSubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}