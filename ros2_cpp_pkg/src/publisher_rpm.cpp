#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include<functional>
#include<chrono>
using namespace std::chrono_literals;


const double RPM_DEFAULT_VALUE = 100.0;

class RPMPubNode : public rclcpp::Node
{
	public:
		RPMPubNode() : Node("rpm_pub_node")
		{
			this->declare_parameter<double>("rpm_value", RPM_DEFAULT_VALUE);
			publisher_ = this->create_publisher<std_msgs::msg::Float64>(
			 "robot_rpm", 10);

			timer_ = this->create_wall_timer(1s, 
			std::bind(&RPMPubNode::publish_robot_rpm, this));
		}

	private:
		void publish_robot_rpm()
		{
			auto message = std_msgs::msg::Float64();
			// this->get_parameter("rpm_value", rpm_value_param_);
			rclcpp::Parameter rpm_value_param_ = this->get_parameter("rpm_value");
			message.data = rpm_value_param_.as_double();
			publisher_->publish(message);

		}

		rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		// double rpm_value_param_ = RPM_DEFAULT_VALUE;


};


int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RPMPubNode>());
	rclcpp::shutdown();
	return 0;
}