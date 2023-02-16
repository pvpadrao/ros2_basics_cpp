#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pkg/srv/odd_even_check.hpp"

#include<iostream>

// saving some extra typing
typedef ros2_cpp_pkg::srv::OddEvenCheck OddEvenCheckSrv;

int main(int argc, char *argv[])

{
rclcpp::init(argc, argv);

// creating a service client node
auto service_client_node = rclcpp::Node::make_shared("odd_even_check_client_node");

// creating client to request info on topic odd_even_check
auto client = service_client_node->create_client<OddEvenCheckSrv>("odd_even_check");

// creating client request
auto request = std::make_shared<OddEvenCheckSrv::Request>();

// printing info on screen to get user input
std::cout << "Type number to check if odd or even: ";
std::cin >> request->number;

// client waits for the server to be up
client->wait_for_service();

// client sends an asynchronous request
auto result = client->async_send_request(request);

// checks if future object (result) succeeded. If not, print error message
if(rclcpp::spin_until_future_complete(service_client_node, result) == rclcpp::FutureReturnCode::SUCCESS)
{
    // service OddEvenCheck: request type int64 "number" and response type string "decision"
    std::cout << "Input number is " << result.get()->decision << std::endl;
} else {
    std::cout << "Input was not processed." << std::endl;
}


rclcpp::shutdown();
return 0;

}

