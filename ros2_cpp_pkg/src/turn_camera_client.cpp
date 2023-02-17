#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pkg/srv/turn_camera.hpp"

#include<iostream>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui.hpp>

// saving some extra typing
typedef ros2_cpp_pkg::srv::TurnCamera TurnCameraSrv;

int main(int argc, char *argv[])

{
rclcpp::init(argc, argv);

// creating a service client node
auto service_client_node = rclcpp::Node::make_shared("turn_camera_client_node");

// creating client to request info on topic turn_camera
auto client = service_client_node->create_client<TurnCameraSrv>("turn_camera");

// creating client request
auto request = std::make_shared<TurnCameraSrv::Request>();

// printing info on screen to get user input
std::cout << "Type degree to turn camera : ";
std::cin >> request->degree_turn;

// client waits for the server to be up
client->wait_for_service();

// client sends an asynchronous request
auto result = client->async_send_request(request);

// checks if future object (result) succeeded. If not, print error message
if(rclcpp::spin_until_future_complete(service_client_node, result) == rclcpp::FutureReturnCode::SUCCESS)
{
    // convert an image from the format used by the ROS message system to an OpenCV Mat object, 
    // which is a format that can be used by OpenCV for image processing
    // Takes two arguments: the image you want to convert (result.get()->camera_image) and the enconding "bgr8"
   auto cv_ptr = cv_bridge::toCvCopy(result.get()->camera_image, "bgr8");
   cv::imshow("The requested image is ", cv_ptr->image);
   cv::waitKey(0);
} else {
    std::cout << "Input was not processed." << std::endl;
}


rclcpp::shutdown();
return 0;

}

