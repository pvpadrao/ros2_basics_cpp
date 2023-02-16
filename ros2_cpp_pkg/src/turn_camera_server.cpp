#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pkg/srv/turn_camera.hpp"
#include <iostream>

// creating a shortcut
typedef ros2_cpp_pkg::srv::TurnCamera TurnCameraSrv;


class TurnCameraServiceNode : public rclcpp::Node 
{
public:
    TurnCameraServiceNode(std::string exe_dir) : Node("odd_even_check_service_node")
    {   
        ws_dir_ = get_ws_dir(exe_dir);
        service_server_= this->create_service<TurnCameraSrv>(
            "odd_even_check",
            std::bind(&TurnCameraServiceNode::get_camera_image, this,
            std::placeholders::_1, std::placeholders::_2
            )
        );
        std::cout << "Turn Camera Service Running... " << std::endl;
    }

private:
    void get_camera_image(
        const TurnCameraSrv::Request::SharedPtr request, 
        TurnCameraSrv::Response::SharedPtr response)
    { 
    // this function selects the image based on the closest angle given
        float closest_num = available_angles_[0];
        float angle_diff;
        float smallest_angle = std::abs(request->degree_turn - available_angles_[0]);

        for (int i=0; i<5; i++){
        
            angle_diff = std::abs(request->degree_turn - available_angles_[i]);

            if (angle_diff < smallest_angle){

                smallest_angle = angle_diff;
                closest_num = available_angles_[0];

            }

        }

        // get the image 
        std::string image_path = ws_dir_ + "src/ros2_basics_cpp/ros2_cpp_pkg/images" + std::to_string((int) closest_num) + ".png";

    }

    // get_ws_dir function takes the path of the executable file as input and returns the path of the working directory by removing the substring "install" from the end of the input string.
    std::string get_ws_dir(std::string exe_dir)
    {
        std::string::size_type substr_index = exe_dir.find("install");
        return exe_dir.substr(0, substr_index);
    }
    rclcpp::Service<TurnCameraSrv>::SharedPtr service_server_;
    const float available_angles_ [5] {-30, -15, 0, 15, 30};
    std::string ws_dir_;
    
    };

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // argv[0] returns the location path of this executable 
    auto node = std::make_shared<TurnCameraServiceNode>(argv[0]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}