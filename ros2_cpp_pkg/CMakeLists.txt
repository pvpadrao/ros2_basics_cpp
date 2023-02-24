cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_ign_gazebo REQUIRED)

# generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/OddEvenCheck.srv"
    "srv/TurnCamera.srv"
    "action/Navigate.action"
    DEPENDENCIES
    sensor_msgs geometry_msgs action_msgs
    ADD_LINTER_TESTS
)

# set support for using custom interfaces in C++ that belong to this package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# create the executables and target dependencies
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(publisher_rpm src/publisher_rpm.cpp)
ament_target_dependencies(publisher_rpm rclcpp std_msgs)

add_executable(subscriber_rpm src/subscriber_rpm.cpp)
ament_target_dependencies(subscriber_rpm rclcpp std_msgs)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp std_msgs)
target_link_libraries(service_server "${cpp_typesupport_target}")

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp std_msgs)
target_link_libraries(service_client "${cpp_typesupport_target}")

add_executable(turn_camera_server src/turn_camera_server.cpp)
ament_target_dependencies(turn_camera_server rclcpp std_msgs sensor_msgs OpenCV cv_bridge)
target_link_libraries(turn_camera_server "${cpp_typesupport_target}")

add_executable(turn_camera_client src/turn_camera_client.cpp)
ament_target_dependencies(turn_camera_client rclcpp std_msgs sensor_msgs OpenCV cv_bridge)
target_link_libraries(turn_camera_client "${cpp_typesupport_target}")

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp std_msgs geometry_msgs rclcpp_action)
target_link_libraries(action_server "${cpp_typesupport_target}")

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp std_msgs geometry_msgs rclcpp_action)
target_link_libraries(action_client "${cpp_typesupport_target}")

# install executables
install(TARGETS
    publisher
    subscriber
    publisher_rpm
    subscriber_rpm
    service_server
    service_client
    turn_camera_server
    turn_camera_client
    action_server
    action_client
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
