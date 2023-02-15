cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# create the executables and target dependencies
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(publisher_rpm src/publisher_rpm.cpp)
ament_target_dependencies(publisher_rpm rclcpp std_msgs)

add_executable(subscriber_rpm src/subscriber_rpm.cpp)
ament_target_dependencies(subscriber_rpm rclcpp std_msgs)

# install executables
install(TARGETS
    publisher
    subscriber
    publisher_rpm
    subscriber_rpm
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
