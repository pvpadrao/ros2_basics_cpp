from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_cpp_pkg",
            executable="publisher_rpm",
            name="rpm_pub_node",
            parameters=[
                {"rpm_value": 5.0}
            ]
        ),

        Node(
            package="ros2_cpp_pkg",
            executable="subscriber_rpm",
            name="rpm_sub_node",
            parameters=[
                {"wheel_radius_value": 10/100.0}
            ]
        ),
        ExecuteProcess(
            cmd = ['ros2', 'topic', 'list'],
            output ='screen'
        ),
        ExecuteProcess(
            cmd = ['ros2', 'param', 'list'],
            output ='screen'
        ),
        ExecuteProcess(
            cmd = ['ros2', 'topic', 'echo', '/robot_speed'],
            output ='screen'
        )

    ])