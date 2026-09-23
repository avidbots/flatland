# Helped by: https://github.com/aws-robotics/ros2-launch-file-migrator
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch.conditions as conditions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("flatland_server"),
                        "test/conestogo_office_test/world.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="200.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.005"),
            DeclareLaunchArgument(name="show_viz", default_value="false"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            DeclareLaunchArgument(name="use_rviz", default_value="false"),
            #  Node(
            #  package="flatland_viz",
            #  executable="flatland_viz",
            #  name="flatland_viz",
            #  output="screen",
            #  condition=conditions.IfCondition("$(var show_viz)"),
            #  ),
            Node(
                package="flatland_server",
                name="flatland_server",
                executable="flatland_server",
                output="screen",
                parameters=[
                    {"world_path": LaunchConfiguration("world_path")},
                    {"update_rate": LaunchConfiguration("update_rate")},
                    {"step_size": LaunchConfiguration("step_size")},
                    {"show_viz": LaunchConfiguration("show_viz")},
                    {"viz_pub_rate": LaunchConfiguration("viz_pub_rate")},
                    {"use_sim_time": True},
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
