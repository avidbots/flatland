from pathlib import Path
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


EXAMPLES = (
    "caster_diff",
    "a300_diff",
    "2910_swerve",
    "rear_drive_ackermann",
    "front_drive_tricycle",
    "articulated_204g",
)


def robot_description(model):
    robot = ElementTree.Element("robot", name="flatland_example")
    ElementTree.SubElement(robot, "link", name="base_link")
    control = ElementTree.SubElement(robot, "ros2_control", name="Flatland", type="system")
    hardware = ElementTree.SubElement(control, "hardware")
    ElementTree.SubElement(hardware, "plugin").text = (
        "joint_command_topic_hardware_interface/JointCommandTopicSystem"
    )
    for parameter, value in (
        ("joint_commands_topic", "/robot_joint_commands"),
        ("joint_states_topic", "/robot_joint_states"),
    ):
        ElementTree.SubElement(hardware, "param", name=parameter).text = value

    for plugin in model["plugins"]:
        if plugin["type"] not in ("DriveWheel", "SteeringMotor"):
            continue
        name = plugin["name"]
        is_steering = plugin["type"] == "SteeringMotor"
        ElementTree.SubElement(robot, "link", name=f"{name}_link")
        joint = ElementTree.SubElement(
            robot, "joint", name=name, type="revolute" if is_steering else "continuous"
        )
        ElementTree.SubElement(joint, "parent", link="base_link")
        ElementTree.SubElement(joint, "child", link=f"{name}_link")
        ElementTree.SubElement(joint, "axis", xyz="0 0 1" if is_steering else "0 1 0")
        bounds = plugin.get("limit", {})
        limits = {"effort": "1000", "velocity": "100"}
        if is_steering:
            limits.update(lower=str(bounds["lower"]), upper=str(bounds["upper"]))
        ElementTree.SubElement(joint, "limit", **limits)

        interface = ElementTree.SubElement(control, "joint", name=name)
        ElementTree.SubElement(interface, "command_interface", name=plugin["mode"])
        position = ElementTree.SubElement(interface, "state_interface", name="position")
        ElementTree.SubElement(position, "param", name="initial_value").text = "0.0"
        ElementTree.SubElement(interface, "state_interface", name="velocity")
        ElementTree.SubElement(interface, "state_interface", name="effort")
    return ElementTree.tostring(robot, encoding="unicode")


def launch_example(context):
    name = LaunchConfiguration("robot").perform(context)
    if name not in EXAMPLES:
        raise ValueError(f"Unknown robot {name!r}; choose one of {', '.join(EXAMPLES)}")
    show_viz = LaunchConfiguration("show_viz").perform(context).lower() == "true"
    share = Path(get_package_share_directory("flatland_ros2_control_examples"))
    nodes: list[Action] = [
        Node(
            package="flatland_server",
            executable="flatland_server",
            name="flatland_server",
            output="screen",
            parameters=[{
                "world_path": str(share / "worlds" / f"{name}.world.yaml"),
                "update_rate": 100.0,
                "step_size": 0.01,
                "show_viz": show_viz,
                "viz_pub_rate": 30.0,
                "use_sim_time": True,
            }],
        )
    ]
    viz_node = (
        Node(
            package="flatland_viz",
            executable="flatland_viz",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ) if show_viz else None
    )
    if LaunchConfiguration("use_ros2_control").perform(context).lower() != "true":
        if viz_node:
            nodes.append(viz_node)
        return nodes

    model = yaml.safe_load((share / "models" / f"{name}.model.yaml").read_text())
    config_path = share / "config" / f"{name}.yaml"
    config = yaml.safe_load(config_path.read_text())
    controllers = [key for key in config if key != "controller_manager"]
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description(model), "use_sim_time": True}],
        output="screen",
    ))
    nodes.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[str(config_path), {"use_sim_time": True}],
        output="screen",
    ))
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager",
                       "--param-file", str(config_path)] + (
                ["--controller-ros-args", "-r ~/reference:=~/cmd_vel"]
                if name == "rear_drive_ackermann" and controller == "drive" else []
            ),
            output="screen",
        )
        for controller in ("joint_state_broadcaster", *controllers)
    ]
    nodes.extend(spawners)
    if viz_node:
        nodes.append(RegisterEventHandler(OnProcessExit(
            target_action=spawners[-1], on_exit=[viz_node]
        )))
    if LaunchConfiguration("use_joystick").perform(context).lower() == "true":
        nodes.extend([
            Node(
                package="joy",
                executable="joy_node",
                parameters=[{
                    "device_id": int(LaunchConfiguration("joy_device_id").perform(context)),
                    "use_sim_time": True,
                }],
                output="screen",
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                parameters=[{
                    "use_sim_time": True,
                    "publish_stamped_twist": True,
                    "axis_linear.x": 1,
                    "axis_angular.yaw": 3,
                    "scale_linear.x": 0.5,
                    "scale_angular.yaw": 0.8,
                    "enable_button": 4,
                }],
                remappings=[("cmd_vel", "/drive/cmd_vel")],
                output="screen",
            ),
        ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="caster_diff", choices=EXAMPLES),
        DeclareLaunchArgument("show_viz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_ros2_control", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_joystick", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        OpaqueFunction(function=launch_example),
    ])