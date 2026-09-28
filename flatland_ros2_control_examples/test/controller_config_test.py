import importlib.util
import math
import unittest
from pathlib import Path
from xml.etree import ElementTree

import yaml
from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location("example_launch", ROOT / "launch" / "example.launch.py")
assert SPEC is not None and SPEC.loader is not None
EXAMPLE_LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXAMPLE_LAUNCH)
ADAPTER_SPEC = importlib.util.spec_from_file_location(
    "twist_to_joint_commands", ROOT / "scripts" / "twist_to_joint_commands.py")
assert ADAPTER_SPEC is not None and ADAPTER_SPEC.loader is not None
ADAPTER = importlib.util.module_from_spec(ADAPTER_SPEC)
ADAPTER_SPEC.loader.exec_module(ADAPTER)


class ControllerConfigTest(unittest.TestCase):
    def test_launch_constructs_both_modes(self):
        for example in EXAMPLE_LAUNCH.EXAMPLES:
            with self.subTest(example=example):
                config = yaml.safe_load((ROOT / "config" / f"{example}.yaml").read_text())
                context = LaunchContext()
                context.launch_configurations.update({
                    "robot": example, "use_ros2_control": "false",
                    "use_joystick": "true", "joy_device_id": "0",
                    "show_viz": "true",
                })
                for show_viz in ("true", "false"):
                    context.launch_configurations["show_viz"] = show_viz
                    viz_count = int(show_viz == "true")
                    actions = EXAMPLE_LAUNCH.launch_example(context)
                    self.assertEqual([action.node_package for action in actions],
                                     ["flatland_server"] + (["flatland_viz"] if viz_count else []))
                    if viz_count:
                        self.assertEqual(actions[1].node_executable, "flatland_viz")
                    context.launch_configurations["use_ros2_control"] = "true"
                    control_actions = EXAMPLE_LAUNCH.launch_example(context)
                    has_adapter = example in ("2910_swerve", "articulated_204g")
                    self.assertEqual(len(control_actions), 5 + len(config) + viz_count + has_adapter)
                    self.assertEqual(
                        [action.node_executable for action in control_actions
                         if isinstance(action, Node)
                         and action.node_package == "flatland_ros2_control_examples"],
                        ["twist_to_joint_commands.py"] if has_adapter else [],
                    )
                    spawners = [action for action in control_actions
                                if isinstance(action, Node) and action.node_executable == "spawner"]
                    for spawner in spawners:
                        arguments = [perform_substitutions(context, part) for part in spawner.cmd[1:]]
                        is_ackermann_drive = example == "rear_drive_ackermann" and arguments[0] == "drive"
                        self.assertEqual("--controller-ros-args" in arguments, is_ackermann_drive)
                        if is_ackermann_drive:
                            self.assertEqual(arguments[arguments.index("--controller-ros-args") + 1],
                                             "-r ~/reference:=~/cmd_vel")
                    teleop = next(action for action in control_actions
                                  if isinstance(action, Node) and action.node_package == "teleop_twist_joy")
                    teleop._perform_substitutions(context)
                    self.assertEqual(teleop.expanded_remapping_rules, [("cmd_vel", "/drive/cmd_vel")])
                    handlers = [action for action in control_actions
                                if isinstance(action, RegisterEventHandler)]
                    self.assertEqual(len(handlers), viz_count)
                    for handler in handlers:
                        self.assertIsInstance(handler.event_handler, OnProcessExit)
                    context.launch_configurations["use_joystick"] = "false"
                    no_joystick = EXAMPLE_LAUNCH.launch_example(context)
                    self.assertEqual(len(no_joystick), 3 + len(config) + viz_count + has_adapter)
                    self.assertEqual(sum(isinstance(action, Node) and
                                         action.node_package == "flatland_ros2_control_examples"
                                         for action in no_joystick), has_adapter)
                    context.launch_configurations["use_joystick"] = "true"
                    context.launch_configurations["use_ros2_control"] = "false"

    def test_twist_to_joint_commands(self):
        speeds, angles = ADAPTER.swerve_commands(1, 0.2, 0.5, 0.051, 0.28, 0.28)
        for index, (x, y) in enumerate(((0.28, 0.28), (0.28, -0.28),
                                        (-0.28, 0.28), (-0.28, -0.28))):
            self.assertAlmostEqual(speeds[index] * 0.051 * math.cos(angles[index]), 1 - 0.5 * y)
            self.assertAlmostEqual(speeds[index] * 0.051 * math.sin(angles[index]), 0.2 + 0.5 * x)
            self.assertLessEqual(abs(angles[index]), math.pi / 2)
        speeds, angles = ADAPTER.swerve_commands(0, 0, 0, 0.051, 0.28, 0.28)
        self.assertEqual(speeds, [0.0] * 4)
        self.assertEqual(angles, [0.0] * 4)

        speeds, angles = ADAPTER.articulated_commands(1, 0.3, 0.29, 0.32, 0.45, 0.6981317)
        self.assertAlmostEqual(angles[0], 2 * math.atan(0.45 * 0.3))
        self.assertAlmostEqual(speeds[0], (1 - 0.3 * 0.32) / 0.29)
        self.assertAlmostEqual(speeds[1], (1 + 0.3 * 0.32) / 0.29)
        self.assertEqual(speeds, [speeds[0], speeds[1]] * 2)
        self.assertEqual(ADAPTER.articulated_commands(0, 1, 0.29, 0.32, 0.45, 0.6981317),
                         ([0.0] * 4, [0.0]))
        self.assertEqual(ADAPTER.articulated_commands(1, 100, 0.29, 0.32, 0.45, 0.6981317)[1],
                         [0.6981317])

    def test_wheel_footprints_match_plugins(self):
        for example in EXAMPLE_LAUNCH.EXAMPLES:
            model = yaml.safe_load((ROOT / "models" / f"{example}.model.yaml").read_text())
            bodies = {body["name"]: body for body in model["bodies"]}
            for wheel in model["plugins"]:
                if wheel["type"] not in ("DriveWheel", "FreeWheel"):
                    continue
                with self.subTest(example=example, wheel=wheel["name"]):
                    center_x, center_y, _ = wheel.get("offset", [0, 0, 0])
                    rectangles = []
                    for footprint in bodies[wheel["body"]]["footprints"]:
                        if footprint["type"] != "polygon" or not footprint.get("sensor"):
                            continue
                        points = footprint["points"]
                        min_x, max_x = min(point[0] for point in points), max(point[0] for point in points)
                        min_y, max_y = min(point[1] for point in points), max(point[1] for point in points)
                        if (abs((min_x + max_x) / 2 - center_x) < 1e-9
                                and abs((min_y + max_y) / 2 - center_y) < 1e-9):
                            rectangles.append((points, min_x, max_x, min_y, max_y))
                    self.assertEqual(len(rectangles), 1)
                    points, min_x, max_x, min_y, max_y = rectangles[0]
                    self.assertEqual({tuple(point) for point in points},
                                     {(min_x, min_y), (max_x, min_y),
                                      (max_x, max_y), (min_x, max_y)})
                    self.assertAlmostEqual(max_x - min_x, 2 * wheel["radius"])
                    self.assertGreaterEqual(max_y - min_y, 0.05 - 1e-9)

    def test_joint_interfaces_match_plugins_and_controllers(self):
        for example in EXAMPLE_LAUNCH.EXAMPLES:
            with self.subTest(example=example):
                model = yaml.safe_load((ROOT / "models" / f"{example}.model.yaml").read_text())
                config = yaml.safe_load((ROOT / "config" / f"{example}.yaml").read_text())
                plugins = {
                    plugin["name"]: plugin["mode"]
                    for plugin in model["plugins"]
                    if plugin["type"] in ("DriveWheel", "SteeringMotor")
                }
                description = ElementTree.fromstring(EXAMPLE_LAUNCH.robot_description(model))
                control = description.find("ros2_control")
                assert control is not None
                hardware_plugin = control.find("hardware/plugin")
                assert hardware_plugin is not None
                self.assertEqual(
                    hardware_plugin.text,
                    "joint_command_topic_hardware_interface/JointCommandTopicSystem",
                )
                interfaces = {}
                for joint in control.findall("joint"):
                    command_interface = joint.find("command_interface")
                    assert command_interface is not None
                    interfaces[joint.attrib["name"]] = command_interface.attrib["name"]
                self.assertEqual(interfaces, plugins)
                self.assertEqual(
                    {joint.attrib["name"] for joint in description.findall("joint")},
                    set(plugins),
                )
                controller_types = config["controller_manager"]["ros__parameters"]
                for controller, definition in config.items():
                    if controller == "controller_manager":
                        continue
                    self.assertIn("type", controller_types[controller])
                    settings = definition["ros__parameters"]
                    for names_key in (
                        "left_wheel_names", "right_wheel_names", "traction_joints_names",
                        "steering_joints_names", "joints",
                    ):
                        for name in settings.get(names_key, []):
                            self.assertEqual(
                                plugins[name],
                                settings.get("interface_name", "position" if "steering" in names_key else "velocity"),
                            )
                    for name_key, mode in (
                        ("traction_joint_name", "velocity"),
                        ("steering_joint_name", "position"),
                    ):
                        if name_key in settings:
                            self.assertEqual(plugins[settings[name_key]], mode)


if __name__ == "__main__":
    unittest.main()