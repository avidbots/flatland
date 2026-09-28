#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def swerve_commands(linear_x, linear_y, angular_z, wheel_radius, module_x, module_y):
    speeds = []
    angles = []
    for x, y in ((module_x, module_y), (module_x, -module_y),
                 (-module_x, module_y), (-module_x, -module_y)):
        velocity_x = linear_x - angular_z * y
        velocity_y = linear_y + angular_z * x
        speed = math.hypot(velocity_x, velocity_y)
        angle = math.atan2(velocity_y, velocity_x) if speed > 1e-9 else 0.0
        if abs(angle) > math.pi / 2:
            angle = math.remainder(angle + math.pi, 2 * math.pi)
            speed = -speed
        speeds.append(speed / wheel_radius)
        angles.append(angle)
    return speeds, angles


def articulated_commands(linear_x, angular_z, wheel_radius, half_track, half_wheelbase, max_angle):
    if abs(linear_x) < 1e-6:
        return [0.0] * 4, [0.0]
    angle = max(-max_angle, min(max_angle, 2 * math.atan(half_wheelbase * angular_z / linear_x)))
    achievable_yaw = linear_x * math.tan(angle / 2) / half_wheelbase
    left = (linear_x - achievable_yaw * half_track) / wheel_radius
    right = (linear_x + achievable_yaw * half_track) / wheel_radius
    return [left, right, left, right], [angle]


class TwistToJointCommands(Node):
    def __init__(self):
        super().__init__("twist_to_joint_commands")
        self.declare_parameter("robot", "")
        self.declare_parameter("wheel_radius", 0.0)
        self.declare_parameter("module_x", 0.0)
        self.declare_parameter("module_y", 0.0)
        self.declare_parameter("half_track", 0.0)
        self.declare_parameter("half_wheelbase", 0.0)
        self.declare_parameter("max_angle", 0.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.robot = self.get_parameter("robot").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.module_x = self.get_parameter("module_x").value
        self.module_y = self.get_parameter("module_y").value
        self.half_track = self.get_parameter("half_track").value
        self.half_wheelbase = self.get_parameter("half_wheelbase").value
        self.max_angle = self.get_parameter("max_angle").value
        self.timeout = self.get_parameter("cmd_timeout").value
        if (self.robot not in ("2910_swerve", "articulated_204g") or self.wheel_radius <= 0
                or self.timeout <= 0 or (self.robot == "2910_swerve" and (self.module_x <= 0 or self.module_y <= 0))
                or (self.robot == "articulated_204g" and
                    (self.half_track <= 0 or self.half_wheelbase <= 0 or self.max_angle <= 0))):
            raise ValueError("Invalid robot or geometry for twist_to_joint_commands")

        self.wheels = self.create_publisher(Float64MultiArray, "/wheels/commands", 10)
        self.steering = self.create_publisher(Float64MultiArray, "/steering/commands", 10)
        self.create_subscription(TwistStamped, "/drive/cmd_vel", self.on_twist, 10)
        self.command = None
        self.received_at = None
        self.create_timer(0.05, self.publish_commands)

    def on_twist(self, message):
        self.command = message.twist
        self.received_at = time.monotonic()

    def publish_commands(self):
        if (self.command is None or
                time.monotonic() - self.received_at > self.timeout):
            linear_x = linear_y = angular_z = 0.0
        else:
            linear_x = self.command.linear.x
            linear_y = self.command.linear.y
            angular_z = self.command.angular.z
        if self.robot == "2910_swerve":
            speeds, angles = swerve_commands(
                linear_x, linear_y, angular_z, self.wheel_radius, self.module_x, self.module_y)
        else:
            speeds, angles = articulated_commands(
                linear_x, angular_z, self.wheel_radius, self.half_track,
                self.half_wheelbase, self.max_angle)
        self.wheels.publish(Float64MultiArray(data=speeds))
        self.steering.publish(Float64MultiArray(data=angles))


def main():
    rclpy.init()
    node = TwistToJointCommands()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()