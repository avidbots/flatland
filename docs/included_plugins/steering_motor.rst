SteeringMotor
=============

SteeringMotor actuates a model's revolute Box2D joint and publishes its angle,
angular speed and applied torque as joint state.
This plugin is provided by ``flatland_controls_plugins``.

.. code-block:: yaml

  plugins:
    - type: flatland_controls_plugins::SteeringMotor
      name: front_steering
      joint: steering_joint
      mode: position
      max_effort: 100.0
      max_speed: 10.0
      position_gain: 10.0
      limit:
        lower: -0.5
        upper: 0.5
      joint_commands_topic: /robot_joint_commands
      joint_states_topic: /robot_joint_states

``joint`` names the required revolute joint. The plugin's ``name`` is the joint
name used in ROS messages. ``mode`` is ``position`` (default), ``velocity``
or ``effort``; commands have units of radians, rad/s or N m respectively.
``max_effort`` (default 100 N m) limits motor or applied torque. In position
mode ``position_gain`` (default 10 1/s) turns angle error into motor speed;
``max_speed`` (default 10 rad/s) caps commanded speed in both position and
velocity modes. Joint limits configured on the model remain active.

The optional ``limit`` map sets lower and upper joint angles in radians.
Both values default to NaN, which leaves any model-defined joint limits unchanged.
If set, both must be finite and within [-0.95*pi, 0.95*pi], with ``upper``
strictly greater than ``lower``. A valid pair enables the joint limits and
replaces any model-defined bounds.

The plugin subscribes to ``control_msgs/msg/JointCommand`` on
``<joint_commands_topic>/<mode>`` and publishes ``sensor_msgs/msg/JointState``
after each physics step on ``joint_states_topic`` using sensor-data QoS.
Commands take effect only when ``interface_name`` matches ``mode`` and
``joint_names`` includes the plugin's ``name``, paired with
a finite value. Multiple steering joints can share the same command topic;
each plugin uses only its named value. Published joint states use that same name.
The default topics are ``/robot_joint_commands`` and ``/robot_joint_states``
for the ros2_control joint command topic hardware interface. Absolute topic
overrides remain global; relative overrides use the model namespace.