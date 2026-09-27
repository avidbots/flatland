DriveWheel
==========

DriveWheel applies a traction-limited force at a wheel center on a model body.
Use one plugin per driven wheel; the wheel has no Box2D rotational body. Wheel
position is integrated from measured forward speed, so slip or external motion
is reflected in the reported state. A wheel also resists sideways motion.

.. code-block:: yaml

  plugins:
    - type: DriveWheel
      name: left_drive
      body: base
      offset: [0.0, 0.3, 0.0]
      radius: 0.1
      mode: velocity
      friction: 1.0
      lateral_resistance: 1.0
      joint_commands_topic: /robot_joint_commands
      joint_states_topic: /robot_joint_states

``body`` and a positive ``radius`` (meters) are required. ``offset`` is the
wheel center (x, y in meters) and forward heading (radians) in body coordinates;
it defaults to ``[0, 0, 0]``. ``joint_name`` defaults to the plugin's name.
``mode`` is ``velocity`` (default) or ``effort``; commands are wheel speed in
rad/s or axle torque in N m. Velocity control accelerates toward the requested
wheel speed, subject to available traction. ``friction`` defaults
to 1.0 and limits the combined longitudinal and lateral force to the wheel's
allocated normal load times the coefficient. ``lateral_resistance`` defaults
to 1.0 and scales the lateral velocity correction by supported mass and step
duration. At low slip the wheel resists sideways motion; beyond the friction
limit it slides. This load-limited approximation does not model a fitted tire
slip curve or wheel rotational inertia.

Commands are ``control_msgs/msg/JointCommand`` on
``<joint_commands_topic>/<mode>``. A message must have a matching
``interface_name`` and a ``joint_names`` entry equal to the plugin's ``name``
(not ``joint_name``), with a finite corresponding ``values`` entry. Commands
without that plugin name are ignored, so multiple wheels can share the same
command topic. Until the first command the wheel applies no drive force. State is
published after each step as ``sensor_msgs/msg/JointState`` (position,
velocity, applied effort) on ``joint_states_topic`` with sensor-data QoS.
The topic defaults match the ros2_control
``joint_command_topic_hardware_interface/JointCommandTopicSystem``. Absolute
topics remain global; relative overrides are prefixed by the model namespace.

Flatland assigns each model's weight (total body mass times 9.81 m/s²) across
its contact-aware plugins each step according to their world-space points and
the model's center of mass. This is a planar support approximation, not a
terrain contact sensor; unloaded points outside the support polygon receive
zero force. Configure at least two separated wheels for load transfer.