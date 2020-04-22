.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Tricycle Drive
==============

This plugin provides features for a specific implementation of tricycle robots.

* Subscribes to a topic publishing `geometry_msgs/Twist <http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_
  messages. Note the yaw rate is the angle of the front wheel instead of actual
  yaw rate.

* Publishes to two topics of `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_
  messages, one for robot odometry which has noise, the other the ground truth
  odometry

The plugins makes several assumptions about the robot drive train, and uses
these assumptions to extract required geometry parameters such as wheel base
and axle track from the robot.

* Twist yaw rate is the angle of the front wheel
* Twist x velocity is the velocity at the front wheel
* The origin of the body is at the front wheel
* The model has front wheel with a revolute joint to the body, the anchor point
  on the wheel is at the origin, the anchor point on the body is at the origin
* The model has a rear left wheel with a weld joint to the body, the anchor
  point on the wheel is at the origin
* The model has rear right wheel with a weld joint to the body, the anchor point
  on the wheel is at the origin
* The perpendicular line from front wheel to rear axle (line segment from the
  rear left wheel to the rear right wheel) bisects the rear axle
* The x velocity and angle given are physically feasible, no additional checks
  are applied, and the tricycle model is used directly
* Kinematic limits can be specified for the steering angle, steering angular
  velocity, and steering angular acceleration, to model the tricycle model's
  front wheel steering response to commanded steering angle changes.
  This does not fully account for lag in the steering response, but provides a
  slightly more realistic acceleration- and velocity-bounded steering response.
  A future update will introduce the steering lag model to the tricycle plugin.

The plugin also sets the angle of the front wheel for visualization purposes, it
has no effect on the actual motion of the robot.

.. code-block:: yaml

  plugins:

      # required, specify TricycleDrive type to load the plugin
    - type: TricycleDrive

      # required, name of the plugin
      name: robot_drive

      # required, body of a model to set velocities and obtain odometry
      body: base

      # required, the front wheel joint
      front_wheel_joint: front_wheel_revolute

      # required, the rear left wheel joint
      rear_left_wheel_joint: rear_left_wheel_weld

      # required, the rear right wheel joint
      rear_right_wheel_joint: rear_right_wheel_weld

      # optional, defaults to odom, the name of the odom frame
      odom_frame_id: odom

      # optional, defaults to inf, rate to publish odometry at, in Hz
      pub_rate: inf

      # optional, defaults to "cmd_vel", the topic to subscribe for velocity
      # commands
      twist_sub: cmd_vel

      # optional, defaults to "odometry/filtered", the topic to advertise for
      # publish noisy odometry
      odom_pub: odometry/filtered

      # optional, defaults to "cmd_vel", the topic to advertise for publish
      # no noise ground truth odometry
      ground_truth_pub: odometry/ground_truth

      # optional, defaults to [0, 0, 0], corresponds to noise on [x, y, yaw],
      # the variances of gaussian noise to apply to the pose components of the
      # odometry message
      odom_pose_noise: [0, 0, 0]

      # optional, defaults to [0, 0, 0], corresponds to noise on
      # [x velocity, y velocity, yaw rate], the variances of gaussian noise to
      # apply to the twist components of the odometry message
      odom_twist_noise: [0, 0, 0]

      # optional, defaults to the diagonal [x, y, yaw] components replaced by
      # odom_pose_noise with all other values equals zero, must have length of 36,
      # represents a 6x6 covariance matrix for x, y, z, roll, pitch, yaw.
      # This does not involve in any of the noise calculation, it is simply
      # the output values of odometry pose covariance
      odom_pose_covariance: [0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0]

      # optional, defaults to the diagonal [x velocity, y velocity, yaw rate]
      # components replaced by odom_twist_noise with all other values equals zero,
      # must have length of 36, represents a 6x6 covariance matrix for rates x,
      # y, z, roll, pitch, yaw. This does not involve in any of the noise
      # calculation, it is simply the output values of odometry twist covariance
      odom_twist_covariance: [0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0]

      # optional, defaults to 0.0 which means no limit
      # sets the steering angular limit of the front wheel (absolute, rad)
      # the steering angle will not exceed this absolute limit
      max_steer_angle: 0.0

      # optional, defaults to 0.0 which means no limit
      # sets the steering angular velocity limit (absolute, rad/s)
      # the steering angular velocity will not exceed this absolute limit
      max_angular_velocity: 0.0

      # optional, defaults to 0.0 which means no limit
      # sets the steering angular acceleration limit (absolute, rad/s^2)
      # the steering angular acceleration will not exceed this absolute limit
      max_steer_acceleration: 0.0