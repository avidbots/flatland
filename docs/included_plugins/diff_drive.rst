.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png


Diff Drive
==========
This plugin provides common features for a differential drive robots. All
velocities and odometries are w.r.t. the robot origin

* Subscribes to a topic publishing `geometry_msgs/Twist <http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_
  messages, and move the robot at the desired forward and rotation velocities

* Publishes to two topics with `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_
  messages, one for robot odometry which has noise, the other the ground truth
  odometry

.. code-block:: yaml

  plugins:

      # required, specify DiffDrive type to load the plugin
    - type: DiffDrive 

      # required, name of the plugin
      name: turtlebot_drive 

      # required, body of a model to set velocities and obtain odometry
      body: base

      # optional, defaults to odom, the name of the odom frame
      odom_frame_id: odom

      # optional, defaults to inf, rate to publish odometry at, in Hz
      pub_rate: .inf

      # optional, defaults to "cmd_vel", the topic to subscribe for velocity
      # commands
      twist_sub: cmd_vel

      # optional, defaults to "odometry/filtered", the topic to advertise for
      # publish noisy odometry
      odom_pub: odometry/filtered

      # optional, defaults to "odometry/ground_truth", the topic to advertise for publish
      # no noise ground truth odometry
      ground_truth_pub: odometry/ground_truth

      # optional, defaults to "twist", the topic to publish noisy local frame velocity
      # that simulates encoder readings
      twist_pub: twist
      
      # optional, defaults to true, enables the advertising and publishing of both
      # ground truth and noisy odometry
      enable_odom_pub: true
      
      # optional, defaults to true, enables the advertising and publishing of noisy local
      # frame velocity
      enable_twist_pub: true

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
      odom_pose_covariance: [0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0]

      # optional, defaults to the diagonal [x velocity, y velocity, yaw rate] 
      # components replaced by odom_twist_noise with all other values equals zero,
      # must have length of 36, represents a 6x6 covariance matrix for rates x, 
      # y, z, roll, pitch, yaw. This does not involve in any of the noise 
      # calculation, it is simply the output values of odometry twist covariance
      odom_twist_covariance: [0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0
                             0, 0, 0, 0, 0, 0]