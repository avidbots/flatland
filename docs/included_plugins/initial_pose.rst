Initial Pose
============

The ``InitialPose`` model plugin publishes the model's starting world pose as a
``geometry_msgs/msg/PoseWithCovarianceStamped`` estimate. It waits for a subscriber
and publishes once, so a localization node such as Nav2 AMCL can receive the pose
after it starts. The pose comes from the first body in the model, after the
model's initial world transform has been applied.

.. code-block:: yaml

  plugins:
    - type: InitialPose
      name: initial_pose
      # Optional, defaults to map.
      frame_id: map
      # Optional, defaults to /initialpose.
      topic: /initialpose
      # Optional covariance diagonal. x and y are in m^2; yaw is in rad^2.
      variance:
        x: 0.01
        y: 0.01
        yaw: 0.00030461741978670857

The default x and y variances are ``0.01 m^2`` each. The default yaw variance
is ``(pi / 180)^2 rad^2``, corresponding to a one-degree standard deviation.
All other covariance entries are zero. Each variance value can be set
independently; omitted values retain their defaults.