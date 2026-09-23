.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png


IMU
==========
This plugin provides a simple simulation of an IMU (three-axis accelerometer and three-axis gyroscope).

* The reference frame of the IMU is x-forward, y-left, z-up. Because Flatland is 2D, only the x and y axes are populated with accelerations only the z axis is populated with an angular rate, and the orientation has components only in the z and w fields.

* The measurements are modelled in a simple manner; they are simply the ground truth corrupted with zero-mean Gaussian noise.

* Publishes a `sensor_msgs/Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_ message with the simulated measurements

.. code-block:: yaml

  plugins:

      # required, specify Imu type to load the plugin
    - type: Imu

      # required, name of the plugin
      name: turtlebot_imu

      # required, body of a model to set location of simulated IMU
      body: base

      # optional, defaults to true, whether to publish the IMU data
      enable_imu_pub: true

      # optional, defaults to imu, TF frame name of the IMU
      imu_frame_id: imu

      # optional, defaults to imu/filtered, the topic on which the noisy imu data is published
      imu_pub: imu/filtered

      # optional, defaults to imu/ground_truth, the topic on which the true (no noise added) imu data is published
      ground_truth_pub: imu/ground_truth

      # optional, defaults to [0.0, 0.0, 0.0], the diagonal of the covariance matrix of the zero-mean noise added to the orientation . Note that because the simulation is 2D, only the third value (i.e., yaw noise) is used.
      orientation_noise: [0.0, 0.0, 0.0]

      # optional, defaults to [0.0, 0.0, 0.0], the diagonal of the covariance matrix of the zero-mean noise added to the angular velocity. Note that because the simulation is 2D, only the third value (i.e., yaw rate noise) is used.
      angular_velocity_noise: [0.0, 0.0, 0.0]

      # optional, defaults to [0.0, 0.0, 0.0], the diagonal of the covariance matrix of the zero-mean noise added to the linear acceleration. Note that because the simulation is 2D, only the first two values (i.e., x and y acceleration noise) are used.
      linear_acceleration_noise: [0.0, 0.0, 0.0]

      # optional, defaults to inifinity (i.e., as fast as the simulation updates), rate to publish IMU measurements, in Hz
      update_rate: 40

      # optional, defaults to true, whether to publish TF
      broadcast_tf: true
