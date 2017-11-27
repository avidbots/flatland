.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png


GPS
==========
This plugin provides a simple simulation of a perfectly-accurate GPS receiver.

* Reference latitude and longitude are set in the plugin's YAML parameters. The reference coordinates correspond to (0, 0) in the Flatland world frame.

* The model's ground truth position in the Flatland world frame is treated as the position in an East-North-Up (ENU) reference frame relative to (0, 0).

* The ENU coordinates are converted to latitude and longitude using the specified reference coordinates and plugging everything into standard equations as defined on `Wikipedia <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF>`_

* Publishes a `sensor_msgs/NavSatFix <http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html>`_ message with the current geodetic position of the vehicle.

.. code-block:: yaml

  plugins:

      # required, specify Gps type to load the plugin
    - type: Gps

      # required, name of the plugin
      name: turtlebot_gps

      # required, body of a model to set location of simulated GPS antenna
      body: base

      # optional, defaults to "gps/fix", the topic to advertise for GPS fix outputs
      topic: gps/fix

      # optional, defaults to 10, rate to publish GPS fix, in Hz
      update_rate: 10

      # optional, defaults to true, whether to publish TF
      broadcast_tf: true

      # optional, default to name of this plugin, the TF frame id to publish TF with
      # only used when broadcast_tf=true
      frame: my_frame_name

      # optional, defaults to 0.0, latitude in degrees corresponding
      # to (0, 0) in world frame
      ref_lat: 0.0

      # optional, defaults to 0.0, longitude in degrees corresponding
      # to (0, 0) in world frame
      ref_lon: 0.0

      # optional, default to [0, 0, 0], in the form of [x, y, yaw], the position
      # and orientation to place GPS antenna relative to specified model body
      origin: [0, 0, 0]
