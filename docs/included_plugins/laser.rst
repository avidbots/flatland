.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Laser
=====

The laser plugin pushes 2D laser data. It attached to a body of the model with
at a specified position and orientation. It can also publish the static TF between
the body and the laser if asked to. It publishes `sensor_msgs/LaserScan <http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html>`_
messages.

.. code-block:: yaml

  plugins:

      # required, specify Laser to load this plugin
    - type: Laser

      # required, name of the laser plugin, must be unique
      name: laser_back

      # optional, default to "scan", topic to publish the laser data
      topic: scan

      # required, name of the body to attach the laser to
      body: base_link

      # optional, default to [0, 0, 0], in the form of [x, y, yaw], the position
      # and orientation to place laser's coordinate system
      origin: [0, 0, 0]

      # optional, default to true, whether to publish TF
      broadcast_tf: true

      # optional, default to name of this plugin, the TF frame id to publish TF with
      # only used when broadcast_tf=true
      frame: laser_back

      # required, maximum range of the laser, minimum range assumed to be zero, in meters
      range: 20

      # optional, default to 0.0, standard deviation of a gaussian noise
      noise_std_dev: 0

      # required, w.r.t to the coordinate system, scan from min angle to max angle
      # at steps of specified increments
      angle: {min: -2.356194490192345, max: 2.356194490192345, increment: 0.004363323129985824}

      # optional, default to inf (as fast as possible), rate to publish laser scan messages
      update_rate: .inf

      # optional, default to ["all"], the layers to operate the laser at, 
      # lasers only detects objects in the specified layers
      layers: ["all"]

    # another example
    - type: Laser
      name: laser_front
      body: base_link
      range: 30
      angle: {min: -1.5707963267948966, max: 1.5707963267948966, increment: 1.5707963267948966}
      layers: ["layer_1", "layer_2", "layer_3"]
      update_rate: 100
      noise_std_dev: 0.01
      