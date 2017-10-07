.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Bool Sensor
===========

The boolean sensor plugin attaches to a single body of a model and publishes :command:`true` `std_msgs/Bool <http://docs.ros.org/api/std_msgs/html/msg/Bool.html>`_ if something collides with it, and :command:`false` otherwise.

This can be used on solid bodies (default), or sensor bodies (:command:`sensor: true`).

The plugin also respects layers. For example a sensor body that exists only on layer "robot" will only emit :command:`true` if an entity on layer :robot" collides with it.

The sensor plugin "latches" collisions, so if there is a collision since the previous publishing, it will always publish at least one :command:`true`.


.. code-block:: yaml

  plugins:

      # required, specify BoolSensor type to load the plugin
    - type: BoolSensor

      # required, name of the plugin, unique within the model
      name: MyBoolSensor

      # The ROS topic name to publish on ("/detector_out")
      # This will respect model namespaces
      # e.g. if this model has namespace "foo", it will publish on "/foo/detector_out"
      topic: detector_out

      # The update rate in hz
      update_rate: 10

      # The model body to detect collisions on
      # Currently only supports collisions on a single body
      body: detector