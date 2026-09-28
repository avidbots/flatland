FreeWheel
=========

FreeWheel models a passive wheel on a dynamic model body. It resists motion
sideways to the wheel's local forward direction while allowing free rolling
along that direction. The resistance is applied at the wheel's offset, so it
also affects the body's rotation. Use one instance per wheel.

.. code-block:: yaml

  plugins:
    - type: FreeWheel
      name: trailer_left_free_wheel
      body: trailer_left_wheel
      offset: [0, 0, 0]
      radius: 0.05
      lateral_resistance: 1.0
      encoder_topic: trailer/left_wheel/rad_s

``body`` is the required body name. ``offset`` defaults to ``[0, 0, 0]``
and specifies the wheel center (x, y, in meters) and forward orientation
(theta, in radians) relative to that body. ``radius`` is required and must be
positive, in meters. ``lateral_resistance`` defaults to 1.0 and scales lateral
velocity correction by the wheel's allocated normal load and step duration;
zero disables sideways resistance. ``friction`` defaults to 1.0 and caps the
lateral force at the allocated normal load times the coefficient, allowing
sliding when the required force exceeds the available grip.

``encoder_topic`` is optional. If provided, it publishes the signed forward
velocity at the wheel center divided by its radius as a
``std_msgs/msg/Float64`` in rad/s. No encoder topic is advertised when it is
omitted. This measures rolling speed without modeling wheel spin or slip.