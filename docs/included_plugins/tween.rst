.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Tween
=====

The tween plugin moves a body from it's start location to a relative end position.

This can be done using a variety of tween modes and easings.

Modes
^^^^^

- ``yoyo`` - move back and forth between the start and end position repeatedly
- ``once`` - move to the end position and stop
- ``loop`` - move to the end position then teleport back, then repeat indefinitely
- ``trigger`` - move towards the end position if the "trigger_topic" ros topic recieves ``true``, move towards start otherwise.

Easings
^^^^^^^

The default easing is "linear", but the following is a complete list of supported easing modes:

- ``linear``
- ``quadraticIn``, ``quadraticOut``, ``quadraticInOut``
- ``cubicIn``, ``cubicOut``, ``cubicInOut``
- ``quarticIn``, ``quarticOut``, ``quarticInOut``
- ``quinticIn``, ``quinticOut``, ``quinticInOut``
- ``exponentialIn``, ``exponentialOut``, ``exponentialInOut``
- ``circularIn``, ``circularOut``, ``circularInOut``
- ``backIn``, ``backOut``, ``backInOut``
- ``elasticIn``, ``elasticOut``, ``elasticInOut``
- ``bounceIn``, ``bounceOut``, ``bounceInOut``

You can see visual examples of these easing modes `here at easings.net <http://easings.net/>`_.

Configuration
^^^^^^^^^^^^^

.. code-block:: yaml

  plugins:

      # required, specify Tween type to load the plugin
    - type: Tween

      # required, name of the plugin, unique within the model
      name: MyTweenPlugin

      # The tween mode (documented above, default 'yoyo')
      mode: yoyo

      # The easing mode (documented above, default 'linear')
      easing: cubicInOut

      # The ROS topic name to subscribe to for 'trigger' mode ("/tween_trigger")
      # This will respect model namespaces
      # e.g. if this model has namespace "foo", it will publish on "/foo/tween_trigger"
      trigger_topic: tween_trigger

      # animation duration in seconds (default 1 second)
      duration: 10

      # The tween delta pose (delta x, y and angle)
      # The following will move the object to x += 2, y += 3, and angle += 1.1
      # relative to the start position
      delta: [2, 3, 1.1]

      # The model body to move
      body: some_body