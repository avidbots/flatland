.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Bumper
======

The bumper plugins publishes the state of Collisions of bodies in the model. The
Collision and Collisions message are defined in flatland_msgs. It publishes 
empty collisions if there are nothing colliding. Note that the force values 
published by the bumper is a ball park approximation of impact strength.

The definition of Collisions and Collision messages are shown below.

.. code-block:: bash
  
  string entity_A              # the name of this model
  string body_A                # this model's body
  string entity_B              # name of the other entity, model or layer, collided with this model
  string body_B                # body of the other entity
  float64[] magnitude_forces   # force of impact at each point
  Vector2[] contact_positions  # list of contact points
  Vector2[] contact_normals    # list of contact normals, normals always go from body_A to body_B

.. code-block:: bash
  
  std_msgs/Header header
  Collision[] collisions  # list of Collision message

Vector2 is defined in flatland_msgs

.. code-block:: bash

  float64 x
  float64 y

The descriptions of parameters are shown below.

.. code-block:: yaml

  plugins:

      # required, specify Bumper type to load the plugin
    - type: Bumper

      # required, name of the plugin, unique within the model
      name: MyBumper

      # optional, default to "map", the frame_id of the world coordinate system, 
      # Collision contain collision points that must have a reference frame
      world_frame_id: world

      # optional, default to "collisions", the topic name to publish collision
      # messages, you begin with "/" to ignore model namespace
      topic: collisions

      # optional, default to inf (publishes every time step), the rate in Hz to
      # publish the Collisions messages
      update_rate: .inf

      # optional, default to true, this works together with update_rate. Collision 
      # states may change in between updates with a given update_rate. It might 
      # appear and disappear between when the messages published. This option
      # forces the bumper plugin to always publish if there are non-zero number
      # collisions regardless of update_rate. If there are no collisions, it publishes
      # empty list of collisions at update_rate
      publish_all_collisions: true

      # optional, default to [], the list of bodies to ignore, ignored bodies
      # will not have their collision state published
      exclude: []
    
    # another example
    - type: Bumper
      name: MyOtherBumper
      publish_all_collisions: false
      update_rate: 60
      exclude: ["left_wheel", "right_wheel"]

