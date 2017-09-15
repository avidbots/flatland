.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Model TF Publisher
==================

Model TF publisher broadcasts TF of bodies in the model. It obtains world position
and orientation of bodies from the simulation, calculates relative transformation
w.r.t. the specifies reference body, and broadcast the TF.

.. code-block:: yaml

  plugins:

      # required, specify model tf publisher plugin to load this plugin
    - type: ModelTfPublisher

      # required, name of the model tf publisher plugin, must be unique
      name: state_publisher

      # optional, defaults to inf (broadcast every iteration)
      update_rate: .inf

      # optional, defaults to false, whether to broadcast TF w.r.t. to a frame frame
      publish_tf_world: false      

      # optional, defaults to map, the world frame ID, only used when 
      # publish_tf_world is set to true
      world_frame_id: map

      # optional, defaults to the first body in the model, the reference body to
      # broadcast all other bodies' TF with respect to. Does not affect actual
      # transformation, only affects how the TF tree looks
      reference: base_link

      # optional, defaults to [], bodies to not broadcast TF for
      exclude: [] 

    # another example
    - type: ModelTfPublisher
      name: state_publisher_2

    # another example
    - type: ModelTfPublisher
      name: state_publisher_3
      exclude: ["castor_wheel"]