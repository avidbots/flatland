.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Configuring World
=================

The simulation world is defined by a YAML file as shown below. Examples can
be found in flatland_server/tests.

.. code-block:: yaml

  # required, world properties
  properties:

    # optional, defaults to 10, number of velocity iterations for the Box2D 
    # physics solver
    velocity_iterations: 10

    # optional, defaults to 10, number of position iterations for the Box2D 
    # physics solver
    position_iterations: 10
  


  # required, specifies a list of layers, maximum number of layers is 16,
  # constrained by Box2D's collision masks
  layers: 

      # required, name of the layer, each layer name must be unique
    - name: "layer_1"

      # required, path to the map yaml file, begin with "/" to indicate
      # absolute path, otherwise relative path w.r.t this file is used
      map: "layer_1.yaml"

      # optional, defaults to [1, 1, 1, 1], color for visualization specified
      # by [r, g, b, alpha]
      color: [1, 1, 1, 1] 

      # you can also specify a list of names. These names will point to the same
      # entity in the physics engine. This introduces an efficient way of organizing
      # entities into the same physical layer without loading the same map more 
      # than once. Only the first name in this list is used to identify this entity,
      # such as in visualizations or collision handling. The number of layers here
      # also counts towards the maximum number of 16 layers. 
    - name: ["layer_2", "layer_3", "layer_4"]
      map: "/absolute/path/layer_2.yaml"
  


  # optional, specifies a list of models to be loaded into the world, if you
  # don't need models to load right at the start, you don't need provide this 
  # yaml entry
  models:  
      
      # required, name of the model, must be unique in the world
    - name: turtlebot1

      # optional, defaults to "", specifies the namespace of the robot. Used
      # for situations where multiple models of the same type are instantiated,
      # in order to avoid conflicts in TF transforms and ROS topic names. It is
      # used to initialize the namespace of the node handle of model plugins
      # under this model. Not enforced to be unique when set, but it should be
      # unique for all practical purposes
      namespace: ""

      # optional, defaults to [0, 0, 0], pose to put the model, in the format
      # of [position x, position y, angle yaw] in the world coordinate
      pose: [0, 0, 0]

      # required, path to the model yaml file, begin with "/" to indicate
      # absolute path, otherwise relative path w.r.t this file is used
      model: "turtlebot.model.yaml"

    - name: turtlebot12
      namespace: "turtlebot2"
      model: "turtlebot.model.yaml"

    - name: person 
      model: "/absolute/path/person.model.yaml"
      