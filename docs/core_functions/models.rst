Configuring Models
==================
In Flatland, a model is a collection of bodies, and it can be used to represent
any physical things. A example of a model file for a turtlebot (not exactly) is
shown here.

.. code-block:: yaml

  # required, list of bodies of the model, must have at least one body
  bodies: 

      # required, name of the body, must be unique within the model
    - name: base_link

      # optional, default to [0, 0, 0], in the form of [x, y, theta] w.r.t the
      # model pose specified in world yaml. Pose is simply the initial pose
      # of the body, it does not add additional constrains in any way
      pose: [0, 0, 0] 

      # optional, default to dynamic, must be one of dynamic, static, kinematic,
      # these are consistent with Box2D's body types
      type: dynamic

      # optional, default to [1 ,1, 1, 0.5], in the form of [r, g, b, alpha],
      # for visualization
      color: [1, 1, 1, 0.5] 

      # optional, defaults to 0.0, specifies the Box2D body linear damping
      linear_damping: 0

      # optional, defaults to 0.0, specifies the Box2D body angular damping
      angular_damping: 0

      # required, list of footprints for the body, you can have any number of
      # footprints
      footprints:

          # required, type of footprint, valid options are circle and polygon
        - type: circle

          # required, radius of the circle footprint, only used for circle type
          radius: 0.5

          # optional, defaults to [0, 0], in the form of [x, y] w.r.t the origin
          # of the body, only used for circle type
          center: [0.0, 0.0]
          
          # optional, defaults to ["all"], a list of layers specifying which
          # layer the footprint belongs to, collisions/contacts only occurs
          # between objects in the same layer, used for all footprint types
          layers = ["all"]

          # required, the density of footprint in kg/m^2 in Box2D, used for all
          # footprint types
          density: 1

          # optional, default to 0.0, specifies Box2D fixture friction, used for
          # all footprint types
          friction: 0

          # optional, default to 0.0, specifies Box2D fixture restitution, used
          # for all footprint types
          restitution: 0

          # optional, default to false, whether to set as Box2D sensor, sensor
          # detects collisions/contacts but does not have responses, used for
          # all footprint types
          sensor: false


    # body for the rear wheel
    - name: rear_wheel
      footprints:

          # an example for polygon type
        - type: polygon

          # required, vertices for polygon, 3 <= length <= 8 (limited by Box2D), 
          # in the form of [x, y] coordinates w.r.t body origin, only used for
          # polygon type
          points: [[-0.05, -0.05], [-0.05, 0.05], [0.05, 0.05], [0.05, -0.05]]
          density: 1

    # body for left wheel
    - name: left_wheel
      footprints:
        - type: polygon
          points: [[-.125, -0.05], [-.125, 0.05], [.125, 0.05], [.125, -0.05]]
          density: 1

    # body for right wheel, notice how left wheel and right wheel are the same
    # they will be spawned in the same location with pose [0, 0, 0] by default,
    # joints need to be used to snap them in place
    - name: right_wheel
      footprints:
        - type: polygon
          points: [[-.125, -0.05], [-.125, 0.05], [.125, 0.05], [.125, -0.05]]
          density: 1
  
  # optional, list of joints in the model that constrains bodies
  joints:
    - type: revolute
      name: rear wheel

  
  plugins:
    - type: DiffDrive 
      name: turtlebot_drive 
      body: base
  
    - type: Laser
      name: kinect
      body: base_link
      range: 20
      angle: {min: -2.356194490192345, max: 2.356194490192345, increment: 0.004363323129985824}
      noise_std_dev: 0.01
      update_rate: 10
