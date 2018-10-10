.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Configuring Models
==================
In Flatland, a model is a collection of bodies, and it can be used to represent
any physical things. An example of a model file for a turtlebot (not exactly) is
shown here. Examples can be found in flatland_server/tests.

.. code-block:: yaml

  # required, list of bodies of the model, must have at least one body
  bodies:

      # required, name of the body, must be unique within the model
    - name: base_link

      # optional, default true. If false (see yaml preprocessor for details), this body is skipped
      enabled: true

      # optional, defaults to [0, 0, 0], in the form of [x, y, theta] w.r.t the
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
          layers: ["all"]

          # optional, defaults to true
          # If set to "false" entirely disables collision with this object
          # Does not disable interactions like the laser scan plugin.
          collision: true

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



  # optional, list of model joints
  joints:

      # required, type of the joint, available options are revolute or weld,
      # corresponds to Box2D joint types, applies to all joint types
    - type: revolute

      # required, name of the joint, unique within the body, applies to all
      # joint types
      name: rear_wheel_revolute

      # optional, default true. If false (see yaml preprocessor for details), this joint is skipped
      enabled: true

      # optional, default to false, specifies whether two bodies connected a
      # this joint should collide with each other, applies to all joint types
      collide_connected: false

      # optional, in the format of [lower limit, upper limit], if specified
      # confines the rotation of the joint within the limits, or it is free to
      # rotate 360 degrees otherwise, applies only to revolute joints
      limits: [0, 0]

      # required, specifies the anchor point on each body, applies to all joint
      # types
      bodies:

          # required, name of a body from this body
        - name: rear_wheel

          # required, an anchor point w.r.t. the origin of the body
          anchor: [0, 0]

          # required, name of another body in the model
        - name: base_link

          # required, an anchor point w.r.t. the origin of the body
          anchor: [-0.83, -0.29]


      # now specifying a weld joint, note that weld joint is not 100% fixed due
      # to how the physics is numerically solved, i.e. if an infinite force is
      # applied to immoveable object or a high impact collision, then the joint
      # will deform. Maximum rigidity can be achieved by setting zero to frequency
      # and damping, and increase velocity and position iterations in world
      # properties. For 100% zero deformation, use a single body with multiple
      # fixtures
    - type: weld
      name: left_wheel_weld

      # optional, defaults to 0, specifies the angle of the weld, applies only
      # to revolute joints
      angle: 0

      # optional, defaults to 0, specifies the frequency of the weld joint in
      # Box2D, unit is in Hz, applies only to weld joints
      frequency: 0

      # optional, defaults to 0, specifies the damping ratio of the weld joint
      # in Box2D, applies only to weld joints
      damping: 0
      bodies:
        - name: left_wheel
          anchor: [0, 0]
        - name: base_link
          anchor: [-0.83, -0.29]


    - type: weld
      name: right_wheel_weld
      bodies:
        - name: left_wheel
          anchor: [0, 0]
        - name: base_link
          anchor: [-0.83, -0.29]



  # optional, list of plugins for the model
  plugins:

      # required, type of the plugin to load. Note the plugin must be configured
      # property to be discovered. See the Writing Model Plugins page
    - type: Laser

      # optional, default true. If false (see yaml preprocessor for details), this plugin is skipped
      enabled: true

      # required, name of the plugin to load, must be unique in a model
      name: kinect

      # the rest of the parameters are extracted by the corresponding model plugins
      body: base_link
      range: 20
      angle: {min: -2.356194490192345, max: 2.356194490192345, increment: 0.004363323129985824}
      noise_std_dev: 0.01
      update_rate: 10


    - type: DiffDrive
      name: turtlebot_drive
      body: base_link
