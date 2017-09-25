.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png


Create Model Tutorial
======================
  


**Description:** This tutoial provides the information needed to create a model for use in the flatland simulator.

**Tutorial Level:** BEGINNER

This tutorial provides step by step instruction on how to create your own 
model and get it working inside the flatland simulator.
   
1. Prerequisites
----------------

The following tutorials provide a good foundation for understanding the flatland 
simulator architecture:

create_plugin_.

.. _create_plugin: file:///home/mikeb/Dev/flatland_github/src/flatland/docs/_build/html/flatland_tutorials/create_plugin.html

2. Getting Started
------------------

.. note:: Flatland includes the Box2d physics engine library. 

Box2D is a 2D rigid body simulation library. For the simulation of a robotic 
cleaner and its environment, the vehicle moves along the floor: it is 
constrained to translate in only two dimensions. 

We take advantage of this fact to simplify the problem of simulating a physical 
body from six dimensions to three dimensions. Specifically, the vehicle is constrained 
by gravity to move only in the x-y plane. The vehicle's rotation is also constrained by 
the floor to rotate only about the z-axis.

Our 2D approximation of the 3D world uses x, y and theta. The Box2d library is specifically 
designed for this type of simulation. Flatland provides Box2d with the information needed to
setup the 2D simulation including initial position and model properties

3. Box2d Core Concepts
----------------------

.. _b2d_docs: http://box2d.org/manual.pdf

For a complete description of the Box2d core concepts, see the user doccumentation here b2d_docs_.
This section provides a brief description of the Box2d core concepts that apply to making a model.

+------------------------------------------------------------------------------+
| | **shape**: A shape is 2D geometrical object, such as a circle or polygon.  |
+------------------------------------------------------------------------------+
| | **rigid body**: A chunk of matter that can not be deformed                 |
+------------------------------------------------------------------------------+
| | **fixture**: A fixture binds a shape to a body and adds material           |
| | properties such as density, friction, and restitution.                     | 
+------------------------------------------------------------------------------+
| |  **constraint**: A constraint is a physical connection that removes        |
| |   degrees of freedom from bodies.                                          | 
+------------------------------------------------------------------------------+
| |  **joint**: This is a constraint used to hold two or more bodies together. |
+------------------------------------------------------------------------------+
| |  **joint limit**: A joint limit restricts the range of motion of a joint.  |
+------------------------------------------------------------------------------+
| |  **world**: A physics world is a collection of bodies, fixtures, and       |
| |   constraints that interact together                                       | 
+------------------------------------------------------------------------------+

You can create a brand new model in flatland using a model definition yaml file.
In that file, you define the Box2d model parameters including the shape, fixtures,
joints and limits.

You can also write a plugin and asscociate it with your model. Through the plugin 
you have access to the Box2d physics engine. So you can do things like apply a force
or an impact to any body in your model. You can also get back the new vehicle pose
so it can be displayed inside the Flatland world.


3. Yaml Model File Format
-------------------------

.. code-block:: Yaml

    bodies:
    - name: base
        type: dynamic  
        color: [1, 1, 1, 0.75] 
        footprints:
        - type: polygon
            density: 100
            points: [ [-1.03, -0.337],
                    [.07983, -0.337],
                    [.30, -.16111],
                    [.30, .16111],
                    [.07983, 0.337],
                    [-1.03, 0.337] ]


    - name: front_wheel
        color: [1, 1, 1, 0.75] 
        footprints:
        - type: polygon
            density: 1.0
            points: [[ 0.0875, -0.0250],
                    [ 0.0875,  0.0250],
                    [-0.0875, 0.0250],
                    [-0.0875, -0.0250]]

    - name: rear_left_wheel
        color: [1, 1, 1, 0.75] 
        footprints:
        - type: polygon
            density: 1.0
            points: [[ 0.0875, -0.0255],
                    [ 0.0875,  0.0255],
                    [-0.0875, 0.0255],
                    [-0.0875, -0.0255]]

    - name: rear_right_wheel
        color: [1, 1, 1, 0.75] 
        footprints:
        - type: polygon
            density: 1.0
            points: [[ 0.0875, -0.0255],
                    [ 0.0875,  0.0255],
                    [-0.0875, 0.0255],
                    [-0.0875, -0.0255]]

    joints:
    - type: revolute
        name: front_wheel_revolute
        bodies: 
        - name: front_wheel
            anchor: [0, 0]
        - name: base
            anchor: [0, 0]

    - type: weld
        name: rear_right_wheel_weld
        bodies: 
        - name: rear_left_wheel
            anchor: [0, 0]
        - name: base
            anchor: [-0.83, 0.29]

    - type: weld
        name: rear_left_wheel_weld  
        bodies: 
        - name: rear_right_wheel
            anchor: [0, 0]
        - name: base
            anchor: [-0.83, -0.29]

    plugins:

    - type: ModelTfPublisher
        name: tf_publisher
        publish_tf_world: true

    - type: TricycleDrive
        name: cleaner_drive 
        body: base
        front_wheel_joint: front_wheel_revolute
        rear_left_wheel_joint: rear_left_wheel_weld
        rear_right_wheel_joint: rear_right_wheel_weld
        odom_frame_id: map

    - type: Laser
        name: laser_front
        frame: laser_front
        topic: scan
        body: base
        broadcast_tf: true
        origin: [0.28, 0, 0]
        range: 20
        angle: {min: -2.356194490192345, max: 2.356194490192345, increment: 0.004363323129985824}
        noise_std_dev: 0.05
        update_rate: 40




