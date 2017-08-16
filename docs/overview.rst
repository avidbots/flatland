Flatland Simulator Overview
===========================

This document provides a high level overview of the flatland simulator.

Flatland simulator is a light weight, performance centric 2D robot simulator. It
is built to integrate directly with ROS and uses Box2D for physics simulation.
This project is divided into three ROS packages: flatland_server, flatland_plugins,
and flatland_viz. 

Flatland Server
---------------
Flatland server contains the core functionalities of the flatland simulator
including simulation environment, objects, plugin interfaces, and ROS services.
All of flatland server runs in a single ROS node, in a single thread. The 
following are some of the commonly used terminologies in the Flatland Server.

* **Simulation Manager**: Simulation manager is responsible for managing the
  state of the simulator. It manages how the world progresses through time,
  ROS Services, and plugins.

* **World**: World is everything that lives in the simulation. Including the 
  layers, models, time, and other physical properties.

* **Layer**: Layer represents the static environment features in the simulation
  such as walls in a building. It allows large scale static features to be loaded
  and represented in a efficient manner. Flatland allows a powerful feature where
  multiple layers can be used to represent a effectively 2.5D world. 

* **Model**: Models are collection of bodies. It can be any other physical 
  entities in the world. It can be made to represent robots, people, automatic
  doors, or anything else.

* **Body**: A body is a indeformable and inseparable piece of entity. A body does
  not need to contiguous, the actual physical outlines are defined by a collection
  of footprints. Body manages physical states such as position and velocity.
  The concept is equivalent to a Body in Box2D physics engine.

* **Footprint**: A footprint defines the actual physical size, shape, 
  and material properties. It manages physical properties such as density and 
  friction. It is contiguous.  The concept is equivalent to a Fixture in Box2D
  physics engine.

* **Joint**: A joint is used to connect two pieces of bodies. It defines additional
  constraints on how the two bodies must interact with each other. The concept is
  equivalent to a Joint in Box2D physics engine.

* **Pose**: Pose is always the x, y, and angle of an object

* **Origin**: Origin is the location of (0, 0) in 

