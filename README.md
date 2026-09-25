# README #

### What is Flatland? ###

Flatland is a lightweight 2.5D robot simulator for ROS 1 and ROS 2.

- Simulate 2D motion with a variety of drivetrains.
- Extend worlds and robots with plugins for dynamic maps, lidar, and drive control.
- Run faster than real time; connected ROS nodes may encounter timing issues if they cannot keep up.
- Allows headless execution, with RViz based visualization if needed

### Dev container (recommended) ###

1. Install Docker, VS Code, and the Dev Containers extension.
2. Open the Flatland repository in VS Code and run **Dev Containers: Reopen in Container**.
3. Choose Humble, Jazzy, Kilted, Lyrical, or Rolling when prompted.

Once the container opens, Flatland is ready to use. Run **Dev Containers: Rebuild Container** if you change package dependencies.

### Without a dev container (ROS 2) ###

Install ROS 2 on your host, or in a container, then run the following:

```bash
mkdir -p ~/flatland_ws/src
git clone https://github.com/avidbots/flatland.git ~/flatland_ws/src/flatland
source /opt/ros/*/setup.bash
cd ~/flatland_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

For older ROS 1 / catkin setups, check out a ROS 1-compatible Flatland version in your catkin workspace's `src` directory. From the workspace root, run `rosdep install --from-paths src --ignore-src -y` and `catkin build`.

### Who do I talk to? ###

* Please direct any questions to @josephduchesne

### Documentation ###

* How to use: http://flatland-simulator.readthedocs.io
* Doxygen: http://flatland-simulator-api.readthedocs.io
* For a quick start use: https://github.com/avidbots/turtlebot_flatland

### License ###
All Flatland code is BSD 3-clause licensed (see LICENSE for details)

Flatland uses a number of open source libraries that it includes in its source tree:
- [ThreadPool](https://github.com/progschj/ThreadPool) Copyright (c) 2012 Jakob Progsch, Václav Zeman (zlib license)
- [Tweeny](https://github.com/mobius3/tweeny) Copyright (c) 2016 Leonardo Guilherme de Freitas (MIT license)
- [Box2d](https://github.com/erincatto/Box2D) Copyright (c) 2006-2017 Erin Catto [http://www.box2d.org](http://www.box2d.org) (zlib license)

