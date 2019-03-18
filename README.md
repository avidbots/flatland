# README #

![ci test result](https://travis-ci.com/avidbots/flatland.svg?branch=master "CI Test Result")


### What is this repository for? ###

* Flatland is a performance centric 2d robot simulator
* [Roadmap on trello](https://trello.com/b/s9poP2Jg/flatland-2d-simulator)
* Release Version: None

### How do I get set up? ###

* Git clone flatland into your catkin workspace's src folder, and catkin build.
* Optionally check out [turtlebot_flatland](https://github.com/avidbots/turtlebot_flatland) and run the turtlebot nav stack
* Run `rosdep install --from-paths src --ignore-src` in your catkin workspace to install any missing rosdeps

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

