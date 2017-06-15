# README #

### What is this repository for? ###

* Flatland is a performance centric 2d robot simulator
* Release Version: None

### How do I get set up? ###

* Git clone flatland into your catkin workspace's src folder, and catkin_make.

### Contribution guidelines ###

* code should be unit tested using gtest/rosunit when practical.
* code must be formatted as per clang-format-3.8 --style=file
* code must pass scan-build's static analysis. To run, clean your build directory and run:
  
  scan-build-3.8 --status-bugs catkin_make -j1

### Who do I talk to? ###

* Please direct any questions to @josephduchesne