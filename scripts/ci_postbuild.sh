#!/bin/bash -e

sudo apt-get install sudo clang-tidy-3.8 -y 

# cd into catkin workspace
cd /root/catkin_ws

# run static analyzer on the first build in the install space
# -DCMAKE_EXPORT_COMPILE_COMMANDS=ON is set in travis.yml
run-clang-tidy-3.8.py -j1 -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_server
run-clang-tidy-3.8.py -j1 -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_plugins
run-clang-tidy-3.8.py -j1 -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_viz

# run tests
catkin config --no-install
catkin clean -y
catkin build
catkin build --catkin-make-args run_tests

echo "ci_postbuild.sh completed."
