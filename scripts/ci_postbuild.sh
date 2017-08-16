#!/bin/bash -e

# cd into catkin workspace
cd /root/catkin_ws
catkin config --install
catkin clean -y
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_server
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_plugins
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_viz

echo "ci_postbuild.sh completed."
