#!/bin/bash -e

# cd into catkin workspace
cd /root/catkin_ws

# run static analyzer
catkin clean -y
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "running clang tidy on flatland_server..."
run-clang-tidy-3.8.py -j1 -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_server | /root/catkin_ws/src/flatland/scripts/parse_clang_tidy.py

echo "running clang tidy on flatland_plugins..."
run-clang-tidy-3.8.py -j1 -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_plugins | /root/catkin_ws/src/flatland/scripts/parse_clang_tidy.py

echo "running clang tidy on flatland_viz..."
run-clang-tidy-3.8.py -j1 -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_viz | /root/catkin_ws/src/flatland/scripts/parse_clang_tidy.py

echo "ci_postbuild.sh completed."
