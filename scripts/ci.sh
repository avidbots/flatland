#!/bin/bash -e

apt-get install sudo clang-format-3.8 clang-3.8 lcov -y 

# change to the file's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd DIR/../

# check files are correctly formatted
git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -i && git diff --exit-code || { git reset --hard; false; } 

# cd into catkin workspace
cd ~/catkin_ws
catkin clean
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_server
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_plugins
run-clang-tidy-3.8.py -clang-tidy-binary=clang-tidy-3.8 -p=build/flatland_viz