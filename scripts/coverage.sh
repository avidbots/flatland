#!/bin/bash -e
# This script is used in CI testing (and can be used manually)
# to generate code coverage statistics, showing which code has been unit tested
# to use it, you must install "lcov" (`sudo apt-get install lcov`)
#
# Run from catkin workspace to generate coverage stats
PROJECT_HOME="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/.."

rm -f $PROJECT_HOME/lcov.info
catkin_make -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=ON -j1
catkin_make run_tests -j1
catkin_test_results
lcov --path . --directory . --capture --output-file $PROJECT_HOME/lcov.info
lcov --remove $PROJECT_HOME/lcov.info '/thirdparty/' '/test/' '/usr/' '/opt/' --output-file $PROJECT_HOME/lcov.info
lcov --list $PROJECT_HOME/lcov.info