#!/bin/bash -e
# Run from catkin workspace to generate coverage stats
PROJECT_HOME="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

rm -f $PROJECT_HOME/lcov.info
catkin_make -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=ON
catkin_make run_tests
catkin_test_results
lcov --path . --directory . --capture --output-file $PROJECT_HOME/lcov.info
lcov --remove $PROJECT_HOME/lcov.info '/test/*' '/usr/*' '/opt/*' --output-file $PROJECT_HOME/lcov.info
lcov --list $PROJECT_HOME/lcov.info