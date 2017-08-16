#!/bin/bash

dpkg -s doxygen

# if doxygen is not installed, install doxygen
if [ $? -ne 0 ]; then
    echo "Doxygen not installed, installing doxygen"
    sudo apt-get install doxygen -y
fi

# change directory to the doxygen file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR/../

# run doxygen
doxygen Doxyfile