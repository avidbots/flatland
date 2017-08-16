#!/bin/bash -e

sudo apt-get install sudo clang-format-3.8 clang-3.8 lcov -y 

# change to the file's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR/../

# check files are correctly formatted
git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -i && git diff --exit-code || { git reset --hard; false; } 

echo "ci_prebuild.sh completed."