#!/bin/bash -e

sudo apt-get install clang-3.8 clang-format-3.8 clang-tidy-3.8 -y 

# change to the file's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR/../

# check files are correctly formatted

echo "running clang format..."
git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -output-replacements-xml | grep -c "<replacement "

if [ $? -ne 1 ]
then 
    printf "\n\n"
    echo "Clang Format Relacements:"
    git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -output-replacements-xml | grep "<replacement "
    printf "\n"
    
    echo "ERROR!"
    echo 'File not formatted correctly, please execute the command below in flatland repo to see what needs to be changed'
    echo 'git ls-files | grep -E "\.[ch](pp)?$"" | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -i && git diff'
    exit 1;
fi

echo "ci_prebuild.sh completed."