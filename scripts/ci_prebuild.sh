#!/bin/bash -e

function print_er {
    if [ -n "$1" ]; then
        echo -e "\e[31m$1\e[0m" # print error in red
    fi
}


apt-get install clang clang-format clang-tidy -y 

# change to the file's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR/../

# check files are correctly formatted

echo "running clang format..."
CLANG_CHANGES_CNT=$(git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -output-replacements-xml | grep -c "<replacement " || true)

if [ $CLANG_CHANGES_CNT -ne 0 ]
then 
    printf "\n\n"

    print_er "clang-format was operated on following files:"
    git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" 

    printf "\n"

    print_er "the files processed by clang-format need following replacements"
    git ls-files | grep -E '\.[ch](pp)?$' | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -output-replacements-xml

    printf "\n^^^^ See above for clang format output, each <replacements> corresponds to one file ^^^^\n"    

    print_er "Clang Format Error!"
    echo 'File not formatted correctly, please execute the command below in flatland repo to see what needs to be changed'
    echo 'git ls-files | grep -E "\.[ch](pp)?$" | grep -v "thirdparty/" |  xargs clang-format-3.8 --style=file -i && git diff --exit-code'
    exit 1;
fi

echo "ci_prebuild.sh completed."
