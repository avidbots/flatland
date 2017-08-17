#!/usr/bin/env python2.7

import sys
import os
import yaml
import re

def match_ignores(line, ignores):
    for ignore in ignores:
        ignore_pattern = re.compile(ignore)
        if ignore_pattern.match(line):
            return True
    return False

def main():
  
    directory = os.path.dirname(os.path.realpath(__file__))
    ignore_file = open(os.path.join(directory, "clang_tidy_ignore.yaml"), "r")
    ignores = yaml.load(ignore_file)

    clang_tidy_lines = sys.stdin.readlines()
    print("".join(clang_tidy_lines));

    caught_problems = []

    for line in clang_tidy_lines:
        line = line.strip()

        line_pattern = re.compile("/.*:\d+:\d+: (warning|error):.*\[.*\]")
        if not line_pattern.match(line):
            continue

        if match_ignores(line, ignores):
            continue

        caught_problems.append(line)

    print("\n\nIgnore List:")
    for ignore in ignores:
        print(ignore)


    if (len(caught_problems) > 0):

        print("\nCaught %d problem(s) not in the ignore list:" % len(caught_problems))
        for problem in caught_problems:
            print(problem)

        print("\nSee clang tidy output for more information")

        exit(1)
    else:
        print("\nNo problems caught!\n\n\n")
        exit(0)

if __name__ == "__main__":
    main()
