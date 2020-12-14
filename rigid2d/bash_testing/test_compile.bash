#!/bin/bash
file1="/home/luxi/winter_2020/ros/homework/src/main-assignment-luxi-huang/rigid2d/bash_testing/test1_output.txt"
file2="/home/luxi/winter_2020/ros/homework/src/main-assignment-luxi-huang/rigid2d/bash_testing/test1_answer.txt"

# compile 
g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp

# PLACE test1_input.txt INTO TEST, THEN PLACE OUTPUT OF TEST INTO output.txt
./rigid2d_test< "test1_input.txt" > "test1_output.txt"

cmp -s "$file1" "$file2" && echo "sucess"
cmp -s "$file1" "$file2" || echo "sucess"