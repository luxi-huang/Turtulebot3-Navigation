#!/bin/bash
file1="/home/luxi/winter_2020/ros/homework/src/main-assignment-luxi-huang/rigid2d/test1_output.txt"
file2="/home/luxi/winter_2020/ros/homework/src/main-assignment-luxi-huang/rigid2d/test1_answer.txt"

g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp
./rigid2d_test< "test1_input.txt" > "test1_output.txt"

if cmp -b "$file1" "$file2"; then
    printf 'success'
else
    printf 'failed'
fi
