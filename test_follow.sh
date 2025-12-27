#!/bin/bash
# 一键启动跟随系统

# 杀死所有相关进程
pkill -f roscore
pkill -f turtlesim
pkill -f leader.py
pkill -f follower.py

# 启动 roscore
gnome-terminal -- roscore

sleep 2

# 启动 turtlesim
gnome-terminal -- rosrun turtlesim turtlesim_node

sleep 1

# 启动 leader
gnome-terminal -- rosrun turtle_follower leader.py

sleep 1

# 启动 follower
gnome-terminal -- rosrun turtle_follower follower.py
