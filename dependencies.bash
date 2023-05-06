#!/bin/bash
sudo apt update
sudo apt install -y python3-rosinstall
rosinstall src/ ai_msc.rosinstall
ws_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/../..
rm -rf src/gazebo_ros2_control
mv src/ros2_controllers/diff_drive_controller src/
rm -rf src/ros2_controllers/*
mv src/diff_drive_controller src/ros2_controllers/
