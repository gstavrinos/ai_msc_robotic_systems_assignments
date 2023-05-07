#!/bin/bash
curr_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ws_dir=$curr_dir/../..
sudo apt update
sudo apt install -y python3-rosinstall
rosinstall $ws_dir/src/ $curr_dir/ai_msc.rosinstall
rm -rf $ws_dir/src/gazebo_ros2_control
mv $ws_dir/src/ros2_controllers/diff_drive_controller $ws_dir/src/
rm -rf $ws_dir/src/ros2_controllers/*
mv $ws_dir/src/diff_drive_controller $ws_dir/src/ros2_controllers/
