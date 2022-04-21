#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun pointgrey_camera_driver list_cameras &
sleep 2
roslaunch pointgrey_camera_driver bumblebee.launch &
sleep 2
rosparam set /camera/camera_nodelet/frame_rate 30 &
roslaunch pointgrey_camera_driver camera.launch &
wait
