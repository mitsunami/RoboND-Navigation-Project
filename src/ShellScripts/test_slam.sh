#!/bin/sh

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"${HOME}/catkin_ws/src/World/MyWorld.world\"" &
sleep 5
#xterm  -e  " roslaunch gmapping slam_gmapping_pr2.launch" &
#sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" &

