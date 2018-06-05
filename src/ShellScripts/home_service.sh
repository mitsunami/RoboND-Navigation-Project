#!/bin/sh

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"$HOME/catkin_ws/src/World/MyWorld.world\"" &
sleep 5
xterm  -e  " roslaunch turtlebot_navigation amcl_demo.launch map_file:=\"$HOME/catkin_ws/src/World/MyWorld.yaml\"" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects" &
sleep 5
xterm  -e  " rosrun add_markers add_markers" 

