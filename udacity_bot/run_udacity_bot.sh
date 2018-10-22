#!/bin/bash
gnome-terminal --working-directory="~/catkin_ws" -e "roslaunch udacity_bot udacity_world.launch" 
gnome-terminal --working-directory="~/catkin_ws" -e "roslaunch udacity_bot amcl.launch" 
gnome-terminal --working-directory="~/catkin_ws" -e "roslaunch udacity_bot navigation_goal"