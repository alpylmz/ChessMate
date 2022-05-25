#!/bin/bash

gnome-terminal -x roslaunch franka_example_controllers one_to_rule_them_all.launch 

sleep 5

gnome-terminal -x rosrun franka_gripper franka_gripper_service 10.0.0.2 0 0 
