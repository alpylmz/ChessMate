#!/bin/bash

gnome-terminal -x roslaunch franka_example_controllers pick_and_place_test.launch 

sleep 5

gnome-terminal -x rosrun franka_gripper franka_gripper_service 10.0.0.2 0 0 
