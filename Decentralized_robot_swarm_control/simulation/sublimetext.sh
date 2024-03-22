#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./singularity.sh exec "source ~/.bashrc && cd ~/task_03_swarm/simulation/user_ros_workspace/src/swarm && subl src/boids.cpp src/swarm.cpp include/student_headers/swarm.h config/user_params_boids.yaml config/user_params_hunt_the_robot.yaml"
