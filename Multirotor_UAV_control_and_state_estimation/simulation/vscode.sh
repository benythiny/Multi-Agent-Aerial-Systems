#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./singularity.sh exec "source ~/.bashrc && cd ~/task_01_controller/simulation/user_ros_workspace/src/controller && code ./ src/controller.cpp src/lkf.cpp include/student_headers/controller.h config/user_params.yaml"
