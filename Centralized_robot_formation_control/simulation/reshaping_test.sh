#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./singularity.sh exec "source ~/.bashrc && rosrun task_02_evaluation rviz.sh && roslaunch task_02_evaluation reshaping_debug.launch"
