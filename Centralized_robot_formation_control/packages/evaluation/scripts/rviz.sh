#!/bin/bash

source ~/.bashrc

RVIZ_RUNNING=$( rosnode list | grep debugrviz | wc -l )

if [[ "$RVIZ_RUNNING" == "0" ]]; then
  roslaunch task_02_evaluation reshaping_debug_rviz.launch 2>&1 >> /dev/null &
  waitForRos
fi
