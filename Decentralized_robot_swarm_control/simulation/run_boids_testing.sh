#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH
  
if [ "$#" -gt 0 ]; then
  TASK_VARIANT="$1"
else
  TASK_VARIANT="testing"
fi

./singularity.sh exec "source ~/.bashrc && cd ~/task_03_swarm/simulation/tmux/boids && ./start_testing.sh ${TASK_VARIANT}"
