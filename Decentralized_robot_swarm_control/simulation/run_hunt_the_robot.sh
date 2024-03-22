#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./singularity.sh exec "source ~/.bashrc && cd ~/task_03_swarm/simulation/tmux/hunt_the_robot && ./start.sh"
