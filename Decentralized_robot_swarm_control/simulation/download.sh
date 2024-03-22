#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

PATH_DIR_DEPT_CYBERNETICS="/opt/singularity/robolab/mrs_uav_system"
PATH_SIF_DEPT_CYBERNETICS="/opt/singularity/robolab/mrs_uav_system.sif"

cd images

# Dept. of Cybernetics: link extracted folder
if [ -d "$PATH_DIR_DEPT_CYBERNETICS" ]; then
  echo "Lab computer: linking image folder ${PATH_DIR_DEPT_CYBERNETICS}"
	ln -sf "$PATH_DIR_DEPT_CYBERNETICS" .

# Dept. of Cybernetics: link image
elif [ -f "$PATH_SIF_DEPT_CYBERNETICS" ]; then
  echo "Lab computer: linking .sif image ${PATH_SIF_DEPT_CYBERNETICS}"
	ln -sf "$PATH_SIF_DEPT_CYBERNETICS" .

# Download image
else
  echo "Downloading .sif image from the servers"
	wget -c https://nasmrs.felk.cvut.cz/index.php/s/CQz8Z733OK5hsjO/download -O mrs_uav_system.sif --no-check-certificate
fi
