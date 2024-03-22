#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

TASK_VARIANT="$1"

# start tmuxinator
tmuxinator start -p ./session_$TASK_VARIANT.yml
