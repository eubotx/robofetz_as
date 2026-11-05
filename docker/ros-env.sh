#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/local_setup.bash ]; then
    source /ws/install/local_setup.bash
fi
exec "$@"
