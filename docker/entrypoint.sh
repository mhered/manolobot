#!/bin/bash

# Enable error signals 
set -e

# Source ROS installation
source /opt/ros/humble/setup.bash

# Prints whatever is passed as arguments
echo "Provided arguments: $@"

# Execute whatever is passed as arguments
exec $@