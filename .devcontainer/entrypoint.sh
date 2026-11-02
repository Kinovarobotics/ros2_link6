#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
# set -e

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f /base_ws/install/setup.bash ]
then
  source /base_ws/install/setup.bash
  echo "Sourced base workspace"
fi

# Build the overlay workspace if it is not built yet
if [ ! -f /overlay_ws/install/setup.bash ]
then
  echo "Overlay workspace not built, running colcon build..."
  (cd /overlay_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release)
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  source /overlay_ws/install/setup.bash
  echo "Sourced overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"