#!/bin/bash
# Basic entrypoint for ROS / catkin Docker containers

# Source ROS 
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f /underlay_ws/devel/setup.bash ]
then
  source /underlay_ws/devel/setup.bash
  echo "Sourced underlay workspace"
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/devel/setup.bash ]
then
  source /overlay_ws/devel/setup.bash
  echo "Sourced overlay workspace"
fi

export ROS_PACKAGE_PATH=/overlay_ws/devel/share/ubt_msgs:$ROS_PACKAGE_PATH
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1

# Execute the command passed into this entrypoint
exec "$@"
