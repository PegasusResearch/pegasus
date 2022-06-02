#!/usr/bin/env bash
set -e

# Setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"