#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/cannon/install/setup.bash 

exec "$@"