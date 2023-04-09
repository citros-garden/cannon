#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/citros_cannon/install/setup.bash 

exec "$@"