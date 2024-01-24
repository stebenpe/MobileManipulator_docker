#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
cd /home/moma

echo "Provided arguments: $@"

exec $@