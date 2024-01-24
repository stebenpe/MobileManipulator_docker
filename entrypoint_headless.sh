#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/moma/MobileManipulator_ros2/ros2_ws/install/setup.bash

cd /home/moma/MobileManipulator_website/vue-webpanel && npm run serve &

ros2 launch omron_moma server_headless.launch.py robot_ip:=192.168.44.14 &

ros2 launch omron_moma visualization_headless.launch.py &

ros2 run omron_moma moma_coffee &