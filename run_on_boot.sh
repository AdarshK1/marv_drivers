#!/bin/bash
sleep 3
source /home/marv/marv_ros_ws/.bashrc
source /home/marv/marv_ros_ws/environment.bash

roscore &

roslaunch /home/marv/marv_ros_ws/src/marv_drivers/launch/joy.launch &

source /home/marv/marv_ros_ws/.bashrc
source /home/marv/marv_ros_ws/environment.bash
sleep 2
nohup /usr/bin/python3 /home/marv/marv_ros_ws/src/marv_drivers/src/joystick_to_pwm.py
