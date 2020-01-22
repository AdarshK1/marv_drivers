#!/bin/bash
source /home/marv/.bashrc
source /home/marv/marv_ros_ws/environment.bash

#roscore &

roslaunch /home/marv/marv_ros_ws/src/marv_drivers/launch/marv.launch &

source /home/marv/.bashrc
source /home/marv/marv_ros_ws/environment.bash
/usr/bin/python3 /home/marv/marv_ros_ws/src/marv_drivers/src/joystick_to_pwm.py
