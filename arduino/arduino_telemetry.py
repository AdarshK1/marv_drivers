#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from time import sleep, time
import serial

ser = serial.Serial('/dev/ttyACM0', 115200) # Establish the connection on a specific port

pub_compass = rospy.Publisher("/arduino_telemtry/compass_orientation", PoseStamped, queue_size=1)
pub_accel = rospy.Publisher("/arduino_telemtry/compass_imu", Imu, queue_size=1)

rospy.init_node("compass_pub")

while not rospy.is_shutdown():

     telem_array = str(ser.readline()).split(",")[1:-1] 

     if len(telem_array) != 6:
         continue

     telem_array[0] = telem_array[0].split(":")[1].strip()
     
     pose_msg = PoseStamped()
     pose_msg.header.frame_id = "earth"
     pose_msg.header.stamp = rospy.Time.now()
     pose_msg.pose.orientation.x = float(telem_array[2])
     pose_msg.pose.orientation.y = float(telem_array[1])
     pose_msg.pose.orientation.z = float(telem_array[0])
     pub_compass.publish(pose_msg)
     print(telem_array)

     sleep(.1) 
