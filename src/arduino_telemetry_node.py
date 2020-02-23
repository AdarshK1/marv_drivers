from time import sleep
import serial
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


class ArduinoTelemtryNode:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200) # Establish the connection on a specific port
        self.sub_hb = rospy.Subscriber("/mission_manager/status", )

    def hb_cb(self):

    def led_cb(self):
        pass

    def parse_serial(self):
        pass

if name == "__main__":
    rospy.init_node("arduino_telemtry_node")


rospy.init_node("arduino_telemtry_node")

while not rospy.is_shutdown():

     print (ser.readline()) # Read the newest output from the Arduino
     
     sleep(.1) # Delay for one tenth of a second
     # if counter == 255:
     # 	counter = 32 
