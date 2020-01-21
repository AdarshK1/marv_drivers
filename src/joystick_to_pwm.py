# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL

from adafruit_servokit import ServoKit
import board
import busio
import sys
sys.path.append("/home/marv/.local/lib/python3.6/site-packages")
import time
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

#from approxeng.input.selectbinder import ControllerResource


# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...


class JoyToPWM:
    def __init__(self):
        
        print("Initializing Servos")
        self.i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
        #i2c_bus0=(busio.I2C(board.SCL, board.SDA))
    
        print("Initializing ServoKit")
        self.kit = ServoKit(channels=16, i2c=self.i2c_bus0)
        self.kit.servo[0].fraction = 0.6
        self.kit.servo[4].fraction = 0.55
   
        print("Waiting for 5, should initialize")
    
        time.sleep(1)
        self.kit.servo[0].fraction = 0.55
        self.kit.servo[4].fraction = 0.55
    

        time.sleep(1)
        self.kit.servo[0].fraction = 0.55
        self.kit.servo[4].fraction = 0.55
    
        print("Done initializing")


        self.pub = rospy.Publisher("/sent_command", TwistStamped, queue_size=10) 
        self.sub = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=10)


        self.lp_frac = 0.05
        self.last_right = 0.55
        self.last_left = 0.55

    def joy_callback(self, joy_msg):

        ax_val_left = joy_msg.axes[1]
        ax_val_right = -joy_msg.axes[4]

        #ax_val_right = self.last_right + 0.01
        #ax_val_left = self.last_left + 0.01
#        print(ax_val_right, ax_val_left)
        clipped_value_right = max(min(ax_val_right / 2 + 0.55, 0.9), 0.2)
        clipped_value_left = max(min(ax_val_left / 2 + 0.55, 0.9), 0.2)
    

        #self.kit.servo[0].fraction = self.last_right
        #self.kit.servo[4].fraction = self.last_left
        #time.sleep(1)

        if joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 1:
            lp_right = self.last_right + self.lp_frac * (clipped_value_right - self.last_right)
            lp_left =  self.last_left + self.lp_frac * (clipped_value_left - self.last_left)

            self.last_right = lp_right
            self.last_left = lp_left
            
            print("Dead Man Switches activated")
            
            self.kit.servo[0].fraction = self.last_right
            self.kit.servo[4].fraction = self.last_left #0.6 - (self.last_right - 0.6)

            #if joy_msg.axes[5] < -0.9:
            #    self.kit.servo[4].fraction = self.last_left
            #else:
            #    self.kit.servo[4].fraction = self.last_right


            print("Clipped --> Right: {}, Left: {}".format(clipped_value_right, clipped_value_left))
            print("Low Passed --> Right: {}, Left: {}".format(lp_right, lp_left))
    
            twist = TwistStamped()
        
            twist.header.stamp = rospy.Time.now()
    
            twist.twist.linear.x = lp_right
            twist.twist.linear.y = lp_left

            self.pub.publish(twist)


   # def joy_to_pwm():
   #     global pub
   #     rospy.init_node("joy_to_pwm", anonymous=True)
   #     #pub = rospy.Publisher("/sent_command", TwistStamped, queue_size=10) 
   #     sub = rospy.Subscriber("/joy", Joy, joy_callback, queue_size=10)
   #     rospy.spin()   

    
if __name__ == "__main__":
    
    rospy.init_node("joy_to_pwm", anonymous=True, disable_signals=False)
    jp = JoyToPWM()
    rospy.spin()   
    
