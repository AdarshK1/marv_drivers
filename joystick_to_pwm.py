# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL

from adafruit_servokit import ServoKit
import board
import busio
import time
import rospy

from sensor_msgs.msg import Joy



#from approxeng.input.selectbinder import ControllerResource


# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...


def joy_callback(joy_msg):
    global kit
    ax_val_left = joy_msg.axes[1]
    ax_val_right = joy_msg.axes[4]
    kit.servo[0].angle = ax_val_left * 90 + 90
    
    print("Right: {}, Left: {}".format(ax_val_right, ax_val_left))



def joy_to_pwm():
    rospy.init_node("joy_to_pwm", anonymous=True)
    sub = rospy.Subscriber("/joy", Joy, joy_callback, queue_size=1)
    
    rospy.spin()


    

    
if __name__ == "__main__":

    print("Initializing Servos")
    i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
    print("Initializing ServoKit")
    kit = ServoKit(channels=16, i2c=i2c_bus0)
    # kit[0] is the bottom servo
    # kit[1] is the top servo
    print("Done initializing")
    joy_to_pwm()
   # kit.servo[0].angle = 137


    #sweep = range(0,180)
    #for degree in sweep :
    #    kit.servo[0].angle=degree
    #    # kit.servo[1].angle=degree
    #    time.sleep(0.02)

