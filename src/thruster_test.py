# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL

from adafruit_servokit import ServoKit
import board
import busio
import time



#from approxeng.input.selectbinder import ControllerResource


# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...


def joy_callback(joy_msg):
    global kit
    ax_val_left = joy_msg.axes[1]
    ax_val_right = joy_msg.axes[4]
    kit.servo[0].angle = ax_val_left * 5
    
    print("Right: {}, Left: {}".format(ax_val_right, ax_val_left))



def joy_to_pwm():
    rospy.init_node("joy_to_pwm", anonymous=True)
    sub = rospy.Subscriber("/joy", Joy, joy_callback, queue_size=1)
    
    rospy.spin()   

    
if __name__ == "__main__":

    print("Initializing Servos")
    i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
    #i2c_bus0=(busio.I2C(board.SCL, board.SDA))
    
    print("Initializing ServoKit")
    kit = ServoKit(channels=16, i2c=i2c_bus0)
    #kit.continuous_servo[0].set_pulse_width_range(1100, 1900)
    kit.servo[0].set_pulse_width_range(1100, 1900)
    
    print("Waiting for 1, should initialize")
    kit.servo[0].angle = 90
    time.sleep(5.0)
    #for i in range(10):
        #kit.servo[0].angle = 90    
        #kit.continuous_servo[0].throttle = 0
        #time.sleep(0.2)

    print("Done initializing, setting a command")
    #for i in range(10):
    #    kit.servo[0].angle = 120
   #     #kit.continuous_servo[0].throttle = 0.2
  #      time.sleep(.50)
    kit.servo[0].angle = 145
    time.sleep(4.0)

    kit.servo[0].angle = 90
    time.sleep(1.0)



#    joy_to_pwm()
   # kit.servo[0].angle = 137


    #sweep = range(0,180)
    #for degree in sweep :
    #    kit.servo[0].angle=degree
    #    # kit.servo[1].angle=degree
    #    time.sleep(0.02)

