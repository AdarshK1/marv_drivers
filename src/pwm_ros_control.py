#!/usr/bin/python3

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
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, PoseStamped

# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...


class JoyToPWM:
    def __init__(self):
        
        self.right_top_thruster_idx = 0
        self.left_top_thruster_idx = 3
        self.right_bot_thruster_idx = 4
        self.left_bot_thruster_idx = 7

        self.front_right_servo_idx = 8
        self.front_left_servo_idx = 11
        self.back_right_servo_idx = 12
        self.back_left_servo_idx = 15

        rospy.logwarn("Initializing Servos")
        self.i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
        rospy.logwarn("Initializing ServoKit")
        self.kit = ServoKit(channels=16, i2c=self.i2c_bus0)

        self.pub = rospy.Publisher("/sent_command", TwistStamped, queue_size=1)

        # self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        self.servo_cmd_sub = rospy.Subscriber("/servo_cmd/position", PoseStamped,
                                              self.servo_cmd_cb, queue_size=1)
        self.thruster_cmd_sub = rospy.Subscriber("/thruster_command/cmd_vel", TwistStamped,
                                                 self.thruster_cmd_cb, queue_size=1)

        self.state_sub = rospy.Subscriber("/mission_manager/state", Float32MultiArray, self.state_cb, queue_size=1)

        self.initialize_thrusters()

        self.lp_frac = 0.05
        self.last_right = 0.55
        self.last_left = 0.55

        self.current_state = 0
        self.current_orientation = 0

    def initialize_thrusters(self):
        self.kit.servo[self.right_top_thruster_idx].fraction = 0.6
        self.kit.servo[self.right_bot_thruster_idx].fraction = 0.55
        self.kit.servo[self.left_bot_thruster_idx].fraction = 0.55
        self.kit.servo[self.left_top_thruster_idx].fraction = 0.55

        time.sleep(1)
        self.kit.servo[self.right_top_thruster_idx].fraction = 0.55
        self.kit.servo[self.right_bot_thruster_idx].fraction = 0.55
        self.kit.servo[self.left_bot_thruster_idx].fraction = 0.55
        self.kit.servo[self.left_top_thruster_idx].fraction = 0.55

        time.sleep(1)
        self.kit.servo[self.right_top_thruster_idx].fraction = 0.55
        self.kit.servo[self.right_bot_thruster_idx].fraction = 0.55
        self.kit.servo[self.left_bot_thruster_idx].fraction = 0.55
        self.kit.servo[self.left_top_thruster_idx].fraction = 0.55

        rospy.logwarn("Done initializing")

    def state_cb(self, state_msg):
        self.current_state = state_msg.data[0]
        self.current_orientation = state_msg.data[1]

    '''
    callback for the thruster's command
    '''
    def thruster_cmd_cb(self, twist_msg):

        throttle_value = twist_msg.twist.linear.x
        differential_value = twist_msg.twist.linear.y

        right_nominal = throttle_value * (1 + differential_value)
        left_nominal = throttle_value * (1 - differential_value)

        clipped_value_right = max(min(right_nominal / 2 + 0.55, 0.9), 0.2)
        clipped_value_left = max(min(left_nominal / 2 + 0.55, 0.9), 0.2)

        lp_right = self.last_right + self.lp_frac * (clipped_value_right - self.last_right)
        lp_left = self.last_left + self.lp_frac * (clipped_value_left - self.last_left)

        self.set_thruster_cmd(lp_left, lp_right)

        self.last_right = lp_right
        self.last_left = lp_left

    '''
    helper method that sends commands informed by orientation (top vs bot thrusters)
    '''
    def set_thruster_cmd(self, left_val, right_val):
        if self.current_orientation == -1:
            self.kit.servo[self.right_bot_thruster_idx].fraction = right_val
            self.kit.servo[self.left_bot_thruster_idx].fraction = left_val
        else:
            self.kit.servo[self.right_top_thruster_idx].fraction = right_val
            self.kit.servo[self.left_top_thruster_idx].fraction = left_val

    def servo_cmd_cb(self, pose_msg):
        if pose_msg.header.frame_id == "manual":
            self.kit.servo[self.front_right_servo_idx].angle = pose_msg.pose.orientation.x
            self.kit.servo[self.front_left_servo_idx].angle = pose_msg.pose.orientation.y
            self.kit.servo[self.back_right_servo_idx].angle = pose_msg.pose.orientation.z
            self.kit.servo[self.back_left_servo_idx].angle = pose_msg.pose.orientation.w

        elif pose_msg.header.frame_id == "set_positions":
            if pose_msg.pose.position.z == 1:
                self.kit.servo[self.front_right_servo_idx].angle = 160
                self.kit.servo[self.front_left_servo_idx].angle = 160
                self.kit.servo[self.back_right_servo_idx].angle = 160
                self.kit.servo[self.back_left_servo_idx].angle = 160
            elif pose_msg.pose.position.z == -1:
                self.kit.servo[self.front_right_servo_idx].angle = 20
                self.kit.servo[self.front_left_servo_idx].angle = 20
                self.kit.servo[self.back_right_servo_idx].angle = 20
                self.kit.servo[self.back_left_servo_idx].angle = 20

    # TODO this needs to be re-written to handle the updated structure
    def joy_callback(self, joy_msg):
        ax_val_left = joy_msg.axes[1]
        ax_val_right = -joy_msg.axes[4]

        clipped_value_right = max(min(ax_val_right / 2 + 0.55, 0.9), 0.2)
        clipped_value_left = max(min(ax_val_left / 2 + 0.55, 0.9), 0.2)

        if joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 1:
            lp_right = self.last_right + self.lp_frac * (clipped_value_right - self.last_right)
            lp_left = self.last_left + self.lp_frac * (clipped_value_left - self.last_left)

            self.last_right = lp_right
            self.last_left = lp_left

            print("Dead Man Switches activated")

            self.kit.servo[0].fraction = self.last_right
            self.kit.servo[4].fraction = self.last_left  # 0.6 - (self.last_right - 0.6)

            # if joy_msg.axes[5] < -0.9:
            #    self.kit.servo[4].fraction = self.last_left
            # else:
            #    self.kit.servo[4].fraction = self.last_right

            print("Clipped --> Right: {}, Left: {}".format(clipped_value_right, clipped_value_left))
            print("Low Passed --> Right: {}, Left: {}".format(lp_right, lp_left))

            twist = TwistStamped()

            twist.header.stamp = rospy.Time.now()

            twist.twist.linear.x = lp_right
            twist.twist.linear.y = lp_left

            self.pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("joy_to_pwm", anonymous=True, disable_signals=False)
    jp = JoyToPWM()
    rospy.spin()
