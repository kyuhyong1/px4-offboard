#!/usr/bin/env python3

__author__ = "Kyuhyong You"
__contact__ = "kyuhyong.you@nearthlab.com"

import sys
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import String as StringMsg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class VelocityCommand():
    v_x = 0.0
    v_y = 0.0
    v_z = 0.0
    rot_z = 0.0

    def reset(self):
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        self.rot_z = 0.0

class JoyControl(Node):

    def printDescription(self):
        msg = """
        This node takes joy input from the joy_node and publishes them
        as Twist messages. 

        Left Stick input
         - Up/Down: Vx (Fwd/Rev)
         - Left/Right: Vy (Left/Right)

        Right Stick input
         - Up/Down: Vz (Up/Down)
         - Left/Right: Yaw (Left/Right)

        Button press input
         - Button 7: Set Offboard
         - Button A: Set Arm 
         - Button X: Set Take off
         - Button B: Set Landing
         - Button Y: Disarm
        
        """
        print(msg)


    def vels(self, speed, turn):
        return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

    def send_command_msg(self, msg):
        str_msg = StringMsg()
        str_msg.data = msg
        self.pub_command.publish(str_msg)
        print(f"Command sent: {msg}")

    def send_twist_msg(self, vel_com):
        twist = Twist()
        twist.linear.x = vel_com.v_x
        twist.linear.y = vel_com.v_y
        twist.linear.z = vel_com.v_z
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = vel_com.rot_z
        self.pub_twist.publish(twist)
        #print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)

    def cb_joy(self, joymsg):
        self.vel_com.v_x = joymsg.axes[1] * self.vx_max
        self.vel_com.v_y = joymsg.axes[0] * self.vy_max
        self.vel_com.v_z = joymsg.axes[4] * self.vz_max
        self.vel_com.rot_z = joymsg.axes[3] * self.yaw_max
        #print(joymsg.axes)
        if joymsg.buttons[0] == 1:              # A button pressed
            self.send_command_msg("SET_ARM")
            print("Set ARM")
        elif joymsg.buttons[1] == 1:            # B button pressed
            self.send_command_msg("SET_TAKEOFF")
            print("Set TAKE OFF")
        elif joymsg.buttons[2] == 1:            # X button pressed
            self.send_command_msg("SET_LAND")
            print("Set LAND")
        elif joymsg.buttons[7] == 1:            # ]] button pressed
            self.send_command_msg("SET_DISARM")
            print("Set DISARM")
        elif joymsg.buttons[4] == 1:            # L button pressed
            self.send_command_msg("SET_POSITION")
            print("Set POSITION")
        elif joymsg.buttons[5] == 1:            # R button pressed
            self.send_command_msg("SET_ATTITUDE")
            print("Set ATTITUDE")

    def __init__(self):
        super().__init__('minimal_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        #Create publishers
        self.pub_twist = self.create_publisher(Twist, '/twist_joy', qos_profile)

        self.pub_command = self.create_publisher(StringMsg, '/teleop_command', qos_profile)

        #Create subscribers
        self.sub_joy = self.create_subscription(Joy, '/joy', self.cb_joy, 10)

        self.vx_max = 1.0
        self.vy_max = 1.0
        self.vz_max = 1.0
        self.yaw_max = 1.0

        self.vel_com = VelocityCommand ()

        self.status = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.printDescription()

    def timer_callback(self):
        self.send_twist_msg(self.vel_com)


def main(args=None):
    rclpy.init(args=args)
    joy_control = JoyControl()

    rclpy.spin(joy_control)

    joy_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()