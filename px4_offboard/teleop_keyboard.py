#!/usr/bin/env python3

__author__ = "Kyuhyong You"
__contact__ = "kyuhyong.you@nearthlab.com"

import sys
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import String as StringMsg
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

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

class KeyboardControl(Node):

    def printDescription(self):
        msg = """
        This node takes keypresses from the keyboard and publishes them
        as Twist messages. 
        Using the arrow keys and 'WAXD' you have Mode 2 RC controls.
        W: Up
        X: Down
        A: Yaw Left
        D: Yaw Right
        S: Reset command
        Up Arrow: Pitch Forward
        Down Arrow: Pitch Backward
        Left Arrow: Roll Left
        Right Arrow: Roll Right

        Press   'O'   to set to offboard mode
        Press   'T'   to take off the drone
        Press 'SPACE' to arm the drone
        Press 'BACKSPACE' to disarm the drone
        """
        print(msg)

    moveBindings = {
        'w': (0, 0, 1, 0), #Z+
        'x': (0, 0, -1, 0),#Z-
        'a': (0, 0, 0, -1), #Yaw+
        'd': (0, 0, 0, 1),#Yaw-
        '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
        '\x1b[B' : (0, -1, 0, 0), #Down Arrow
        '\x1b[C' : (-1, 0, 0, 0), #Right Arrow
        '\x1b[D' : (1, 0, 0, 0),  #Left Arrow
    }
    speedBindings = {
        # 'q': (1.1, 1.1),
        # 'z': (.9, .9),
        # 'w': (1.1, 1),
        # 'x': (.9, 1),
        # 'e': (1, 1.1),
        # 'c': (1, .9),
    }

    def getKey(self, settings):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            #print(key+" ")
            if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
                additional_chars = sys.stdin.read(2)  # read the next two characters
                key += additional_chars  # append these characters to the key
                #print(additional_chars)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def saveTerminalSettings(self):
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)


    def restoreTerminalSettings(self, old_settings):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


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
        print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)

    def __init__(self):
        super().__init__('minimal_publisher')
        self.settings = self.saveTerminalSettings()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        #Create publishers
        self.pub_twist = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)

        self.pub_command = self.create_publisher(StringMsg, '/command_message', qos_profile)

        self.v_step = 0.5
        self.rot_step = .2
        self.vel_com = VelocityCommand ()

        self.status = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.printDescription()

    def timer_callback(self):
        
        # print(vels(speed, turn))
        key = self.getKey(self.settings)
        vx = 0.0
        vy = 0.0
        vz = 0.0
        rz = 0.0
        if key in self.moveBindings.keys():
            vx = self.moveBindings[key][0]
            vy = self.moveBindings[key][1]
            vz = self.moveBindings[key][2]
            rz = self.moveBindings[key][3]
            self.vel_com.v_x = (vx * self.v_step) + self.vel_com.v_x
            self.vel_com.v_y = (vy * self.v_step) + self.vel_com.v_y
            self.vel_com.v_z = (vz * self.v_step) + self.vel_com.v_z
            self.vel_com.rot_z = (rz * self.rot_step) + self.vel_com.rot_z
            self.send_twist_msg(self.vel_com)    
        
        if key == 'o': 
            self.send_command_msg("SET_OFFBOARD")
        elif key == 's':
            self.vel_com.reset()
            self.send_twist_msg(self.vel_com)    
        elif key == 't':
            self.send_command_msg("SET_TAKEOFF")
        elif key == ' ':  # ASCII value for space
            self.send_command_msg("SET_ARM")
        elif key == 'l':
            self.send_command_msg("SET_LAND")
        elif key == '\x7f':
            self.send_command_msg("SET_DISARM")
        
        


def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()

    rclpy.spin(keyboard_control)

    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()