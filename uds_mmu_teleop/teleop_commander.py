#!/usr/bin/env python

__author__ = "Kyuhyong You"
__contact__ = "kyuhyong.you@nearthlab.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from uds_mmu_msgs.msg import SetVehicleId, StickCommand
from uds_mmu_msgs.msg import VehicleCommand, VehicleCommandAck
from uds_mmu_msgs.msg import MissionCommand, MissionType, VehicleStatus
#from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleRatesSetpoint, VehicleAttitudeSetpoint
#from px4_msgs.msg import VehicleStatus
#from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
#from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import String as StringMsg


class TeleopCommander(Node):

    def __init__(self):
        super().__init__('teleop_commander')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        #Create subscriptions from mmu_gcs_interface
        self.sub_status = self.create_subscription(
            VehicleStatus,'/vehicle_status', self.cb_vehicle_status, qos_profile)
        
        self.sub_command_ack = self.create_subscription(
            VehicleCommandAck, 'gcs/vehicle_command_ack/in', self.cb_vehicle_command_ack, qos_profile)
        
        # Subscribe from teleop interfaces
        self.my_bool_sub = self.create_subscription(
            StringMsg, '/teleop_command', self.cb_teleop_command, qos_profile)
        
        self.sub_twist_key = self.create_subscription(
            Twist, '/keyboard_cmd', self.cb_twist_key, qos_profile)
        
        self.sub_twist_joy = self.create_subscription(
            Twist, '/twist_joy', self.cb_twist_joy, qos_profile)
        
        #self.sub_vehicle_local_position = self.create_subscription(
        #    VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb_vehicle_local_position, qos_profile)

        #Create publishers
        self.pub_vehicle_command = self.create_publisher(VehicleCommand, "/gcs/vehicle_command/out", 10)
        self.pub_stick = self.create_publisher(StickCommand, "/gcs/stick_command/out", 10)
        self.pub_set_vehicle_id = self.create_publisher(SetVehicleId, '/gcs/set_vehicle_id/out', qos_profile)
        self.pub_mission_command = self.create_publisher(MissionCommand, '/gcs/mission_command/out', qos_profile)
        

        # Initialize variables
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        #arm_timer_period = .1 # seconds
        #self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        
        # Command value
        self.stick_cmd_joy = StickCommand()
        self.stick_cmd_key = StickCommand()

        self.vehicle_cmd_msg = VehicleCommand()
        self.vehicle_cmd_msg.vehicle_command_id = 0
        self.vehicle_cmd_msg.vehicle_command_type = 0
        self.vehicle_cmd_msg.flight_mode = 0

        self.command_ack_msg = VehicleCommandAck()

        self.vehicle_status = VehicleStatus()
        self.timer_count = 0


    def cb_vehicle_local_position(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def cb_teleop_command(self, msg):
        self.com_message = msg.data
        valid = False
        if msg.data == "SET_ARM":
            valid = True
            self.vehicle_cmd_msg.vehicle_command_type = 10
            self.get_logger().info("ARM")
        elif msg.data == "SET_DISARM":
            valid = True
            self.vehicle_cmd_msg.vehicle_command_type = 11
            self.get_logger().info("DISARM")
        elif msg.data == "SET_TAKEOFF":
            valid = True
            self.vehicle_cmd_msg.vehicle_command_type = 1
            self.vehicle_cmd_msg.flight_mode = 11
            self.get_logger().info("TAKEOFF")
        elif msg.data == "SET_LAND":
            valid = True
            self.vehicle_cmd_msg.vehicle_command_type = 1
            self.vehicle_cmd_msg.flight_mode = 12
            self.get_logger().info("LAND")
        elif msg.data == "SET_POSITION":
            valid = True
            self.vehicle_cmd_msg.vehicle_command_type = 1
            self.vehicle_cmd_msg.flight_mode = 1
            self.get_logger().info("SET MODE: MANUAL POSITION")
        elif msg.data == "SET_ATTITUDE":
            valid = True
            self.vehicle_cmd_msg.vehicle_command_type = 1
            self.vehicle_cmd_msg.flight_mode = 2
            self.get_logger().info("SET MODE: MANUAL ATTITUDE")
        if valid == True:
            self.send_vehicle_command(self.vehicle_cmd_msg)

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        if msg.data == True:
            self.get_logger().info("ARM")
        else:
            self.get_logger().info("DISARM")

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        
        #self.get_logger().info(self.current_state)
        self.myCnt += 1

    def send_heartbeat_signal(self):
        """Publish the offboard control mode."""
        self.offboard_control_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(self.offboard_control_mode)

    def land(self):
        """Switch to land mode."""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    #publishes command to /fmu/in/vehicle_command
    def send_vehicle_command(self, cmd_msg: VehicleCommand()):
        """Publish a vehicle command."""
        #cmd_msg.stamp = int(self.get_clock().now().nanoseconds / 1000) # time in microseconds
        cmd_msg.stamp = self.get_clock().now().to_msg()
        self.pub_vehicle_command.publish(cmd_msg)

    def cb_vehicle_command_ack(self, ack_msg):
        self.command_ack_msg = ack_msg

    def send_stick_command(self, stick_msg: StickCommand()):
        #stick_msg.stamp = int(self.get_clock().now().nanoseconds / 1000)
        stick_msg.stamp = self.get_clock().now().to_msg()
        self.pub_stick.publish(stick_msg)

    #receives and sets vehicle status values 
    def cb_vehicle_status(self, vehicle_status):
        if not self.vehicle_status.armed and vehicle_status.armed == True:
            self.get_logger().info("ARMED")
        if not self.vehicle_status.pre_flight_checks_passed and vehicle_status.pre_flight_checks_passed == True:
            self.get_logger().info("PRE FLIGHT CHECK PASSED")
        

        self.vehicle_status = vehicle_status
        
        #self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        #self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        #self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

    #receives Twist commands from Teleop and converts NED -> FLU
    def cb_twist_key(self, twist):
        #implements NED -> FLU Transformation
        self.stick_cmd_key.roll = twist.linear.y
        self.stick_cmd_key.pitch = twist.linear.x
        self.stick_cmd_key.throttle = twist.linear.z
        self.stick_cmd_key.yaw = twist.angular.z
        self.send_stick_command(self.stick_cmd_key)
        #self.get_logger().info(f"Keyboard input={self.stick_cmd_key.roll, self.stick_cmd_key.pitch}")

    def cb_twist_joy(self, twist):
        #implements twist to stick conversion
        self.stick_cmd_joy.roll = twist.linear.y
        self.stick_cmd_joy.pitch = twist.linear.x
        self.stick_cmd_joy.throttle = twist.linear.z
        self.stick_cmd_joy.yaw = twist.angular.z
        self.send_stick_command(self.stick_cmd_joy)
        #self.get_logger().info(f"Joy input={self.stick_cmd_joy.roll, self.stick_cmd_joy.pitch}")

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def cb_attitude(self, msg):
        orientation_q = msg.q
        #trueYaw is the drones current yaw value
        #self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
        #                          1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    #Command loop called every 0.1sec
    def cmdloop_callback(self):
        self.timer_count += 1
        #self.send_heartbeat_signal()


def main(args=None):
    rclpy.init(args=args)

    teleop_commander = TeleopCommander()

    rclpy.spin(teleop_commander)

    teleop_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()