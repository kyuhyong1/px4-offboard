#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__maintainer__ = "Kyuhyong You"
__contact__ = "kyuhyong.you@nearthlab.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleRatesSetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import String as StringMsg


class OffboardCommander(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        #Create subscriptions
        self.sub_status = self.create_subscription(
            VehicleStatus,'/fmu/out/vehicle_status', self.cb_vehicle_status, qos_profile)
        
        self.sub_twist_position = self.create_subscription(
            Twist, '/offboard_position_cmd', self.cb_twist_position, qos_profile)
        
        self.sub_twist_velocity = self.create_subscription(
            Twist, '/offboard_velocity_cmd', self.cb_twist_velocity, qos_profile)
        
        self.sub_attitude = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.cb_attitude, qos_profile)
        self.my_bool_sub = self.create_subscription(
            StringMsg, '/command_message', self.cb_command_message, qos_profile)
        self.sub_vehicle_local_position = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb_vehicle_local_position, qos_profile)

        #Create publishers
        # Some message names changed???
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        #self.pub_velocity = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.pub_velocity = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_vehicle_command = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # Initialize variables
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_MAX
        
        # Command value
        self.velocity = Vector3()
        self.position = Vector3()
        self.heading = 0.0  #yaw value we send as command
        self.yaw_rad_s = 0.0    # Rot_z speed

        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = True
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_control_mode = OffboardControlMode()
        self.offboard_control_mode.position = True
        self.offboard_control_mode.velocity = False
        self.offboard_control_mode.acceleration = False
        self.offboard_control_mode.attitude = False
        self.offboard_control_mode.body_rate = False
        #self.offboard_control_mode.thrust_body = False
        self.takeoff_height = -5.0
        self.is_in_velocity_control = False

        #states with corresponding callback functions that run once when state switches
        # self.states = {
        #     "IDLE": self.state_init,
        #     "ARMING": self.state_arming,
        #     "TAKEOFF": self.state_takeoff,
        #     "LOITER": self.state_loiter,
        #     "OFFBOARD": self.state_offboard
        # }
        # self.current_state = "IDLE"

    def cb_vehicle_local_position(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def cb_command_message(self, msg):
        self.com_message = msg.data
        if msg.data == "SET_ARM":
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("ARM")
        elif msg.data == "SET_DISARM":
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            self.get_logger().info("DISARM")
        elif msg.data == "SET_TAKEOFF":
            self.take_off()
            self.get_logger().info("TAKEOFF")
        elif msg.data == "SET_LAND":
            self.land()
            self.get_logger().info("LAND")
        elif msg.data == "SET_OFFBOARD":
            self.offboardMode = True;
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info("SET OFFBOARD")

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

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            #self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
            pose = Vector3()
            pose.x = 0.0
            pose.y = 0.0
            pose.z = self.takeoff_height
            yaw = 1.57
            self.send_position_setpoint(pose, yaw)
            self.get_logger().info("Takeoff command send")

    #publishes command to /fmu/in/vehicle_command
    def send_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command  # command ID
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1       # system which should execute the command
        msg.target_component =  1   # component which should execute the command, 0 for all components
        msg.source_system = 1       # system sending the command
        msg.source_component = 1    # component sending the command
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # time in microseconds
        self.pub_vehicle_command.publish(msg)
    
    def send_position_setpoint(self, pose: Vector3(), yaw: float):
        """Publish the trajectory setpoint."""
        self.offboard_control_mode.position = True
        self.offboard_control_mode.velocity = False
        #self.offboard_control_mode.thrust_body = False
        msg = TrajectorySetpoint()
        msg.position = [pose.x, pose.y, pose.z]
        msg.yaw = yaw #1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.get_logger().info(f"Publishing position setpoints {[pose.x, pose.y, pose.z]}")
        self.pub_trajectory.publish(msg)

    def send_velocity_setpoint(self, vel: Vector3(), yaw: float):
        """Publish the trajectory setpoint."""
        self.offboard_control_mode.position = False
        self.offboard_control_mode.attitude = False
        self.offboard_control_mode.body_rate = True
        #msg = VehicleAttitudeSetpoint()
        msg = VehicleRatesSetpoint()
        #msg.roll_body = 0.0
        msg.roll = 0.0
        #msg.pitch_body = 0.0
        msg.pitch = 0.0
        #msg.yaw_sp_move_rate = yaw
        msg.yaw = yaw
        msg.thrust_body[0] = vel.x
        msg.thrust_body[1] = vel.y
        msg.thrust_body[2] = vel.z
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.get_logger().info(f"Publishing atitiude setpoints {[vel.x, vel.y, vel.z]}")
        self.pub_velocity.publish(msg)

    #receives and sets vehicle status values 
    def cb_vehicle_status(self, vehicle_status):
        #self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        #self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        #self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")
        self.vehicle_status = vehicle_status
        self.nav_state = vehicle_status.nav_state
        self.arm_state = vehicle_status.arming_state
        self.failsafe = vehicle_status.failsafe
        self.flightCheck = vehicle_status.pre_flight_checks_pass
        #if msg.pre_flight_checks_pass == True:
        #    self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

    #receives Twist commands from Teleop and converts NED -> FLU
    def cb_twist_position(self, twist):
        #implements NED -> FLU Transformation
        self.is_in_velocity_control = True
        self.position.x = twist.linear.y
        self.position.y = -twist.linear.x
        self.position.z = self.takeoff_height - twist.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.heading = twist.angular.z
        #self.send_velocity_setpoint(self.velocity, self.yaw)
        self.send_position_setpoint(self.position, self.heading)
        self.get_logger().info(f"Position={self.position.x, self.position.y}")

    def cb_twist_velocity(self, twist):
        #implements NED -> FLU Transformation
        self.is_in_velocity_control = True
        self.velocity.x = twist.linear.y
        self.velocity.y = -twist.linear.x
        self.velocity.z = -twist.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw_rad_s = twist.angular.z
        #self.send_velocity_setpoint(self.velocity, self.yaw)
        self.send_velocity_setpoint(self.velocity, self.yaw_rad_s)
        self.get_logger().info(f"Velocity={self.velocity.x, self.velocity.y}")

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def cb_attitude(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    #Command loop called every 0.1sec
    def cmdloop_callback(self):
        self.send_heartbeat_signal()


def main(args=None):
    rclpy.init(args=args)

    offboard_commander = OffboardCommander()

    rclpy.spin(offboard_commander)

    offboard_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()