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
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleRatesSetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import String as StringMsg


class OffboardControl(Node):

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
        self.sub_offboard_velocity = self.create_subscription(
            Twist, '/offboard_velocity_cmd', self.cb_offboard_velocity, qos_profile)
        self.sub_attitude = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.cb_attitude, qos_profile)
        self.my_bool_sub = self.create_subscription(
            StringMsg, '/command_message', self.cb_command_message, qos_profile)
        self.sub_vehicle_local_position = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb_vehicle_local_position, qos_profile)

        #Create publishers
        # Some message names changed???
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
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
        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = True
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

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
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(msg)

    def land(self):
        """Switch to land mode."""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
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

    #receives and sets vehicle status values 
    def cb_vehicle_status(self, msg):
        #self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        #self.get_logger().info(f"ARM STATUS: {msg.arming_state}")
        #self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass
        if msg.pre_flight_checks_pass == True:
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

    #receives Twist commands from Teleop and converts NED -> FLU
    def cb_offboard_velocity(self, msg):
        #implements NED -> FLU Transformation
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

        # X (FLU) is -Y (NED)
        self.velocity.x = -msg.linear.y

        # Y (FLU) is X (NED)
        self.velocity.y = msg.linear.x

        # Z (FLU) is -Z (NED)
        self.velocity.z = -msg.linear.z

        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw = msg.angular.z

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def cb_attitude(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.offboardMode == True):
            # Publish offboard control modes
            self.send_heartbeat_signal()        

            # Compute velocity in the world frame
            # cos_yaw = np.cos(self.trueYaw)
            # sin_yaw = np.sin(self.trueYaw)
            # velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            # velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            # # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            # trajectory_msg = TrajectorySetpoint()
            # trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            # trajectory_msg.velocity[0] = velocity_world_x
            # trajectory_msg.velocity[1] = velocity_world_y
            # trajectory_msg.velocity[2] = self.velocity.z
            # trajectory_msg.position[0] = float('nan')
            # trajectory_msg.position[1] = float('nan')
            # trajectory_msg.position[2] = float('nan')
            # trajectory_msg.acceleration[0] = float('nan')
            # trajectory_msg.acceleration[1] = float('nan')
            # trajectory_msg.acceleration[2] = float('nan')
            # trajectory_msg.yaw = float('nan')
            # trajectory_msg.yawspeed = self.yaw

            # self.pub_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()