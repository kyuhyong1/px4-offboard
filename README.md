# px4-offboard
This `repository` contains a python examples for offboard control on ROS2 with [PX4](https://px4.io/)

The `px4_offboard` package contains the following nodes
- `offboard_control.py`: Example of offboard position control using position setpoints
- `visualizer.py`: Used for visualizing vehicle states in Rviz
- `velocity_control.py`: Example of offboard velocity control based from [ROS2_PX4_Offboard_Example](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example)
- `control.py` : Keyboard User input to be sent to velocity control

The source code is released under a BSD 3-Clause license.

- **Author**: Jaeyoung Lim
- **Affiliation**: Autonomous Systems Lab, ETH Zurich
- **Maintainer**: Kyuhyong You

## Setup
Add the repository to the ros2 workspace
```
git clone https://github.com/kyuhyong1/px4-offboard
```

## Running

### Start PX4 in simulation

You will make use of 3 different terminals to run the offboard demo.

On the first terminal, run a SITL instance from the PX4 Autopilot firmware.
```
make px4_sitl gazebo
```

On the second terminal terminal, run the micro-ros-agent which will perform the mapping between Micro XRCE-DDS and RTPS. So that ROS2 Nodes are able to communicate with the PX4 micrortps_client.
```
micro-ros-agent udp4 --port 8888
```

### Start PX4 hardware

First you will need to start micro-ros-agent either from the Host PC or Companion computer(CC) depending on how PX4 is connected to.

If CC is connected to PX4 via serial port(/dev/ttyHS1) with baud rate of 2000000,
```
$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyHS1 -b 2000000
```

### Launch velocity control

To run the offboard velocity control example, open a terminal and run the the node.
```
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

This will launch control.py which takes keyboard input from user to send Twist command to velocity_control node.  

Available commands as below

- W: &emsp;Velocity +Z (Move up)
- X: &emsp;Velocity -Z (Move down)
- A: &emsp;Velocity +Y (Move Left)
- D: &emsp;Velocity -Y (Move Right)

- Arrow Up: &emsp;Velocity +X (FWD)
- Arrow Down: &emsp;Velocity -X (REV)
- Arrow Left: &emsp;Rotate +Z (Yaw Left)
- Arrow Right: &emsp;Rotate -Z (Yaw Right)

- S:&emsp;&emsp;&emsp;Reset velocity to zero
- O:&emsp;&emsp;&emsp;Set to **OFFBOARD** mode
- SPACE:&emsp;&emsp;Set **ARM**
- T:&emsp;&emsp;&emsp;Set to **TAKEOFF** mode
- L:&emsp;&emsp;&emsp;Set to **LAND** mode
- BACKSPACE: &emsp;Set **DISARM**


### Launch offboard_position_control

In order to run the offboard position control example, open a third terminal and run the the node.
This runs two ros nodes, which publishes offboard position control setpoints and the visualizer.
```
ros2 launch px4_offboard offboard_position_control.launch.py
```

![offboard](https://user-images.githubusercontent.com/5248102/194742116-64b93fcb-ec99-478d-9f4f-f32f7f06e9fd.gif)

In order to just run the visualizer,
```
ros2 launch px4_offboard visualize.launch.py
```
