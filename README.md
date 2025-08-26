<h1 align="center">MDDS30 ROS Controller</h1>

<p align="center">
  <img alt="Github top language" src="https://img.shields.io/github/languages/top/r1b4z01d/MDDS30-ROS-Controller?color=56BEB8">

  <img alt="Github language count" src="https://img.shields.io/github/languages/count/r1b4z01d/MDDS30-ROS-Controller?color=56BEB8">

  <img alt="Repository size" src="https://img.shields.io/github/repo-size/r1b4z01d/MDDS30-ROS-Controller?color=56BEB8">

  <img alt="License" src="https://img.shields.io/github/license/r1b4z01d/MDDS30-ROS-Controller?color=56BEB8">

  <!-- <img alt="Github issues" src="https://img.shields.io/github/issues/r1b4z01d/MDDS30-ROS-Controller?color=56BEB8" /> -->

  <!-- <img alt="Github forks" src="https://img.shields.io/github/forks/r1b4z01d/MDDS30-ROS-Controller?color=56BEB8" /> -->

  <!-- <img alt="Github stars" src="https://img.shields.io/github/stars/r1b4z01d/MDDS30-ROS-Controller?color=56BEB8" /> -->
</p>

<hr> 
<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0; 
  <a href="#sparkles-features">Features</a> &#xa0; | &#xa0;
  <a href="#zap-wiring">Wiring</a> &#xa0; | &#xa0;
  <a href="#white_check_mark-setup">Setup</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-run-node">Run Node</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="https://github.com/r1b4z01d" target="_blank">Author</a>
</p>
b4z01d
<br>

## :dart: About ##

This is a ROS2 node to allow easy integration with the [Cytron MDDS30 SmartDriveDuo-30 motor controller](https://www.cytron.io/p-30amp-7v-35v-smartdrive-dc-motor-driver-2-channels). This node utilizes the Serial Simplified mode of the MDDS30. Each channel has a topic that accepts -100 (full speed reverse) to 100 (full speed forward).

**Updated for ROS2 and optimized for NVIDIA Jetson Orin Nano.**

## :sparkles: Features ##

:heavy_check_mark: UART control;\
:heavy_check_mark: Forward and Reverse;\
:heavy_check_mark: 2 wire connection;

## :zap: Wiring ##
The MDDS30 only needs two wires connected to the host device. One is the ground and the other is the TX of the host should connect to the IN1 of the MDDS30.

### For NVIDIA Jetson Orin Nano:
- Connect GND to GND
- Connect UART TX (Pin 8, /dev/ttyTHS0) to MDDS30 IN1

### For other Jetson devices:
- Jetson Nano: Use /dev/ttyTHS1 (Pin 8)
- Generic USB-Serial: Use /dev/ttyUSB0

![Screenshot from 2022-09-29 21-30-04](https://user-images.githubusercontent.com/3535710/193180211-cb1f48a3-3c1e-4c58-82ff-066d18714961.jpg)


The dips switches on the MDDS30 should match the following for connecting with a baudrate of 9600bps.

![dipswitch](https://user-images.githubusercontent.com/3535710/193176624-9a9f3896-a6ad-4569-9c4d-40d4cf9d798a.png)

If you want to use another baudrate use the following to determin what switches to flip:

![other options](https://user-images.githubusercontent.com/3535710/193176898-bc102955-00d3-41d2-9246-97929ccf0183.png)



## :white_check_mark: Setup ##

```bash
# Install Python dependencies
$ pip install pyserial

# CD to workspace src folder
$ cd ~/ros2_ws/src 

# Clone this project
$ git clone https://github.com/Scrappybots/MDDS30-ROS-Controller

# Drop back to workspace root
$ cd ~/ros2_ws

# Build with colcon
$ colcon build --packages-select mdds30_controller

# Source the workspace
$ source install/setup.bash
```



## :checkered_flag: Run Node ##

```bash
# Source the workspace bash file
$ source ~/ros2_ws/install/setup.bash

# Run the node with default Jetson Orin Nano port (/dev/ttyTHS0)
$ ros2 run mdds30_controller mdds30_controller

# Or run with custom port
$ ros2 run mdds30_controller mdds30_controller --ros-args -p port:=/dev/ttyUSB0

# Or use launch file
$ ros2 launch mdds30_controller node_launch.py

# Or launch with custom port
$ ros2 launch mdds30_controller node_launch.py port:=/dev/ttyUSB0
 
# Publish -100 to 100 to one of the two topics:
# ros2 topic pub /MDDS30_controller/motor_left std_msgs/msg/Int16 "data: 50"
# ros2 topic pub /MDDS30_controller/motor_right std_msgs/msg/Int16 "data: -30"
```

### Jetson Orin Nano Serial Port Notes:
- Default port: `/dev/ttyTHS0` (UART1)
- Make sure UART is enabled in device tree
- Check permissions: `sudo chmod 666 /dev/ttyTHS0`
## :memo: License ##

This project is released under CC BY-SA 4.0 license. For more details read this: [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).


Made with :heart: by <a href="https://github.com/R1B4Z01D" target="_blank">R1B4Z01D</a>

&#xa0;

<a href="#top">Back to top</a>
