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
The MDDS30 only needs two wires connected to the host device. One is the ground and the other is the TX of the host should connect to the IN1 of the MDDS30 This is the example for a Jetson Nano.

![Screenshot from 2022-09-29 21-30-04](https://user-images.githubusercontent.com/3535710/193180211-cb1f48a3-3c1e-4c58-82ff-066d18714961.jpg)


The dips switches on the MDDS30 should match the following for connecting with a baudrate of 9600bps.

![dipswitch](https://user-images.githubusercontent.com/3535710/193176624-9a9f3896-a6ad-4569-9c4d-40d4cf9d798a.png)

If you want to use another baudrate use the following to determin what switches to flip:

![other options](https://user-images.githubusercontent.com/3535710/193176898-bc102955-00d3-41d2-9246-97929ccf0183.png)



## :white_check_mark: Setup ##

```bash
# CD to workspace src folder
$ cd $/catkin_ws/src 

# Clone this project
$ git clone https://github.com/r1b4z01d/MDDS30-ROS-Controller

# Drop back down
$ cd ..

# Install dependencies
$ catkin build
#or
$ catkin_make

```



## :checkered_flag: Run Node ##

```bash
# Source the workspace bash file
$ source ~/catkin_ws/devel/setup.bash

# Run the node
$ roslaunch mdds30_controller node.launch port:=/dev/ttyUSB0
 
# publish -100 to 100 to one of the two topics:
/MDDS30_controller/motor_left
/MDDS30_controller/motor_Right
```
## :memo: License ##

This project is released under CC BY-SA 4.0 license. For more details read this: [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).


Made with :heart: by <a href="https://github.com/R1B4Z01D" target="_blank">R1B4Z01D</a>

&#xa0;

<a href="#top">Back to top</a>
