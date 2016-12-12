## Synopsis

ROS package for Thermal Seek camera

## Motivation

The Thermal Seek camera is a thermal camera, very cheap in the reign of thermal cameras and a ROS node to take picture with this camera allows a robot to obtain thermal data from environment, which may use to detect possible failures in monitored systems or in everyday activities like cooking.

## Installation

### ROS
A working installation of ROS must be installed, before building and running this package.

Install ROS

http://wiki.ros.org/indigo/Installation/Ubuntu

Then set up the ROS environment

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment


### Download and build seek_cam package

$ git clone git@github.com:arcoslab/ros_seek_cam.git

$ cd ros_seek_cam/

$ catkin_make

then source setup.bash

$ source devel/setup.bash

this last step must be executed from every terminal before using this package

## Run
Now from the same terminal where we source the setup.bash

We may run the launch file view_seek.launch to see the camera's output

$ roslaunch seek_cam view_seek.launch

We may also use rosrun

$ roscore

$ rosrun seek_cam seek_cam_node.py

and see the published messages with 

$ rostopic echo /thermalview/compressed


## API Reference

The node publishes png images in /thermalview/compressed topic.

## Examples

## Todo

- dead pixels
- define temperature scale

## Author 
Alexander Marin Drobinoga alexanderm2230@gmail.com

