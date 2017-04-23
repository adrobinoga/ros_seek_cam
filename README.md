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

### Install pyseek
The library to use the camera must be downloaded

$ git clone git@github.com:adrobinoga/pyseek.git

In order to use this lib you must append the path to this lib to PYTHONPATH, you may do this appending the next line at the end of your .bashrc file

export PYTHONPATH="${PYTHONPATH}:/path/of/your/download/pyseek"

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

## FAQs

1. Why a separate repo for pyseek?
The the tridge fork for pyseek just works for PILLOW versions older than 2.9.0 , the current fork, used for this project supports newer versions.

## Todo

- fix dead pixels
- get absolute temperature
- consider using zougloub/libseek instead of pyseek
## Author 
Alexander Marin Drobinoga alexanderm2230@gmail.com

