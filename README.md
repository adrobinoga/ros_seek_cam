## Synopsis


## Motivation


## Installation

### ROS
A working installation of ROS must be installed, before building and running this package.

Install ROS

http://wiki.ros.org/indigo/Installation/Ubuntu

Then set up the ROS environment

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment


### Download and build sony_cam package

$ git clone git@github.com:arcoslab/ros_sony_cam.git

$ cd ros_sony_cam/

$ catkin_make

then source setup.bash

$ source devel/setup.bash

this last step must be executed from every terminal before using this package

## Run
Now from the same terminal were we source the setup.bash

We may run the launch file to see the camera's output, liveview (low quality images) and high quality photos.

$ roslaunch sony_cam pic_liveview_test.launch

We may also use rosrun

$ roscore

$ rosrun sony_cam sony_cam_node.py

and see the published messages with 

$ rostopic echo /liveview/compressed


## API Reference

## Examples


## Author 
Alexander Marin Drobinoga alexanderm2230@gmail.com

