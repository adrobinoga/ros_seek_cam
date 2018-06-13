# ROS Node for Thermal Seek Camera

[TOC]

## Synopsis

ROS package for Thermal Seek camera.

## Motivation

The Thermal Seek camera is a thermal camera, very cheap in the reign of thermal cameras and a ROS node to take picture with this camera allows a robot to obtain thermal data from environment, which may use to detect possible failures in systems or in everyday activities like cooking.

## Installation

### ROS

A working installation of ROS must be installed, before building and running this package.

Install ROS, see:

http://wiki.ros.org/indigo/Installation/Ubuntu

Then set up the ROS environment, see:

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

You may also need to install:

- `ros-<distro>-image-transport-plugins`
- `ros-<distro>-image-view`

to visualize images.

### Setup pyseek

The library to use the camera must be downloaded

```bash
$ git clone git@github.com:adrobinoga/pyseek.git
```

In order to use this lib you must append the path to this lib to `PYTHONPATH`, you may do this appending the next line in your .bashrc file

```bash
export PYTHONPATH="${PYTHONPATH}:/path/of/your/download/pyseek"
```

### Download and build seek_cam package

Use the following commands to get and build the ROS node:

```bash
$ git clone git@github.com:arcoslab/ros_seek_cam.git
$ cd ros_seek_cam/
$ catkin_make
```

Then append the following command to .bashrc file:

```bash
source /<full>/<path>/devel/setup.bash
```

## Run

Now we may run the launch file view_seek.launch to see the camera's output

```bash
$ roslaunch seek_cam view_seek.launch
```

### Debug

When running with the roslaunch program it is hard to find errors in the setup.

You may perform the roslaunch tasks manually.

First, launch roscore:

```bash
$ roscore
```

Then in another terminal, launch main node:

```bash
$ rosrun seek_cam seek_cam_node.py
```

Open another terminal and run the following command to visualize thermal images with the image_view package:

```bash
$ rosrun image_view image_view image:=/thermalview _image_transport:=compressed
```

## API Reference

The node publishes png images in /thermalview/compressed topic.

## FAQs

1. Why a separate repo for pyseek?
The tridge fork for pyseek just works for PILLOW versions older than 2.9.0 , the current fork, used for this project supports newer versions.

## Todo

- fix dead pixels
- get absolute temperature
- consider using zougloub/libseek instead of pyseek

## License

GPLv3, see <http://www.gnu.org/licenses/>

## Contact

Alexander Marin Drobinoga <alexanderm2230@gmail.com>
