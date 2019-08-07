# People Counter Example Application Using oCamS-1CGN-U Stereo Camera

## Introduction
This example is to show how to implement a simple people counter example using the oCamS-1CGN-U stereo camera with ROS.
The stereo camera generates depth map and detects significant change of the depth(i.e., distance) within a specific region (ROI).

### Step 1. Preparation
Linux system with ROS installed.
Ref.: http://wiki.ros.org/ROS/Installation

### Step 2. Install ROS Dependence Package
Download and install “rqt”, ther ROS visualization package.
```
$ sudo apt-get install ros-[ROS_VERSION]-rqt-*
```

Install and the package.
```
$ svn export https://github.com/withrobot/oCamS/trunk/Example/People_counter
```
Build the package.
```
$ cd ~/catkin_ws && catkin_make
```

### Step 3. Set and Run Package
Set the path.
```
$ source ~/catkin_ws/devel/setup.bash
```
Run the package.
For calibration
```
$ roslaunch People_counter calibration.launch
```
To see disparity map
```
$ roslaunch People_counter disparity.launch
```
To see left and right camera images
```
$ roslaunch People_counter ocams_ros.launch
```
