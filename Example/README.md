
# Example Application Using oCamS-1CGN-U Stereo Camera

## 1. Camera Parameter Control Example Application Using oCamS-1CGN-U Stereo Camera

### Introduction
This example is to show how to control camera parameter c++ example using the oCamS-1CGN-U stereo camera with openCV and Withrobot API.</br>
It can control gain, exposure, white balance and auto exposure on/off.</br>

#### Step 1. Preparation
OpenCV 4.x.x installed
if you use openCV version 3.x.x, modify something in Makefile and main.cpp

#### Step 2. Build Example
Download an example and build
```
$ cd oCamS_Linux_ParameterControl_Example && make
```
Then, will be generate oCamS_Linux_ParameterControl_Example

#### Step 3. Set and Run Package
Run an example
```
$ ./oCamS_Linux_ParameterControl_Example
```


## 2. People Counter Example Application Using oCamS-1CGN-U Stereo Camera

### Introduction
This example is to show how to implement a simple people counter example using the oCamS-1CGN-U stereo camera with ROS.</br>
The stereo camera generates depth map and detects significant change of the depth(i.e., distance) within a specific region (ROI).</br>

#### Step 1. Preparation
Linux system with ROS installed.
Ref.: http://wiki.ros.org/ROS/Installation

#### Step 2. Install ROS Dependence Package
Download and install “rqt”, the ROS visualization package.
```
$ sudo apt-get install ros-[ROS_VERSION]-rqt-*
```

Install the package.
```
$ svn export https://github.com/withrobot/oCamS/trunk/Example/People_counter
```
Build the package.
```
$ cd ~/catkin_ws && catkin_make
```

#### Step 3. Set and Run Package
Set the path.
```
$ source ~/catkin_ws/devel/setup.bash
```
Run the package.</br>

* For calibration
```
$ roslaunch People_counter calibration.launch
```
* To see disparity map
```
$ roslaunch People_counter disparity.launch
```
* To see left and right camera images
```
$ roslaunch People_counter ocams_ros.launch
```

