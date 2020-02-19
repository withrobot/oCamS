
# Example Application Using oCamS-1CGN-U Stereo Camera


## 1. Stereo Matching and Object Detection Using oCamS-1CGN-U Stereo Camera on the NVIDIA Xavier

### Introduction
This example is detect object and measure the distance between camera and object using the oCamS-1CGN-U stereo camera with VisionWorks and Withrobot API.</br>
It use VisionWorks Stereo matching Algorithm for generate disparity map and darknet Yolo V3 for detect objects on the NVIDIA Xavier.</br>
So, NVIDIA Xavier is required in this example.</br>

#### Step 1. Preparation
OpenCV 3.4, Darknet installed

#### Step 2. Build Example
Download an example and build
```
$ cd stereo_vw_example && make
```
Then, will be generate stereo_vw_example

#### Step 3. Set and Run Package
Modify Stereo matching parameter file location in main.cpp 9 line
```
std::string configFile ="[MODIFY YOUR LOCATION]/stereo_vw_example/stereo_matching_demo_config.ini";
```
Check your camera path and modify in main.cpp 48 line
```
const char* devPath = "/dev/video0";
```
Set your camera gain, exposure, fps, resolution in main.hpp 28~30 line
```
#define WIDTH		640
#define HEIGHT		480
#define FPS		30
#define GAIN		150
#define EXPOSURE	150
```
Move your camera calibration file to calib folder
You can use ROS Stereo Calibration package and if you use that, add a '%YAML:1.0' in first line
```
[example]
%YAML:1.0
image_width: 640
...
```
Replace libdarknet.so file in darknet folder to use DNN

Run an example
```
$ ./stereo_vw_example
```


## 2. Camera Parameter Control Example Application Using oCamS-1CGN-U Stereo Camera

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


## 3. People Counter Example Application Using oCamS-1CGN-U Stereo Camera

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
