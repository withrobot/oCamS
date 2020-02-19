Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.

(L4T R24) NVIDIA GStreamer Camera Capture Sample App
@brief NVIDIA GStreamer Camera Capture Sample user guide.

## Introduction ##

`nvx_sample_nvgstcamera_capture` demonstrates NVIDIA GStreamer camera access. The sample is tested with
the E2146 camera module.

@note This sample is supported only on Jetson Embedded platforms running L4T R24 OS.

`nvx_sample_nvgstcamera_capture` is installed in the following directory:

    /usr/share/visionworks/sources/samples/nvgstcamera_capture

For the steps to build sample applications, see the see: nvx_sample_building_linux section.

## Available Camera modes

|  Resolution | Frames per second |
|:-----------:|:-----------------:|
| `2592x1944` |        30         |
| `2592x1458` |        30         |
|  `1280x720` |       120         |

## Executing the NVIDIA GStreamer Camera Capture Sample ##

    ./nvx_sample_nvgstcamera_capture [options]

### Command Line Options ###

This topic provides a list of supported options and the values they consume.

#### \-r, \--resolution ####
- Parameter: [resolution]
- Description: Specifies the resolution.
- Usage: `--resolution=1280x720` for capturing `720p` frames.
- Supported resolution settings:

	- `2592x1944`
	- `2592x1458`
	- `1280x720`
	- `640x480`

#### \-f, \--fps ####
- Parameter: [fps]
- Description: Specifies the frame per second.
- Usage: `--fps=30` to capture 30 frames per second.
- Supported FPS settings: `[10, 120]`

@note The actual FPS value may differ from the specified one:
1. Custom resolutions such as `640x480` cannot be played with FPS higher than 30.
2. FPS for native resolutions cannot be higher than the native value.

#### \-h, \--help ####
- Description: Prints the help message.

### Operational Keys ###
- Use `Space` to pause/resume the sample.
- Use `ESC` to close the sample.

