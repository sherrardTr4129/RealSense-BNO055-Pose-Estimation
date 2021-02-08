# Kinect-BNO055-Pose-Estimation
This repository contains required software to construct a Pose of a vision target by fusing its x,y,z coordinates from the kinect, with its quaternion orientation from the BNO055 sensor.

## System Description
More to come

### Kinect Position Capture
More to come

### BNO055 Orientation Capture
More to come

## System Dependencies
The various dependencies of the system can be seen in the following sub-sections.

### Kinect Dependencies
To run the kinect interfacing software in this repository, make sure to install the libfreenect library and python wrappers. Two bash scripts can be found [here](https://github.com/alwynmathew/libfreenect-with-python] that do exactly that.)

### MCU Serial Interface
The wirelss orientation data reciever MCU serial ROS driver depends on the pyserial library for serial connectiviy. This library can be installed by running the following command in a terminal:

```bash
sudo apt install python-serial
```

## Usage
More to come
