# Orientation Capture Sub-System Designs (Electrical and Mechanical) and Firmware
This directory contains the hardware designs (mechanical and electrical) and firmware needed to construct the wireless orientation capture system used for more robust motion capture.

# Overall System Specifications
The table below outlines some basic specifications for the overall orientation capture sub-system.

# Electrical Hardware Designs
For this project, a PCB was designed containing all required hardware to read data packets from the BNO055 sensor and transmit them wirelessly to a basestation, where they can be read in by the aforementioned ROS driver. The kiCAD project containng the schematic and board layout files can be found in the electricalHardware/wirelessOrientationSensorSystem directory in this repository. Please see below for a wiring diagram for both the base-station and wireless transmitter board and well as real pictures of each device. 

## Sensor Data Transmitter
![Wireless Data TX Real Picture](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/wirelessDataTX.jpg)

## Sensor Data Reciever 
![Wireless Data RX Real Picture](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/wirelessDataRX.jpg)


# Firmware
The firmware flashing process for each of the board assemblies can be seen in each respective sub-section.

## Sensor Data Transmitter Firmware Flashing Procedure
Just a stub for now

## Sensor Data Reciever Firmware Flashing Procedure
Just a stub for now

# Mechanical Hardware Designs 
Just a stub for now
