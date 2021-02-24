# Orientation Capture Sub-System Designs (Electrical and Mechanical) and Firmware
This directory contains the hardware designs (mechanical and electrical) and firmware needed to construct the wireless orientation capture system used for more robust motion capture.

# Overall System Specifications
The table below outlines some basic specifications for the overall orientation capture sub-system:

|                       Specification                       |                              Value                              |
|:---------------------------------------------------------:|:---------------------------------------------------------------:|
| Transmitter Range                                         | At least 5m (probably much more even at the NRF24_PA_LOW value) |
| Transmitter Sub-System Current Consumption (steady state) | ~50mA                                                           |
| Battery Life (Assuming 1Ah cell)                          | ~20 hours                                                       |
| Transmitter Input Voltage (4 AA battery cell recommended) | 5V - 6V                                                         |
| Overall System Data Frame Rate                            | ~30 FPS                                                         |

# Electrical Hardware Designs
For this project, a PCB was designed containing all required hardware to read data packets from the BNO055 sensor and transmit them wirelessly to a basestation, where they can be read in by the aforementioned ROS driver. The kiCAD project containng the schematic and board layout files can be found in the electricalHardware/wirelessOrientationSensorSystem directory in this repository. Please see below for a wiring diagram for both the base-station and wireless transmitter board and well as real pictures of each device. 

## Sensor Data Transmitter
![Wireless Data TX Real Picture](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/wirelessDataTX.jpg)
![Wireless Data TX Wiring Diagram](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/wirelessTxDiagram.png)

Please note that this diagram depicts the wireless data tranmission module in its operational state. More connections are required to flash the firmware to the device. These connections and the firmware flashing process will be outlined later in this readme.

## Sensor Data Reciever 
![Wireless Data RX Real Picture](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/wirelessDataRX.jpg)
![Wireless Data RX Wiring Diagram](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/basestationDiagram.png)


# Firmware
The firmware flashing process for each of the board assemblies can be seen in each respective sub-section.

## Sensor Data Transmitter Firmware Flashing Procedure
Just a stub for now

## Sensor Data Reciever Firmware Flashing Procedure
Just a stub for now

# Mechanical Hardware Designs 
The mechanical hardware designs for both the vision target and the base station can be seen in the rendered images below. Note that both the these STL files are available in this repository. 

![vision target](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/visionTarget.PNG)
![base station](https://github.com/sherrardTr4129/Kinect-BNO055-Pose-Estimation/blob/main/documentation/images/basestation.PNG)
