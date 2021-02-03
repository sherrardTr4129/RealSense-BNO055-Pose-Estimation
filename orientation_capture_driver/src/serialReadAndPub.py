#!/usr/bin/python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: Febuary 3rd, 2021
# Description: This file instantiates a node that reads from 
#              the RX arduino and publishes quaternion data
#              to the rest of the system

import rospy
import time
from SerialManagerClass import SerialManager 
from geometry_msgs.msg import Quaternion

# define global variables
bno055Topic = "bno055_quat"
port = "/dev/ttyACM0"
baud = 57600
serObj = SerialManager(port, baud)
sleepTime = 1

def procSerialData(RxText):
    """
    This function extracts the individual quaternion components from the recieved
    serial text. The individual components are packed into a dictionary and returned
    for processing

    params:
        RxText (str): The string returned from the serial port
    returns:
        quatDict (dict): a dictionary representation of the extracted quaternion data.
                         the dictionary will be empty if data extraction fails
        status (bool): a boolean indicating the status of the data extraction
    """

    print(RxText.isalnum())
    status = True
    # try to extract substring indecies.
    try:
        Xind = RxText.index('X')
        Yind = RxText.index('Y')
        Zind = RxText.index('Z')
        Wind = RxText.index('W')

    except ValueError, err:
        rospy.logerr("no substring found for quaternion component: %s" % err)
        status = False

    if(status):
        # grab actual substrings
        Xdata = RxText[Xind+1:Yind]
        Ydata = RxText[Yind+1:Zind]
        Zdata = RxText[Zind+1:Wind]
        Wdata = RxText[Wind+1:]

        # remove whitespace
        Xdata.strip()
        Ydata.strip()
        Zdata.strip()
        Wdata.strip()
        
        # construct quatDict
        quatDict = {"X":Xdata, "Y":Ydata, "Z":Zdata, "W":Wdata}
        return status, quatDict

    else:
        return status, dict()

def main():
    # init node
    rospy.init_node("bno055_ros_driver")
    rospy.loginfo("bno055_ros_driver_node initialized")

    # create publisher object
    quatPub = rospy.Publisher(bno055Topic, Quaternion, queue_size=1)

    # try to open serial port
    res = serObj.open()

    # wait a second before procedeing to make sure
    # that serial port has been set up
    time.sleep(sleepTime)

    if(not res):
        return -1

    # start publishing data
    else:
        while(not rospy.is_shutdown()):
            # grab line and process
            lineToProc = serObj.read()
            if(len(lineToProc) == 0):
                continue
            else:
                status, quatDict = procSerialData(lineToProc)
                print(quatDict) 
                if(status):
                    try:
                        # construct Quaternion message
                        quatMessage = Quaternion()
                        quatMessage.x = float(quatDict["X"])
                        quatMessage.y = float(quatDict["Y"])
                        quatMessage.z = float(quatDict["Z"])
                        quatMessage.w = float(quatDict["W"])

                        # publish data
                        quatPub.publish(quatMessage)
                    except ValueError, err:
                        rospy.logerr("could not extract all quaternion components from serial string: %s" % err)
                        continue
        
        # close the serial port
        serObj.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
