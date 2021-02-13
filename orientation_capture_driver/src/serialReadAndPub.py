#!/usr/bin/python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: Febuary 3rd, 2021
# Description: This file instantiates a node that reads from 
#              the RX arduino and publishes quaternion data
#              to the rest of the system

import rospy
import time
import math
from SerialManagerClass import SerialManager 
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

# define global variables
bno055Topic = "bno055_quat"
port = "/dev/ttyACM0"
baud = 9600
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

    status = True
    # try to extract substring indecies.
    try:
        Rind = RxText.index('R')
        Pind = RxText.index('P')
        Yind = RxText.index('Y')

    except ValueError, err:
        rospy.logerr("no substring found for quaternion component: %s" % err)
        status = False

    if(status):
        # grab actual substrings
        Rdata = RxText[Rind+1:Pind]
        Pdata = RxText[Pind+1:Yind]
        Ydata = RxText[Yind+1:]

        # remove whitespace
        Rdata.strip()
        Pdata.strip()
        Ydata.strip()
        
        # construct quatDict
        rpyDict = {"R":Rdata, "P":Pdata, "Y":Ydata}
        return status, rpyDict

    else:
        return status, dict()

def main():
    # init node
    rospy.init_node("bno055_ros_driver")
    rospy.loginfo("bno055_ros_driver_node initialized")

    # create publisher object
    quatPub = rospy.Publisher(bno055Topic, QuaternionStamped, queue_size=1)

    # try to open serial port
    res = serObj.open()

    # wait a second before procedeing to make sure
    # that serial port has been set up
    time.sleep(sleepTime)

    if(not res):
        rospy.logerr("could not open serial device! Double check port and baud settings ")
        return -1

    # start publishing data
    else:
        while(not rospy.is_shutdown()):
            # grab line and process
            lineToProc = serObj.read()
            if(len(lineToProc) == 0):
                continue
            else:
                status, rpyDict = procSerialData(lineToProc)
                if(status):
                    # extract rpy data
                    roll = float(rpyDict["R"])
                    pitch = float(rpyDict["P"])
                    yaw = float(rpyDict["Y"])

                    # convert to radians
                    roll = roll * (math.pi/180)
                    pitch = pitch * (math.pi/180)
                    yaw = yaw * (math.pi/180)

                    # convert to quaternion
                    quaternion = quaternion_from_euler(roll, pitch, yaw)

                    # rotate pitch by -pi/2
                    pitchRotateQuat = quaternion_from_euler(0, -math.pi/2, 0)

                    # perform quaternion multiplication
                    tempQuat = quaternion_multiply(quaternion, pitchRotateQuat)

                    # move back to euler 
                    rpyNew = euler_from_quaternion(tempQuat)
                    newRoll = rpyNew[0]
                    newPitch = rpyNew[1]
                    newYaw = rpyNew[2]

                    # swap roll and yaw
                    tempRoll = newRoll
                    newRoll = newYaw
                    newYaw = tempRoll

                    # back to quaternion
                    newQuat = quaternion_from_euler(newRoll, newPitch, newYaw)

                    try:
                        # construct Quaternion message
                        quatMessage = QuaternionStamped()
                        quatMessage.header.stamp = rospy.Time.now()
                        quatMessage.quaternion.x = newQuat[0]
                        quatMessage.quaternion.y = newQuat[1]
                        quatMessage.quaternion.z = newQuat[2]
                        quatMessage.quaternion.w = newQuat[3]

                        # publish data
                        print(quatMessage)
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
