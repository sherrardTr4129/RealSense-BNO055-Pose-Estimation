#!/usr/bin/env python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: 02/06/2021
# Description: This script extracts synced RGB and Depth frames from 
#              the kinect and attempts to locate a given vision target.
#              if the vision target is located, the XYZ coordinates are
#              scaled and published.

import freenect
import cv2
import numpy as np
import frame_convert2
import rospy
from geometry_msgs.msg import PointStamped

cv2.namedWindow('Depth')
cv2.namedWindow('Video')

# define various global variables
pointPubTopic = "kinectXYZPoint"
lowerColorBound = (0, 187, 83)
upperColorBound = (255, 255, 255)
MIN_AREA = 100

def getDepth():
    """
    This function extracts a synced depth image from the kinect.
    params:
        None
    returns:
        depth (uint8 frame): The depth image
    """
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth(format=freenect.DEPTH_REGISTERED)[0])

def getRGB():
    """
    This function extracts a synced RGB image from the kinect.
    params:
        None
    returns:
        RGB (uint8 frame): The RGB image
    """
    return frame_convert2.video_cv(freenect.sync_get_video()[0])

def threshColorImage(colorImg, lowerColorBound, upperColorBound):
    """
    This function takes a HSV image and two HSV color tuples and
    returns a thresholded image. 

    params:
        colorImg (int8 HSV image): the HSV image to be thresholded
        lowerColorBound (int,int,int): the lower color threshold bound
        upperColorBound (int,int,int): the upper color threshold bound
    returns:
        threshedImg (uint8 image): a thresholded binary image
    """
    threshedImage = cv2.inRange(colorImg, lowerColorBound, upperColorBound)
    return threshedImage

def findMinEnclosingCircle(frame):
    # blur image
    blurKernel = (11, 11)
    blurred = cv2.GaussianBlur(frame, blurKernel, 0)

    # convert to HSV
    hsvImage = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # threshold
    thresh = threshColorImage(hsvImage, lowerColorBound, upperColorBound)

    # open image
    kernel = np.ones((5,5), np.uint8)
    erode = cv2.erode(thresh, kernel, iterations=2)
    openedImage = cv2.dilate(erode, kernel, iterations=2)

    # find edges
    edges = cv2.Canny(openedImage, 0, 255)

    # find image contours from edge image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    goodContours = []
    for cont in contours:
        area = cv2.contourArea(cont)
        if(area > MIN_AREA):
            goodContours.append(cont)

    defaultCircle = ((0,0),0)
    if(len(goodContours) > 0):
        #find biggest contour
        maxC = max(goodContours, key=cv2.contourArea)

        # find enclosing circle of biggest contour
        ((cx, cy), radius) = cv2.minEnclosingCircle(maxC)
        defaultCircle = ((cx, cy), radius)
        return True, defaultCircle
    else:
        return False, defaultCircle

def findDepthAvg(depthImg, circle):
    # construct ROI
    (x,y), radius = circle
    x=int(x)
    y=int(y)
    radius=int(radius)
    circleRoi = depthImg[y-radius:y+radius, x-radius:x+radius]

    # grab circle ROI height and width
    w = circleRoi.shape[1]
    h = circleRoi.shape[0]

    # create circle mask
    circleMask = np.zeros((w,h), circleRoi.dtype)
    cv2.circle(circleMask, (int(w/2), int(h/2)), radius, (255,255,255), -1)

    # issolate circle ROI in depth image
    combined = cv2.bitwise_and(circleRoi, circleMask)

    # compute average of non-black pixels on depth image
    nonZeroIndecies = np.where(combined != 0)[0]
    average = np.mean(combined[nonZeroIndecies])

    return average

def mapCircleCoordinates(numpyDepthFrame, circleCenter, depthAvg):
    # grab shape of frame
    w = numpyDepthFrame.shape[1]
    h = numpyDepthFrame.shape[0]

    # find image center
    imgCx = int(w/2)
    imgCy = int(h/2)

    # extract center of circle
    (circleX, circleY) = circleCenter

    # map circle center to new coordinate system
    circleCenterMappedX = circleX - imgCx
    circleCenterMappedY = -(circleY - imgCy)

    # scale circle coordinates to unit length
    circleScaledX = circleCenterMappedX / (imgCx)
    circleScaledY = circleCenterMappedY / (imgCy)
    circleScaled = (circleScaledX, circleScaledY)

    # scale depth average
    depthAvgScaled = depthAvg / 255
    return circleScaled, depthAvgScaled

def main():
    # start node
    rospy.init_node("kinect_find_xyz")
    rospy.loginfo("kinect_find_xyz node initialized")

    # create Point32 publisher
    pointPub = rospy.Publisher(pointPubTopic, PointStamped, queue_size=1)

    while (not rospy.is_shutdown()):
        # grab frames from kinect
        depthImage = getDepth()
        rgbImage = getRGB()

        # convert to numpy arrays
        numpyRGB = np.array(rgbImage)
        numpyDepth = np.array(depthImage)

        # find min enclosing circle 
        status, circle = findMinEnclosingCircle(numpyRGB)

        if(status):
            # draw circles if they are found
            (x,y), radius = circle
            cv2.circle(numpyRGB, (int(x), int(y)), int(radius), (0,255,0),2)

            #find average depth of the pixels within the circle
            average = findDepthAvg(numpyDepth, circle)
            print(numpyDepth.shape[0])

            # map circle coordinates to meters (approx)
            (mappedX, mappedY), depthAvgScaled = mapCircleCoordinates(numpyDepth, (x,y), average)

            # create PointStamped message and publish
            pointMessage = PointStamped()
            pointMessage.header.stamp = rospy.Time.now()
            pointMessage.point.x = mappedX
            pointMessage.point.y = mappedY
            pointMessage.point.z = depthAvgScaled
            pointPub.publish(pointMessage)

        cv2.imshow('Depth', numpyDepth)
        cv2.imshow('Video', numpyRGB)
        if(cv2.waitKey(1) & 0xFF == ord('q')):
            break

if(__name__ == "__main__"):
    main()
