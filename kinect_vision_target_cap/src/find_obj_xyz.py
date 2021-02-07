#!/usr/bin/env python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: 02/06/2021
# Description: This script extracts synced RGB and Depth frames from 
#              the kinect and attempts to locate a given vision target.

import freenect
import cv2
import numpy as np
import frame_convert2

def nothing(x):
    pass

cv2.namedWindow('Depth')
cv2.namedWindow('Video')

# define various global variables
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
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])

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

    # generate circle mask image from ROI
    w = circleRoi.shape[0]
    h = circleRoi.shape[1]

    circleMask = np.zeros((w,h), circleRoi.dtype)
    cv2.circle(circleMask, (int(w/2), int(h/2)), radius, (255,255,255), -1)

    combined = cv2.bitwise_and(circleRoi, circleMask)
    nonZeroIndecies = np.where(combined != 0)[0]
    average = np.mean(combined[nonZeroIndecies])
    print(average)
    return combined

def main():
    while 1:
        # grab frames from kinect
        depthImage = getDepth()
        rgbImage = getRGB()

        # convert to numpy arrays
        numpyRGB = np.array(rgbImage)
        numpyDepth = np.array(depthImage)

        status, circle = findMinEnclosingCircle(numpyRGB)
        if(status):
            (x,y), radius = circle
            cv2.circle(numpyRGB, (int(x), int(y)), int(radius), (0,255,0),2)
            numpyDepth = findDepthAvg(numpyDepth, circle)
        cv2.imshow('Depth', numpyDepth)
        cv2.imshow('Video', numpyRGB)
        if(cv2.waitKey(1) & 0xFF == ord('q')):
            break

if(__name__ == "__main__"):
    main()
