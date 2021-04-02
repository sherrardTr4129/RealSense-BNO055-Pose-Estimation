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
import rospy
import pyrealsense2 as rs
from geometry_msgs.msg import PointStamped

cv2.namedWindow('Depth')
cv2.namedWindow('Video')

# define various global variables
pointPubTopic = "kinectXYZPoint"
lowerColorBound = (0, 187, 83)
upperColorBound = (255, 255, 255)
XYScaleFactor = 20
ZScaleFactor = 50
MIN_AREA = 100
WINDOW_SIZE = 3

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
    """
    This function finds a circle that encloses the largest contour
    in a given image that is over a given size of MIN_AREA. If a circle is found,
    a True status boolean is returned, along with the circle's center coordinates and
    radius.

    params:
        frame (uint8 image): the RGB image from the kinect to process
    returns:
        status, circle (boool, ((x,y), radius)): the status indicating if a circle was found
                                                 and the detected circle center point and radius
    """
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

def circle_to_realworld(depthImg, circle, depth_intrin):
    """
    This function takes a given depth image from the Intel RealSense camera
    and the center point of the detected circle, and de-projects it into real
    world points. 

    params:
        depthImg (depth frame): The depth image from the Intel RealSense
        circle ((int x, int y), int radius): the center point and radius of the detected circle
        depth_intrin (depth intrinsics): Intrinsic paramters of the depth frame.
    returns:
        real_world_point (int): the real world point of the detected circle 
    """
    # construct ROI
    (x,y), radius = circle
    x=int(x)
    y=int(y)
    radius=int(radius)

    # get depth value of circle center
    depth = depthImg.get_distance(x, y)

    # perform deprojection
    real_world_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)

    return real_world_point


def main():
    # start node
    rospy.init_node("kinect_find_xyz")
    rospy.loginfo("kinect_find_xyz node initialized")

    # create Point32 publisher
    pointPub = rospy.Publisher(pointPubTopic, PointStamped, queue_size=1)

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    # enable depth and color streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)


    while (not rospy.is_shutdown()):
        # grab frames from intel RealSense
        frames = pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # get depth frame intrinsics
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        # convert to numpy arrays
        numpyRGB = np.asanyarray(color_frame.get_data())
        numpyDepth = np.asanyarray(aligned_depth_frame.get_data())

        # find min enclosing circle 
        status, circle = findMinEnclosingCircle(numpyRGB)

        if(status):
            # draw circles if they are found
            (x,y), radius = circle
            cv2.circle(numpyRGB, (int(x), int(y)), int(radius), (0,255,0),2)

            # find depth of detected circle
            average = circle_to_realworld(aligned_depth_frame, circle, depth_intrin)

            # invert sign of y-point to align with standard cartesian
            # views
            average[1] = -1*average[1]

            # create PointStamped message and publish
            pointMessage = PointStamped()
            pointMessage.header.stamp = rospy.Time.now()
            pointMessage.point.x = average[0]
            pointMessage.point.y = average[1]
            pointMessage.point.z = average[2]
            pointPub.publish(pointMessage)

        # display images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(numpyDepth, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow('Depth', depth_colormap)
        cv2.imshow('Video', numpyRGB)
        if(cv2.waitKey(1) & 0xFF == ord('q')):
            break

if(__name__ == "__main__"):
    main()
