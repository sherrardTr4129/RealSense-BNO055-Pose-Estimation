#!/usr/bin/env python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: 02/07/2021
# Description: This script fuses the XYZ datastream from the kinect target tracking
#              with the quaternion datastream from the BNO055 orientation capture system
import rospy
from geometry_msgs.msg import PointStamped, QuaternionStamped, Pose
from message_filters import ApproximateTimeSynchronizer, Subscriber

# misc variables
kinect_topic = "kinectXYZPoint"
bno_quat_topic = "bno055_quat"
poseTopic = "fused_BNO_Kinect_Pose"
queue = 10
slop_time = 0.3

# create Pose publisher object
posePub = rospy.Publisher(poseTopic, Pose, queue_size=1)

def gotStreams(xyz_point, bno_quat):
    # create new pose object
    new_Pose = Pose()

    # populate pose position
    new_Pose.position.x = xyz_point.point.x
    new_Pose.position.y = xyz_point.point.y
    new_Pose.position.z = xyz_point.point.z

    # populate pose orientation
    new_Pose.orientation.x = bno_quat.quaternion.x
    new_Pose.orientation.y = bno_quat.quaternion.y
    new_Pose.orientation.z = bno_quat.quaternion.z
    new_Pose.orientation.w = bno_quat.quaternion.w

    # publish topic
    posePub.publish(new_Pose)

def main():
    rospy.init_node("kinect_bno_fusion_node")
    rospy.loginfo("kinect_bno_fusion_node started")

    # create subscribers
    XYZ_sub = Subscriber(kinect_topic, PointStamped)
    bno_quat_sub = Subscriber(bno_quat_topic, QuaternionStamped)

    # try to sync datastreams
    approxSync = ApproximateTimeSynchronizer([XYZ_sub, bno_quat_sub], queue_size=queue, slop=slop_time)
    approxSync.registerCallback(gotStreams)
    rospy.spin()

if(__name__ == "__main__"):
    main()
