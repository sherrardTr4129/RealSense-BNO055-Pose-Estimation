#!/usr/bin/python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: 02/07/2021
# Description: This script subscribes to the pose constructed by fusing Kinect
#              and BNO055 sensor data and moves an object in gazebo accordingly.

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose

# misc variables
poseTopic = "fused_BNO_Kinect_Pose"
gazeboServiceName = "/gazebo/set_model_state"

def poseCallback(msg):
    """
    This function subscribes to the pose of the detected vision target
    and contructs a model state message from this pose. From here, the
    ModelState message is used to make a rosservice call to move an object in 
    gazebo accordingly

    params:
        msg (geometry_msgs/Pose): The vision target pose message
    returns;
        None
    """
    # wait fo gazebo to come up
    rospy.wait_for_service(gazeboServiceName)

    # constuct modelState message from recieved pose
    stateMsg = ModelState()
    stateMsg.model_name = "simpleBox"
    stateMsg.pose.position.x = msg.position.z
    stateMsg.pose.position.y = msg.position.x
    stateMsg.pose.position.z = msg.position.y
    stateMsg.pose.orientation.x = msg.orientation.x
    stateMsg.pose.orientation.y = msg.orientation.y
    stateMsg.pose.orientation.z = msg.orientation.z
    stateMsg.pose.orientation.w = msg.orientation.w

    # attempt to make ROS service call to change gazebo model Pose
    try:
        setModelState = rospy.ServiceProxy(gazeboServiceName, SetModelState)
        response = setModelState(stateMsg)
    except rospy.ServiceException, err:
        rospy.logerr("service call failed: %s" % err)

def main():
    # init node
    rospy.init_node("set_gazebo_box_pose")
    rospy.loginfo("set_gazebo_box_pose initialized")

    # start subscriber
    rospy.Subscriber(poseTopic, Pose, poseCallback)
    rospy.spin()

if(__name__ == "__main__"):
    main()
