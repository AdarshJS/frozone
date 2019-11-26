#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def callback(data):
    # rospy.loginfo(" %s",data.data)
    print("Inside Callback")
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    orientation_list = [x,y,z,w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print("Yaw = {}".format(yaw))



if __name__ == '__main__':
    rospy.init_node('Model_State_Sub')
    rospy.Subscriber("/gazebo/model_states", ModelState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
