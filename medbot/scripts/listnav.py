#!/usr/bin/env python3

import rospy, math
from std_msgs.msg import String
import std_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
import matplotlib.pyplot as plt
import random
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


def movebase_client(data):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if data == 'Stanza 1':
        goal.target_pose.pose.position.x = 2.3
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0.0
	    #roll 0, pitch 0 , yaw 1.57
        quat = quaternion_from_euler(0, 0, 1.57079)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
 

    elif data == 'Stanza 2':
        goal.target_pose.pose.position.x = -2.5
        goal.target_pose.pose.position.y = -9
         #roll 0, pitch 0 , yaw 0
        quat = quaternion_from_euler(0, 0, 0.00001)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

    elif data == 'Reception':
        goal.target_pose.pose.position.x = -2.5
        goal.target_pose.pose.position.y = 3.5
         #roll 0, pitch 0 , yaw 1.57
        quat = quaternion_from_euler(0, 0, 1.57079)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]


    elif data == 'Ricarica':
        goal.target_pose.pose.position.x = -7.2
        goal.target_pose.pose.position.y = -1
        #roll 0, pitch 0 , yaw 1.57
        quat = quaternion_from_euler(0, 0, 1.57079)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]



    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def callback(data):
    result = movebase_client(data.data)
    if result:
        rospy.loginfo("Goal execution done!")

def listener():
    rospy.Subscriber('/robot/command', String, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('nav_goal', anonymous=True)
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
