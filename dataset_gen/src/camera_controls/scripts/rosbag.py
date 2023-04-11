#!/usr/bin/env python3
import sys
import re
import rospy
import roslib
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_conjugate


def processing(data):
    pass


def listener(topic_name):
    rospy.init_node('rosbaglistener')
    rospy.Subscriber(topic_name, msg_transposition, processing)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener(sys.argv[1])
    except (rospy.ROSInterruptException, IndexError):
        print("Check argument!")
