#!/usr/bin/env python3
import sys
import re
import rospy
import roslib
import rosbag
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_conjugate


recording_status = False


def processing(data, topic_name):
    global recording_status
    if data.data == "start record" and not recording_status:
        recording_status = True
        print("start record ", topic_name)
    elif data.data == "stop record":
        recording_status = False
        print("stop record ", topic_name)


def listener(topic_name):
    rospy.init_node('rosbaglistener')
    rospy.Subscriber("rosbag_command", String, processing, topic_name)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener(sys.argv[1])
    except (rospy.ROSInterruptException, IndexError):
        print("Check argument!")
