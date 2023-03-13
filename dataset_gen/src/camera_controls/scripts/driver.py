#!/usr/bin/env python3
import roslib
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_name() + " ПОЛУЧИЛ " + data.data)

def listener():
    rospy.init_node('driver')
    rospy.Subscriber('drivercontroller', String, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

