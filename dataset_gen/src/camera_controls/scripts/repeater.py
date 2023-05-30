#!/usr/bin/env python

import sys

import rospy
from sensor_msgs.msg import Image


def repeater(inp, out):
    rospy.init_node("repeater")
    pub = rospy.Publisher(out, Image, queue_size=10)

    def callback(msg):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

    sub = rospy.Subscriber(inp, Image, callback)
    rospy.spin()


def usage():
    return f"{sys.argv[0]} [input topic] [output topic]"


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 3:
        inp = args[1]
        out = args[2]
    else:
        print(usage())
        sys.exit(1)
    repeater(inp, out)
