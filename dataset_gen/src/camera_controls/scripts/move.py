#!/usr/bin/env python

import sys

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler


def move_client(model_name, position, orientation):
    sms_service = "/gazebo/set_model_state"
    rospy.wait_for_service(sms_service)
    try:
        sms = rospy.ServiceProxy(sms_service, SetModelState)

        state = ModelState()
        state.model_name = model_name

        state.pose.position = Point(*position)

        state.pose.orientation = Quaternion(*quaternion_from_euler(*orientation))

        return sms(state)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def usage():
    return f"{sys.argv[0]} [model_name] [x] [y] [z] [roll] [pitch] [yaw]"


if __name__ == "__main__":
    if len(sys.argv) != 8:
        print(usage())
        sys.exit(1)
    model_name = sys.argv[1]
    x, y, z, roll, pitch, yaw = map(float, sys.argv[2:])
    print(f"Moving {model_name} to XYZ=({x},{y},{z}), RPY=({roll},{pitch},{yaw})")
    res = move_client(model_name, (x, y, z), (roll, pitch, yaw))
    print(res)
