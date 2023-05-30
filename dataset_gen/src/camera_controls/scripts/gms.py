#!/usr/bin/env python

import sys

import rospy
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion


def gms_client(model_name, relative_entity_name):
    gms_service = "/gazebo/get_model_state"
    rospy.wait_for_service(gms_service)
    try:
        gms = rospy.ServiceProxy(gms_service, GetModelState)
        model_state = gms(model_name, relative_entity_name)
        position = model_state.pose.position
        orientation = euler_from_quaternion(
            [
                model_state.pose.orientation.x,
                model_state.pose.orientation.y,
                model_state.pose.orientation.z,
                model_state.pose.orientation.w,
            ]
        )
        return position, orientation
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def usage():
    return f"{sys.argv[0]} [model_name]"


if __name__ == "__main__":
    if len(sys.argv) == 2:
        model_name = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)
    print(f"Requesting {model_name}")
    position, orientation = gms_client(model_name, "")
    print(position)
    print("rpy:", orientation)
