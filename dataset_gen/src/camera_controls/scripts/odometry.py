#!/usr/bin/env python3
import time

import roslib
import rospy
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from camera_controls.msg import msg_odometry, msg_transposition


def odometry_processing():
    drone_names = []
    try:
        world_s = rospy.ServiceProxy("gazebo/get_world_properties", GetWorldProperties)
        properties = world_s()
        if properties.success:
            drone_names = list(filter(lambda x: "drone" in x, properties.model_names))
        else:
            print("Simulation is not running!")
    except rospy.ServiceException:
        print("Simulation is not running!")
    odom = "/gazebo/get_model_state"
    try:
        odom_s = rospy.ServiceProxy(odom, GetModelState)
        for drone_name in drone_names:
            drone_state = odom_s(drone_name, "")
            orientation = drone_state.pose.orientation
            pub = rospy.Publisher(drone_name + "/odom", msg_odometry, queue_size=1)
            pub.publish(orientation.x, orientation.y, orientation.z, orientation.w)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


if __name__ == "__main__":
    try:
        rospy.init_node("odometrypublisher")
        while not rospy.is_shutdown():
            odometry_processing()
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass
