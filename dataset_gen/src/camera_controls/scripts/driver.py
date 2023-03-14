#!/usr/bin/env python3
import roslib
import rospy
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, Point
from gazebo_msgs.srv import SetModelState


def callback(data):
    rospy.loginfo(f"\nx = {data.x},\ny = {data.y},\nz = {data.z},\nroll = {data.roll},\npitch = {data.pitch}, \nyaw = {data.yaw}")
    rospy.wait_for_service('gazebo/set_model_state')
    try:
        sms = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = 'drone1'

        state.pose.position = Point(data.x, data.y, data.z)

        state.pose.orientation = Quaternion(*quaternion_from_euler(data.roll, data.pitch, data.yaw))
        return sms(state)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def listener():
    rospy.init_node('driver')
    rospy.Subscriber('drivercontroller', msg_transposition, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

