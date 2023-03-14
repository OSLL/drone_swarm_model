#!/usr/bin/env python3
import roslib
import rospy
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, Point
from gazebo_msgs.srv import SetModelState, GetModelState


def callback(data):
    rospy.loginfo(
        f"\nx = {data.x},\ny = {data.y},\nz = {data.z},\nroll = {data.roll},\npitch = {data.pitch}, \nyaw = {data.yaw}")
    # Получение текущей позиции и ориентации дрона
    position, orientation = get_drone_location('drone1')
    # Преобразование координат
    data = count_new_drone_location(data, position, orientation)
    # Телепортация дрона
    teleport_drone(data)

# Вычисление положения робота в глобальной системе координат после перемещения
def count_new_drone_location(data, position, orientation):
    data.x += position.x
    data.y += position.y
    data.z += position.z
    data.roll += orientation[0]
    data.pitch += orientation[1]
    data.yaw += orientation[2]
    return data

# Получить текущее положение робота до начала перемещения
def get_drone_location(name_drone):
    rospy.wait_for_service('gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        model_state = gms(name_drone, '')
        position = model_state.pose.position
        orientation = euler_from_quaternion([model_state.pose.orientation.x,
                                             model_state.pose.orientation.y,
                                             model_state.pose.orientation.z,
                                             model_state.pose.orientation.w])
        return position, orientation
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

# Телепортация дрона в глобальной системе координат
def teleport_drone(data):
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

