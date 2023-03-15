#!/usr/bin/env python3
import roslib
import rospy
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply,quaternion_conjugate, unit_vector
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, Point
from gazebo_msgs.srv import SetModelState, GetModelState

# Функция обработки сообщений
def callback(data):
    #rospy.loginfo(
     #   f"\nx = {data.x},\ny = {data.y},\nz = {data.z},\nroll = {data.roll},\npitch = {data.pitch}, \nyaw = {data.yaw}")
    # Получение текущей позиции и ориентации дрона в глобальной системе координат
    rec_pos, rec_dir = get_drone_location('drone1')

    # Преобразование координат, data -- локальные значения, rec_pos/rec_dir -- глобальные
    data = coordinate_transformation(data, rec_pos, rec_dir)
    # Телепортация дрона
    teleport_drone(data)

# Поворот вектора через кватернион
def qv_mult(q1, v1):
    q2 = [v1.x, v1.y, v1.z, 0]
    q1 = [q1.x, q1.y, q1.z, q1.w]

    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[:3]

# Преобразование координат и углов. rec_dir -- кватернион
def coordinate_transformation(data, rec_pos, rec_dir):
    # Локальные координаты робота
    robot_pos = [data.x, data.y, data.z]
    # Локальный кватернион
    robot_dir = Quaternion(*quaternion_from_euler(data.roll, data.pitch, data.yaw))

    # Поворот вектора rec_pos на кватернион robot_dir
    delta_pos = qv_mult(robot_dir, rec_pos)
    # Результирующее положение робота
    robot_pos = list(map(sum, zip(delta_pos,robot_pos)))
    # Результирующие углы
    robot_dir.x += rec_dir.x
    robot_dir.y += rec_dir.y
    robot_dir.z += rec_dir.z
    robot_dir.w += rec_dir.w

    data.x = robot_pos[0]
    data.y = robot_pos[1]
    data.z = robot_pos[2]
    data.roll, data.pitch, data.yaw = euler_from_quaternion([robot_dir.x,
                                                             robot_dir.y,
                                                             robot_dir.z,
                                                             robot_dir.w])
    return data

# Получить текущее положение робота до начала перемещения
def get_drone_location(name_drone):
    rospy.wait_for_service('gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        model_state = gms(name_drone, '')
        position = model_state.pose.position
        orientation = model_state.pose.orientation

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
