#!/usr/bin/env python3
import roslib
import rospy
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply,quaternion_conjugate, unit_vector
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, Point
from gazebo_msgs.srv import SetModelState, GetModelState

# Поворот вектора через кватернион
def qv_mult(q1, v1):
    q2 = [v1[0], v1[1], v1[2], 0]
    q1 = [q1.x, q1.y, q1.z, q1.w]

    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )[0:3]

# Умножение кватернионов друг на друга
def qq_mult(q1,q2):
    q1 = [q1.x , q1.y , q1.z , q1.w]
    q2 = [q2.x , q2.y , q2.z , q2.w]
    return quaternion_multiply(q1, q2)

# Функция обработки сообщений
def callback(data):
    robot_pos, robot_dir = get_drone_location('drone1')
    # Преобразование координат, data -- локальные значения, robot_pos/robot_dir -- глобальные
    data = coordinate_transformation(data, robot_pos, robot_dir)
    # Телепортация дрона
    teleport_drone(data)

# Преобразование координат и углов
def coordinate_transformation(data, robot_pos, robot_dir):
    # Локальные координаты робота
    rec_pos = [data.x, data.y, data.z]
    # Локальный кватернион
    rec_dir = Quaternion(*quaternion_from_euler(data.roll, data.pitch, data.yaw))

    # Поворот вектора rec_pos на кватернион robot_dir
    delta_pos = qv_mult(robot_dir, rec_pos)

    # Результирующее положение робота
    robot_pos.x += delta_pos[0]
    robot_pos.y += delta_pos[1]
    robot_pos.z += delta_pos[2]
    
    # Результирующие углы
    robot_dir = qq_mult(rec_dir, robot_dir)

    data.x = robot_pos.x
    data.y = robot_pos.y
    data.z = robot_pos.z
    data.roll, data.pitch, data.yaw = euler_from_quaternion([robot_dir[0],
                                                             robot_dir[1],
                                                             robot_dir[2],
                                                             robot_dir[3]])
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
