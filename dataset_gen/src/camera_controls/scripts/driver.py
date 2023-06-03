#!/usr/bin/env python3
import time

import rosgraph
import roslib
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SetModelState
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
from tf.transformations import (
    euler_from_quaternion,
    quaternion_conjugate,
    quaternion_from_euler,
    quaternion_multiply,
    unit_vector,
)
from camera_controls.msg import msg_transposition


# Определяем, запущена ли симуляция
def wait_for_gazebo():
    while not rospy.is_shutdown():
        try:
            gwp = rospy.ServiceProxy("gazebo/get_world_properties", GetWorldProperties)
            world_properties = gwp()
            return world_properties
        except rospy.ServiceException:
            print("Simulation is not running")
            time.sleep(1)
        except (rospy.ROSInterruptException, TypeError):
            break
    return None


# Определяем, есть ли дрон с именем drone_name на карте
def wait_for_drone(world_properties, drone_name):
    while not rospy.is_shutdown():
        if world_properties.success:
            if drone_name in world_properties.model_names:
                return
            print(world_properties.model_names)
            print(f"Drone named '{drone_name}' is not on the map")
            time.sleep(1)


# Поворот вектора на кватернион
def rotate_by_quaternion(q1, v1):
    q2 = v1 + [0]

    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))[
        0:3
    ]


# Функция обработки сообщений
def cmd_move_event(data, drone_name):
    robot_pos, robot_dir = get_drone_location(drone_name)
    # Преобразование координат, data -- локальные значения, robot_pos/robot_dir -- глобальные
    data = coordinate_transformation(data, robot_pos, robot_dir)
    # Телепортация дрона
    teleport_drone(data, drone_name)


# Преобразование координат и углов
def coordinate_transformation(data, robot_pos, robot_dir):
    # Локальные координаты робота
    rec_pos = [data.x, data.y, data.z]
    # Локальный кватернион
    rec_dir = quaternion_from_euler(data.roll, data.pitch, data.yaw)

    # Поворот вектора rec_pos на кватернион robot_dir
    delta_pos = rotate_by_quaternion(robot_dir, rec_pos)

    # Результирующее положение робота
    robot_pos.x += delta_pos[0]
    robot_pos.y += delta_pos[1]
    robot_pos.z += delta_pos[2]

    # Результирующие углы
    robot_dir = quaternion_multiply(rec_dir, robot_dir)

    data.x = robot_pos.x
    data.y = robot_pos.y
    data.z = robot_pos.z
    data.roll, data.pitch, data.yaw = euler_from_quaternion(
        [robot_dir[0], robot_dir[1], robot_dir[2], robot_dir[3]]
    )
    return data


# Получить текущее положение робота до начала перемещения
def get_drone_location(drone_name):
    rospy.wait_for_service("gazebo/get_model_state")
    try:
        gms = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        model_state = gms(drone_name, "")
        position = model_state.pose.position
        orientation = [
            model_state.pose.orientation.x,
            model_state.pose.orientation.y,
            model_state.pose.orientation.z,
            model_state.pose.orientation.w,
        ]

        return position, orientation
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return None


# Телепортация дрона в глобальной системе координат
def teleport_drone(data, drone_name):
    rospy.wait_for_service("gazebo/set_model_state")
    try:
        sms = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = drone_name

        state.pose.position = Point(data.x, data.y, data.z)

        state.pose.orientation = Quaternion(
            *quaternion_from_euler(data.roll, data.pitch, data.yaw)
        )

        return sms(state)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return None


def listener():
    # Если не указан параметр __name, то имя ноды по-умолчанию -- 'driver'
    rospy.init_node("driver")

    world_properties = wait_for_gazebo()

    # Имя дрона без слеша (e.g. 'drone1')
    drone_name = rospy.get_name()[1:]

    wait_for_drone(world_properties, drone_name)
    print(f"'{drone_name}' driver ready to receive messages")

    rospy.Subscriber(
        f"{drone_name}/cmd_move", msg_transposition, cmd_move_event, drone_name
    )
    rospy.spin()


if __name__ == "__main__":
    listener()
