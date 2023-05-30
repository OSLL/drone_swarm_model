#!/usr/bin/env python3
import time

import roslib
import rospy
from camera_controls.msg import msg_transposition
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


# Класс сообщения, которое получает драйвер
class RobotState:
    # Конструктор
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    # Запись в поля класа значений, получаемых из data
    def change_values(self, data):
        self.x = data.x
        self.y = data.y
        self.z = data.z
        self.roll = data.roll
        self.pitch = data.pitch
        self.yaw = data.yaw


# Поворот вектора на кватернион
def rotate_by_quaternion(q1, v1):
    q2 = v1 + [0]

    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))[
        0:3
    ]


# Проверяем, запущена ли симуляция и есть ли в мире дрон с именем drone_name
def is_drone_is_simulation_running(drone_name):
    try:
        gwp = rospy.ServiceProxy("gazebo/get_world_properties", GetWorldProperties)
        world_properties = gwp()
        if world_properties.success:
            if drone_name in world_properties.model_names:
                return True, ""
            else:
                return False, f"Drone named '{drone_name}' is not on the map"
        else:
            return False, "Simulation is not running"
    except rospy.ServiceException as e:
        return False, "Simulation is not running"


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


# Изменение координат робота в соответствии с содержимым msg
def coordinate_transformation(robot_pos, robot_dir, msg):
    # Величины, на которые изменятся координаты дрона
    delta_pos = [x / 60 for x in [msg.x, msg.y, msg.z]]
    # Величины, на которые изменятся углы дрона
    delta_angle = quaternion_from_euler(
        *[x / 60 for x in [msg.roll, msg.pitch, msg.yaw]]
    )

    # Изменение координат дрона
    delta_pos = rotate_by_quaternion(robot_dir, delta_pos)
    robot_pos.x += delta_pos[0]
    robot_pos.y += delta_pos[1]
    robot_pos.z += delta_pos[2]

    # Изменение углов дрона
    robot_dir = quaternion_multiply(delta_angle, robot_dir)

    return robot_pos, robot_dir


# Телепортация дрона в глобальной системе координат
def teleport_drone(robot_pos, robot_dir, drone_name):
    rospy.wait_for_service("gazebo/set_model_state")
    try:
        sms = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = drone_name

        state.pose.position = Point(robot_pos.x, robot_pos.y, robot_pos.z)

        state.pose.orientation = Quaternion(*robot_dir)

        return sms(state)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


# Функция обработки сообщений
def cmd_vel_event(data, msg):
    msg.change_values(data)


def listener():
    # Создание экземпляра сообщения
    msg = RobotState()
    # Если не указан параметр __name, то имя ноды по умолчанию -- 'driver'
    rospy.init_node("driver")

    # Имя дрона без слеша (e.g. 'drone1')
    drone_name = rospy.get_name()[1:]

    rospy.Subscriber(f"{drone_name}/cmd_vel", msg_transposition, cmd_vel_event, msg)

    is_drone_prev = False
    while not rospy.is_shutdown():
        is_drone, err = is_drone_is_simulation_running(drone_name)
        # Если симуляция запущена и дрон с именем drone_name есть на карте, то перемещаем его
        if is_drone:
            if is_drone and not is_drone_prev:
                print(f"'{drone_name}' driver ready to receive messages")
            try:
                now = time.time()
                robot_pos, robot_dir = get_drone_location(drone_name)
                robot_pos, robot_dir = coordinate_transformation(
                    robot_pos, robot_dir, msg
                )
                teleport_drone(robot_pos, robot_dir, drone_name)
                delta_time = time.time() - now
                rospy.sleep(1 / 60 - delta_time)
            except (rospy.ROSInterruptException, TypeError):
                rospy.signal_shutdown("Done")
        else:
            print(err)
            time.sleep(1)
        is_drone_prev = is_drone


if __name__ == "__main__":
    # Создание объекта сообщения
    listener()
