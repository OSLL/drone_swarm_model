#!/usr/bin/env python3
import sys
import re
import rospy
import roslib
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_conjugate


def is_number(string):
    return re.fullmatch(r'[-+]?\d+(\.\d+)?(e[+-]\d+)?', string)


def odometry_client(model_name):
    odom = '/gazebo/get_model_state'
    rospy.wait_for_service(odom)
    try:
        odom_s = rospy.ServiceProxy(odom, GetModelState)
        model_state = odom_s(model_name, "")
        position = model_state.pose.position
        orientation = model_state.pose.orientation
        return position, orientation
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def rotate_by_quat(q1, v1):
    q2 = v1 + [0.0]
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))


def translation_coor(x, y, z, roll, pitch, yaw, pos, quaternion_global):
    # глобальные координаты pos quaternion_global тут кватернионы
    # глобальные координаты введенные x y z roll pitch yaw тут эйлеры
    # преобразование в список
    quaternion_global = [quaternion_global.x, quaternion_global.y, quaternion_global.z, quaternion_global.w]
    # перевод глобальных в эйлеры
    roll_global, pitch_global, yaw_global = euler_from_quaternion(quaternion_global)
    # расчет локального поворота
    roll_local, pitch_local, yaw_local = roll - roll_global, pitch - pitch_global, yaw - yaw_global
    # кватернион из локальных
    quaternion_global_in = quaternion_from_euler(roll_local, pitch_local, yaw_local)
    # поворот, который говорит поворот осей
    quaternion_global_new = quaternion_multiply(quaternion_global, quaternion_global_in)
    # расчет координат для новых
    x, y, z = rotate_by_quat(quaternion_global_new, [x, y, z])[:3]
    # расчет координат для текущей
    pos.x, pos.y, pos.z = rotate_by_quat(quaternion_global_new, [pos.x, pos.y, pos.z])[:3]
    return x - pos.x, y - pos.y, z - pos.z, roll_local, pitch_local, yaw_local


if __name__ == '__main__':
    try:
        rospy.init_node('dronecontroller')
        while not rospy.is_shutdown():
            command = input()
            if len(command) == 0:
                continue
            if command.split(" ")[0] not in ['move', 'move_direct', 'rotate', 'rotate_direct', 'translate', 'translate_direct']:
                print("Check command!")
            else:
                command_list = command.split(" ")
                command_list = list(filter(None, command_list))
                if command_list[0] in ['move', 'move_direct'] and len(command_list) == 5 and all(is_number(x) for x in command_list[2:]) or \
                        command_list[0] in ['rotate', 'rotate_direct'] and len(command_list) == 5 and all(is_number(x) for x in command_list[2:]) or \
                        command_list[0] in ['translate', 'translate_direct'] and len(command_list) == 8 and all(is_number(x) for x in command_list[2:]):
                    commands = [float(i) for i in command_list[2:]]
                    if command_list[0] in ['move', 'move_direct']:
                        commands.extend([float(0), ] * 3)
                    elif command_list[0] in ['rotate', 'rotate_direct']:
                        commands = [float(0), ] * 3 + commands
                    x, y, z, roll, pitch, yaw = commands
                    topic_name = command_list[1] + "/cmd_move"
                    pub = rospy.Publisher(topic_name, msg_transposition, queue_size=1)
                    if command_list[0] in ['move_direct', 'rotate_direct', 'translate_direct']:
                        pub.publish(x, y, z, roll, pitch, yaw)
                    else:
                        model_name = command_list[1]
                        position, orientation = odometry_client(model_name)
                        x, y, z, roll, pitch, yaw = translation_coor(x, y, z, roll, pitch, yaw, position, orientation)
                        pub.publish(x, y, z, roll, pitch, yaw)
                else:
                    print("Check arguments!")
    except rospy.ROSInterruptException:
        pass
