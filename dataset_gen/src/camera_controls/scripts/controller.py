#!/usr/bin/env python3
import sys
import rospy
import roslib
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_conjugate
from geometry_msgs.msg import Quaternion


def my_is_digit(string):
    if string.isdigit():
        return True
    else:
        try:
            float(string)
            return True
        except ValueError:
            return False


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


def quaternion_mult(q1, q2):
    q1 = [q1.x, q1.y, q1.z, q1.w]
    q2 = [q2.x, q2.y, q2.z, q2.w]
    return quaternion_multiply(q1, q2)


def quatvect_mult(q1, v1):
    q2 = v1 + [0.0]
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))


def translation_coor(x, y, z, roll, pitch, yaw, pos, quaternion_global):
    # глобальные координаты pos orient тут кватернионы
    # глобальные координаты введенные x y z roll pitch yaw тут эйлеры
    # кватернион из глобальных введенных
    quaternion_global_in = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
    # поворот, который говорит поворот осей
    quaternion_global = quaternion_mult(quaternion_global, quaternion_global_in)
    # расчет локальных координат
    x, y, z = quatvect_mult(quaternion_global, [x, y, z])[:3]
    # перевод углов в эйлеры
    roll, pitch, yaw = euler_from_quaternion(quaternion_global)
    return x, y, z, roll, pitch, yaw


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
                if command_list[0] in ['move', 'move_direct'] and len(command_list) == 5 and all(my_is_digit(x) for x in command_list[2:]) or \
                        command_list[0] in ['rotate', 'rotate_direct'] and len(command_list) == 5 and all(my_is_digit(x) for x in command_list[2:]) or \
                        command_list[0] in ['translate', 'translate_direct'] and len(command_list) == 8 and all(my_is_digit(x) for x in command_list[2:]):
                    commands = [float(i) for i in command_list[2:]]
                    if command_list[0] in ['move', 'move_direct']:
                        commands.extend([float(0), ] * 3)
                    elif command_list[0] in ['rotate', 'rotate_direct']:
                        commands = [float(0), ] * 3 + commands
                    x, y, z, roll, pitch, yaw = commands
                    topic_name = command_list[1] + "_driver"
                    pub = rospy.Publisher(topic_name, msg_transposition, queue_size=1)
                    if command_list[0] in ['move_direct', 'rotate_direct', 'translate_direct']:
                        pub.publish(x, y, z, roll, pitch, yaw)
                    else:
                        model_name = command_list[1]
                        position, orientation = odometry_client(model_name)
                        # print(position, orientation)
                        x, y, z, roll, pitch, yaw = translation_coor(x, y, z, roll, pitch, yaw, position, orientation)
                        # print(x, y, z, roll, pitch, yaw)
                        pub.publish(x, y, z, roll, pitch, yaw)
                else:
                    print("Check arguments!")
    except rospy.ROSInterruptException:
        pass
