#!/usr/bin/env python3
import sys
import re
import rospy
import roslib
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_conjugate


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


class Handler:
    commands = {}
    topic_name = ""

    def run_command(self, name, topic, *args):
        try:
            self.topic_name = topic + "/cmd_move"
            return self.commands[name]["func"](self, *args)
        except KeyError as e:
            print(f"Wrong command! - {name}")
            raise AttributeError

    def publish(self, x, y, z, roll, pitch, yaw):
        pub = rospy.Publisher(self.topic_name, msg_transposition, queue_size=1)
        pub.publish(x, y, z, roll, pitch, yaw)


def command(description):
    def __command_wrapper(func):
        name = func.__name__
        Handler.commands[name] = {
            "func": func,
            "description": description
        }
        return func
    return __command_wrapper


class GlobalStorage(Handler):
    @command("move drone to x y z in global coordinates")
    def move(self, x, y, z):
        position, orientation = odometry_client(self.topic_name.split("/")[0])
        x, y, z, roll, pitch, yaw = translation_coor(x, y, z, 0.0, 0.0, 0.0, position, orientation)
        self.publish(x, y, z, roll, pitch, yaw)

    @command("move drone to x y z in local coordinates")
    def move_direct(self, x, y, z):
        self.publish(x, y, z, 0.0, 0.0, 0.0)

    @command("rotate drone by roll pitch yaw in global coordinates")
    def rotate(self, roll, pitch, yaw):
        position, orientation = odometry_client(self.topic_name.split("/")[0])
        x, y, z, roll, pitch, yaw = translation_coor(0.0, 0.0, 0.0, roll, pitch, yaw, position, orientation)
        self.publish(x, y, z, roll, pitch, yaw)

    @command("rotate drone by roll pitch yaw in local coordinates")
    def rotate_direct(self, roll, pitch, yaw):
        self.publish(0.0, 0.0, 0.0, roll, pitch, yaw)

    @command("move drone to x y z and rotate drone by roll pitch yaw in global coordinates")
    def translate(self, x, y, z, roll, pitch, yaw):
        position, orientation = odometry_client(self.topic_name.split("/")[0])
        x, y, z, roll, pitch, yaw = translation_coor(x, y, z, roll, pitch, yaw, position, orientation)
        self.publish(x, y, z, roll, pitch, yaw)

    @command("move drone to x y z and rotate drone by roll pitch yaw in local coordinates")
    def translate_direct(self, x, y, z, roll, pitch, yaw):
        self.publish(x, y, z, roll, pitch, yaw)

    @command("prints help")
    def help(self):
        for command in self.commands:
            print(f"{command}: {self.commands[command]['description']}")


if __name__ == '__main__':
    try:
        rospy.init_node('dronecontroller')
        global_storage = GlobalStorage()
        while not rospy.is_shutdown():
            command = input()
            if len(command) == 0:
                continue
            else:
                c, *args = command.split(" ")
                try:
                    global_storage.run_command(c, args[0], *map(float, args[1:]))
                except (AttributeError, TypeError, ValueError, IndexError) as e:
                    print("Check commands!")
                    global_storage.help()
    except rospy.ROSInterruptException:
        pass
