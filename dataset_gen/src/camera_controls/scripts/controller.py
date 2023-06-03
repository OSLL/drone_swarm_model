#!/usr/bin/env python3

import roslib
import rospy
from camera_controls.msg import msg_transposition
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
from tf.transformations import (
    euler_from_quaternion,
    quaternion_conjugate,
    quaternion_from_euler,
    quaternion_multiply,
)


def odometry_client(model_name):
    try:
        odom = "/gazebo/get_model_state"
        odom_s = rospy.ServiceProxy(odom, GetModelState)
        model_state = odom_s(model_name, "")
        if model_state.success is False:
            raise rospy.ServiceException
        position = model_state.pose.position
        orientation = model_state.pose.orientation
        return position, orientation
    except rospy.ServiceException:
        print(f"Service odometry call failed for drone with name {model_name}: \
            Check drone or simulation!")
        return None


def rotate_by_quat(q1, v1):
    q2 = v1 + [0.0]
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))


def translation_coor(x, y, z, roll, pitch, yaw, pos, quaternion_global):
    # глобальные координаты pos quaternion_global тут кватернионы
    # глобальные координаты введенные x y z roll pitch yaw тут эйлеры
    # преобразование в список
    quaternion_global = [
        quaternion_global.x,
        quaternion_global.y,
        quaternion_global.z,
        quaternion_global.w,
    ]
    # перевод глобальных в эйлеры
    roll_global, pitch_global, yaw_global = euler_from_quaternion(quaternion_global)
    # расчет локального поворота
    roll_local, pitch_local, yaw_local = (
        roll - roll_global,
        pitch - pitch_global,
        yaw - yaw_global,
    )
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
        except KeyError as exc:
            print(f"Wrong command! - {name}")
            raise AttributeError from exc

    def publish(self, x, y, z, roll, pitch, yaw):
        pub = rospy.Publisher(self.topic_name, msg_transposition, queue_size=1)
        pub.publish(x, y, z, roll, pitch, yaw)


def command(description):
    def __command_wrapper(func):
        name = func.__name__
        Handler.commands[name] = {"func": func, "description": description}
        return func

    return __command_wrapper


class GlobalStorage(Handler):
    @command("move drone to x y z in global coordinates")
    def move(self, x, y, z):
        position, orientation = odometry_client(self.topic_name.split("/")[0])
        x, y, z, roll, pitch, yaw = translation_coor(
            x, y, z, 0.0, 0.0, 0.0, position, orientation
        )
        self.publish(x, y, z, roll, pitch, yaw)

    @command("move drone to x y z in local coordinates")
    def move_direct(self, x, y, z):
        self.publish(x, y, z, 0.0, 0.0, 0.0)

    @command("rotate drone by roll pitch yaw in global coordinates")
    def rotate(self, roll, pitch, yaw):
        position, orientation = odometry_client(self.topic_name.split("/")[0])
        x, y, z, roll, pitch, yaw = translation_coor(
            0.0, 0.0, 0.0, roll, pitch, yaw, position, orientation
        )
        self.publish(x, y, z, roll, pitch, yaw)

    @command("rotate drone by roll pitch yaw in local coordinates")
    def rotate_direct(self, roll, pitch, yaw):
        self.publish(0.0, 0.0, 0.0, roll, pitch, yaw)

    @command(
        "move drone to x y z and rotate drone by roll pitch yaw in global coordinates"
    )
    def translate(self, x, y, z, roll, pitch, yaw):
        position, orientation = odometry_client(self.topic_name.split("/")[0])
        x, y, z, roll, pitch, yaw = translation_coor(
            x, y, z, roll, pitch, yaw, position, orientation
        )
        self.publish(x, y, z, roll, pitch, yaw)

    @command(
        "move drone to x y z and rotate drone by roll pitch yaw in local coordinates"
    )
    def translate_direct(self, x, y, z, roll, pitch, yaw):
        self.publish(x, y, z, roll, pitch, yaw)

    @command("prints help")
    def help(self):
        for command, content in self.commands.items():
            print(f"{command}: {content['description']}")


if __name__ == "__main__":
    try:
        rospy.init_node("dronecontroller")
        global_storage = GlobalStorage()
        global_storage.help()
        while not rospy.is_shutdown():
            command = input()
            if len(command) == 0:
                continue
            c, *args = command.split(" ")
            if c == "exit":
                break
            try:
                global_storage.run_command(c, args[0], *map(float, args[1:]))
            except (AttributeError, TypeError, ValueError, IndexError):
                print("Check commands!")
                global_storage.help()
    except rospy.ROSInterruptException:
        pass
