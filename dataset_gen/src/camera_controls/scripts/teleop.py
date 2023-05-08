#!/usr/bin/env python3
import roslib
import rospy
import sys
import time
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_conjugate, \
    unit_vector
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion, Point
from gazebo_msgs.srv import SetModelState, GetModelState, GetWorldProperties
from select import select
import termios
import tty
import subprocess


# Получение нажатой клавиши
def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class RobotState:
    # Конструктор
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.delta = 0.1

    def __str__(self):
        return ' '.join(map(str, [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]))

    def increase_x(self):
        self.x += self.delta

    def decrease_x(self):
        self.x -= self.delta

    def increase_y(self):
        self.y += self.delta

    def decrease_y(self):
        self.y -= self.delta

    def increase_z(self):
        self.z += self.delta

    def decrease_z(self):
        self.z -= self.delta

    def increase_pitch(self):
        self.pitch += self.delta

    def decrease_pitch(self):
        self.pitch -= self.delta

    def increase_roll(self):
        self.roll += self.delta

    def decrease_roll(self):
        self.roll -= self.delta

    def increase_yaw(self):
        self.yaw += self.delta

    def decrease_yaw(self):
        self.yaw -= self.delta

def float_to_string(num):
    sign = '+'
    if num < 0:
        sign = ''
    return sign + f'{num:.4f}'

def print_UI(msg):
    print(f"""                                                              U
                     B    W                                   ↑ 
           [{float_to_string(msg.z)}] |   / [{float_to_string(msg.x)}]                         |│  /
                     |  /                      [{float_to_string(msg.yaw)}] O ←──┘ / 
                     | /                                     | /
                     |/                                      |/    ┌→ K [{float_to_string(msg.pitch)}] 
            A -------*------- D [{float_to_string(msg.y)}]             -------*-----|-
                    /|                                      /|     ↓           
                   / |                      [{float_to_string(msg.roll)}] J ←──┐ |     I          
                  /  |                                    /│ |                
                 /   |                                   / ↓ |                
                S  SPACE                                   L
                """)

def talker():
    msg = RobotState()

    settings = termios.tcgetattr(sys.stdin)
    key_timeout = 0.5

    hot_keys = {'w': lambda message: message.increase_x(),
                's': lambda message: message.decrease_x(),
                'd': lambda message: message.increase_y(),
                'a': lambda message: message.decrease_y(),
                'b': lambda message: message.increase_z(),
                ' ': lambda message: message.decrease_z(),
                'i': lambda message: message.increase_pitch(),
                'k': lambda message: message.decrease_pitch(),
                'l': lambda message: message.increase_roll(),
                'j': lambda message: message.decrease_roll(),
                'o': lambda message: message.increase_yaw(),
                'u': lambda message: message.decrease_yaw()}

    rospy.init_node('teleop')
    # Имя дрона без слеша (e.g. 'drone1')
    drone_name = rospy.get_name()[1:-7]

    pub = rospy.Publisher(f'{drone_name}/cmd_vel', msg_transposition, queue_size=4)

    subprocess.call('clear', stdin=True, shell=True)
    print_UI(msg)

    while not rospy.is_shutdown():
        key = getKey(settings, key_timeout)
        if key == '\x03':
            sys.exit(0)
        if key:
            subprocess.call('clear', stdin=True, shell=True)
            if key in hot_keys:
                hot_keys[key](msg)
                print_UI(msg)
                pub.publish(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
