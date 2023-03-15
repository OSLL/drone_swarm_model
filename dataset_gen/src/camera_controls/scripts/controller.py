#!/usr/bin/env python3
import sys
import rospy
import roslib
from std_msgs.msg import String
from camera_controls.msg import msg_transposition


def my_is_digit(string):
    if string.isdigit():
        return True
    else:
        try:
            float(string)
            return True
        except ValueError:
            return False


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('drivercontroller', msg_transposition, queue_size=1)
        rospy.init_node('dronecontroller')
        while not rospy.is_shutdown():
            command = input()
            if len(command) == 0:
                continue
            if command.split(" ")[0] not in ['move', 'rotate', 'translate']:
                print("Check command!")
            else:
                command_list = command.split(" ")
                command_list = list(filter(None, command_list))
                if command_list[0] == 'move' and len(command_list) == 5 and all(my_is_digit(x) for x in command_list[2:]) or \
                        command_list[0] == 'rotate' and len(command_list) == 5 and all(my_is_digit(x) for x in command_list[2:]) or \
                        command_list[0] == 'translate' and len(command_list) == 8 and all(my_is_digit(x) for x in command_list[2:]):
                    commands = [float(i) for i in command_list[2:]]
                    if command_list[0] == 'move':
                        commands.extend([float(0), ] * 3)
                    elif command_list[0] == 'rotate':
                        commands = [float(0), ] * 3 + commands
                    x, y, z, roll, pitch, yaw = commands
                    pub.publish(x, y, z, roll, pitch, yaw)
                else:
                    print("Check arguments!")
    except rospy.ROSInterruptException:
        pass
