#!/usr/bin/env python3
import sys
import rospy
import roslib
from std_msgs.msg import String


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
        pub = rospy.Publisher('drivercontroller', String, queue_size=1)
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
                    pub.publish(command)
                else:
                    print("Check arguments!")
    except rospy.ROSInterruptException:
        pass
