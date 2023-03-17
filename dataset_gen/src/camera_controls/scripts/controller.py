#!/usr/bin/env python3
import sys
import rospy
import roslib
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion


def my_is_digit(string):
    if string.isdigit():
        return True
    else:
        try:
            float(string)
            return True
        except ValueError:
            return False


def odometry_client(model_name, relative_entity_name):
    odom = '/gazebo/get_model_state'
    rospy.wait_for_service(odom)
    try:
        odom_s = rospy.ServiceProxy(odom, GetModelState)
        model_state = odom_s(model_name, relative_entity_name)
        position = model_state.pose.position
        orientation = euler_from_quaternion([model_state.pose.orientation.x,
                                             model_state.pose.orientation.y,
                                             model_state.pose.orientation.z,
                                             model_state.pose.orientation.w])
        return position, orientation
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


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
                    model_name = command_list[1]
                    position, orientation = odometry_client(model_name, '')
                    print(position)
                    print('rpy:', orientation)
                    topic_name = command_list[1] + "_driver"
                    pub = rospy.Publisher(topic_name, msg_transposition, queue_size=1)
                    if command_list[0] in ['move_direct', 'rotate_direct', 'translate_direct']:
                        pub.publish(x, y, z, roll, pitch, yaw)
                    else:
                        pub.publish(x, y, z, roll, pitch, yaw)
                else:
                    print("Check arguments!")
    except rospy.ROSInterruptException:
        pass
