#!/usr/bin/env python3
import sys
import re
import subprocess
import signal
import os
import rospy
import roslib
import rosbag
from std_msgs.msg import String
from camera_controls.msg import msg_transposition
from camera_controls.msg import msg_odometry
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_conjugate


class RosbagProcess:
    def __init__(self, drone_name, topics_name):
        self.drone_name = drone_name
        self.topics_name = topics_name
        self.process = None
        self.recording_status = False
        self.count = 0
        self.sync_mode = False
        self.sync_listener_one = None
        self.sync_listener_two = None
        self.msg_type_one = msg_odometry
        self.msg_type_two = msg_odometry
        self.data_one = []
        self.special_topic_name = "/rosbag_sync_topic"

    def processing(self, data):
        if data.data == "start record" and not self.recording_status:
            self.recording_status = True
            command = f"rosbag record -O rosbag{self.count}.bag"
            if len(self.topics_name) == 1:
                command += f" /{self.drone_name}/{self.topics_name[0]}"
            elif len(self.topics_name) == 2:
                self.sync_mode = True
                self.sync_listener()
                command += f" {self.special_topic_name}"
            print(command)
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
            print("start record ", *self.topics_name)
        elif data.data == "stop record" and self.recording_status:
            if self.sync_mode:
                self.sync_listener_one.unregister()
                self.sync_listener_two.unregister()
                self.sync_mode = False
            self.recording_status = False
            self.stop_process(self.process)
            self.count += 1
            print("stop record ", *self.topics_name)

    def sync_processing_one(self, data):
        self.data_one = [data.x, data.y, data.z, data.w]

    def sync_processing_two(self, data):
        if self.data_one:
            pub = rospy.Publisher(self.special_topic_name, self.msg_type_one, queue_size=1)
            pub.publish(*self.data_one)
            pub = rospy.Publisher(self.special_topic_name, self.msg_type_two, queue_size=2)
            pub.publish(data.x, data.y, data.z, data.w)

    def sync_listener(self):
        self.sync_listener_one = rospy.Subscriber(f"{self.drone_name}/{self.topics_name[0]}", self.msg_type_one, self.sync_processing_one)
        self.sync_listener_two = rospy.Subscriber(f"{self.drone_name}/{self.topics_name[1]}", self.msg_type_two, self.sync_processing_two)

    def stop_process(self, process):
        cmd = subprocess.Popen(f"ps -o pid --ppid {process.pid} --noheaders", stdout=subprocess.PIPE, shell=True)
        cmd_out = cmd.stdout.read()
        code = cmd.wait()
        assert code == 0, f"Cmd return {code}"
        cmd_out = cmd_out.decode('utf-8')
        for pid in cmd_out.split("\n")[:-1]:
            os.kill(int(pid), signal.SIGINT)
        process.terminate()

    def listener(self):
        rospy.init_node('rosbaglistener')
        rospy.Subscriber("rosbag_command", String, self.processing)
        rospy.spin()


if __name__ == '__main__':
    try:
        rosbag = RosbagProcess(sys.argv[1], list(sys.argv[2:]))
        rosbag.listener()
    except (rospy.ROSInterruptException, IndexError):
        print("Check argument!")
