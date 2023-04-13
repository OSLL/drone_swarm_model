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
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_conjugate


class RosbagProcess:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.process = None
        self.recording_status = False
        self.count = 0

    def processing(self, data):
        if data.data == "start record" and not self.recording_status:
            self.recording_status = True
            command = f"rosbag record -O rosbag{self.count}.bag /{self.topic_name}"
            print(command)
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
            print("start record ", self.topic_name)
        elif data.data == "stop record" and self.recording_status:
            self.recording_status = False
            self.stop_process(self.process)
            self.count += 1
            print("stop record ", self.topic_name)

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
        rosbag = RosbagProcess(sys.argv[1])
        rosbag.listener()
    except (rospy.ROSInterruptException, IndexError):
        print("Check argument!")
