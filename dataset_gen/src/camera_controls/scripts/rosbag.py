#!/usr/bin/env python3
import sys
import re
import subprocess
import signal
import os
import rospy
import roslib
import rosbag
import rostopic
import message_filters
from std_msgs.msg import String


class RosbagProcess:
    def __init__(self, drone_name, topics_name):
        self.drone_name = drone_name
        self.topics_name = topics_name
        self.full_topics_name = [f"/{self.drone_name}/{topics_name[i]}" for i in range(len(topics_name))]
        self.process = None
        self.recording_status = False
        self.count = 0
        self.sync_mode = False
        self.sync_one = None
        self.sync_two = None
        self.sync_listener_both = None
        self.msg_types = [rostopic.get_topic_class(i)[0] for i in self.full_topics_name]
        self.special_topic_name = "/rosbag_sync_topic"

    def processing(self, data):
        if data.data == "start record" and not self.recording_status:
            self.recording_status = True
            command = f"rosbag record -O rosbag{self.count}.bag"
            if len(self.topics_name) == 1:
                command += f" {self.full_topics_name[0]}"
            elif len(self.topics_name) == 2:
                self.sync_mode = True
                self.sync_listener()
                command += f" {self.special_topic_name}"
            print(command)
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
            print(f"start record", *self.full_topics_name)
        elif data.data == "stop record" and self.recording_status:
            if self.sync_mode:
                self.sync_one.unregister()
                self.sync_two.unregister()
                self.sync_mode = False
            self.recording_status = False
            self.stop_process(self.process)
            self.count += 1
            print(f"stop record", *self.full_topics_name)

    def sync_processing_both(self, data1, data2):
        pub = rospy.Publisher(self.special_topic_name, self.msg_types[0], queue_size=1)
        pub.publish(data1)
        pub = rospy.Publisher(self.special_topic_name, self.msg_types[1], queue_size=2)
        pub.publish(data2)

    def sync_listener(self):
        self.sync_one = message_filters.Subscriber(f"{self.full_topics_name[0]}", self.msg_types[0])
        self.sync_two = message_filters.Subscriber(f"{self.full_topics_name[1]}", self.msg_types[1])
        self.sync_listener_both = message_filters.ApproximateTimeSynchronizer([self.sync_one, self.sync_two], 10, 1, allow_headerless=True)
        self.sync_listener_both.registerCallback(self.sync_processing_both)

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
