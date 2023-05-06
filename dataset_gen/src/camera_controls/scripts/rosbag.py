#!/usr/bin/env python3
import sys
import re
import subprocess
import signal
import os
import time
import rospy
import roslib
import rosbag
import rostopic
import message_filters
from std_msgs.msg import String


class Data:
    def __init__(self, data):
        self.data = data


class RosbagProcess:
    def __init__(self, drone_name, topics_name):
        self.drone_name = drone_name
        self.topics_name = topics_name
        self.full_topics_name = [f"/{self.drone_name}/{topics_name[i]}" for i in range(len(topics_name))]
        self.command = f"rosbag record"
        self.process = None
        self.recording_status = False
        self.count = 0
        self.during = False
        self.sync_mode = False
        self.sync_one = None
        self.sync_two = None
        self.sync_listener_both = None
        self.msg_types = [rostopic.get_topic_class(i)[0] for i in self.full_topics_name]
        self.timer_count = 0
        self.available_flag = [False, False]
        self.first_time = True
        self.check_availability_topics()
        self.first_time = False
        self.special_topic_name = "/rosbag_sync_topic"

    def check_availability_topics(self):
        for i in range(len(self.full_topics_name)):
            try:
                a = rostopic.get_info_text(self.full_topics_name[i])
                self.available_flag[i] = True
                if len(self.full_topics_name) == 1:
                    self.available_flag[1] = True
            except rostopic.ROSTopicException:
                self.available_flag[i] = False
                if len(self.full_topics_name) == 1:
                    self.available_flag[1] = False
        while False in self.available_flag:
            if self.timer_count == 0:
                print("Waiting for ", end="", flush=True)
                for i in [i for i, e in enumerate(self.available_flag) if e == False]:
                    if len(self.full_topics_name) == 1:
                        print(self.full_topics_name[0], end=" ", flush=True)
                        break
                    else:
                        print(self.full_topics_name[i], end=" ", flush=True)
                print("publishers", end="", flush=True)
            print(".", end="", flush=True)
            self.timer_count += 1
            if self.timer_count == 10:
                self.timer_count = 0
                print("There is no publishers for topics: ", end="", flush=True)
                for i in [i for i, e in enumerate(self.available_flag) if e == False]:
                    if len(self.full_topics_name) == 1:
                        print(self.full_topics_name[0], end=" ", flush=True)
                        break
                    else:
                        print(self.full_topics_name[i], end=" ", flush=True)
                print()
                if self.first_time:
                    exit()
                return False
            time.sleep(1)
            for i in range(len(self.full_topics_name)):
                try:
                    a = rostopic.get_info_text(self.full_topics_name[i])
                    self.available_flag[i] = True
                    if len(self.full_topics_name) == 1:
                        self.available_flag[1] = True
                except rostopic.ROSTopicException:
                    self.available_flag[i] = False
                    if len(self.full_topics_name) == 1:
                        self.available_flag[1] = False
        return True

    def processing(self, data):
        if data.data == "start record" and not self.recording_status:
            if self.check_availability_topics():
                self.start_record()
        elif data.data == "stop record" and self.recording_status:
            self.stop_record()
        elif re.fullmatch(r'record during \d+', data.data) and not self.during and not self.recording_status:
            if self.check_availability_topics():
                self.during = True
                time_r = data.data.split()[2]
                self.command += f" --duration={time_r}"
                self.start_record()
                time.sleep(int(time_r) + 1)
                self.stop_record()
                self.during = False

    def start_record(self):
        self.recording_status = True
        self.command += f" -O rosbag{self.count}.bag"
        if len(self.topics_name) == 1:
            self.command += f" {self.full_topics_name[0]}"
        elif len(self.topics_name) == 2:
            self.sync_mode = True
            self.sync_listener()
            self.command += f" {self.special_topic_name}"
        print("\n" + self.command)
        self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE, shell=True)
        print(f"start record", *self.full_topics_name)

    def stop_record(self):
        if self.sync_mode:
            self.sync_one.unregister()
            self.sync_two.unregister()
            self.sync_mode = False
        self.recording_status = False
        if not self.during:
            self.stop_process(self.process)
        self.count += 1
        self.command = f"rosbag record"
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
        rosbag.listener_command_line()
        rospy.spin()

    def listener_command_line(self):
        while not rospy.is_shutdown():
            data = Data(input())
            self.processing(data)


if __name__ == '__main__':
    try:
        topic_check = sys.argv[2]
        rosbag = RosbagProcess(sys.argv[1], list(sys.argv[2:]))
        print("\nstart record: enable recording")
        print("stop record: disable recording")
        print("record during n: enable recording during n seconds")
        rosbag.listener()
    except (rospy.ROSInterruptException, IndexError):
        print("Check argument!")
