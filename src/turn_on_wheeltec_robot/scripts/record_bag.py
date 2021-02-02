#!/usr/bin/env python

from __future__ import print_function
import subprocess
import signal
import time
import rospy
from std_msgs.msg import UInt8


class HandleListener:
    def __init__(self):
        self.saver_process = None
        self.cmd= 'rosbag record -a -b 0'
    def callback(self, handle_key_msg):
        handle_key = handle_key_msg.data
        # rospy.loginfo("key pressed: %d", handle_key)
        if handle_key == 13 and self.saver_process is None:
            self.saver_process = subprocess.Popen(self.cmd, shell=True, executable="/bin/bash")
            rospy.loginfo("start recording...")
            # self.saver_process = 1
        if handle_key == 14 and self.saver_process is not None:
            self.saver_process.send_signal(signal.SIGINT)
            self.saver_process=None
            rospy.loginfo("stop recording...")

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/HandleKey", UInt8, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener = HandleListener()
    listener.listener()
