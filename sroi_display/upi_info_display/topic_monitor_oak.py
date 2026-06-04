#!/usr/bin/env python3

import rospy
import rostopic
import time
from datetime import datetime
from topic_monitor import TopicMonitor

class TopicMonitoOAK(TopicMonitor):
    def __init__(self):
        super().__init__()

    def define_topics(self):
        self.topics_to_monitor = [
            "/oak/imu/data",
            "/oak/left/image_raw/compressed",
            "/oak/right/image_raw/compressed"
        ]
        print("Defined topics to monitor oak:", self.topics_to_monitor)

if __name__ == '__main__':
    try:
        topic_monitor = TopicMonitoOAK()
        topic_monitor.run()
    except rospy.ROSInterruptException:
        pass