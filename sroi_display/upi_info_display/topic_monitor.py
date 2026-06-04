#!/usr/bin/env python3

import rospy
import rostopic
import time
from datetime import datetime

class TopicMonitor:
    def __init__(self):
        rospy.init_node('topic_monitor', anonymous=True)
        self.last_message_times = {}
        self.hz_values = {}
        self.write_to_file = True
        self.file_path = "/home/codes/rosbags/recording_status.txt"
        if self.write_to_file:
            with open(self.file_path, "w") as f:
                f.write("No topic info")
                
    def subscribe_topics(self):
        for topic in self.topics_to_monitor:
            self.last_message_times[topic] = None
            self.hz_values[topic] = 0.0

            # Subscribe to topics (even if we don't process the messages)
            rospy.Subscriber(topic, rospy.AnyMsg, self.message_callback, callback_args=topic)
        self.print_timer = rospy.Timer(rospy.Duration(1.0), self.print_hz)  # Print every 1 second
    
    def define_topics(self):
        self.topics_to_monitor = [
            "/camera/infra1/image_rect_raw/compressed",
            "/camera/infra2/image_rect_raw/compressed",
            "/camera/color/image_raw/compressed",
            "/camera/imu"
        ]
        print("Defined topics to monitor:", self.topics_to_monitor)

    def message_callback(self, msg, topic):
        current_time = rospy.Time.now()

        if self.last_message_times[topic] is not None:
            time_diff = (current_time - self.last_message_times[topic]).to_sec()
            if time_diff > 0: # avoid division by zero
                self.hz_values[topic] = 1.0 / time_diff
        else:
            rospy.loginfo(f"Topic {topic} started publishing.")

        self.last_message_times[topic] = current_time

    def print_hz(self, event):
        rospy.loginfo("--- Topic Frequencies ---")
        all_topics_publishing = True
        for topic in self.topics_to_monitor:
            if self.last_message_times[topic] is None:
                rospy.logwarn(f"Topic {topic} is NOT publishing.")
                all_topics_publishing = False
            else:
                rospy.loginfo(f"{topic}: {self.hz_values[topic]:.2f} Hz")
        if all_topics_publishing:
            rospy.loginfo("All monitored topics are publishing.")
        rospy.loginfo("-------------------------")

        if self.write_to_file:
            with open(self.file_path, "w") as f:
                current_time = datetime.now().strftime("%M:%S")
                f.write(f"{current_time}: ")
                for topic in self.topics_to_monitor:
                    if "imu" in topic:
                        f.write(f"IMU:{self.hz_values[topic]:.0f} ")
                    if "infra1" in topic or "left" in topic:
                        f.write(f"IL:{self.hz_values[topic]:.0f} ") 
                    if "infra2" in topic or "right" in topic:
                        f.write(f"IR:{self.hz_values[topic]:.0f} ")
                    if "color" in topic:
                        f.write(f"CL:{self.hz_values[topic]:.0f} ") 

                print("Wrote to file:", self.file_path)

    def run(self):
        self.define_topics()
        self.subscribe_topics()
        rospy.spin()

if __name__ == '__main__':
    try:
        topic_monitor = TopicMonitor()
        topic_monitor.run()
    except rospy.ROSInterruptException:
        pass