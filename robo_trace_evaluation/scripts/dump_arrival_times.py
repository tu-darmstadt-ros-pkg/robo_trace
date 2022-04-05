#!/usr/bin/env python3

import rospy
import csv
import os
import argparse
import time

CSV_HEADER = ["topic", "time"]
CSV_BUFFER_SIZE = 1
QUERY_NEW_TOPICS_DELAY = 0.02

class ArrivalTimeListener():

    def __init__(self, topic, writer):
        
        self.topic = topic
        self.writer = writer
        self.subscrber = rospy.Subscriber(
            name=self.topic,
            data_class=rospy.msg.AnyMsg,
            callback=self.on_message_received,
            queue_size=100000
        )

    def on_message_received(self, _):

        arrival_time = time.time()

        self.writer.writerow(
            [self.topic, arrival_time]
        )

class ArrivalTimeDumper():

    def __init__(self, file):

        print("Setting up node ...")

        self.listeners = []

       
        print(" + creating csv writer.")
        self.file_name = file
        os.makedirs(os.path.dirname(self.file_name), exist_ok=True)
        print(" + file is " + self.file_name)
       

        self.handle = open(
            file = self.file_name, 
            mode='w+',
            encoding="UTF8"
        )
        self.writer = csv.writer(self.handle)

        # The CSV header
        self.writer.writerow(CSV_HEADER)

        print(" + creating update timer.")

        self.updater = rospy.Timer(
            period=rospy.Duration(QUERY_NEW_TOPICS_DELAY),
            callback=self.update_active_topics
        )

        print("Setup completed.")

    def is_topic_beeing_recorderd(self, topic):
        return any(map(lambda listener: listener.topic == topic, self.listeners))

    def update_active_topics(self, event):

        topics = rospy.get_published_topics()

        for topic, type in topics:
            
            if self.is_topic_beeing_recorderd(topic):
                continue
            
            print(" + recording: " + topic)

            self.listeners.append(
                ArrivalTimeListener(
                    # topic
                    topic,
                    # writer
                    self.writer
                )
            )

if __name__ == '__main__':

    rospy.init_node('arrival_times_dumper', anonymous=True)
    
    parser = argparse.ArgumentParser(
        description="Records timestamps for messages to a CSV file."
    )
    parser.add_argument("--file", 
        dest="file", 
        type=str, 
        default="timings.csv",
        help="The file name to dump to"
    )

    args = parser.parse_args() 

    dumper = ArrivalTimeDumper(
        file=args.file
    )

    rospy.spin()
