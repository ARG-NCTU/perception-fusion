#!/usr/bin/env python3

import rospy
import rospkg
import rosbag
import csv
import os

rospack = rospkg.RosPack()

def bag_to_csv(bag_file, csv_file):
    rospy.loginfo(f"Processing bag file: {bag_file}")
    with rosbag.Bag(bag_file) as bag:
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])  # CSV header

            for topic, msg, t in tqdm(bag.read_messages(topics=['/mavros/global_position/global']), desc='Converting', leave=False):
                writer.writerow([t.to_sec(), msg.latitude, msg.longitude, msg.altitude])

    rospy.loginfo(f"CSV file created: {csv_file}")

if __name__ == "__main__":
    rospy.init_node('bag_to_csv_node')
    bag_file = rospy.get_param('~bag_file', None)
    bag_file = os.path.join(rospack.get_path('bag_to_tools'), bag_file)
    csv_file = rospy.get_param('~csv_file', None)
    csv_file = os.path.join(rospack.get_path('bag_to_tools'), csv_file)

    if not os.path.exists(bag_file):
        rospy.logerr(f"Bag file not found: {bag_file}")
        exit(1)

    bag_to_csv(bag_file, csv_file)
