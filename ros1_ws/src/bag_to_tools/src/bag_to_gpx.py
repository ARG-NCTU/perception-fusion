#!/usr/bin/env python3

import rospy
import rospkg
import rosbag
import gpxpy
import gpxpy.gpx
import os
from tqdm import tqdm

rospack = rospkg.RosPack()

def bag_to_gpx(bag_file, gpx_file):
    gpx = gpxpy.gpx.GPX()

    rospy.loginfo(f"Processing bag file: {bag_file}")
    with rosbag.Bag(bag_file) as bag:
        track = gpxpy.gpx.GPXTrack()
        gpx.tracks.append(track)
        segment = gpxpy.gpx.GPXTrackSegment()
        track.segments.append(segment)

        for topic, msg, t in tqdm(bag.read_messages(topics=['/mavros/global_position/global']), desc='Converting', leave=False):
            gpx_point = gpxpy.gpx.GPXTrackPoint(
                latitude=msg.latitude,
                longitude=msg.longitude,
                elevation=msg.altitude,
                time=None  # Optionally add timestamp here
            )
            segment.points.append(gpx_point)

    with open(gpx_file, mode='w') as file:
        file.write(gpx.to_xml())

    rospy.loginfo(f"GPX file created: {gpx_file}")

if __name__ == "__main__":
    rospy.init_node('bag_to_gpx_node')
    bag_file = rospy.get_param('~bag_file', None)
    bag_file = os.path.join(rospack.get_path('bag_to_tools'), bag_file)
    gpx_file = rospy.get_param('~gpx_file', None)
    gpx_file = os.path.join(rospack.get_path('bag_to_tools'), gpx_file)

    if not os.path.exists(bag_file):
        rospy.logerr(f"Bag file not found: {bag_file}")
        exit(1)

    bag_to_gpx(bag_file, gpx_file)
