#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np
import csv

def polar2cartesian(ranges, angles) -> np.ndarray:
    ranges = np.array(ranges, dtype=float)
    angles = np.array(angles, dtype=float)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.stack([x, y]).T

def create_fake_scan_message(file_path) -> LaserScan:
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        data = list(reader)
    
    scan = data[0]

    laser_scan = LaserScan()
    laser_scan.header.frame_id = "map"
    laser_scan.angle_min = 0.
    laser_scan.angle_max = 2.*np.pi
    laser_scan.angle_increment = 2.*np.pi / len(scan)
    laser_scan.time_increment = 0.0
    laser_scan.scan_time = 0.0
    laser_scan.range_min = 0.0
    laser_scan.range_max = 10.0
    laser_scan.ranges = np.array(scan, dtype=float)
    laser_scan.intensities = [0.0 for _ in scan]

    return laser_scan

class TestScanPublisher:
    def __init__(self):
        rospy.init_node("test_scan_publisher")

        # state
        self.initial_scan = None
        self.new_scan = None

        # callback storage
        self.latest_scan_match = None

        # pub / sub
        self.publisher_scan = rospy.Publisher("/scan1", LaserScan, queue_size=1)
        self.publisher_scan_match = rospy.Publisher("/scan2", LaserScan, queue_size=1)

    def callback_scan_match(self, data):
        self.latest_scan_match = data

    def load_scans(self, file_paths):
        # load file
        self.initial_scan = create_fake_scan_message(file_paths[0])
        self.new_scan = create_fake_scan_message(file_paths[1])

    def publish_scans(self):
        if self.initial_scan is not None and self.new_scan is not None:
            self.publisher_scan.publish(self.initial_scan)
            self.publisher_scan_match.publish(self.new_scan)

    def main(self):
        rate = rospy.Rate(10.)

        while not rospy.is_shutdown():
            self.publish_scans()
            rate.sleep()
    
if __name__ == '__main__':
    paths = [
        "/nethome/mlamsey3/catkin_ws/src/lidar_locator/data/scan1.csv",
        "/nethome/mlamsey3/catkin_ws/src/lidar_locator/data/scan2.csv"
    ]
    tsp = TestScanPublisher()
    tsp.load_scans(paths)
    tsp.main()