import rospy
from sensor_msgs.msg import LaserScan

import numpy as np

def polar2cartesian(ranges, angles) -> np.ndarray:
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return np.concatenate(x, y).T

def H(R, x):
    h = np.eye(3)
    h[:2, :2] = R
    h[:2, 2] = x
    return h

def R2d(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

def randomly_perturb_scan(scan: LaserScan) -> LaserScan:
    # do something to transform the scan's range values
    # get XY, apply H, return new scan
    new_scan = scan.copy()
    return new_scan

class TestScanPublisher:
    def __init__(self):
        # state
        self.initial_scan = None
        self.new_scan = None

        # callback storage
        self.latest_scan_match = None

        # pub / sub
        self.publisher_scan = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.subscriber_scan_match = rospy.Subscriber("/topic", type, callback=self.callback_scan_match, queue_size=1)

    def callback_scan_match(self, data):
        self.latest_scan_match = data

    def load_scan(self, file_path):
        # load file
        with open(file_path) as f:
            pass

        # save scans
        self.initial_scan = None
        self.new_scan = None

    def publish_scans(self):
        pass

    def main(self):
        rate = rospy.Rate(10.)

        while not rospy.is_shutdown():
            self.publish_scans()
            rate.sleep()
    
if __name__ == '__main__':
    tsp = TestScanPublisher()
    tsp.load_scan("path")
    tsp.main()