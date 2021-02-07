#!/usr/bin/env python

import cv2
import message_filters
import rospy

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

from .modules.graph_slam import GraphSLAM


def main():
    rospy.init_node('mapping', anonymous=True)

    graph = GraphSLAM()

    odometry_sub = message_filters.Subscriber('odometry', None)  # TODO
    laser_scanner_sub = message_filters.Subscriber(
        'laser_scanner', Float32MultiArray)

    time_sync = message_filters.TimeSynchronizer(
        [odometry_sub, laser_scanner_sub], 1)
    time_sync.registerCallback(graph.mapping)

    cv2.namedWindow('graph')

    while not rospy.is_shutdown():
        img = graph.getImage()
        if img is not None:
            cv2.imshow('graph', img)
            cv2.waitKey(3)

    rospy.spin()


if __name__ == "__main__":
    main()
