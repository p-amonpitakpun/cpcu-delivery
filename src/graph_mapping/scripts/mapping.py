#!/usr/bin/env python

import cv2
import message_filters
import numpy as np
import rospy

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from time import time_ns

from modules.graph_slam import GraphSLAM
from modules.icp.icp import icp


odom_buffer = [0, 0, 0, 0, 0]
odom_last_update = 0
last_odom = None

scanner_buffer = []
scanner_last_update = 0

graphSLAM = GraphSLAM()

cv2.namedWindow('plot')


def odom_callback(msg):
    global odom_buffer, odom_last_update

    now = time_ns()
    # print('odom', msg.data, now)
    odom_buffer[0: 3] = msg.data[0: 3]
    odom_buffer[3] += msg.data[3]
    odom_buffer[4] += msg.data[4]
    odom_last_update = now


def scanner_callback(msg):
    global scanner_buffer, scanner_last_update

    now = time_ns()
    # print('scanner', len(msg.data), now)
    scanner_buffer = []
    n = len(msg.data)
    for i in range(n // 3):
        point = msg.data[3 * i: 3 * (i + 1)]
        scanner_buffer.append(point)
    scanner_last_update = now


def process_data(odom_data, scanner_data):
    global graphSLAM, last_odom

    new_scan = np.array(scanner_data)[:, : 2]

    dl = odom_data[3]
    dr = odom_data[4]
    c = 0.1
    v_ = c * (dl + dr) / 2

    transform = [0, 0, 0]
    if last_odom is not None:
        theta = odom_data[2]
        dtheta = theta - last_odom[2]
        
        transform[0] = v_ * np.sin(theta)
        transform[1] = v_ * np.cos(theta)
        transform[2] = dtheta
        

    graphSLAM.mapping(transform, new_scan)

    last_odom = odom_data


def main():
    rospy.init_node('mapping', anonymous=True)

    odom_sub = rospy.Subscriber('odomData', Float32MultiArray, odom_callback)
    scanner_sub = rospy.Subscriber(
        'scannerData', Float32MultiArray, scanner_callback)

    print('running')

    last_update = 0
    odom_last_calculate = 0
    scan_last_calculate = 0
    delay = 1500e+6

    odom_data = None
    scanner_data = None

    global graphSLAM
    global odom_buffer, odom_last_update
    global scanner_buffer, scanner_last_update


    while not rospy.is_shutdown():
        now = time_ns()
        if now - last_update >= delay:
            if odom_last_update > odom_last_calculate and scanner_last_update > scan_last_calculate:
                print('calculate', now - last_update,
                      odom_buffer, len(scanner_buffer))

                odom_data = odom_buffer.copy()
                scanner_data = scanner_buffer.copy()

                process_data(odom_data, scanner_data)
                points = graphSLAM.getVertexPoints()

                odom_buffer[3] = 0
                odom_buffer[4] = 0

                odom_last_calculate = now
                scan_last_calculate = now
                last_update = now

                print(len(points))
                plot = np.zeros((500, 500, 3), dtype=np.uint8)
                for i, p in enumerate(points):
                    plot = cv2.circle(plot, (int(p[0] * 100 + 250), int(p[1] * 100 + 250)), 2, (0, 0, 255))

                cv2.imshow('plot', plot)
                cv2.waitKey(1)

    rospy.spin()


if __name__ == "__main__":
    print('started')
    main()
