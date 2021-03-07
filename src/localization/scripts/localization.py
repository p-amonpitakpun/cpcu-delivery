#!/usr/bin/env python

import cv2
import glob
import numpy as np
import os
import pprint
import rospkg
import rospy
import sys

from datetime import datetime
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from threading import Lock, Thread

from modules.icp import icp
from modules.PF import ParticleFilter


odom_buffer = [0, 0, 0, 0, 0]
odom_msg_prev = [0, 0, 0, 0, 0]
odom_last_update = datetime.now()
odom_init = False

scanner_buffer = [0, 0, 0, 0]
scanner_last_update = datetime.now()

pf = ParticleFilter()


def odom_callback(msg):
    global mutex
    global odom_buffer, odom_init, odom_last_update, odom_msg_prev

    odom_msg = list(msg.data)

    mutex.acquire()
    now = datetime.now()
    # print('odom', msg.data[3: 5])
    if odom_init:
        odom_buffer[0: 2] = odom_msg[0: 2]
        odom_buffer[2] = odom_msg[2] * np.pi / 180
        odom_buffer[3] += odom_msg[3] - odom_msg_prev[3]
        odom_buffer[4] += odom_msg[4] - odom_msg_prev[4]
    odom_msg_prev = odom_msg.copy()
    # print(odom_buffer)
    odom_init = True
    odom_last_update = now
    mutex.release()


def scanner_callback(msg):
    global scanner_buffer, scanner_last_update

    mutex.acquire()
    now = datetime.now()
    # print('scanner', len(msg.data), now)
    scanner_buffer = []
    n = len(msg.data)
    for i in range(n // 3):
        point = list(msg.data[3 * i: 3 * (i + 1)])
        point[2] *= -1
        scanner_buffer.append(point)
    scanner_last_update = now
    mutex.release()


def main():
    rospy.init_node('Localization', anonymous=True, log_level=rospy.INFO)

    odom_sub = rospy.Subscriber('odomData', Float32MultiArray, odom_callback)
    scanner_sub = rospy.Subscriber(
        'scannerData', Float32MultiArray, scanner_callback)

    loc_pub = rospy.Publisher('location', Float32MultiArray, queue_size=5)

    global delay_ms, last_update
    global odom_buffer, odom_last_update
    global scanner_buffer, scanner_last_update

    odom_last_calculate = datetime.now()
    scan_last_calculate = datetime.now()

    print('  Thread: started')

    mapping_log['starttime'] = datetime.timestamp(datetime.now())

    while not rospy.is_shutdown():
        try:
            now = datetime.now()
            if last_update is None or (now - last_update).total_seconds() * 1e+3 >= delay_ms:
                if odom_last_update > odom_last_calculate and \
                        scanner_last_update > scan_last_calculate:

                    mutex.acquire()
                    odom_data = odom_buffer.copy(),
                    scanner_data = scanner_buffer.copy()

                    odom_buffer[3] = 0
                    odom_buffer[4] = 0

                    odom_last_calculate = now
                    scan_last_calculate = now
                    last_update = now
                    mutex.release()



                    scan = np.zeros((500, 500, 3), dtype=np.uint8)
                    scale = 50
                    scan = cv2.circle(scan, (250, 250), 5, (100, 250, 50), -1)
                    for p in scanner_buffer:
                        scan = cv2.circle(
                            scan, (int(250 + p[2] * scale), int(250 - p[1] * scale)), 2, (0, 125, 255))

                    cv2.imshow('scan', scan)
                    cv2.waitKey(1)
        except Exception as e:
            print('  ERROR: ', e)


if __name__ == '__main__':
    main()
