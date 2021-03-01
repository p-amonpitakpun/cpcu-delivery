#!/usr/bin/env python3

import cv2
import glob
import json
import matplotlib.pyplot as plt
import numpy as np
import os
import pprint
import rospkg
import rospy
import sys

from cv_bridge import CvBridge
from datetime import datetime
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from threading import Lock, Thread

from modules.graph_slam import GraphSLAM
from modules.log_processing import compute
from modules.icp.icp import icp


pp = pprint.PrettyPrinter(indent=4)
mutex = Lock()

PACKAGE_PATH = rospkg.RosPack().get_path('graph_mapping')
LOG_PATH = '/logs/mapping_logs/'

odom_buffer = [0, 0, 0, 0, 0]
odom_msg_prev = [0, 0, 0, 0, 0]
odom_last_update = datetime.now()
odom_init = False
last_odom = None

scanner_buffer = [0, 0, 0, 0]
scanner_last_update = datetime.now()

delay_ms = 100
mapping_log = {
    'starttime': None,
    'delay_ms': delay_ms,
    'data': []
}
is_running = False
last_update = None


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


def thread_function():
    global is_running, delay_ms, mutex, last_update
    global mapping_log
    global odom_buffer, odom_last_update
    global scanner_buffer, scanner_last_update

    odom_last_calculate = datetime.now()
    scan_last_calculate = datetime.now()

    print('  Thread: started')

    mapping_log['starttime'] = datetime.timestamp(datetime.now())

    while not rospy.is_shutdown() and is_running:
        try:
            now = datetime.now()
            if last_update is None or (now - last_update).total_seconds() * 1e+3 >= delay_ms:
                if odom_last_update > odom_last_calculate and \
                        scanner_last_update > scan_last_calculate:

                    data = {
                        'timestamp': datetime.timestamp(now),
                        'odom': odom_buffer.copy(),
                        'scanner': scanner_buffer.copy()
                    }
                    print('  Thread: append data ', data['timestamp'])
                    mapping_log['data'].append(data)

                    mutex.acquire()
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
    print('  Thread: stopped')


def main():

    global is_running, last_update
    global mapping_log

    rospy.init_node('Mapping', anonymous=True)

    odom_sub = rospy.Subscriber('odomData', Float32MultiArray, odom_callback)
    scanner_sub = rospy.Subscriber(
        'scannerData', Float32MultiArray, scanner_callback)

    thread = Thread(target=thread_function)

    print('  Mapping')
    print('  control:')
    print('  - Q: start')
    print('  - W: stop&save')
    print('  - E: exit')

    while not rospy.is_shutdown():
        print('  time:', datetime.now())
        print('  input: ', end='')
        ans = input().strip().lower()

        if ans == 'q':
            is_running = True
            thread.start()

        elif ans == 'w':
            is_running = False
            thread = Thread(target=thread_function)
            filepath = PACKAGE_PATH + LOG_PATH + \
                'mapping_log-{}.json'.format(mapping_log['starttime'])
            with open(filepath, 'w') as fp:
                json.dump(mapping_log, fp, indent=4)
            print('  Mapping: saved at', filepath)

        elif ans == 'r':
            logs = glob.glob(PACKAGE_PATH + LOG_PATH +
                             'mapping_log/mapping_log-*.json')
            print('  Mapping: compute from log')
            for i, logpath in enumerate(logs):
                print('  [{}]'.format(i), logpath)
            ans2 = input('  answer: ').strip()
            try:
                i = int(ans2)

                if i < len(logs):
                    log = None
                    with open(logs[i], 'r') as fp:
                        log = json.load(fp)
                    graph = compute(log)

            except ValueError as e:
                print('  Error: ', e)

        elif ans == 'e':
            sys.exit()

    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
