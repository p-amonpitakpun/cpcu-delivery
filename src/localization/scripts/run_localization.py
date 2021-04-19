#!/usr/bin/env python3

import cv2
import glob
import numpy as np
import os
import pprint
import rospkg
import rospy
import sys
import traceback

from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from threading import Lock, Thread

from modules.PF import ParticleFilter


PACKAGE_PATH = rospkg.RosPack().get_path('localization')

mutex = Lock()

odom_buffer = [0, 0, 0, 0, 0]
odom_msg_prev = [0, 0, 0, 0, 0]
odom_last_update = datetime.now()
odom_init = False

scanner_buffer = [0, 0, 0, 0]
scanner_last_update = datetime.now()

last_odom = None
last_update = None

pf = ParticleFilter(N=100)

real_pose = [0, 0, 0]
real_pose_origin = [0, 0, 0]
real_pose_init = False

def odom_callback(msg):
    global mutex
    global odom_buffer, odom_init, odom_last_update, odom_msg_prev
    global real_pose, real_pose_origin, real_pose_init

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

    if real_pose_init:
        real_pose = [odom_msg[-2] / 100 - real_pose_origin[0], odom_msg[-1] / 100 - real_pose_origin[1], odom_buffer[2] - real_pose_origin[2]]
    else:
        real_pose_origin = [odom_msg[-2] / 100, odom_msg[-1] / 100, odom_buffer[2]]
        real_pose_init = True

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
        # point[2] *= -1
        scanner_buffer.append(point)
    scanner_last_update = now
    mutex.release()


def process_data(odom_data, scanner_data, last_odom):

    new_scan = np.array(scanner_data)[:, 1: 3]

    dl = odom_data[3]
    dr = odom_data[4]

    c = 55.5
    v_ = c * (dl + dr) / 2

    transform = [0, 0, 0]
    if last_odom is not None:
        theta = odom_data[2]
        dtheta = (theta - last_odom[2])

        transform[1] = v_ * np.sin(theta)
        transform[0] = v_ * np.cos(theta)
        transform[2] = dtheta

    return np.array(transform), new_scan, odom_data.copy()


def main():
    cvBridge = CvBridge()

    rospy.init_node('Localization', anonymous=True, log_level=rospy.INFO)

    # Subscribers
    odom_sub = rospy.Subscriber('odomData', Float32MultiArray, odom_callback)
    scanner_sub = rospy.Subscriber(
        'scannerData', Float32MultiArray, scanner_callback)

    # Publisher
    pose_pub = rospy.Publisher('pose', Float32MultiArray, queue_size=5)
    map_pub = rospy.Publisher('map', Image, queue_size=5)

    global delay_ms, last_update
    global odom_buffer, odom_last_update, last_odom, real_pose
    global scanner_buffer, scanner_last_update

    odom_last_calculate = datetime.now()
    scan_last_calculate = datetime.now()

    saved_maps = glob.glob(PACKAGE_PATH + '/../../saves/*.grid.npy')
    saved_config = glob.glob(PACKAGE_PATH + '/../../saves/*.config.json')

    for i, x in enumerate(saved_maps):
        print(i, ':', x)
    map_idx = int(input('choose saved map: ').strip())

    for i,  x in enumerate(saved_config):
        print(i, ':', x)
    config_idx = int(input('choose saved map config: ').strip())

    pf.init(ref_map_path=saved_maps[map_idx],
            ref_map_config_path=saved_config[config_idx])

    # rate = rospy.Rate(10)
    delay_ms = 10

    while not rospy.is_shutdown():
        try:
            now = datetime.now()
            if last_update is None or (now - last_update).total_seconds() * 1e+3 >= delay_ms:
                if odom_last_update > odom_last_calculate and \
                        scanner_last_update > scan_last_calculate:

                    mutex.acquire()
                    odom_data = odom_buffer.copy()
                    scanner_data = scanner_buffer.copy()

                    odom_buffer[3] = 0
                    odom_buffer[4] = 0

                    odom_last_calculate = now
                    scan_last_calculate = now
                    last_update = now
                    mutex.release()

                    transform, new_scan, last_odom = process_data(
                        odom_data, scanner_data, last_odom)
                    pf.update(transform, new_scan)
                    # print('PF updated with \tT:', transform)
                    # print('loc', pf.getLoc())

                    scan = np.zeros((500, 500, 3), dtype=np.uint8)
                    scale = 50
                    scan = cv2.circle(scan, (250, 250), 5, (100, 250, 50), -1)
                    for p in scanner_buffer:
                        scan = cv2.circle(
                            scan, (int(250 - p[2] * scale), int(250 - p[1] * scale)), 2, (0, 125, 255))

                    cv2.imshow('scan', scan)
                    name, img = pf.getImage()
                    if img is not None:
                        cv2.imshow(name, img)
                    cv2.waitKey(1)

                    last_update = datetime.now()

                    pose = pf.getPose()
                    cell = pf.getCell()
                    print(f'\nloc: {pose[: -1]}, dir: {(pose[-1] * 180 / np.pi + 360) % 360}')
                    print(f'cell: {cell}')

                    # Publish
                    try:
                        pose_msg = Float32MultiArray(data=pf.getPose())
                        pose_pub.publish(pose_msg)
                    except CvBridgeError as e:
                        rospy.logerr(e)

                    try:
                        map_msg = cvBridge.cv2_to_imgmsg(pf.getMap())
                        map_pub.publish(map_msg)
                    except CvBridgeError as e:
                        rospy.logerr(e)
                    
        except Exception as e:
            print('  ERROR: ', e)
            traceback.print_exc()

        # loc_pub.publish()
        # rate.sleep()


if __name__ == '__main__':
    main()
