#!/usr/bin/env python3

import cv2
import glob
import numpy as np
import os
import pprint
import rospkg
import rospy
import sched
import sys
import time
import traceback

from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from threading import Lock, Thread

from robot_state.srv import Localization, LocalizationResponse

from modules.PF import ParticleFilter


PACKAGE_PATH = rospkg.RosPack().get_path('localization')


class LocalizationNode():
    def __init__(self):
        # Subscribers
        self.odom_sub = rospy.Subscriber(
            'odomData', Float32MultiArray, self.odom_callback)
        self.scanner_sub = rospy.Subscriber(
            'scannerData', Float32MultiArray, self.scanner_callback)

        # Publisher
        self.pose_pub = rospy.Publisher(
            'pose', Float32MultiArray, queue_size=5)
        self.map_pub = rospy.Publisher('map', Image, queue_size=5)

        # Services
        self.map_srv = rospy.Service(
            'localization', Localization, self.localization_handler)

        self.cvBridge = CvBridge()
        self.mutex = Lock()

        self.odom_buffer = [0, 0, 0, 0, 0]
        self.odom_msg_prev = [0, 0, 0, 0, 0]
        self.odom_last_update = datetime.now()
        self.odom_init = False

        self.scanner_buffer = [0, 0, 0, 0]
        self.scanner_last_update = datetime.now()

        self.odom_last_calculate = datetime.now()
        self.scan_last_calculate = datetime.now()

        self.last_odom = None
        self.last_update = None

        self.pf = ParticleFilter(N=100)

        self.real_pose = [0, 0, 0]
        self.real_pose_origin = [0, 0, 0]
        self.real_pose_init = False

        self.pf_initialized = None

    def begin(self):
        saved_maps = glob.glob(PACKAGE_PATH + '/../../saves/*.grid.npy')
        saved_config = glob.glob(PACKAGE_PATH + '/../../saves/*.config.json')

        for i, x in enumerate(saved_maps):
            print(i, ':', x)
        map_idx = int(input('choose saved map: ').strip())

        for i,  x in enumerate(saved_config):
            print(i, ':', x)
        config_idx = int(input('choose saved map config: ').strip())

        self.is_logging = input('error logging (y / n): ').strip() == 'y'

        self.pf.init(ref_map_path=saved_maps[map_idx],
                     ref_map_config_path=saved_config[config_idx])

        delay_ms = 100

        self.odom_last_calculate = datetime.now()
        self.scan_last_calculate = datetime.now()

        self.start_time = datetime.now()
        timestamp = datetime.timestamp(self.start_time)

        if self.is_logging:
            self.error_log_path = PACKAGE_PATH + f'/logs/error/error_{timestamp}.log.txt'
            with open(self.error_log_path, 'w') as fp:
                fp.write('# info\n')
                fp.write(f'time: {timestamp}')
                fp.write(f'map: {saved_maps[map_idx]}\n')
                fp.write(f'config: {saved_config[config_idx]}\n')
                fp.write('\n')
                fp.write('# data\n')

        self.timer = rospy.Timer(rospy.Duration(secs=0, nsecs=delay_ms * 1000),
                                 self.timer_callback)

    def odom_callback(self, msg):
        odom_msg = list(msg.data)

        self.mutex.acquire()
        now = datetime.now()
        # print('odom', msg.data[3: 5])
        if self.odom_init:
            self.odom_buffer[0: 2] = odom_msg[0: 2]
            self.odom_buffer[2] = odom_msg[2] * np.pi / 180
            self.odom_buffer[3] += odom_msg[3] - self.odom_msg_prev[3]
            self.odom_buffer[4] += odom_msg[4] - self.odom_msg_prev[4]
        self.odom_msg_prev = odom_msg.copy()
        # print(self.odom_buffer)
        self.odom_init = True
        self.odom_last_update = now

        if self.real_pose_init:
            self.real_pose[0] = odom_msg[5] / 100 - self.real_pose_origin[0]
            self.real_pose[1] = odom_msg[6] / 100 - self.real_pose_origin[1]
            self.real_pose[2] = self.odom_buffer[2] - self.real_pose_origin[2]
        else:
            self.real_pose_origin = [
                odom_msg[5] / 100,
                odom_msg[6] / 100,
                self.odom_buffer[2]
            ]
            self.real_pose_init = True

        self.mutex.release()

    def scanner_callback(self, msg):
        self.mutex.acquire()
        now = datetime.now()
        self.scanner_buffer = []
        n = len(msg.data)
        for i in range(n // 3):
            point = list(msg.data[3 * i: 3 * (i + 1)])
            self.scanner_buffer.append(point)
        self.scanner_last_update = now
        self.mutex.release()

    def localization_handler(self, req):
        img = self.cvBridge.cv2_to_imgmsg(self.pf.getMap())
        grid_pos = self.pf.getCell()
        pos = self.pf.getPose()
        pos[2] = (pos[2] * 180 / np.pi + 360) % 360

        return LocalizationResponse(img, grid_pos, pos)

    def process_data(self, odom_data, scanner_data, last_odom):

        new_scan = np.array(scanner_data)[:, 1: 3]

        dl = odom_data[3]
        dr = odom_data[4]

        c = 2.5 * 25
        v_ = c * (dl + dr) / 2

        transform = [0, 0, 0]
        if self.last_odom is not None:
            theta = odom_data[2]
            dtheta = (theta - self.last_odom[2])

            transform[0] = v_ * np.cos(theta)
            transform[1] = v_ * np.sin(theta)
            transform[2] = dtheta

        return np.array(transform), new_scan, odom_data.copy()

    def timer_callback(self, timer):
        try:
            now = datetime.now()
            if self.odom_last_update > self.odom_last_calculate and \
                    self.scanner_last_update > self.scan_last_calculate:

                self.mutex.acquire()
                odom_data = self.odom_buffer.copy()
                scanner_data = self.scanner_buffer.copy()

                self.odom_buffer[3] = 0
                self.odom_buffer[4] = 0

                self.odom_last_calculate = now
                self.scan_last_calculate = now
                self.mutex.release()

                transform, new_scan, self.last_odom = self.process_data(
                    odom_data, scanner_data, self.last_odom)
                self.pf.update(transform, new_scan, real_pose=self.real_pose)

                if self.pf_initialized is None:
                    self.pf_initialized = False
                    print('pf initializing...')

                if self.pf.getState() == 1:
                    if not self.pf_initialized:
                        cv2.destroyAllWindows()
                        self.pf_initialized = True
                        print('pf initialized !')

                    # scan = np.zeros((500, 500, 3), dtype=np.uint8)
                    # scale = 50
                    # scan = cv2.circle(scan, (250, 250), 5, (100, 250, 50), -1)
                    # for p in self.scanner_buffer:
                    #     scan = cv2.circle(
                    #         scan, (int(250 - p[2] * scale), int(250 - p[1] * scale)), 2, (0, 125, 255))
                    # cv2.imshow('scan', scan)

                    name, img = self.pf.getImage()
                    if img is not None:
                        cv2.imshow(name, img)
                    cv2.waitKey(1)

                    pose = self.pf.getPose()
                    cell = self.pf.getCell()

                    print()
                    # print(f'dt: {(now - self.last_update).total_seconds() * 1000}')
                    print(
                        f'loc:\t {pose[0]:0.06f},\t {pose[1]:0.06f}\t dir:\t {(np.rad2deg(pose[2]) + 360) % 360}')
                    # print(f'cell: {cell}')
                    print(
                        f'real:\t {self.real_pose[0]:0.6f},\t {self.real_pose[1]:0.6f}\t dir:\t {(np.rad2deg(self.real_pose[2]) + 360) % 360}')
                    print(
                        f'err:\t {self.real_pose[0] - pose[0]:0.6f}\t {self.real_pose[1] - pose[1]:0.6f}\t dir_err:\t {(np.rad2deg(self.real_pose[2] - pose[2]) + 360) % 360:0.6f}')
                    
                    if self.is_logging:
                        with open(self.error_log_path, 'a') as fp:
                            fp.write('{},\t {},\t {},\t {}\n'.format(
                                datetime.timestamp(now),
                                self.real_pose[0] - pose[0],
                                self.real_pose[1] - pose[1],
                                self.real_pose[2] - pose[2]
                            ))

                    # Publish
                    try:
                        pose_data = np.concatenate([pose, cell])
                        pose_msg = Float32MultiArray(data=pose_data)
                        self.pose_pub.publish(pose_msg)
                    except CvBridgeError as e:
                        rospy.logerr(e)

                    try:
                        map_msg = self.cvBridge.cv2_to_imgmsg(self.pf.getMap())
                        self.map_pub.publish(map_msg)
                    except CvBridgeError as e:
                        rospy.logerr(e)

                self.last_update = datetime.now()

        except Exception as e:
            print('  ERROR: ', e)
            traceback.print_exc()


def main():

    rospy.init_node('Localization', anonymous=True, log_level=rospy.INFO)

    try:
        node = LocalizationNode()
        node.begin()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)


if __name__ == '__main__':
    main()
