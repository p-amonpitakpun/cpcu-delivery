#!/usr/bin/env python

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pprint
import rospy

from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from threading import Lock
from time import time_ns

from modules.graph_slam import GraphSLAM
from modules.icp.icp import icp


pp = pprint.PrettyPrinter(indent=4)
mutex = Lock()

odom_buffer = [0, 0, 0, 0, 0]
odom_msg_prev = [0, 0, 0, 0, 0]
odom_last_update = 0
odom_init = False
last_odom = None

scanner_buffer = [0, 0, 0, 0]
scanner_last_update = 0

graphSLAM = GraphSLAM()

odom_hist_l = []
odom_hist_r = []


def odom_callback(msg):
    global mutex
    global odom_buffer, odom_init, odom_last_update, odom_msg_prev

    odom_msg = list(msg.data)

    mutex.acquire()
    now = time_ns()
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
    now = time_ns()
    # print('scanner', len(msg.data), now)
    scanner_buffer = []
    n = len(msg.data)
    for i in range(n // 3):
        point = list(msg.data[3 * i: 3 * (i + 1)])
        point[2] *= -1
        scanner_buffer.append(point)
    scanner_last_update = now
    mutex.release()


def process_data(odom_data, scanner_data):
    global graphSLAM, last_odom, odom_hist_l, odom_hist_r

    new_scan = np.array(scanner_data)[:, 1: 3]

    dl = odom_data[3] #- (0.28 if odom_data[3] > 0 else 0)
    dr = odom_data[4] #- (0.48 if odom_data[3] > 0 else 0)

    odom_hist_l.append(dl)
    odom_hist_r.append(dr)

    c = 1E+2
    v_ = c * (dl + dr) / 2

    transform = [0, 0, 0]
    if last_odom is not None:
        theta = odom_data[2]
        dtheta = theta - last_odom[2]

        # print(v_)

        transform[0] = v_ * np.sin(theta)
        transform[1] = v_ * np.cos(theta)
        transform[2] = dtheta

    print('>>  transf\t', transform)
    graphSLAM.mapping(transform, new_scan)

    last_odom = odom_data.copy()


def main():
    rospy.init_node('mapping', anonymous=True)

    odom_sub = rospy.Subscriber('odomData', Float32MultiArray, odom_callback)
    scanner_sub = rospy.Subscriber(
        'scannerData', Float32MultiArray, scanner_callback)

    print('running')

    last_update = 0
    odom_last_calculate = 0
    scan_last_calculate = 0
    delay = 500e+3

    odom_data = None
    scanner_data = None

    global graphSLAM
    global odom_buffer, odom_last_update
    global scanner_buffer, scanner_last_update

    start_time_ns = time_ns()

    while not rospy.is_shutdown():
        now = time_ns()
        if now - last_update >= delay:
            if odom_last_update > odom_last_calculate and scanner_last_update > scan_last_calculate:
                # print('calculate', now - last_update,
                #       odom_buffer, len(scanner_buffer))


                print(odom_buffer[3: 5])
                process_data(odom_buffer, scanner_buffer)

                mutex.acquire()
                odom_buffer[3] = 0
                odom_buffer[4] = 0

                odom_last_calculate = now
                scan_last_calculate = now
                last_update = now
                mutex.release()

                points = graphSLAM.getVertexPoints()
                print('Vertices', len(points))
                # for i, p in enumerate(points):
                #     print(i, p)

                edges = graphSLAM.getEdges()
                print('Edges', len(edges))
                # for e in edges.values():
                #     # print(e)
                #     print(e.from_x, e.to_x, e.dx, '\t',
                #           e.z.T[0], '\t', e.z.T[0] / e.dx)

                # print()

                plot = np.zeros((500, 500, 3), dtype=np.uint8)
                scan = np.zeros((500, 500, 3), dtype=np.uint8)
                scale = 50
                for i, p in enumerate(points):
                    # plot = cv2.circle(
                    #     plot, (int(p[0] * scale + 250), int(250 - p[1] * scale)), 3, (105, 105, 105))
                    if i > 0:
                        plot = cv2.line(
                            plot,
                            (int(points[i - 1][0] * scale + 250),
                             int(250 - points[i - 1][1] * scale)),
                            (int(p[0] * scale + 250), int(250 - p[1] * scale)),
                            (0, 0, 150),
                            2
                        )
                scan = cv2.circle(scan, (250, 250), 5, (100, 250, 50), -1)
                for p in scanner_buffer:
                    scan = cv2.circle(
                        scan, (int(250 + p[2] * scale), int(250 - p[1] * scale)), 2, (0, 125, 255))

                # cv2.imshow('plot', plot)
                # cv2.imshow('scan', scan)
                cv2.imshow('all', cv2.hconcat([plot, scan]))
                cv2.waitKey(1)


                if now - start_time_ns > 30E+9:
                    global odom_hist_l, odom_hist_r
                    odom_l = odom_hist_l
                    odom_r = odom_hist_r
                    # odom_x = [x for x in odom_hist_x if np.fabs(x) > 0.01]
                    # odom_y = [y for y in odom_hist_y if np.fabs(y) > 0.01]

                    # print(min(odom_l), '\t', max(odom_l))
                    # print(min(odom_r), '\t', max(odom_r))

                    # n_odom = min(len(odom_l), len(odom_r))
                    # plt.plot(range(n_odom), odom_l[- n_odom: ], c='r')
                    # plt.plot(range(n_odom), odom_r[- n_odom: ], c='cyan')
                    # plt.show()

                # scanarr = np.array(scanner_buffer)
                # plt.scatter(scanarr[:, 2], scanarr[:, 1])
                # plt.ylim(-2.5, 2.5)
                # plt.xlim(-2.5, 2.5)
                # plt.show()

    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print('started')
    main()
