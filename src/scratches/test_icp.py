import glob
import json
import numpy as np
import os

from datetime import datetime

from graph_mapping.scripts.modules.icp.icp import icp


dir_path = os.path.dirname(os.path.realpath(__file__))
paths = glob.glob(dir_path + '/graph_mapping/logs/mapping_log/mapping_log-*.json')

print('read log (found {})'.format(len(paths)))
for i, path in enumerate(paths):
    print('{} >> {}'.format(i, path))

log = None
try:
    i = int(input('choose file ').strip())

    with open(paths[i], 'r') as fp:
        log = json.load(fp)

except Exception as e:
    print('error :', e)

if log is not None:
    print('read log from {}'.format(datetime.fromtimestamp(log['starttime'])))

    odom_prev = None
    scanner_prev = None
    for i, data in enumerate(log['data']):

        odom_data = data['odom']
        dl = odom_data[3]
        dr = odom_data[4]

        c = 1E+2
        v_ = c * (dl + dr) / 2

        transform = [0, 0, 0]
        if odom_prev is not None:
            theta = odom_data[2]
            dtheta = theta - odom_prev[2]

            transform[0] = v_ * np.sin(theta)
            transform[1] = v_ * np.cos(theta)
            transform[2] = dtheta

        scanner_data = np.array(data['scanner'])[:, 1: 3]
        if scanner_prev is not None:
            RT_hist, _ = icp(scanner_prev, scanner_data)
            R = np.eye(2)
            T = np.zeros((2, 1))
            for RT in RT_hist:
                R_ = RT[:, : 2]
                T_ = RT[:, 2:]
                R = np.dot(R_, R)
                T = np.dot(R_, T) + T_

            dtheta = np.arctan2(R[0, 1], R[0, 0])
            dx = T[0, 0]
            dy = T[1, 0]

            z = np.array([[dx], [dy], [dtheta]])
            print(transform, z.flatten())

        odom_prev = odom_data
        scanner_prev = scanner_data