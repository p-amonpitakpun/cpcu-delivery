import glob
import json
import matplotlib.pyplot as plt
import numpy as np
import os

from datetime import datetime

from graph_mapping.scripts.modules.icp import tr_icp

print("This is", __file__)
dir_path = os.path.dirname(os.path.realpath(__file__))
regex = dir_path + '/graph_mapping/logs/mapping_log/mapping_log-*.json'
paths = glob.glob(regex)
print('search with', regex)

if len(paths) > 0:
    print('read log (found {})'.format(len(paths)))
    for i, path in enumerate(paths):
        print('{} >> {}'.format(i, path))

    log = None
    try:
        i = int(input('choose file ').strip()) if len(paths) > 1 else 0

        with open(paths[i], 'r') as fp:
            log = json.load(fp)

    except Exception as e:
        print('error :', e)

    if log is not None:
        print('read log from {}'.format(
            datetime.fromtimestamp(log['starttime'])))

        odom = [data['odom'] for data in log['data']]
        scanner = [data['scanner'] for data in log['data']]

        i, j = [int(s) for s in input(
            'choose a pair from {}: '.format(len(scanner))).strip().split()]

        P_i = np.array(scanner[i])[:, 1: 3]
        P_j = np.array(scanner[j])[:, 1: 3]
        # print('odom', odom[j])

        # dx, dy, dtheta = transform

        # R = np.array([[np.cos(dtheta), - np.sin(dtheta)],
        #                 [np.sin(dtheta), np.cos(dtheta)]])
        # T = np.array([[dx], [dy]])


        # P_i = P_i @ R.T + T.T

        
        R, T = tr_icp(P_i, P_j, N_iter=20)
        # print(R)
        # print(T)

        dtheta = np.arctan2(R[1, 0], R[0, 0])
        dx = T[0]
        dy = T[1]

        z = np.array([[dx, dy, dtheta]]).T
        print(z.T)
        
        R = np.array([[np.cos(dtheta), - np.sin(dtheta)],
                        [np.sin(dtheta), np.cos(dtheta)]])
        T = np.array([[dx], [dy]])

        Q = P_j @ R.T + T.T
    

        _, ax = plt.subplots()
        ax.set_aspect(1)

        ax.scatter(P_i.T[0], P_i.T[1], s=1, c='c', label=f'scanner {i}')
        ax.scatter(P_j.T[0], P_j.T[1], s=1, c='m', label=f'scanner {j}')
        ax.scatter(Q.T[0], Q.T[1], s=1, c='k', label=f'scanner {j} (new)')

        plt.legend()
        plt.show()

else:
    print('log not found')
