# import matplotlib.pyplot as plt
# import numpy as np

import cv2
import glob
import json
import matplotlib.pyplot as plt
import numpy as np
import os
import traceback

from datetime import datetime

from graph_mapping.scripts.modules.log_processing import compute
from graph_mapping.scripts.modules.icp import icp, tr_icp

import os
dir_path = os.path.dirname(os.path.realpath(__file__))
LOG_PATH = dir_path + '/graph_mapping/logs/mapping_log'

LOG_DIR = LOG_PATH + '/mapping_log-*.json'
print(LOG_DIR)
logs = glob.glob(LOG_DIR)

slam = None
if len(logs) > 0:
    print('create map from log (found {})'.format(len(logs)))
    for i, logpath in enumerate(logs):
        print('[{}]'.format(i), logpath)

    if len(logs) == 1:
        ans2 = 0
    else:
        ans2 = input('answer: ').strip()

    i = int(ans2)

    if i < len(logs):
        log = None
        with open(logs[i], 'r') as fp:
            log = json.load(fp)
        graph_name = 'map {}'.format(datetime.fromtimestamp(log['starttime']))
        slam, occ, g = compute(log, optimized=False, name=graph_name,
                           run_graph=False, progress=True)

if slam is None:
    print('compute error')
    exit(1)

V = slam.getVertices()
E = slam.getEdges()

print('slam result', len(V), len(E))

edge = E[40]
i = edge.from_x
j = edge.to_x
v_i = V[i]
v_j = V[j]

print(f'from\t{i}\t{v_i.point}')
print(f'to\t{j}\t{v_j.point}')

transform = v_j.point - v_i.point
print(f'with transform\t{transform}')

P_i = v_i.laser_scanner_data
P_j = v_j.laser_scanner_data

dx, dy, dtheta = transform

R = np.array([[np.cos(dtheta), np.sin(dtheta)],
                [- np.sin(dtheta), np.cos(dtheta)]])
T = np.array([[dx], [dy]])

Q = P_i @ R.T - T.T
R, T = tr_icp(Q, P_j, N_iter=15)

Q_ = Q @ R.T + T.T

plt.scatter(P_i.T[0], P_i.T[1], s=5, alpha=0.3, c='c', label=f'scanner {i}')
plt.scatter(P_j.T[0], P_j.T[1], s=5, alpha=0.3, c='m', label=f'scanner {j}')
plt.scatter(Q.T[0], Q.T[1], s=5, alpha=0.3, c='k', label=f'scanner {i} (transformed)')
plt.scatter(Q_.T[0], Q_.T[1], s=5, alpha=0.3, c='r', label=f'scanner {i} (icp)')

plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.legend()
plt.show()