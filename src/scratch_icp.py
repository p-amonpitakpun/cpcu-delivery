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
from graph_mapping.scripts.modules.icp import icp

import os
dir_path = os.path.dirname(os.path.realpath(__file__))
LOG_PATH = dir_path + '/graph_mapping/logs/mapping_log'

LOG_DIR = LOG_PATH + '/mapping_log-*.json'
print(LOG_DIR)
logs = glob.glob(LOG_DIR)

graph = None
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
        graph, g = compute(log, optimized=False, name=graph_name,
                           run_graph=False, progress=False)

if graph is not None:
    p = np.array(graph.getVertexPoints())
    q = p[1:] - p[: -1]

# print(q[: 5])

q_ = np.mean(q, axis=0)
r = q - q_
X = r[:, 0]
Y = r[:, 1]
Z = r[:, 2]
print(X.shape, Y.shape, Z.shape, len(q))

A = np.array([[np.dot(X, X), np.dot(X, Y), np.dot(X, Z)],
              [np.dot(Y, X), np.dot(Y, Y), np.dot(Y, Z)],
              [np.dot(Z, X), np.dot(Z, Y), np.dot(Z, Z)]]) / len(q)

print(A)

rng = np.random.default_rng(0)

P = np.array([0, 0, 0])
# A = np.array([[2, 0.1, 0.1],
#               [0.1, 2, 0.1],
#               [0.1, 0.1, 2]]) * 0.0001
Q = rng.multivariate_normal(P, A, size=1000, method='svd')

# print(Q[0])

plt.scatter(Q[:, 0], Q[:, 1], s=10, alpha=0.3, c='b')
plt.scatter(q[:, 0], q[:, 1], s=10, c='r')
plt.show()
