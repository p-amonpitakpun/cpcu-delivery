import glob
import os
import matplotlib.pyplot as plt
import numpy as np
import os

from datetime import datetime
from ext.basic_units import radians

dir_path = os.path.dirname(os.path.realpath(__file__))
paths = glob.glob(dir_path + '/localization/logs/error/error_*.log.txt')

class Reader:
    def __init__(self):
        self.state = -1
        self.prefix = '#'
        self.keywords = ['info', 'data']

        self.info = {}
        self.data = []
    
    def read(self, idx, line):
        if line[0] == self.prefix:
            words = line.strip().split(' ')
            try: 
                self.state = self.keywords.index(words[1])
            except ValueError:
                print(f'Reeader: Cannot find keyword {words[1]}')
                self.state = -1
        elif self.state >= 0 and self.state < len(self.keywords):
            self.translate(self.keywords[self.state], idx, line)
        else:
            pass

    def translate(self, state, idx, line):
        line = line.strip()
        if len(line) > 0:
            try:
                if state == 'info':
                    w_idx = line.find(':')
                    k = line[: w_idx].strip()
                    v = line[w_idx + 1: ].strip()
                    self.info[k] = v
                elif state == 'data':
                    row = [float(s.strip()) for s in line.split(',')]
                    self.data.append(row)
            except:
                print(f'Reader: incorrect format at line {idx}')


if len(paths) > 0:
    print('filepath:')
    for i, path in enumerate(paths):
        print(f'[{i}]\t {path}')

    idx = int(input('choose index: ').strip())
    path = paths[idx]

    reader = Reader()

    lines = []
    with open(path, 'r') as fp:
        lines = fp.readlines()

    for i, line in enumerate(lines):
        reader.read(i, line)

    info = reader.info
    data = reader.data

    T = []
    X = []
    Y = []
    Z = []

    timestamp  = float(info['time'])
    log_time = datetime.fromtimestamp(timestamp)
    start_time = datetime.fromtimestamp(data[0][0])

    for t, x, y, z in data:
        T.append((datetime.fromtimestamp(t) - start_time).total_seconds())
        X.append(x)
        Y.append(y)
        z = np.fmod(np.rad2deg(z), 360)
        Z.append(z if z < 180 else z - 360)

    fig1, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 10))
    fig1.suptitle(f'Localization Error of Test case {log_time}')
    axs[0].plot(T, X, c='b', label='dx')
    axs[0].plot(T, Y, c='r', label='dy')
    axs[1].plot(T, Z, c='k', label='dTheta')

    axs[1].set_ylim(-20, 20)

    axs[0].set_ylabel('Position Error (m)')
    axs[1].set_ylabel('Orientation Error (degree)')
    axs[1].set_xlabel('Time (s)')

    axs[0].legend(loc='upper left')
    axs[1].legend(loc='upper left')
    
    fig1.savefig(dir_path + f'/../images/error_{timestamp}.png')

    # fig, axt = plt.subplots()
    # axt.scatter(X, Y, s=1)
    # axt.set_xlim(-0.20, 0.20)
    # axt.set_ylim(-0.20, 0.20)
    
    plt.show()