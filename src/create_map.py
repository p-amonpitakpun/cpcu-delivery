import cv2
import glob
import json
import matplotlib.pyplot as plt
import numpy as np
import os
import traceback

from datetime import datetime

from graph_mapping.scripts.modules.log_processing import compute


dir_path = os.path.dirname(os.path.realpath(__file__))
LOG_DIR = dir_path + '/graph_mapping/logs/mapping_log/'
IMAGE_DIR = dir_path + '/../images/'
SAVE_DIR = dir_path + '/../saves/'


def validate(slam, log):
    P = [v.point for v in slam.getVertices()]
    T_ = [float(dat['timestamp']) for dat in log['data']]
    R = [dat['realpos'] for dat in log['data']]

    t0 = datetime.fromtimestamp(T_[0])
    p0 = P[0]
    r0 = np.array(R[0])

    X = []
    Y = []
    Z = []
    T = []

    for t, p, r in zip(T_, P, R):
        t = (datetime.fromtimestamp(t) - t0).total_seconds()
        p[0] -= p0[1]
        p[1] -= p0[1]
        r[0] = r[0] - r0[0]
        r[1] = r[1] - r0[1]
        z = (np.rad2deg(p[2] - r[2]) + 360) % 360

        T.append(t)
        X.append(p[0] - r[0])
        Y.append(p[1] - r[1])
        Z.append(z if z < 180 else z - 360)

    return T, X, Y, Z 

def main():
    LOG_PATH = LOG_DIR + 'mapping_log-*.json'
    print(LOG_PATH)
    logs = glob.glob(LOG_PATH)

    if len(logs) > 0:
        print('create map from log (found {})'.format(len(logs)))
        for i, logpath in enumerate(logs):
            print('[{}]'.format(i), logpath)

        if len(logs) == 1:
            ans2 = 0
        else:
            ans2 = input('answer: ').strip()

        try:
            i = int(ans2)

            if i < len(logs):

                real = input('real ? (y / n): ').strip() == 'y'
                optimized = input('optimized ? (y / n): ').strip() == 'y'

                log = None
                with open(logs[i], 'r') as fp:
                    log = json.load(fp)
                graph_name = 'map_{}'.format(str(log['starttime']))
                slam, occ_grid, grid = compute(
                    log, slam_type='real' if real else 'icp', optimized=optimized, name=graph_name, run_graph=True, size=500)
                
                u_vertices = [v.point for v in slam.getVertices()]
                p = np.array(u_vertices).T

                _, ax = plt.subplots()
                ax.set_aspect(1)
                ax.scatter(p[0, :], p[1, :], s=5, c='c', label=graph_name)

                plt.legend()

                if grid is not None:
                    save_path = IMAGE_DIR + graph_name + '.png'
                    print('saved to ', save_path)
                    cv2.imwrite(save_path, grid)
                    cv2.imshow(graph_name, grid)
                    cv2.waitKey(-1)

                T, X, Y, Z = validate(slam, log)

                fig1, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 10))
                start_time = datetime.fromtimestamp(log['starttime'])
                fig1.suptitle(f'Mapping Error of Test case {start_time}')

                axs[0].plot(T, X, c='b', label='dx')
                axs[0].plot(T, Y, c='r', label='dy')
                axs[1].plot(T, Z, c='k', label='dTheta')

                axs[1].set_ylim(-180, 180)

                axs[0].set_ylabel('Position Error (m)')
                axs[1].set_ylabel('Orientation Error (degree)')
                axs[1].set_xlabel('Time (s)')

                axs[0].legend(loc='upper left')
                axs[1].legend(loc='upper left')

                fig1.savefig(dir_path + f'/../images/mapping_error_{start_time}.png')

                print('close the plot to continue...')
                plt.show()

                if occ_grid is not None:
                    occ_config = {
                        'resolution': occ_grid.resolution,
                        'logOdd_occ': occ_grid.logOdd_occ,
                        'logOdd_free': occ_grid.logOdd_free,
                        'min_treshold': occ_grid.min_treshold,
                        'max_treshold': occ_grid.max_treshold,
                    }
                    occ_config_path = SAVE_DIR + graph_name + '.config.json'
                    with open(occ_config_path, 'w') as fp:
                        json.dump(occ_config, fp, indent=2)
                    print('saved occ config at', occ_config_path)

                    occ_grid_path = SAVE_DIR + graph_name + '.grid.npy'
                    np.save(occ_grid_path, occ_grid.grid)
                    print('saved occ grid at', occ_grid_path)

        except ValueError as e:
            print('Error: ', e)
            traceback.print_exc()

    else:
        print('log not found')

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
