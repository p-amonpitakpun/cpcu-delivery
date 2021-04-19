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
                log = None
                with open(logs[i], 'r') as fp:
                    log = json.load(fp)
                graph_name = 'map {}'.format(
                    datetime.fromtimestamp(log['starttime']))
                graph, grid, g = compute(
                    log, slam_type='graph', optimized=False, name=graph_name, size=1000)
                # graph_optimized, g = compute(log, optimized=True)

            # u_vertices = [v.point for v in graph.getVertices()]
            # p = list(zip(*u_vertices))
            # plt.scatter(p[0], p[1], s=1, c='c', label='unoptimized')

            # o_vertices = [v.point for v in graph_optimized.getVertices()]
            # q = list(zip(*o_vertices))
            # plt.scatter(q[0], q[1], s=1, c='m', label='optimized')

            # try:
            #     r = list(zip(*log['valid']))
            #     plt.scatter(r[0], r[1], s=1, c='k', label='validation')
            # except:
            #     print('  Mapping: cannot show validation')

            save_path = IMAGE_DIR + graph_name + '.png'
            print('saved to ', save_path)
            cv2.imwrite(save_path, g)
            cv2.imshow(graph_name, g)
            cv2.waitKey(-1)
            # plt.legend()
            # plt.xlim(-3, 3)
            # plt.ylim(-3, 3)
            # print(gu)
            # print('close the plot to continue...')
            # plt.show()

            occ_config = {
                'resolution': grid.resolution,
                'logOdd_occ': grid.logOdd_occ,
                'logOdd_free': grid.logOdd_free,
                'min_treshold': grid.min_treshold,
                'max_treshold': grid.max_treshold,
            }
            occ_config_path = SAVE_DIR + graph_name + '.config.json'
            with open(occ_config_path, 'w') as fp:
                json.dump(occ_config, fp, indent=2)
            print('saved occ config at', occ_config_path)

            occ_grid_path = SAVE_DIR + graph_name + '.grid.npy'
            np.save(occ_grid_path, grid.grid)
            print('saved occ grid at', occ_grid_path)

        except ValueError as e:
            print('Error: ', e)
            traceback.print_exc()

    else:
        print('log not found')

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
