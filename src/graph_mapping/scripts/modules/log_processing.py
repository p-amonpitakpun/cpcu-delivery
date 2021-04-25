import cv2
import numpy as np

from progressbar import ProgressBar

from .graph_slam import GraphSLAM
from .icp_slam import ICPSLAM
from .occupancy_grid import OccupancyGrid


SLAM = {
    'graph': GraphSLAM,
    'icp': ICPSLAM
}


def process_data(slam, odom_data, scanner_data, last_odom):

    new_scan = np.array(scanner_data)[:, 1: 3]

    dl = odom_data[3]
    dr = odom_data[4]

    c = 0.5
    v_ = c * (dl + dr) / 2

    transform = [0, 0, 0]
    if last_odom is not None:
        theta = odom_data[2]
        dtheta = (theta - last_odom[2])

        transform[0] = v_ * np.cos(theta)
        transform[1] = v_ * np.sin(theta)
        transform[2] = dtheta

    slam.mapping(np.array(transform), new_scan)

    return slam, odom_data.copy()


def create_OccupancyGrid(slam, name='', size=500):
    min_scanner = 20
    resolution = min_scanner * 2 / size

    occupancy_grid = OccupancyGrid(
        shape=(size, size),
        resolution=resolution,
        logOdd_occ=0.9,
        logOdd_free=0.7
    )
    occupancy_grid.min_treshold = -50
    occupancy_grid.max_treshold = 50
    offset = min_scanner

    plot = np.zeros((500, 500, 3), dtype=np.uint8)

    V = slam.getVertices()
    with ProgressBar(max_value=len(V)) as bar:
        for i, v in enumerate(V):

            X = v.point

            dx = v.point
            psi = dx[2]
            R = np.array([[np.cos(psi), - np.sin(psi)],
                          [np.sin(psi), np.cos(psi)]])
            T = dx[0: 2].reshape((2, 1))

            P = v.laser_scanner_data

            all_plot = []

            for p in P:
                p = np.dot(p, R.T) + T.T
                p = p.reshape((2,))
                occupancy_grid.updateOccupy(
                    (X[0] + offset, X[1] + offset), (p[0] + offset, p[1] + offset))
                plot = cv2.circle(plot, (int(size // 2 + p[1] // resolution), int(size // 2 - p[0] // resolution)), 1, (0, 0, 255))
            all_plot.append(plot)

            grid = occupancy_grid.getImage(
                occupancy_grid.min_treshold, occupancy_grid.max_treshold)
            center = (int(X[0] // resolution) + (size // 2),
                      (size // 2) - int(X[1] // resolution))
            grid = cv2.circle(grid, center, 2, (0, 0, 255), -1)
            # cv2.imshow(name, grid)
            all_plot.append(grid)

            scan = np.zeros((500, 500, 3), dtype=np.uint8)
            scale = 50
            scan = cv2.circle(scan, (250, 250), 5, (100, 250, 50), -1)
            for p in P:
                scan = cv2.circle(scan, (int(250 - p[1] * scale), int(250 - p[0] * scale)), 2, (0, 125, 255))
            # cv2.imshow('scan', scan)
            all_plot.append(scan)

            img = cv2.hconcat(all_plot)
            img = cv2.putText(img, f'{i}', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100))
            cv2.imshow('all plot', img)

            key = cv2.waitKey(1)
            if key > -1:
                break

            bar.update(i)

    grid = occupancy_grid.getImage(
        occupancy_grid.min_treshold, occupancy_grid.max_treshold)

    return grid, occupancy_grid


def compute(log, slam_type='graph', optimized=True, name='', run_graph=True, size=500, progress=True):

    print(f'computing {slam_type}SLAM...')
    slam = SLAM[slam_type](optimized=optimized)

    last_odom = None
    if progress:
        with ProgressBar(max_value=len(log['data'])) as bar:
            for i, data in enumerate(log['data']):
                # print('process', i, data['timestamp'])
                slam, last_odom = process_data(
                    slam, data['odom'], data['scanner'], last_odom)
                bar.update(i)

    else:
        for i, data in enumerate(log['data']):
            slam, last_odom = process_data(
                slam, data['odom'], data['scanner'], last_odom)

    if run_graph:
        print('creating Occupancy Grid...')
        grid, occupancy_grid = create_OccupancyGrid(slam, name=name, size=size)
    else:
        grid = None
        occupancy_grid = None

    return slam, occupancy_grid, grid
