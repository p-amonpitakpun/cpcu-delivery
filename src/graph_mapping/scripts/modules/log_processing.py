import cv2
import numpy as np

from progressbar import ProgressBar

from .graph_slam import GraphSLAM
from .occupancy_grid import OccupancyGrid


def process_data(graphSLAM, odom_data, scanner_data, last_odom):

    new_scan = np.array(scanner_data)[:, 1: 3]

    dl = odom_data[3]
    dr = odom_data[4]

    c = 55
    v_ = c * (dl + dr) / 2

    transform = [0, 0, 0]
    if last_odom is not None:
        theta = odom_data[2]
        dtheta = theta - last_odom[2]

        transform[1] = - v_ * np.sin(theta)
        transform[0] = v_ * np.cos(theta)
        transform[2] = dtheta

    graphSLAM.mapping(transform, new_scan)

    return graphSLAM, odom_data.copy()


def create_OccupancyGrid(graphSLAM):
    size = 1000
    min_scanner = 20
    resolution = min_scanner * 2 / size

    occupancy_grid = OccupancyGrid(shape=(size, size), resolution=resolution, logOdd_occ=0.9, logOdd_free=0.7)
    occupancy_grid.min_treshold = -50
    occupancy_grid.max_treshold = 50
    offset = min_scanner / 2

    R = np.eye(2)
    T = np.zeros(shape=(2, 1))

    V = graphSLAM.getVertices()
    E = graphSLAM.getEdges()

    X = V[0].point
    P = V[0].laser_scanner_data
    for p in P:
        p = p.flatten()
        occupancy_grid.updateOccupy((X[0] + offset, offset - X[1]), (p[0] + offset, offset + p[1]))

    with ProgressBar(max_value=len(E)) as bar:
        for e in E:
            i = e.from_x
            j = e.to_x

            X = V[j].point

            dx = V[j].point - V[i].point
            psi = dx[2]
            R_ = np.array([[np.cos(psi), - np.sin(psi)], [np.sin(psi), np.cos(psi)]])
            T_ = dx[0: 2].reshape((2, 1))

            R = np.dot(R_, R)
            T = np.dot(R_, T) + T_

            P = V[0].laser_scanner_data

            for p in P:
                p = p.reshape((2, 1))
                p[0, 0] *= -1
                p = np.dot(R, - p) + T
                p = p.reshape((2,))
                occupancy_grid.updateOccupy((X[0] + offset, X[1] + offset), (p[0] + offset, p[1] + offset))
            

            occupancy_range = occupancy_grid.max_treshold - occupancy_grid.min_treshold
            grid = (
                (occupancy_grid.grid - occupancy_grid.min_treshold) / occupancy_range * 255
            ).astype(np.uint8)
            grid = 255 - cv2.cvtColor(grid, cv2.COLOR_GRAY2RGB)
            grid = cv2.circle(grid, (int(X[0] // resolution) + size // 2, size // 2 - int(X[1] // resolution)), 5, (0, 0, 255), -1)
            cv2.imshow('grid', grid)
            cv2.waitKey(1)

            bar.update(i)


    occupancy_range = occupancy_grid.max_treshold - occupancy_grid.min_treshold
    grid = (
        (occupancy_grid.grid - occupancy_grid.min_treshold) / occupancy_range * 255
    ).astype(np.uint8)

    return grid

def compute(log, optimized=True):
    graph = GraphSLAM(optimized=optimized)

    last_odom = None
    with ProgressBar(max_value=len(log['data'])) as bar:
        for i, data in enumerate(log['data']):
            # print('process', i, data['timestamp'])
            graph, last_odom = process_data(
                graph, data['odom'], data['scanner'], last_odom)
            bar.update(i)

    grid = create_OccupancyGrid(graph)

    return graph, grid
