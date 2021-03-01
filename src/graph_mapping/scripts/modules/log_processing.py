import numpy as np

from .graph_slam import GraphSLAM


def process_data(graphSLAM, odom_data, scanner_data, last_odom):

    new_scan = np.array(scanner_data)[:, 1: 3]

    dl = odom_data[3]
    dr = odom_data[4]

    c = 1E+2
    v_ = c * (dl + dr) / 2

    transform = [0, 0, 0]
    if last_odom is not None:
        theta = odom_data[2]
        dtheta = theta - last_odom[2]

        transform[0] = v_ * np.sin(theta)
        transform[1] = v_ * np.cos(theta)
        transform[2] = dtheta

    graphSLAM.mapping(transform, new_scan)

    return odom_data.copy()

def compute(log):
    graph = GraphSLAM()

    last_odom = None
    for data in log['data']:
        last_odom = process_data(graph, data['odom'], data['scanner'], last_odom)

    return graph