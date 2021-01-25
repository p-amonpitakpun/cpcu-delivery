import numpy as np

from collections import defaultdict


POINT_SHAPE = (3)


class Vertex:
    def __init__(self, point, laser_scanner_data):
        self.point = point
        self.laser_scanner_data = laser_scanner_data


class Edge:
    def __init__(self, dx, info_matrix):
        self.dx = dx
        self.info_matrix = info_matrix


vertices = []
edges = defaultdict(Edge)
adjacency_list = defaultdict(lambda: defaultdict(int))


def do_something(vetices, edges, adjacency_list, transform, laser_scanner_data):
    V = len(vertices)
    E = len(edges)

    new_vertex_id = V
    new_edge_id = E

    new_vertex, from_vertex_id, new_edge = predict(
        vertices, edge, adjacency_list, transform)

    vertices.append(new_vertex)
    edges[new_edge_id] = new_edge
    adjacency_list[from_vertex_id][new_vertex_id] = new_edge_id

    if V > 0:
        z, info_matrix = get_measurement(
            vertices[-1].laser_scanner_data, laser_scanner_data)
