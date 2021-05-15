import numpy as np

from collections import defaultdict
from sympy import Matrix, MatrixSymbol, lambdify

from .icp import icp, tr_icp


POINT_DIM = 3
POINT_SHAPE = (3)


class Vertex:
    def __init__(self, point, laser_scanner_data=None):
        self.point = point
        self.laser_scanner_data = laser_scanner_data


class Edge:
    def __init__(self, from_vertex_id, to_vertex_id, transform=None, z=None, info_matrix=None):
        self.from_x = from_vertex_id
        self.to_x = to_vertex_id
        self.transform = transform
        self.z = z
        self.info_matrix = info_matrix

    def getTransform(self, V):
        return V[self.to_x].point - V[self.from_x].point


class REALSLAM:
    def __init__(self, optimized=True):
        self.vertices = []
        self.edges = []
        self.adjacency_list = defaultdict(lambda: defaultdict(int))

        self.optimized = optimized

    def getVertices(self):
        return self.vertices

    def getEdges(self):
        return self.edges

    def getAdjList(self):
        return self.adjacency_list

    def mapping(self, realpos, laser_scanner_data):
        V = len(self.vertices)

        # find a new vertex and edges
        new_vertex, new_edges = self.predict(realpos)
        new_vertex.laser_scanner_data = laser_scanner_data

        # append a new vertex and edges:
        self.vertices.append(new_vertex)
        for edge in new_edges:
            new_edge_id = len(self.edges)
            self.edges.append(edge)
            self.adjacency_list[edge.from_x][edge.to_x] = new_edge_id

    def predict(self, realpos):
        if len(self.vertices) > 0:
            V = len(self.vertices)
            vi = self.vertices[-1]
            vj = Vertex(np.array(realpos))
            edge = Edge(V - 1, V, transform=(vj.point - vi.point))
            return vj, [edge]
        else:
            vj = Vertex(np.array(realpos))
            return vj, []

    def next_point(self, point, transform):
        return Vertex(point + transform)

    def getImage(self):
        return None

    def getVertexPoints(self):
        points = []
        for vertex in self.vertices:
            points.append(vertex.point)
        return points
