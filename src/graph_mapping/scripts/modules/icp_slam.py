import numpy as np

from collections import defaultdict
from sympy import Matrix, MatrixSymbol, lambdify

from .icp import tr_icp


POINT_DIM = 3
POINT_SHAPE = (3)


Xi = MatrixSymbol('Xi', 3, 1)
Xj = MatrixSymbol('Xj', 3, 1)
Z = MatrixSymbol('Z', 3, 1)
Z_ = Xj - Xi
# z_ = lambdify((Xi, Xj), Matrix(Z_), modules='numpy')
E = Matrix(Z - Z_)
e = lambdify((Z, Xi, Xj), E, modules='numpy')
A = lambdify((Z, Xi, Xj), E.jacobian(Xi), modules='numpy')
B = lambdify((Z, Xi, Xj), E.jacobian(Xj), modules='numpy')


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


class ICPSLAM:
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

    def mapping(self, transform, laser_scanner_data):
        V = len(self.vertices)
        E = len(self.edges)

        # find a new vertex and edges
        new_vertex, new_edges = self.predict(transform)
        new_vertex.laser_scanner_data = laser_scanner_data

        # append a new vertex and edges
        if V == 0 or not np.all(np.fabs(self.vertices[-1].point - new_vertex.point) < 0.01):
            self.vertices.append(new_vertex)
            for edge in new_edges:
                new_edge_id = len(self.edges)
                self.edges.append(edge)
                self.adjacency_list[edge.from_x][edge.to_x] = new_edge_id

            # print('add new point at ', new_vertex.point)

            if V > 0:

                for edge in new_edges:
                    laser_scanner_data_i = self.vertices[edge.from_x].laser_scanner_data
                    laser_scanner_data_j = self.vertices[edge.to_x].laser_scanner_data
                    transform = edge.getTransform(self.vertices)

                    # get z, info_matrix from the measurement
                    z_ij, info_matrix_ij = self.getMeasurement(
                        laser_scanner_data_i, laser_scanner_data_j, transform)

                    # assign z, info_matrix to the edge
                    edge.z = z_ij
                    edge.info_matrix = info_matrix_ij

                if self.optimized:
                    # TODO ICP optimization
                    self.optimize(self.vertices, new_edges)

    def predict(self, transform):
        if len(self.vertices) > 0:
            V = len(self.vertices)
            vi = self.vertices[-1]
            vj = self.next_point(vi.point, transform)
            edge = Edge(V - 1, V, transform=vj.point - vi.point)
            return vj, [edge]
        else:
            vj = Vertex(np.zeros(POINT_SHAPE))
            return vj, []

    def next_point(self, point, transform):
        return Vertex(point + transform)

    def getMeasurement(self, laser_scanner_data_i, laser_scanner_data_j, transform):

        P_i = laser_scanner_data_i.copy()
        P_j = laser_scanner_data_j.copy()

        # dx, dy, dtheta = transform

        # R = np.array([[np.cos(dtheta), np.sin(dtheta)],
        #                 [- np.sin(dtheta), np.cos(dtheta)]])
        # T = np.array([[dx], [dy]])


        # P_i = P_i @ R.T + T.T

        R, T = tr_icp(P_i, P_j, N_iter=15)

        dtheta = np.arctan2(R[1, 0], R[0, 0])
        dx = T[0]
        dy = T[1]

        z = np.array([dx, dy, dtheta])
        # omega = np.linalg.inv(np.array([[2, 0.1, 0.1],
        #                                 [0.1, 2, 0.1],
        #                                 [0.1, 0.1, 2]]))
        return z, None

    def optimize(self, vertices, edges):
        for edge in edges:
            v_i = vertices[edge.from_x]
            v_j = vertices[edge.to_x]

            v_j.point = edge.z

    def getImage(self):
        return None

    def getVertexPoints(self):
        points = []
        for vertex in self.vertices:
            points.append(vertex.point)
        return points
