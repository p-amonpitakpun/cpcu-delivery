import numpy as np

from collections import defaultdict
from sympy import Matrix, MatrixSymbol, lambdify

from .icp.icp import icp


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
    def __init__(self, from_vertex_id, to_vertex_id, dx, z=None, info_matrix=None):
        self.from_x = from_vertex_id
        self.to_x = to_vertex_id
        self.dx = dx
        self.z = z
        self.info_matrix = info_matrix


class GraphSLAM:
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
                    transform = edge.dx

                    # get z, info_matrix from the measurement
                    z_ij, info_matrix_ij = self.getMeasurement(
                        laser_scanner_data_i, laser_scanner_data_j, transform)

                    # assign z, info_matrix to the edge
                    edge.z = z_ij
                    edge.info_matrix = info_matrix_ij

                if self.optimized:
                    N = POINT_DIM
                    X, H = self.optimize(new_edges)
                    for i, v in enumerate(self.vertices):
                        v.point = X[N * i: N * (i + 1), :].reshape((3))
                    for e in new_edges:
                        e.info_matrix = H[N * i: N *
                                          (i + 1), N * i: N * (i + 1)]

                # for edge in new_edges:
                #     print('{} --> {}'.format(edge.from_x, edge.to_x))
                #     print(edge.dx)
                #     print(self.vertices[edge.from_x].point - self.vertices[edge.to_x].point)
                #     print(edge.z.flatten())


    def predict(self, transform):
        if len(self.vertices) > 0:
            V = len(self.vertices)
            vi = self.vertices[-1]
            vj = self.next_point(vi.point, transform)
            edge = Edge(V - 1, V, vj.point - vi.point)
            return vj, [edge]
        else:
            vj = Vertex(np.zeros(POINT_SHAPE))
            return vj, []

    def next_point(self, point, transform):
        return Vertex(point + np.array(transform))

    def getMeasurement(self, laser_scanner_data_i, laser_scanner_data_j, transform):
        RT_hist, _ = icp(laser_scanner_data_i, laser_scanner_data_j)
        R = np.eye(2)
        T = np.zeros((2, 1))
        for RT in RT_hist:
            R_ = RT[:, : 2]
            T_ = RT[:, 2:]
            R = np.dot(R_, R)
            T = np.dot(R_, T) + T_

        dtheta = np.arctan2(R[1, 0], R[0, 0])
        dx = T[0, 0]
        dy = T[1, 0]

        z = np.array([[dx], [dy], [dtheta]])
        omega = np.linalg.inv(np.array([[2, 0.1, 0.1],
                                        [0.1, 2, 0.1],
                                        [0.1, 0.1, 2]]))
        return z, omega

    def optimize(self, edges):

        N = POINT_DIM
        Nv = len(self.vertices)
        X = np.concatenate(
            [vertex.point for vertex in self.vertices]).reshape((-1, 1))
        Nx = len(X)

        # loop until converge
        dx = None
        while dx is None or (np.fabs(dx.flatten()) > np.inf).any():

            # Build Linear System
            # initilize matrices
            H = np.zeros((Nx, Nx), dtype=float)
            b = np.zeros((Nx, 1), dtype=float)

            # compute H, b
            for edge in edges:
                i = edge.from_x
                j = edge.to_x

                x_i = X[N * i: N * (i + 1)]
                x_j = X[N * j: N * (j + 1)]

                z = edge.z
                Omega = edge.info_matrix

                global e, A, B

                e_ij = e(z, x_i, x_j)
                A_ij = A(z, x_i, x_j)
                B_ij = B(z, x_i, x_j)

                H[N * i: N * (i + 1), N * i: N * (i + 1)
                  ] += A_ij.T @ (Omega @ A_ij)
                H[N * i: N * (i + 1), N * j: N * (j + 1)
                  ] += A_ij.T @ (Omega @ B_ij)
                H[N * j: N * (j + 1), N * i: N * (i + 1)
                  ] += B_ij.T @ (Omega @ A_ij)
                H[N * j: N * (j + 1), N * j: N * (j + 1)
                  ] += B_ij.T @ (Omega @ B_ij)

                b[N * i: N * (i + 1), :] += A_ij.T @ Omega @ e_ij
                b[N * j: N * (j + 1), :] += B_ij.T @ Omega @ e_ij

            # compute the Hessian in the original space
            H += np.eye(Nx)

            # Solve Linear System
            dx = np.linalg.solve(H, b)
            X += dx

        H = np.zeros((Nx, Nx), dtype=float)

        for edge in edges:
            i = edge.from_x
            j = edge.to_x

            x_i = X[N * i: N * (i + 1)]
            x_j = X[N * j: N * (j + 1)]

            z = edge.z
            Omega = edge.info_matrix

            A_ij = A(z, x_i, x_j)
            B_ij = B(z, x_i, x_j)

            H[N * i: N * (i + 1), N * i: N * (i + 1)
              ] += A_ij.T @ (Omega @ A_ij)
            H[N * i: N * (i + 1), N * j: N * (j + 1)
              ] += A_ij.T @ (Omega @ B_ij)
            H[N * j: N * (j + 1), N * i: N * (i + 1)
              ] += B_ij.T @ (Omega @ A_ij)
            H[N * j: N * (j + 1), N * j: N * (j + 1)
              ] += B_ij.T @ (Omega @ B_ij)

        return X, H

    def getImage(self):
        return None

    def getVertexPoints(self):
        points = []
        for vertex in self.vertices:
            points.append(vertex.point)
        return points
