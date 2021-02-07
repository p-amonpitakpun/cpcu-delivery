import numpy as np

from collections import defaultdict
from sympy import Matrix, MatrixSymbol, lambdify


POINT_DIM = 3
POINT_SHAPE = (3)


class Vertex:
    def __init__(self, point, laser_scanner_data):
        self.point = point
        self.laser_scanner_data = laser_scanner_data


class Edge:
    def __init__(self, from_vertex_id, to_vertex_id, dx, z, info_matrix):
        self.from_x = from_vertex_id
        self.to_x = to_vertex_id
        self.dx = dx
        self.z = z
        self.info_matrix = info_matrix

        Xi = MatrixSymbol('Xi', 3, 1)
        Xj = MatrixSymbol('Xj', 3, 1)
        Z = MatrixSymbol('Z', 3, 1)
        Z_ = Xj - Xi
        z_ = lambdify((Xi, Xj), Matrix(Z_), modules='numpy')
        E = Matrix(Z - Z_)
        e = lambdify((Z, Xi, Xj), E, modules='numpy')
        A = lambdify((Z, Xi, Xj), E.jacobian(Xi), modules='numpy')
        B = lambdify((Z, Xi, Xj), E.jacobian(Xj), modules='numpy')

        self.error_functions = e, A, B


class GraphSLAM:
    def __init__(self):
        self.vertices = []
        self.edges = defaultdict(Edge)
        self.adjacency_list = defaultdict(lambda: defaultdict(int))

    def mapping(self, transform, laser_scanner_data):
        V = len(self.vertices)
        E = len(self.edges)

        # find a new vertex and edges
        new_vertex, new_edges = self.predict(transform)
        new_vertex.laser_scanner_data = laser_scanner_data

        # append a new vertex and edges
        vertices.append(new_vertex)
        for edge in new_edges:
            new_edge_id = len(self.edges)
            self.edges[new_edge_id] = new_edge
            self.adjacency_list[edge.from_vertex_id][edge.to_vertex_id] = new_edge_id

        if V > 0:

            for egde in new_edges:
                laser_scanner_data_i = vertices[edge.from_vertex_id].laser_scanner_data
                laser_scanner_data_j = vertices[edges.to_vertex_id].laser_scanner_data
                transform = edge.dx

                # get z, info_matrix from the measurement
                z_ij, info_matrix_ij = self.getMeasurement(
                    laser_scanner_data_i, laser_scanner_data_j, transform)

                # assign z, info_matrix to the edge
                edge.z = z_ij
                edge.info_matrix = info_matrix_ij

            X, H = self.optimize(new_edges)

    def predict(self, transform):
        if len(self.vertices) > 0:
            vi = self.vertices[-1]
            vj = next_point(vi.point, transform)
            edge = Edge(vi, vj, vj.point - vi.point, None, None)
            return vj, [edge]
        else:
            vj = np.zeros(POINT_SHAPE)
            return vj, []

    def getMeasurement(self, laser_scanner_data_i, laser_scanner_data_j, transform):
        # TODO
        z = None
        omega = None
        return z, omega

    def optimize(self, edges):

        N = POINT_DIM
        Nv = len(self.vertices)
        X = np.concatenate([vertex.point for vertex in self.vertices])
        Nx = len(X)

        # loop until converge
        for _ in range(1):

            # Build Linear System
            # initilize matrices
            H = np.zeros((Nx, Nx), type=float)
            b = np.zeros((Nx, 1), type=float)

            # compute H, b
            for edge in edges:
                i = edge.from_vertex_id
                j = edge.to_vertex_id

                x_i = X[N * i: N * (i + 1)]
                x_j = X[N * j: N * (j + 1)]

                z = edge.z
                Omega = edge.info_matrix

                e, A, B = self.error_functions

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
            H[: N, : N] += np.eye(N)

            # Solve Linear System
            dx = np.linalg.solve(H, b)
            X += dx

        H = np.zeros((Nx, Nx), type=float)

        for edge in edges:
            i = edge.from_vertex_id
            j = edge.to_vertex_id

            z = edge.z
            Omega = edge.info_matrix

            e = E(z, x_i, x_j)
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
