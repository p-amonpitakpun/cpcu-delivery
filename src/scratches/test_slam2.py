import numpy as np

from collections import defaultdict
from sympy import Matrix, MatrixSymbol, lambdify


POINT_DIM = 3
POINT_SHAPE = (3)
# X = [x, y, t]

Xi = MatrixSymbol('Xi', 3, 1)
Xj = MatrixSymbol('Xj', 3, 1)
Z = MatrixSymbol('Z', 3, 1)
Z_ = Xj - Xi
z_ = lambdify((Xi, Xj), Matrix(Z_), modules='numpy')
E = Matrix(Z - Z_)
e = lambdify((Z, Xi, Xj), E, modules='numpy')
A = lambdify((Z, Xi, Xj), E.jacobian(Xi), modules='numpy')
B = lambdify((Z, Xi, Xj), E.jacobian(Xj), modules='numpy')

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


vertices = []
edges = defaultdict(Edge)
adjacency_list = defaultdict(lambda: defaultdict(int))


def mapping(vetices, edges, adjacency_list, transform, laser_scanner_data):
    V = len(vertices)
    E = len(edges)

    # find a new vertex and edges
    new_vertex, new_edges = predict(
        vertices, edge, adjacency_list, transform)
    new_vertex.laser_scanner_data = laser_scanner_data

    # append a new vertex and edges
    vertices.append(new_vertex)
    for edge in new_edges:
        new_edge_id = len(edges)
        edges[new_edge_id] = new_edge
        adjacency_list[edge.from_vertex_id][edge.to_vertex_id] = new_edge_id

    if V > 0:

        for egde in new_edges:
            laser_scanner_data_i = vertices[edge.from_vertex_id].laser_scanner_data
            laser_scanner_data_j = vertices[edges.to_vertex_id].laser_scanner_data

            # get z, info_matrix from the measurement
            z_ij, info_matrix_ij = get_measurement(
                laser_scanner_data_i, laser_scanner_data_j)

            # assign z, info_matrix to the edge
            edge.z = z_ij
            edge.info_matrix = info_matrix_ij

        X, H = optimize(vertices, new_edges)


def predict(V, E, Adj, t):
    if len(V) > 0:
        vi = V[-1]
        vj = next_point(vi.point, transform)
        edge = Edge(vi, vj, vj.point - vi.point, None, None)
        return vj, [edge]
    else:
        vj = np.zeros(POINT_SHAPE)
        return vj, []


def optimize(V, E):

    N = POINT_DIM
    Nv = len(V)
    X = np.zeros((N * Nv, 1)) #TODO initialize state vector

    for _ in range(1):

        # Build Linear System
        # initilize matrices
        H = np.zeros((N * Nv, N * Nv), type=float)
        b = np.zeros((N * Nv, 1), type=float)

        # compute H, b
        for edge in E:
            i = edge.from_vertex_id
            j = edge.to_vertex_id

            x_i = X[N * i: N * (i + 1)]
            x_j = X[N * j: N * (j + 1)]

            z = edge.z
            Omega = edge.info_matrix

            e = E(z, x_i, x_j)
            A_ij = A(z, x_i, x_j)
            B_ij = B(z, x_i, x_j)

            H[N * i: N * (i + 1), N * i: N * (i + 1)] = A_ij.T @ (Omega @ A_ij)
            H[N * i: N * (i + 1), N * j: N * (j + 1)] = A_ij.T @ (Omega @ B_ij)
            H[N * j: N * (j + 1), N * i: N * (i + 1)] = B_ij.T @ (Omega @ A_ij)
            H[N * j: N * (j + 1), N * j: N * (j + 1)] = B_ij.T @ (Omega @ B_ij)

            b[N * i: N * (i + 1), :] += A_ij.T @ Omega @ e_ij
            b[N * j: N * (j + 1), :] += B_ij.T @ Omega @ e_ij

        # compute the Hessian in the original space
        H += np.eye(N * Nv)

        # Solve Linear System
        dx = np.linalg.solve(H, b))
        X += dx

    H=np.zeros((N * Nv, N * Nv), type = float)

    for edge in E:
        i=edge.from_vertex_id
        j=edge.to_vertex_id

        z=edge.z
        Omega=edge.info_matrix

        e=E(z, x_i, x_j)
        A_ij=A(z, x_i, x_j)
        B_ij=B(z, x_i, x_j)

        H[N * i: N * (i + 1), N * i: N * (i + 1)]=A_ij.T @ (Omega @ A_ij)
        H[N * i: N * (i + 1), N * j: N * (j + 1)]=A_ij.T @ (Omega @ B_ij)
        H[N * j: N * (j + 1), N * i: N * (i + 1)]=B_ij.T @ (Omega @ A_ij)
        H[N * j: N * (j + 1), N * j: N * (j + 1)]=B_ij.T @ (Omega @ B_ij)

    return X, H


if __name__ == "__main__":
    n=20
    start=np.array([[0, 0, 0]])
    vertices.append(Vertex(start, None))
    for i in range(1, n):
        dtheta=np.random.uniform(low = -np.pi/4, high = np.pi/4)
        theta=p[i - 1][2] + dtheta
        r=np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        x=p[i - 1, :2] + (r @ np.random.uniform(low=[[0], [0]],
                                                  high=[[5], [0]])).reshape(1, 2)
        x=np.append(x, [[theta]], axis = -1)
        p=np.append(p, x, axis = 0)

    n=len(p)
    u=0.1
    dp=np.zeros((0, 3))
    for i, p_i in enumerate(p):
        x=p_i
        if i > 0:
            x=x - p[i - 1]
        dp=np.append(
            dp, np.array([x]), axis = 0)
    dp=np.array(dp)
    # print(dp)

    odo_stream=dp + (np.random.rand(n, 3) -
                       np.array([[0.5] * 3] * n)) * u
    sen_stream=dp + (np.random.rand(n, 3) -
                       np.array([[0.5] * 3] * n)) * u / 2

    q=np.cumsum(odo_stream, axis = 0)
    s=np.cumsum(sen_stream, axis = 0)

    # print(odo_stream)
    print('start!')
    x=GraphSLAM().run(odo_stream, sen_stream).reshape((-1, 3))
    # print(x)

    plt.plot(p[:, 0], p[:, 1], c = 'black')
    plt.plot(q[:, 0], q[:, 1], c = 'b')
    plt.plot(s[:, 0], s[:, 1], c = 'g')
    plt.plot(x[:, 0], x[:, 1], c = 'r')
    plt.show()
