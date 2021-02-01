import matplotlib.pyplot as plt
import numpy as np
import scipy as sp

from sympy import Matrix, MatrixSymbol, lambdify


N = 3

Xi = MatrixSymbol('Xi', 3, 1)
Xj = MatrixSymbol('Xj', 3, 1)
Z = MatrixSymbol('Z', 3, 1)
e = Matrix(Z - (Xj - Xi))
a = e.jacobian(Xi)
b = e.jacobian(Xj)
E = lambdify((Xi, Xj, Z), e, modules='numpy')
A = lambdify((Xi, Xj, Z), a, modules='numpy')
B = lambdify((Xi, Xj, Z), b, modules='numpy')


class GraphSLAM:
    def __init__(self):
        pass

    def run(self, odo_stream, sen_stream):
        M = len(odo_stream)
        if M != len(sen_stream):
            print('!')
            return

        # new solution and new info matrix
        self.X = np.zeros((M * 3, 1))
        self.H = np.zeros((M * 3, M * 3))

        for i, (odo, sen) in enumerate(zip(odo_stream, sen_stream)):
            x = np.array(odo, dtype=float).reshape(N, 1)
            z = np.array(sen, dtype=float).reshape(N, 1)

            Omega = np.linalg.inv(np.array([[1, 0.5, 0.5],
                                            [0.5, 1, 0.5],
                                            [0.5, 0.5, 1]]))

            if i > 0:
                self.predict(i - 1, i, x)
                self.optimize(i - 1, i, z, M, Omega)
            else:
                self.X[N * i: N * (i + 1), :] = x

        return self.X

    def predict(self, i, j, x):
        self.X[N * j: N * (j + 1), :] = self.X[N * i: N * (i + 1), :] + x

    def optimize(self, i, j, z, M, Omega=np.zeros((N, N), dtype=float)):
        for _ in range(1):
            H, b = self.buildLinearSystem(i, j, z, M, Omega)
            delta_x = np.linalg.solve(H + np.eye(N * M), b)
            self.X += delta_x
            # print(self.X.T[: N * (j + 1)])
        self.H = H - np.eye(*H.shape)
        return self.X, self.H

    def buildLinearSystem(self, i, j, z, M, Omega):
        x_i = self.X[N * i: N * (i + 1), :]
        x_j = self.X[N * j: N * (j + 1), :]
        e_ij = E(x_i, x_j, z)

        A_ij = A(x_i, x_j, z)
        B_ij = B(x_i, x_j, z)

        H = np.zeros((M * 3, M * 3))
        b = np.zeros((M * 3, 1))

        H[N * i: N * (i + 1), N * i: N * (i + 1)] = A_ij.T @ (Omega @ A_ij)
        H[N * i: N * (i + 1), N * j: N * (j + 1)] = A_ij.T @ (Omega @ B_ij)
        H[N * j: N * (j + 1), N * i: N * (i + 1)] = B_ij.T @ (Omega @ A_ij)
        H[N * j: N * (j + 1), N * j: N * (j + 1)] = B_ij.T @ (Omega @ B_ij)

        b[N * i: N * (i + 1), :] += A_ij.T @ Omega @ e_ij
        b[N * j: N * (j + 1), :] += B_ij.T @ Omega @ e_ij

        return H, b


if __name__ == "__main__":
    n = 20
    p = np.array([[0, 0, 0]])
    for i in range(1, n):
        dtheta = np.random.uniform(low=-np.pi/4, high=np.pi/4)
        theta = p[i - 1][2] + dtheta
        r = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        x = p[i - 1, :2] + (r @ np.random.uniform(low=[[0], [0]],
                                                  high=[[5], [0]])).reshape(1, 2)
        x = np.append(x, [[theta]], axis=-1)
        p = np.append(p, x, axis=0)
    n = len(p)
    u = 0.1
    dp = np.zeros((0, 3))
    for i, p_i in enumerate(p):
        x = p_i
        if i > 0:
            x = x - p[i - 1]
        dp = np.append(
            dp, np.array([x]), axis=0)
    dp = np.array(dp)
    # print(dp)

    odo_stream = dp + (np.random.rand(n, 3) -
                       np.array([[0.5] * 3] * n)) * u
    sen_stream = dp + (np.random.rand(n, 3) -
                       np.array([[0.5] * 3] * n)) * u / 2

    q = np.cumsum(odo_stream, axis=0)
    s = np.cumsum(sen_stream, axis=0)

    # print(odo_stream)
    print('start!')
    x = GraphSLAM().run(odo_stream, sen_stream).reshape((-1, 3))
    # print(x)

    plt.plot(p[:, 0], p[:, 1], c='black')
    plt.plot(q[:, 0], q[:, 1], c='b')
    plt.plot(s[:, 0], s[:, 1], c='g')
    plt.plot(x[:, 0], x[:, 1], c='r')
    plt.show()
