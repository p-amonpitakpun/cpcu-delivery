import numpy as np


def euclidian_distant(point1, point2):
    assert len(point1) == len(point2)
    return np.sqrt(sum([(x1 - x2)**2 for x1, x2 in zip(point1, point2)]))


def closest_points(target, ref, dup=False):
    M_ref = ref.copy()

    C = []
    for p in target:
        index = np.argmin([euclidian_distant(p, m) for m in M_ref])
        C += [M_ref[index]]

        if not dup:
            mask = np.ones(len(M_ref), dtype=bool)
            mask[index] = False
            M_ref = M_ref[mask]
    C = np.array(C)
    np.testing.assert_array_equal(C.shape, target.shape)
    return C


def icp(ref, target, N_iter=10):
    n = len(target)
    P = target.copy()
    # print('shape X', X.shape)
    # print('shape P', P.shape)
    R = np.eye(2)
    T = np.array([0, 0])
    prev_err = None

    for i in range(N_iter):
        # print(f'# {i}')
        np.random.shuffle(P)
        X = closest_points(P, ref)
        np.testing.assert_array_equal(X.shape, P.shape)

        mean_X = np.mean(X, axis=0)
        mean_P = np.mean(P, axis=0)
        # print('mean X', mean_X)
        # print('mean P', mean_P)

        X0 = X - mean_X
        P0 = P - mean_P
        np.testing.assert_almost_equal(np.array([0, 0]), np.mean(X0, axis=0))
        np.testing.assert_almost_equal(np.array([0, 0]), np.mean(P0, axis=0))
        A = np.matmul(X0.T, P0)
        u, s, vh = np.linalg.svd(A)
        # print('u =')
        # print(u)
        # print('s =')
        # print(s)
        # print('vh =')
        # print(vh)
        r = np.matmul(u.T, vh)
        # print('r =')
        # print(r)
        t = mean_X - np.matmul(r, mean_P)
        # print('t =')
        # print(t)
        err = np.sum(
            np.sqrt(np.sum(
                (X[:] - np.matmul(r, P[:].T).T - t)**2, axis=1))) / n

        if prev_err is None or (prev_err is not None and prev_err > err):
            # print('err =', err)
            prev_err = err
            P = np.matmul(r, P[:].T).T + t
            R = np.matmul(r, R)
            T = np.matmul(r, T) + t
    return R, T


def tr_icp(ref, target, N=-1, N_iter=100):
    if N == -1:
        N = len(target)

    P = target.copy()
    R = np.eye(2)
    T = np.array([0, 0])
    S = np.inf

    for i in range(N_iter):
        # np.random.shuffle(P)
        x = closest_points(P, ref, dup=True)
        np.testing.assert_array_equal(x.shape, P.shape)

        pairs = sorted([[p, x] for p, x in zip(P, x)],
                       key=lambda x: euclidian_distant(x[0], x[1]))[:N]
        s_d = sum([euclidian_distant(p, x)**2 for p, x in pairs])

        # print('S_LTS =', S, '\tS\'_LTS =', s_d)
        if s_d < S:
            S = s_d
        else:
            return R, T

        p_d, x_d = list(zip(*pairs))

        mean_x = np.mean(x_d, axis=0)
        mean_p = np.mean(p_d, axis=0)
        X0 = x_d - mean_x
        P0 = p_d - mean_p
        A = np.matmul(X0.T, P0)
        u, s, vh = np.linalg.svd(A)
        r = np.matmul(u.T, vh)
        t = mean_x - np.matmul(r, mean_p)

        P = np.matmul(r, P.T).T + t
        R = np.matmul(r, R)
        T = np.matmul(r, T) + t

        # print('#', end=' ')
    return R, T
