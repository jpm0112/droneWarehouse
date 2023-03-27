

import numpy as np






def create_instance_1(n,multiplier,seed):
    np.random.seed(seed)
    rnd = np.random
    rnd.seed(12)
    coord_x = [0]
    coord_y = [0]
    for i in range(n - 1):
        coord = rnd.rand() * 100
        coord_x.append(coord)

    for i in range(n - 1):
        coord = rnd.rand() * 100
        coord_y.append(coord)

    coord_x[-2] = 200
    coord_y[-2] = 200
    coord_x[-3] = 210
    coord_y[-3] = 180

    coord_x[3] = coord_x[3] + 10
    coord_y[3] = coord_y[3] + 10
    depot = [0]
    orders_list = [i for i in range(1, n)]  # [] es lista
    nodes = depot + orders_list  # suma de lista concatena las dos listas

    coord_x[-1] = coord_x[0]
    coord_y[-1] = coord_y[0]

    # drone distances
    drone_distances = np.random.randint(1, 10, size=(n, n))
    for i in range(n):
        for j in range(n):
            point1 = np.array((coord_x[i], coord_y[i]))
            point2 = np.array((coord_x[j], coord_y[j]))
            drone_distances[i, j] = np.linalg.norm(point1 - point2)

    # WORKER DISTANCE MATRIX
    worker_distances = drone_distances * multiplier  # CHANGE FOR WORKER DISTANCE MATRIX
    np.fill_diagonal(drone_distances, 1000000)
    np.fill_diagonal(worker_distances, 1000000)
    return (coord_x,coord_y,drone_distances,worker_distances, nodes)


def create_instance_2(n,multiplier,seed):
    np.random.seed(seed)
    rnd = np.random
    rnd.seed(12)
    coord_x = [0]
    coord_y = [0]
    for i in range(n - 1):
        coord = rnd.rand() * 100
        coord_x.append(coord)

    for i in range(n - 1):
        coord = rnd.rand() * 100
        coord_y.append(coord)




    depot = [0]
    orders_list = [i for i in range(1, n)]  # [] es lista
    nodes = depot + orders_list  # suma de lista concatena las dos listas

    coord_x[-1] = coord_x[0]
    coord_y[-1] = coord_y[0]

    # drone distances
    drone_distances = np.random.randint(1, 10, size=(n, n))
    for i in range(n):
        for j in range(n):
            point1 = np.array((coord_x[i], coord_y[i]))
            point2 = np.array((coord_x[j], coord_y[j]))
            drone_distances[i, j] = np.linalg.norm(point1 - point2)

    # WORKER DISTANCE MATRIX
    worker_distances = drone_distances * multiplier  # CHANGE FOR WORKER DISTANCE MATRIX
    np.fill_diagonal(drone_distances, 1000000)
    np.fill_diagonal(worker_distances, 1000000)
    return (coord_x,coord_y,drone_distances,worker_distances, nodes)