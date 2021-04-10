import numpy as np


class OccupancyGrid:
    def __init__(self, shape=(10, 10), resolution=1, logOdd_occ=0.9, logOdd_free=0.7):
        self.grid = np.zeros(shape=shape, dtype=float)
        self.resolution = resolution
        self.logOdd_occ = logOdd_occ
        self.logOdd_free = logOdd_free
        self.max_treshold = 100
        self.min_treshold = -100

    def getShape(self):
        return self.grid.shape

    def getOccupy(self, x, y):
        return self.grid[x, y]

    def setOccupiedCell(self, x, y):
        if x >= self.grid.shape[0] or y >= self.grid.shape[1]:
            raise Exception(
                "Error: index "
                + str((x, y))
                + " is out of range "
                + str(self.grid.shape)
                + "!"
            )

        if self.grid[y, x] + self.logOdd_occ <= self.max_treshold:
            self.grid[y, x] += self.logOdd_occ
        else:
            self.grid[y, x] = self.max_treshold

    def setFreeCell(self, x, y):
        if x >= self.grid.shape[0] or y >= self.grid.shape[1]:
            raise Exception(
                "Error: index "
                + str((x, y))
                + " is out of range "
                + str(self.grid.shape)
                + "!"
            )

        if self.grid[y, x] - self.logOdd_free > self.min_treshold:
            self.grid[y, x] -= self.logOdd_free
        else:
            self.grid[y, x] = self.min_treshold

    def updateOccupy(self, start_point, end_point):
        x0, y0 = start_point
        xf, yf = end_point
        x_int0 = int(x0 // self.resolution)
        y_int0 = int(y0 // self.resolution)
        x_intf = int(xf // self.resolution)
        y_intf = int(yf // self.resolution)
        if x_intf != x_int0:
            sign_x = (x_intf >= x_int0) - (x_intf < x_int0)
            X_int = range(x_int0, x_intf, sign_x)
            for x_int in X_int:
                y_int = int(
                    ((yf - y0) * (x_int * self.resolution - x0) / (xf - x0) + y0)
                    // self.resolution
                )
                if (
                    0 <= x_int < self.grid.shape[0]
                    and 0 <= y_int < self.grid.shape[1]
                    and (np.abs(x_int - x_int0) > 2 or np.abs(y_int - y_int0) > 2)
                ):
                    self.setFreeCell(x_int, y_int)

                y_int = int(
                    ((y_intf - y_int0) * (x_int - x_int0) / (x_intf - x_int0) + y_int0)
                )
                if (
                    0 <= x_int < self.grid.shape[0]
                    and 0 <= y_int < self.grid.shape[1]
                    and (np.abs(x_int - x_int0) > 2 or np.abs(y_int - y_int0) > 2)
                ):
                    self.setFreeCell(x_int, y_int)

        sign_x = 1 * (xf > x0) - 1 * (xf < x0)
        sign_y = 1 * (yf > y0) - 1 * (yf < y0)

        for dxy in [0, 1, 2]:

            x_int = x_intf + sign_x * dxy
            y_int = y_intf + sign_y * dxy

            if 0 <= x_int < self.grid.shape[0] and 0 <= y_int < self.grid.shape[1]:
                self.setOccupiedCell(x_int, y_int)


if __name__ == "__main__":
    shape = (10, 10)
    resolution = 1
    occupancy_grid = OccupancyGrid(shape=shape, resolution=resolution)
    # for x, y in [(x, np.sqrt(25 - x ** 2)) for x in range(6)]:
    # print(x, y)
    for _ in range(10):
        occupancy_grid.updateOccupy((0, 0), (10, 10))
    print(occupancy_grid.grid)
