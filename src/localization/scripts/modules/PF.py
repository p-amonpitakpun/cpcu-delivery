import cv2
import json
import numpy as np

from datetime import datetime

from .occupancy_grid import OccupancyGrid
from .LPF import LowPassFilter


RNG_SEED = 0
np.random.seed(RNG_SEED)


class ParticleFilter():
    def __init__(self, N=200):
        self.state = 0

        self.rng = np.random.default_rng(RNG_SEED)
        self.last_update = None
        self.N = N
        self.M = 8 * N
        self.images = dict()
        self.particle_hist = []
        self.real_pose_hist = []

    def init(self, ref_map_path=None, ref_map_config_path=None, init_pose=np.array([0, 0, 0]), init_vel=np.array([0, 0, 0])):
        if ref_map_path is None or ref_map_config_path is None:
            raise Exception('ref_map_path or ref_map_config_path is None')

        # with open(ref_map_path, 'r') as fp:
            # self.ref_map = np.load(fp)
        self.ref_map = np.load(ref_map_path)

        with open(ref_map_config_path, 'r') as fp:
            self.ref_map_config = json.load(fp)

        self.occGrid = OccupancyGrid(
            shape=self.ref_map.shape,
            resolution=self.ref_map_config['resolution'],
            logOdd_occ=self.ref_map_config['logOdd_occ'],
            logOdd_free=self.ref_map_config['logOdd_free'])
        self.occGrid.grid = self.ref_map

        self.particle = init_pose
        self.particle_velocity = init_vel

        self.particles = None
        self.scores = np.array([1] * self.M) / self.M

        self.obs_grid = OccupancyGrid(
            shape=self.ref_map.shape,
            resolution=self.ref_map_config['resolution'],
            logOdd_occ=50.0,
            logOdd_free=50.0)
        self.obs_grid.grid = np.zeros(self.ref_map.shape)

        self.init_pose = init_pose
        self.init_pose_error = [0.1, 0.1, 5 * np.pi / 180]

        self.threshold = 0.6
        self.bias = 0.4

        self.lpf = LowPassFilter(1000)

    def update(self, transform, laser_scanner_data, real_pose=None):
        now = datetime.now()
        dt_s = (
            now - self.last_update).total_seconds() if self.last_update is not None else 0

        new_particles = self.sample()
        new_particles = self.predict(new_particles, transform)
        scores = self.get_scores(new_particles, laser_scanner_data)

        self.particles = new_particles
        self.scores = scores

        # select a particle with maximum score
        selected_particle = new_particles[scores.argmax()]
        dx = selected_particle - self.particle

        # self.updateOcc(self.particle, laser_scanner_data)

        self.particle_velocity = dx / dt_s if dt_s > 0 else self.particle_velocity
        self.particle = self.lpf.calc(selected_particle)
        self.particle[2] = np.fmod(self.particle[2], np.pi * 2)
        self.last_update = now

        if self.M / self.N > 1:
            self.M = int(self.M - 100)
        else:
            self.state = 1
            self.particle_hist.append(self.particle.copy())
            self.updateObsGrid(laser_scanner_data)

            if real_pose is not None:
                self.validate(real_pose)

    def getState(self):
        return self.state

    def getPose(self):
        return self.particle.copy()

    def getCell(self):
        return self.cvtSim2Grid(self.particle.copy())

    def getMap(self):
        k = self.bias
        grid = (k * self.occGrid.getProbabilityMap() + (1 - k) * self.obs_grid.getProbabilityMap()) > self.threshold
        img = (255 - 255 * grid).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img.copy()

    def getImage(self):
        image_list = list(self.images.items())
        N = len(image_list)
        if N > 0:
            ret_name, ret_image = image_list[0]
            ret_image = cv2.hconcat(list(self.images.values()))
            for name, _ in image_list[1:]:
                ret_name += ', ' + name
            return ret_name, ret_image
        else:
            return None, None

    def sample(self):

        if self.particles is None:
            x, y, z = self.init_pose
            dx, dy, dz = self.init_pose_error

            samples = np.random.uniform(low=np.array(
                [x - dx, y - dy]), high=np.array([x + dx, y + dy]), size=(self.N, 2))
            new_samples = []
            for s in samples:
                for X in np.random.uniform(low=np.array([s[0], s[1], z - dz]), high=np.array([s[0], s[1], z + dz]), size=(8, 3)):
                    new_samples.append(X)
            return np.array(new_samples)

        if len(self.scores) != len(self.particles):
            raise Exception(
                f'Error: the length of scores ({len(self.scores)}) is not equal to the number of particles ({len(self.particles)}).')

        new_particles = self.rng.choice(
            self.particles, size=self.M, p=self.scores)
        return new_particles

    def predict(self, particles, transform):
        if particles.shape[-1] != transform.shape[0]:
            raise Exception(
                f'Error: the shape of transformation {transform.shape} is not the same as the shape of particle {particles.shape}.')

        transform_err = np.array([5e-3, 5e-3, 0.01 * np.pi / 180])

        predicted_particles = particles + np.random.uniform(low=transform - transform_err,
                                                            high=transform + transform_err,
                                                            size=particles.shape)

        return predicted_particles

    def cvtSim2Grid(self, sim_pose):
        reso = self.occGrid.resolution
        y_shape, x_shape = self.occGrid.getShape()
        x_offset, y_offset = x_shape // 2, y_shape // 2

        return [int(sim_pose[0] // reso + x_offset), int(- sim_pose[1] // reso + y_offset)]

    def get_scores(self, particles, laser_scanner_data):

        threshold = 0.6
        k = 0.5

        scores_length = len(particles)
        scores = [0] * scores_length

        y_shape, x_shape = self.occGrid.getShape()

        particle_cells = []
        particle_scans = []

        for i, particle in enumerate(particles):
            X = particle
            point_cell = self.cvtSim2Grid(X)
            particle_cells.append(point_cell)

            dx = particle
            psi = dx[2]
            R = np.array([[np.cos(psi), - np.sin(psi)],
                          [np.sin(psi), np.cos(psi)]])
            T = dx[0: 2].reshape((2, 1))

            score = 0
            scan = []
            for p in laser_scanner_data:
                p = np.dot(p, R.T) + T.T
                p = p.reshape((2,))
                point = self.cvtSim2Grid(p)
                if 0 <= point[0] < x_shape and 0 <= point[1] < y_shape:
                    grid_score = self.occGrid.getProbabilty(*point)
                    # grid_score = k * self.occGrid.getProbabilty(*point) + (1 - k) * self.obs_grid.getProbabilty(*point)
                    score += grid_score if grid_score > threshold else 0
                scan.append(point)

            if self.occGrid.getOccupy(*point_cell) == 0:
                score = 0
            scores[i] = score if score > 0 else 0.00001
            particle_scans.append(scan)

        norm_scores = [1 / scores_length] * \
            scores_length if np.sum(scores) == 0 else scores / np.sum(scores)

        max_score = np.max(norm_scores)
        particle_list = list(zip(particle_cells, particle_scans, norm_scores))
        img = self.occGrid.getImage2(threshold)
        show_ghost_particles = False
        if show_ghost_particles:
            for pose, scan, score in particle_list:
                if score == max_score:
                    pass
                else:
                    img = cv2.circle(img, tuple(pose), 2, (0, 255, 0), -1)
                    for point in scan:
                        img = cv2.circle(img, tuple(point), 1, (70, 70, 105), -1)

        idx = np.argmax(norm_scores)
        for point in self.particle_hist:
            img = cv2.circle(img, tuple(
                self.cvtSim2Grid(point)), 2, (255, 255, 0), -1)
        img = cv2.circle(img, tuple(
            particle_list[idx][0]), 2, (255, 50, 0), -1)
        for point in particle_list[idx][1]:
            img = cv2.circle(img, tuple(point), 2, (0, 0, 255), -1)

        self.images['particles'] = img

        return np.array(norm_scores)

    def updateOcc(self, particle, laser_scanner_data):
        offset = 20

        X = particle

        psi = particle[2]
        R = np.array([[np.cos(psi), - np.sin(psi)],
                      [np.sin(psi), np.cos(psi)]])
        T = particle[: 2].reshape((2, 1))

        for p in laser_scanner_data:
            p = np.dot(p, R.T) + T.T
            p = p.reshape((2,))
            self.occGrid.updateOccupy(
                (X[0] + offset, - X[1] + offset), (p[0] + offset, - p[1] + offset))

    def updateObsGrid(self, laser_scanner_data):
        k = self.bias

        y_shape, x_shape = self.occGrid.getShape()
        reso = self.occGrid.resolution
        offset_x, offset_y = reso * x_shape / 2, reso * y_shape / 2

        X = self.particle

        dx = self.particle
        psi = dx[2]
        R = np.array([[np.cos(psi), - np.sin(psi)],
                      [np.sin(psi), np.cos(psi)]])
        T = dx[0: 2].reshape((2, 1))

        for p in laser_scanner_data:
            p = np.dot(p, R.T) + T.T
            p = p.reshape((2,))
            point = self.cvtSim2Grid(p)

            if 0 <= point[0] < x_shape and 0 <= point[1] < y_shape:
                self.obs_grid.updateOccupy(
                    (X[0] + offset_x, - X[1] + offset_y), (p[0] + offset_x, - p[1] + offset_y))

        img = self.obs_grid.getImage2(self.threshold)
        img = cv2.circle(img, tuple(self.cvtSim2Grid(X)), 2, (255, 50, 0), -1)
        # self.images['obs'] = img.copy()


        img = (255 - 255 * ((k * self.occGrid.getProbabilityMap() + (1 - k) * self.obs_grid.getProbabilityMap()) > self.threshold)).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img = cv2.circle(img, tuple(self.cvtSim2Grid(X)), 2, (255, 50, 0), -1)
        self.images['blend'] = img.copy()

    def validate(self, real_pose):
        y_shape, x_shape = self.occGrid.getShape()

        img = self.occGrid.getImage2(0.5)
        img = cv2.circle(img, tuple(self.cvtSim2Grid(real_pose)), 2, (0, 0, 255), -1)
        img = cv2.circle(img, tuple(self.cvtSim2Grid(self.particle)), 2, (255, 50, 0), -1)
        img = cv2.putText(img, f'{self.particle[0]:0.06f}', (0, y_shape - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        img = cv2.putText(img, f'{real_pose[0]:0.06f}', (120, y_shape - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        img = cv2.putText(img, f'{self.particle[1]:0.06f}', (0, y_shape - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        img = cv2.putText(img, f'{real_pose[1]:0.06f}', (120, y_shape - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        img = cv2.putText(img, f'{self.particle[2]:0.06f}', (0, y_shape - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        img = cv2.putText(img, f'{real_pose[2]:0.06f}', (120, y_shape - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        # self.images['validate'] = img.copy()

        img = self.obs_grid.getImage2(self.threshold)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, t = cv2.threshold(img, 125, 100, cv2.THRESH_BINARY_INV)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        contours, h = cv2.findContours(t, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            l = cv2.arcLength(cnt, True)
            print(l)

            if l < 100:
                c = (0, 0, 255)
                # cv2.drawContours(img, cnt, -1, c, 3)
                
                if len(cnt) > 5:
                    e = cv2.fitEllipseAMS(cnt)
                    img = cv2.ellipse(img, e, c, 3)

        # self.images['cnt'] = img.copy()