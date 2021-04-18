import cv2
import json
import numpy as np

from datetime import datetime

from .occupancy_grid import OccupancyGrid


RNG_SEED = 0
np.random.seed(RNG_SEED)


class ParticleFilter():
    def __init__(self, N=200):
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

    def update(self, transform, laser_scanner_data):
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
        self.particle_velocity = dx / dt_s if dt_s > 0 else self.particle_velocity
        self.particle = selected_particle
        self.last_update = now

        if self.M / self.N > 1:
            self.M = int(self.M - 100)
        else:
            self.particle_hist.append(self.particle.copy())

    def getLoc(self):
        now = datetime.now()
        dt_s = (
            now - self.last_update).total_seconds() if self.last_update is not None else 0
        return self.particle + self.particle_velocity * dt_s

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

    def showRealPose(self, pose, laser_scanner_data):
        print('real pose\t', pose)
        reso = self.occGrid.resolution
        y_shape, x_shape = self.occGrid.getShape()
        x_offset, y_offset = x_shape // 2, y_shape // 2
        cell = self.cvSim2Grid(pose, reso, x_offset, y_offset)

        dx = np.array(pose)
        psi = dx[2]
        R = np.array([[np.cos(psi), - np.sin(psi)],
                        [np.sin(psi), np.cos(psi)]])
        T = dx[0: 2].reshape((2, 1))

        scan = []
        for p in laser_scanner_data:
            p = np.dot(p, R) + T.T
            p = p.reshape((2,))
            point = self.cvSim2Grid(p, reso, x_offset, y_offset)
            scan.append(point)

        img = self.occGrid.getImage(-50, 50)
        for prev in self.real_pose_hist:
            img = cv2.circle(img, tuple(self.cvSim2Grid(prev, reso, x_offset, y_offset)), 1, (255, 105, 0), -1)
        img = cv2.circle(img, tuple(cell), 5, (255, 0, 0), -1)
        for point in scan:
            img = cv2.circle(img, tuple(point), 2, (0, 0, 255), -1)

        self.images['real_pose'] = img
        self.real_pose_hist.append(pose)

    def sample(self):

        if self.particles is None:
            samples = np.random.uniform(low=np.array([-2, -3]), high=np.array([2, 0]), size=(self.N, 2))
            new_samples = []
            for s in samples:
                new_samples.append([s[0], s[1], 0])
                new_samples.append([s[0], s[1], np.pi / 2])
                new_samples.append([s[0], s[1], np.pi])
                new_samples.append([s[0], s[1], np.pi * 3 / 2])
                new_samples.append([s[0], s[1], np.pi / 4])
                new_samples.append([s[0], s[1], np.pi * 3 / 4])
                new_samples.append([s[0], s[1], np.pi * 5 / 4])
                new_samples.append([s[0], s[1], np.pi * 7 / 4])
            return np.array(new_samples)

        if len(self.scores) != len(self.particles):
            raise Exception(f'Error: the length of scores ({len(self.scores)}) is not equal to the number of particles ({len(self.particles)}).')

        new_particles = self.rng.choice(
            self.particles, size=self.M, p=self.scores)
        return new_particles

    def predict(self, particles, transform):
        if particles.shape[-1] != transform.shape[0]:
            raise Exception(
                f'Error: the shape of transformation {transform.shape} is not the same as the shape of particle {particles.shape}.')

        # cov_matrix = np.array([[1, 0, 1], [0, 1, 1], [1, 1, 1]]) * 1e-6
        transform_err = np.array([1e-3, 1e-3, 0.05])

        predicted_particles = particles + np.random.uniform(low=transform - transform_err,
                                                            high=transform + transform_err,
                                                            size=particles.shape)

        return predicted_particles

    def cvSim2Grid(self, sim_pose, reso, x_offset, y_offset):
        return [int(sim_pose[0] // reso + x_offset), int( - sim_pose[1] // reso + y_offset)]

    def get_scores(self, particles, laser_scanner_data):
        scores_length = len(particles)
        scores = [0] * scores_length
        # for i in range(scores_length):
        #     score = np.random.rand()
        #     scores.append(score)

        reso = self.occGrid.resolution
        y_shape, x_shape = self.occGrid.getShape()
        x_offset, y_offset = x_shape // 2, y_shape // 2

        particle_cells = []
        particle_scans = []

        for i, particle in enumerate(particles):
            score = 0

            X = particle
            point_cell = self.cvSim2Grid(X, reso, x_offset, y_offset)
            particle_cells.append(point_cell)

            dx = particle
            # dx[1] = - dx[1]
            psi = dx[2]
            R = np.array([[np.cos(psi), - np.sin(psi)],
                          [np.sin(psi), np.cos(psi)]])
            T = dx[0: 2].reshape((2, 1))

            scan = []
            for p in laser_scanner_data:
                p = np.dot(p, R) + T.T
                p = p.reshape((2,))
                point = self.cvSim2Grid(p, reso, x_offset, y_offset)
                if 0 <= point[0] < x_shape and 0 <= point[1] < y_shape:
                    grid_score = self.occGrid.getOccupy(*point)
                    score += grid_score if grid_score > 0 else 0
                scan.append(point)

            if self.occGrid.getOccupy(*point_cell) == 0:
                score = 0
            scores[i] = score if score > 0 else 0.00001
            particle_scans.append(scan)

        norm_scores = [1 / scores_length] * \
            scores_length if np.sum(scores) == 0 else scores / np.sum(scores)

        max_score = np.max(norm_scores)
        particle_list = list(zip(particle_cells, particle_scans, norm_scores))
        img = self.occGrid.getImage(-50, 50)
        for pose, scan, score in particle_list:
            if score == max_score:
                pass
            else:
                img = cv2.circle(img, tuple(pose), 2, (0, 255, 0), -1)
                for point in scan:
                    img = cv2.circle(img, tuple(point), 1, (70, 70, 105), -1)

        idx = np.argmax(norm_scores)
        img = cv2.circle(img, tuple(particle_list[idx][0]), 2, (255, 0, 0), -1)
        for point in particle_list[idx][1]:
            img = cv2.circle(img, tuple(point), 2, (0, 0, 255), -1)

        for point in self.particle_hist:
            img = cv2.circle(img, tuple(self.cvSim2Grid(point, reso, x_offset, y_offset)), 2, (255, 0, 0), -1)

        print(f'\nloc: {particles[idx]}\tmaxscore: {max_score}\tM: {self.M}')

        self.images['particles'] = img

        return np.array(norm_scores)