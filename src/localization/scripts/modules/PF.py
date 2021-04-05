import json
import numpy as np

from datetime import datetime


RNG_SEED = 0
np.random.seed(RNG_SEED)

class ParticleFilter():
    def __init__(self, N=200):
        self.rng = np.random.default_rng(RNG_SEED)
        self.last_update = None

    def init(self, ref_map_path=None, ref_map_config_path=None, init_pose=np.array([0, 0, 0]), init_vel=np.array([0, 0, 0])):
        if ref_map_path is None or ref_map_config_path is None:
            raise Exception('ref_map_path or ref_map_config_path is None')

        with open(ref_map_path, 'r') as fp:
            self.ref_map = np.load(fp)

        with open(ref_map_config_path, 'r') as fp:
            self.ref_map_config = json.load(fp)

        self.particle = init_pose
        self.particle_velocity = init_vel

    def update(self, transform, laser_scanner_data):
        now = datetime.now()
        dt_s = (now - self.last_update).total_seconds() if self.last_update is not None else 0

        new_particles = self.sampling()
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


    def sampling(self):
        if len(self.scores) != len(self.particles):
            raise Exception(f'Error: the length of scores ({len(self.scores)}) is not equal to the number of particles ({len(self.particles)}).')

        new_particles = self.rng.choice(self.particles, size=self.N, p=self.scores)
        return new_particles

    def predict(self, particles, transform):
        if particles.shape[-1] != transform.shape[0]:
            raise Exception(f'Error: the shape of transformation {transform.shape} is not the same as the shape of particle {particles.shape}.')

        # cov_matrix = np.array([[1, 0, 1], [0, 1, 1], [1, 1, 1]]) * 1e-6
        transform_err = 5e-3

        predicted_particles = np.random.uniform(transofrm - transform_err, 
                                                transform + transform_err, 
                                                particles.shape)

        return predicted_particles

    def get_scores(self, particles, laser_scanner_data):
        scores_length = len(particles.shape[0])
        scores = []
        for i in range(scores_length):
            score = np.random.rand()
            scores.append(score)
        return scores

    def getLoc(self):
        now = datetime.now()
        dt_s = (now - self.last_update).total_seconds() if self.last_update is not None else 0
        return self.particle + self.particle_velocity * dt_s