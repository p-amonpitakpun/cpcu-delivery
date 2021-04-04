import numpy as np


RNG_SEED = 0

class ParticleFilter():
    def __init__(self, ref_map=None, N=200):
        self.rng = np.random.default_rng(RNG_SEED)
        pass

    def update(self, transform, laser_scanner_data, info_matrix):

        new_particles = self.sampling()
        new_particles = self.predict(new_particles, transform)
        scores = self.get_scores(laser_scanner_data)

    def sampling(self):
        if len(self.scores) != len(self.particles):
            raise Exception(f'Error: the length of scores ({len(self.scores)}) is not equal to the number of particles ({len(self.particles)}).')

        new_particles = self.rng.choice(self.particles, size=self.N, p=self.scores)
        return new_particles

    def predict(self, particles, transform):
        if particles.shape[-1] != transform.shape[0]:
            raise Exception(f'Error: the shape of transformation {transform.shape} is not the same as the shape of particle {particles.shape}.')

        cov_matrix = np.array()