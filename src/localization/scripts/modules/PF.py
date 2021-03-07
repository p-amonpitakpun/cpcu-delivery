class ParticleFilter():
    def __init__(self, ref_map=None):
        pass

    def update(self, odom_data, scanner_data, info_matrix):

        new_particles = self.sampling()
        new_particles = self.predict(new_particles, odom_data)
        scores = self.get_scores(scanner_data)