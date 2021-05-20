import numpy as np

from datetime import datetime


class LowPassFilter:
    def __init__(self, w):
        self.v = None
        self.w = w
        self.last_update = None

    def calc(self, x):
        now = datetime.now()
        if self.last_update is not None:
            dt = (now - self.last_update).total_seconds()
            b = np.exp(- self.w * dt)
            self.v = b * self.v + (1 - b) * x
        else:
            self.v = x
            self.last_update = now
        
        return self.v

    def getValue(self):
        return self.v
