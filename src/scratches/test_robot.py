import numpy as np
import time
import threading


V_FORWARD = [5.0, 5.0]
V_ROT_LEFT = [-2.0, 2.0]
V_ROT_RIGHT = [2.0, -2.0]

class Robot:
    def __init__(self, pos, orient):
        self.v = [0.0, 0.0]

        self.wheel_r = 1.0
        self.wheel_d = 1.0

        self.pos = pos
        self.orient = orient

        self.thread = None


    def setV(self, vx, vy):
        self.v = [vx, vy]

    def goto(self, pos, pos_threshold, rot_threshold, timeout_s):
        time_start_s = time.time()

        disp = pos - self.pos
        expect_orient = np.arctan2(disp[0], disp[1])

        while np.abs(expect_orient - self.orient) > pos_threshold:
            time_now_s = time.time() - time_start_s
            if time_now_s >= timeout_s:
                raise Exception('ERROR: goto Timeout {:.3f} s')

            orient_diff = (expect_orient - self.orient + 360) % 360

            if orient_diff > 0:
                self.setV(*V_ROT_RIGHT)

            elif orient_diff < 0:
                self.setV(*V_ROT_LEFT)

            else:
                print(0)
                break

        self.setV(*V_FORWARD)
        while np.linalg.norm(pos - self.pos) > pos_threshold:
            time_now_s = time.time() - time_start_s
            if time_now_s >= timeout_s:
                raise Exception('ERROR: goto Timeout {:.3f} s')
        self.setV(0, 0)

    def update(self, dt_s):
        v = (self.v[0] + self.v[1]) * self.wheel_r / 2
        w = (self.v[0] - self.v[1]) * self.wheel_r * 2 / self.wheel_d

        self.orient += w * dt_s
        dPosX = v * dt_s * np.sin(self.orient)
        dPosY = v * dt_s * np.cos(self.orient)
        self.pos = self.pos + np.array([dPosX, dPosY])

    def run(self):
        pass
    
    def start(self):
        pass
    
