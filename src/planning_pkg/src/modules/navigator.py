from math import pi

ROBOT_DIMENSION = 0.2
DEFAULT_SPEED = 3
OMEGA = pi / 4
_OMEGA = OMEGA * ROBOT_DIMENSION / 0.08 / pi * 180

ERRROR_THRES = 10
DIRECTION_THRES = 15

def get_angle_diff(angle1, angle2):
    return (angle1 - angle2 + 180) % 360 - 180

class Navigator:

    def __init__(self):
        self.start_turn_time = 0
        self.turn_period = 0
        self.lastest_speed = (None, None)

    def get_motor_speed(self, current_position, current_planned_position, timer):
        if not current_position or not current_planned_position:
            return (0, 0)
        if abs(get_angle_diff(current_position[2], current_planned_position[2])) >= DIRECTION_THRES and not self.turn_period:
            self.turn_period = abs(get_angle_diff(current_position[2], current_planned_position[2]))/_OMEGA * pi / 180
            self.start_turn_time = timer
        if timer < self.start_turn_time + self.turn_period:
            if get_angle_diff(current_planned_position[2], current_position[2]) < 0:
                self.lastest_speed = (OMEGA*ROBOT_DIMENSION/2, - _OMEGA*ROBOT_DIMENSION/2)
                return (OMEGA*ROBOT_DIMENSION/2, - _OMEGA*ROBOT_DIMENSION/2)
            elif get_angle_diff(current_planned_position[2], current_position[2]) > 0:
                self.lastest_speed = (- OMEGA*ROBOT_DIMENSION/2, _OMEGA*ROBOT_DIMENSION/2)
                return (- OMEGA*ROBOT_DIMENSION/2, _OMEGA*ROBOT_DIMENSION/2)
            else:
                return self.lastest_speed
        else:
            self.lastest_speed = (None, None)
            self.turn_period = 0
            self.start_turn_time = 0
            return (DEFAULT_SPEED, DEFAULT_SPEED)