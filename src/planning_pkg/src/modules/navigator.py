from math import acos, radians, sin

ROBOT_DIMENSION = 0.08
DEFAULT_SPEED = 5
TRUCK_LENGHT = 0.001
TURN_CONSTANT = 2.5
TURN_TRESH = 0


def get_angle_diff(angle1, angle2):
    return (angle1 - angle2 + 180) % 360 - 180


def get_turn_period(angle1, angle2):
    return 2/TURN_CONSTANT*acos(1-(TURN_CONSTANT*TRUCK_LENGHT)/(2*DEFAULT_SPEED)*abs(radians(get_angle_diff(angle1, angle2))))


def get_truck_angle(timer, turn_period):
    if timer <= turn_period/2:
        return TURN_CONSTANT*timer
    return TURN_CONSTANT*(turn_period-timer)


class Navigator:

    def __init__(self):
        self.start_turn_time = 0
        self.end_turn_time = 0
        self.turn_period = 0

    def get_motor_speed(self, current_position, current_planned_position, timer):
        if abs(get_angle_diff(current_position[2], current_planned_position[2])) >= TURN_TRESH or timer/1000 < self.end_turn_time:
            if not self.turn_period:
                self.turn_period = get_turn_period(
                    current_position[2], current_planned_position[2])
            if not self.start_turn_time:
                self.start_turn_time = timer/1000
                self.end_turn_time = self.start_turn_time + self.turn_period
            if timer/1000-self.start_turn_time <= self.turn_period/2:
                omega = DEFAULT_SPEED/TRUCK_LENGHT * \
                    sin(get_truck_angle(timer/1000 - self.start_turn_time,
                                        self.turn_period)) + TURN_CONSTANT
            elif timer/1000-self.start_turn_time <= self.turn_period:
                omega = DEFAULT_SPEED/TRUCK_LENGHT * \
                    sin(get_truck_angle(timer/1000 - self.start_turn_time,
                                        self.turn_period)) - TURN_CONSTANT
            else:
                omega = 0
                self.start_turn_time = 0
                self.end_turn_time = 0
                self.turn_period = 0
            if get_angle_diff(current_position[2], current_planned_position[2]) > 0:
                omega *= -1
        else:
            self.start_turn_time = 0
            self.end_turn_time = 0
            self.turn_period = 0
            omega = 0

        return (DEFAULT_SPEED - omega*ROBOT_DIMENSION/2, DEFAULT_SPEED + omega*ROBOT_DIMENSION/2)

    def clear(self):
        print('Navigator : Clearing data...')
        self.start_turn_time = 0
        self.end_turn_time = 0
        self.turn_period = 0
