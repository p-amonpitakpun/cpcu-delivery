from collections import deque
from math import ceil

ANGLE = {-180, -135, -90, -45, 0, 45, 90, 135, 180}
WALL_DISTANCE_THRES = 3

def get_angle_diff(angle1, angle2):
    return (angle1 - angle2 + 180) % 360 - 180

def normalized_angle(angle):
    return get_angle_diff(angle, 0)

def closest_angle(angle):
    if angle % 45 == 0:
        return normalized_angle(angle)
    return normalized_angle(angle - (angle % 45))

def get_possible_position(position):
    result = tuple()
    _closest_angle = closest_angle(position[2])
    if position[2] == _closest_angle:
        if _closest_angle == 0:
            result += ((position[0] + 1, position[1], _closest_angle),)
        elif position[2] == 45:
            result += ((position[0] + 1, position[1] + 1, _closest_angle),)
        elif position[2] == 90:
            result += ((position[0], position[1] - 1, _closest_angle),)
        elif position[2] == 135:
            result += ((position[0] - 1, position[1] - 1, _closest_angle),)
        elif position[2] == -45:
            result += ((position[0] + 1, position[1] + 1, _closest_angle),)
        elif position[2] == -90:
            result += ((position[0], position[1] + 1, _closest_angle),)
        elif position[2] == -135:
            result += ((position[0] - 1, position[1] + 1, _closest_angle),)
        elif abs(position[2]) == 180:
            result += ((position[0] - 1, position[1], _closest_angle),)
    for angle in ANGLE.difference({position[2]}):
        result += ((position[0], position[1], angle),)
    return result

def check_wall(point, map):
    x, y = point
    for i in map[y - WALL_DISTANCE_THRES:y + WALL_DISTANCE_THRES]:
        for j in i[x - WALL_DISTANCE_THRES:x + WALL_DISTANCE_THRES]:
            if not sum(j):
                return False
    return True

class Planner:

    def __init__(self):
        self.map = None
        self.current_position = None
        self.goal = None
        self.planned = deque()

    def update_map(self, map):
        self.map = map

    def update_position(self, position):
        self.current_position = (ceil(position[0]), ceil(
            position[1]), normalized_angle(position[2]))

    def update_goal(self, goal):
        self.goal = goal

    def plan(self):
        if not self.current_position:
            return
        print('Planner : Calculating...')
        queue = deque([self.current_position])
        start_point = self.current_position
        backtracker = {}
        while len(queue):
            position = queue.popleft()
            if position[:2] == self.goal:
                self.planned = deque()
                _position = position
                while _position != start_point:
                    self.planned.appendleft(_position)
                    _position = backtracker[_position]
                self.planned.appendleft(_position)
                return True
            
            for _position in get_possible_position(position):
                if _position[0] >= 0 and _position[1] >= 0 and _position[0] <= len(self.map[0]) - 1 and _position[1] <= len(self.map) - 1 and \
                _position not in backtracker and check_wall(_position[:2], self.map):
                    backtracker[_position] = position
                    queue.append(_position)

        self.planned = deque()
        return False
    
    def get_path(self):
        if self.planned:
            return list(self.planned)
        return [self.current_position]

    def clear(self):
        print('Planner : Clearing data...')
        self.goal = None
        self.planned = deque()