from collections import deque
from math import ceil, sqrt
import numpy

ANGLE = [-180, -135, -90, -45, 0, 45, 90, 135, 180]
WALL_DISTANCE_THRES = 5


def get_angle_diff(angle1, angle2):
    return (angle1 - angle2 + 180) % 360 - 180


def normalized_angle(angle):
    return get_angle_diff(angle, 0)


def closest_angle(angle):
    if angle % 45 == 0:
        return (angle, normalized_angle(angle - 45), normalized_angle(angle + 45))
    new_angle = angle - (angle % 45)
    return (new_angle, normalized_angle(new_angle - 45), normalized_angle(new_angle + 45), normalized_angle(new_angle + 90))


def offset_angle(angle, offset):
    if angle < 0:
        return angle - offset
    return angle + offset

def check_wall(point, map):
    x, y = point
    for i in map[y - WALL_DISTANCE_THRES:y + WALL_DISTANCE_THRES]:
        for j in i[x - WALL_DISTANCE_THRES:x + WALL_DISTANCE_THRES]:
            if not sum(j):
                return False
    return True


def get_posible_path(x, y, direction):
    result = tuple()
    for angle in closest_angle(direction):
        if angle == 0:
            result += ((x + 1, y, angle),)
        elif angle == 45:
            result += ((x + 1, y - 1, angle),)
        elif angle == 90:
            result += ((x, y + 1, angle),)
        elif angle == 135:
            result += ((x - 1, y - 1, angle),)
        elif angle == -45:
            result += ((x + 1, y + 1, angle),)
        elif angle == -90:
            result += ((x, y - 1, angle),)
        elif angle == -135:
            result += ((x - 1, y + 1, angle),)
        elif abs(angle) == 180:
            result += ((x-1, y, angle),)
    return result


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
            x, y, direction = queue.popleft()
            if (x, y) == self.goal:
                self.planned = deque()
                current_position = (x, y, direction)
                while current_position != start_point:
                    self.planned.appendleft(current_position)
                    current_position = backtracker[current_position]
                self.planned.appendleft(current_position)
                return True
            for _x, _y, _direction in get_posible_path(x, y, direction):
                if _x >= 0 and _y >= 0 and _x <= len(self.map[0]) - 1 and _y <= len(self.map) - 1 and (_x, _y, _direction) not in backtracker and check_wall((_x, _y), self.map):
                    backtracker[(_x, _y, _direction)] = (x, y, direction)
                    queue.append((_x, _y, _direction))

        self.planned = deque()
        print('not found')
        return False
    
    def get_path(self):
        if self.planned:
            return list(self.planned)
        return [self.current_position]

    def clear(self):
        print('Planner : Clearing data...')
        self.goal = None
        self.planned = deque()
