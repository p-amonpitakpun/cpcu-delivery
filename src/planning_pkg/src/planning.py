#!/usr/bin/env python3

import json
import rospy

from robot_state.srv import *
from std_msgs.msg import Float32MultiArray
from datetime import datetime
from collections import defaultdict, deque
from heapq import heappush, heappop
from math import hypot, degrees, radians, sin, acos, atan2

DEFAULT_SPEED = 1
TRUCK_LENGHT = 2
TURN_CONSTANT = 0.5
POSITION_ERROR_THRES = 0.1
DIRECTION_ERROR_THRES = 0.1
Kp = 0.01
Ki = 0.005
Kd = 0.001

SET_GOAL = 0
MOVE = 1
SET_MAP = 2

ROBOT_DIMENSION = 1

def get_angle(timer, period):
    if timer <= period/2:
        return TURN_CONSTANT*timer
    return TURN_CONSTANT*(period-timer)

class LocationError(Exception):
    pass

def default_inf():
    return float('inf')

class Planner:

    def __init__(self):
        self.graph = defaultdict(tuple)
        self.current_position = None
        self.start_direction = None
        self.goal = None
        self.final_direction = None
        self.path = deque()
        self.all_path = tuple()

    def set_position(self, x: int, y: int, z: float):
        if (x, y) not in self.graph:
            raise LocationError
        self.current_position = (x, y)
        self.start_direction = z
        return True

    def set_goal(self, x: int, y: int, z: float):
        self.goal = (x, y)
        self.final_direction = z
        return True

    def add_path(self, u: tuple, v: tuple):
        self.graph[u] = self.graph[u] + (v, )
        self.graph[v] = self.graph[v] + (u, )

    def del_node(self, node: tuple):
        self.graph.pop(node, None)
        for point in self.graph:
            self.graph[point] = tuple(
                e for e in self.graph[point] if e != node)

    def calculate_shortest_path(self):
        if not self.current_position or not self.goal:
            return False
        backtracker = {}
        distances = defaultdict(default_inf)
        distances[self.current_position] = 0
        queue = [(0, self.current_position)]
        while queue:
            current_distance, current_position = heappop(queue)
            if current_distance <= distances[current_position]:
                for neighbor in self.graph[current_position]:
                    distance = hypot(
                        current_position[0]-neighbor[0], current_position[1]-neighbor[1])
                    if distance < distances[neighbor] and neighbor in self.graph:
                        distances[neighbor] = distance
                        backtracker[neighbor] = current_position
                        if neighbor == self.goal:
                            self.path = deque()
                            while neighbor != self.current_position:
                                self.path.appendleft(neighbor)
                                neighbor = backtracker[neighbor]
                            self.path.appendleft(neighbor)
                            return True
                        heappush(queue, (distance, neighbor))
        self.path = deque()
        return False

class DifferentialDrive:

    def __init__(self, dimension: float):
        self.dimension = dimension
        self.robot_motion = []
        self.turn_timer = 0
        self.turn_period = 0
        self.sum_error = 0
        self.prev_error = 0
        self.diff_error = 0

    def create_robot_motion(self, planner: Planner):
        degree = None
        self.robot_motion = deque()
        for idx, path in enumerate(planner.path):
            if not idx:
                continue
            if idx+1 < len(planner.path):
                degree = degrees(atan2(
                    planner.path[idx+1][1]-planner.path[idx][1], planner.path[idx+1][0]-planner.path[idx][0]))
            self.robot_motion.append(path+(degree, ))
        self.robot_motion.appendleft(
            planner.current_position+(planner.start_direction,))
        self.robot_motion[len(self.robot_motion)-1] = (self.robot_motion[len(self.robot_motion)-1][0], 
        self.robot_motion[len(self.robot_motion)-1][1], planner.final_direction)

    def get_motor_speed(self, pos_x: float, pos_y: float, direction: float, time: float, obstacle: bool = False):
        if obstacle:
            return (0, 0)
        try:
            if (((pos_x-self.robot_motion[0][0])**2+(pos_y-self.robot_motion[0][1])**2)**0.5 < POSITION_ERROR_THRES):
                self.robot_motion.popleft()
            if abs(radians(self.robot_motion[0][2]-direction)) > DIRECTION_ERROR_THRES:
                self.prev_error = 0
                self.sum_error = 0
                self.diff_error = 0
                if not self.turn_timer:
                    try:
                        self.turn_period = 2/TURN_CONSTANT * \
                            acos(1-(TURN_CONSTANT*self.dimension/(2*DEFAULT_SPEED)
                                    * radians(self.robot_motion[0][2]-direction)))
                    except ValueError:
                        self.turn_period = 2/TURN_CONSTANT * \
                            acos(1-(TURN_CONSTANT*self.dimension/(2*DEFAULT_SPEED)
                                    * radians(direction-self.robot_motion[0][2])))
                    self.turn_timer = time
                if time-self.turn_timer <= self.turn_period/2:
                    omega = DEFAULT_SPEED/TRUCK_LENGHT * \
                        sin(get_angle(time-self.turn_timer,
                                      self.turn_period)) + TURN_CONSTANT
                else:
                    omega = DEFAULT_SPEED/TRUCK_LENGHT * \
                        sin(get_angle(time-self.turn_timer,
                                      self.turn_period)) - TURN_CONSTANT
            else:
                error = radians(direction-self.robot_motion[0][2])
                self.sum_error += error
                self.diff_error = error-self.prev_error
                self.turn_timer = 0
                omega = error*Kp+self.sum_error*Ki+self.diff_error*Kd
            if direction > self.robot_motion[0][2]:
                omega *= -1
            return (DEFAULT_SPEED + omega*self.dimension/2, DEFAULT_SPEED - omega*self.dimension/2)
        except IndexError:
            return (0, 0)

memory = []
obstacle = False
navigation = Planner()
diffdrive = DifferentialDrive(ROBOT_DIMENSION)

def publish():
    publisher = rospy.Publisher('robotControl', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    msg = Float32MultiArray()
    while not rospy.is_shutdown():
        if navigation.current_position:
            pos_x, pos_y, direction = navigation.current_position
            message = diffdrive.get_motor_speed(pos_x, pos_y, direction, datetime.now().timestamp(), obstacle)
            message = Float32MultiArray(data=message)
            publisher.publish(message)
        else:
            message = Float32MultiArray(data=[1, 1])
            publisher.publish(message)
        rate.sleep()


def dummie():
    navigation.add_path((0, 0), (1, 1))
    navigation.add_path((1, 1), (1, 2))
    navigation.set_position(0, 0, 45)
    navigation.set_goal(1, 2, 45)
    navigation.calculate_shortest_path()
    diffdrive.create_robot_motion(navigation)
    print('dummie')

def robot_state_callback(req):
    
    commands = json.loads(req.req)
    response = {'status':'type error'}
    if commands['type'] == SET_GOAL:
        dummie()
        response = commands
        response['status'] = 'work!'
    elif commands['type'] == MOVE:
        pass
    elif commands['type'] == SET_MAP:
        pass
    return json.dumps(response)

def main():

    rospy.init_node('planning_runner', anonymous=True)
    rospy.Service('planning', Planning, robot_state_callback)
    publish()


if __name__ == "__main__":
    print('started planning')
    main()
