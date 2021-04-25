#!/usr/bin/env python3

import json
import rospy

from robot_state.srv import *
from std_msgs.msg import Float32MultiArray
from datetime import datetime
from collections import defaultdict, deque
from heapq import heappush, heappop
from math import hypot, degrees, radians, sin, acos, atan2, ceil

DEFAULT_SPEED = 5
TRUCK_LENGHT = 2
TURN_CONSTANT = 2.5
POSITION_ERROR_THRES = 0.1
DIRECTION_ERROR_THRES = 1.57
Kp = 0.001
Ki = 0.00005
Kd = 0.000001

SET_GOAL = 0
MOVE = 1
SET_MAP = 2

ROBOT_DIMENSION = 1

def get_angle_diff(angle1, angle2):
    return (angle1-angle2+180)%360-180

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
        print('calculating...')
        queue = deque()
        backtracker = {}
        queue.append(self.current_position)
        while len(queue):
            position = queue.popleft()
            for p in self.graph[position]:
                if p not in backtracker:
                    queue.append(p)
                    backtracker[p] = position
                    if p == self.goal:
                        self.path = deque()
                        backtrack = self.goal
                        while backtrack != self.current_position:
                            self.path.appendleft(backtrack)
                            backtrack = backtracker[backtrack]
                        self.path.appendleft(backtrack)
                        print('Path : ', self.path)
                        return True
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
        self.current_position = None

    def create_robot_motion(self, planner: Planner):
        degree = None
        self.current_position = planner.current_position+(planner.start_direction,)
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
        print('Motion : ', self.robot_motion)

    def get_motor_speed(self, pos_x: float, pos_y: float, direction: float, time: float, obstacle: bool = False):
        self.current_position = (pos_x, pos_y, direction)
        if obstacle:
            return (-0.0001, -0.0001)
        try:
            current_idx = 0
            for idx, (x, y, z) in enumerate(self.robot_motion):
                if x == pos_x and y == pos_y:
                    current_idx = idx
            if current_idx == -1 or current_idx == len(self.robot_motion) - 1:
                raise IndexError
                
            if abs(get_angle_diff(self.robot_motion[current_idx][2],direction)) > DIRECTION_ERROR_THRES:
                self.prev_error = 0
                self.sum_error = 0
                self.diff_error = 0
                if not self.turn_timer:
                    try:
                        self.turn_period = 2/TURN_CONSTANT * \
                            acos(1-(TURN_CONSTANT*self.dimension/(2*DEFAULT_SPEED)
                                    * radians(self.robot_motion[current_idx][2]-direction)))
                    except ValueError:
                        self.turn_period = 2/TURN_CONSTANT * \
                            acos(1-(TURN_CONSTANT*self.dimension/(2*DEFAULT_SPEED)
                                    * radians(direction-self.robot_motion[current_idx][2])))
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
                error = radians(direction-self.robot_motion[current_idx][2])
                self.sum_error += error
                self.diff_error = error-self.prev_error
                self.turn_timer = 0
                omega = error*Kp+self.sum_error*Ki+self.diff_error*Kd
            if get_angle_diff(direction,self.robot_motion[current_idx][2])  > 0:
                omega *= -1
            return (DEFAULT_SPEED + omega*self.dimension/2, DEFAULT_SPEED - omega*self.dimension/2)
        except IndexError:
            print('Done!')
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
        if diffdrive.robot_motion and navigation.current_position:
            pos_x, pos_y = navigation.current_position
            direction = navigation.start_direction
            message = diffdrive.get_motor_speed(pos_x, pos_y, direction, datetime.now().timestamp(), obstacle)
            message = Float32MultiArray(data=message)
            publisher.publish(message)
        else:
            message = Float32MultiArray(data=[0, 0])
            publisher.publish(message)
        rate.sleep()

def create_map(navigation, map):
    for x in range(499):
        for y in range(499):
            if sum(map[y][x]) > 128*3:
                if sum(map[y+1][x]) > 128*3:
                    navigation.add_path((x, y), (x, y+1))
                if sum(map[y][x+1]) > 128*3:
                    navigation.add_path((x, y), (x+1, y))
    for i in range(499):
        if sum(map[499][i]) == sum(map[499][i+1]) and sum(map[499][i]) > 128*3:
            navigation.add_path((i, 499), (i+1, 499))
        if sum(map[i][499]) == sum(map[i+1][499]) and sum(map[i][499]) > 128*3:
            navigation.add_path((499, i), (499, i+1))
            
    navigation.del_node((250,250))
    navigation.add_path((250,250), (251, 250))

def robot_state_callback(req):
    global memory
    commands = json.loads(req.req)
    response = {'status':'type error'}
    if commands['type'] == SET_GOAL:
        navigation.set_goal(commands['goal'][0]+250, commands['goal'][1]+250, 0)
        response['status'] = 'ok'
    elif commands['type'] == MOVE:
        if commands['status'] == 0:
            # Pause
            memory = [navigation.goal]
            navigation.set_position(diffdrive.current_position)
            navigation.set_goal(diffdrive.current_position)
            navigation.calculate_shortest_path()
            diffdrive.create_robot_motion(navigation)
        elif commands['status'] == 1:
            # Go/Resume
            if memory:
                navigation.set_goal(memory[0])
                navigation.calculate_shortest_path()
                diffdrive.create_robot_motion(navigation)
                memory = []
            else:
                navigation.calculate_shortest_path()
                diffdrive.create_robot_motion(navigation)
        else:
            # Cancel
            navigation.set_goal(diffdrive.current_position)
            navigation.calculate_shortest_path()
            diffdrive.create_robot_motion(navigation)
    elif commands['type'] == SET_MAP:
        x = ceil(commands['real_position'][0]+250)
        y = ceil(commands['real_position'][1]+250)
        print('Map : ', x, y, commands['real_position'][2])
        navigation.set_position(x, y, commands['real_position'][2])
        if len(navigation.graph):
            if sum(commands['map'][251][250]) <= 128*3:
                obstacle = True
                navigation.del_node((251, 250))
            else:
                obstacle = False
        else:
            create_map(navigation, commands['map'])
    return json.dumps(response)

def main():

    rospy.init_node('planning_runner', anonymous=True)
    rospy.Service('planning', Planning, robot_state_callback)
    publish()


if __name__ == "__main__":
    print('started planning')
    main()
