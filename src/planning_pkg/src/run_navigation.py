#!/usr/bin/env python3

from rospy import Publisher, Rate, init_node, Service, is_shutdown
from json import loads, dumps
from time import time
from math import radians

from robot_state.srv import Planning
from std_msgs.msg import Float32MultiArray

from modules.planner import Planner
from modules.navigator import Navigator
from modules.position import Position
from modules.wrapper import point_wrapper, map_wrapper

# Response
IDLE = 0
WALKING = 1
PAUSED = 2

# Command
SET_GOAL = 0
MOVE = 1
SET_MAP = 2

COMMAND_PAUSE = 0
COMMAND_GO = 1
COMMAND_CANCEL = 2

# ROS Config
NODE_NAME = 'planning_runner'
SERVICE_NAME = 'planning'
PUBLISH_TO = 'robotControl'

POSITION_ERROR_THRES = 1000


class POSITION_ERROR_EXCEPTOON(Exception):
    pass


def validate_planned(planned, map):
    if not planned:
        return False
    for (x, y, direction) in planned:
        if map[y][x]:
            return False
    return True


def get_angle_diff(angle1, angle2):
    return (angle1 - angle2 + 180) % 360 - 180


class Node:

    def __init__(self):

        init_node(NODE_NAME, anonymous=True)
        Service(SERVICE_NAME, Planning, self.callback)

        self.planner = Planner()
        self.navigator = Navigator()
        self.position = Position()
        self.status = IDLE
        self.publisher = Publisher(
            PUBLISH_TO, Float32MultiArray, queue_size=10)
        self.rate = Rate(10)
        self.goal = None

    def callback(self, request):
        commands = loads(request.req)
        response = {'status': 'type error'}
        if commands['type'] == SET_GOAL:
            self.planner.update_goal((point_wrapper(
                commands['goal'][0]), point_wrapper(commands['goal'][1])))
            self.goal = commands['goal']
            response['status'] = 'ok'
        elif commands['type'] == MOVE:
            if commands['status'] == COMMAND_PAUSE:
                self.status = PAUSED
                response['status'] = 'ok'
            elif commands['status'] == COMMAND_GO:
                self.status = WALKING
                response['status'] = 'ok'
            elif commands['status'] == COMMAND_CANCEL:
                self.planner.clear()
                self.navigator.clear()
                self.status = IDLE
                response['status'] = 'ok'
        elif commands['type'] == SET_MAP:

            self.planner.update_position((point_wrapper(commands['occupancy_grid_position'][0]), 
            point_wrapper(commands['occupancy_grid_position'][1]), 
            commands['real_position'][2]))

            self.planner.update_map(map_wrapper(commands['map']))
            response['state'] = self.status
            response['target'] = self.goal
            response['path'] = list(self.planner.planned)
            response['status'] = 'ok'
        print(response)
        return dumps(response)


    def operate(self):
        planned_position = self.position.get_position(self.planner.current_position[0], self.planner.current_position[1])
        if self.position.threshold_check(POSITION_ERROR_THRES):
            self.publisher.publish(Float32MultiArray(data=[0, 0]))
            print('Node : Hm... where am I?')
            self.planner.plan()
            self.position.update_plan(self.planner.planned)
            planned_position = self.position.get_position(self.planner.current_position[0], self.planner.current_position[1])
        if self.position.at_goal((self.planner.current_position[0], self.planner.current_position[1])):
            self.status = IDLE
            return
        self.publisher.publish(Float32MultiArray(data=self.navigator.get_motor_speed(
            self.planner.current_position, planned_position, time())))

    def publish(self):
        while not is_shutdown():
            if self.status == IDLE or self.status == PAUSED:
                self.publisher.publish(Float32MultiArray(data=[0, 0]))
            elif self.status == WALKING:
                # if not validate_planned(self.planner.planned, self.planner.map):
                #     self.planner.plan()
                self.operate()
            self.rate.sleep()

    def run(self):
        self.publish()


if __name__ == '__main__':
    print('Started Planning')
    navigation_node = Node()
    navigation_node.run()
