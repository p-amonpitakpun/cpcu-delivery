#!/usr/bin/env python3

from rospy import Publisher, Rate, init_node, Service, is_shutdown
from json import loads, dumps
from time import time
from math import radians, degrees
import numpy as np

from robot_state.srv import Planning
import rospy
from std_msgs.msg import Float32MultiArray

from modules.planner import Planner
from modules.navigator import Navigator
from modules.position import Position

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

POSITION_ERROR_THRES = 100


class POSITION_ERROR_EXCEPTOON(Exception):
    pass


def validate_planned(planned, map):
    if not planned:
        return False
    for (x, y, direction) in planned:
        if not sum(map[y][x]):
            return False
    return True


def get_angle_diff(angle1, angle2):
    return (angle1 - angle2 + 180) % 360 - 180


class Node:

    def __init__(self):

        init_node(NODE_NAME, anonymous=True)
        Service(SERVICE_NAME, Planning, self.callback)
        rospy.Subscriber("pose", Float32MultiArray, self.pose_callback)

        self.planner = Planner()
        self.navigator = Navigator()
        self.position = Position()
        self.status = IDLE
        self.publisher = Publisher(
            PUBLISH_TO, Float32MultiArray, queue_size=10)
        self.rate = Rate(10)
        self.goal = None
        self.calculating = False

    def callback(self, request):
        commands = loads(request.req)
        response = {'status': 'type error'}
        if commands['type'] == SET_GOAL:
            self.planner.update_goal((
                commands['goal'][0], commands['goal'][1]))
            self.goal = commands['goal']
            self.calculating = True
            self.planner.plan()
            self.position.update_plan(self.planner.planned)
            self.calculating = False
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
            self.planner.update_map(commands['map'])
            response['state'] = self.status
            response['target'] = self.goal
            response['path'] = self.planner.get_path()
            response['status'] = 'ok'
        return dumps(response)


    def operate(self):
        if self.calculating:
            self.publisher.publish(Float32MultiArray(data=[0, 0]))
            return
        planned_position = self.position.get_position(self.planner.current_position)
        if self.position.threshold_check(POSITION_ERROR_THRES):
            self.publisher.publish(Float32MultiArray(data=[0, 0]))
            print('Node : Hm... where am I?')
            self.planner.plan()
            self.position.update_plan(self.planner.planned)
            planned_position = self.position.get_position(self.planner.current_position)
            return
        if not planned_position:
            self.status = IDLE
            self.publisher.publish(Float32MultiArray(data=[0, 0]))
            return
        self.publisher.publish(Float32MultiArray(data=self.navigator.get_motor_speed(
            self.planner.current_position, planned_position, time())))
    
    def pose_callback(self, input):
        new_pose = (input.data[3], input.data[4], (input.data[2] * 180 / np.pi + 360) % 360)
        self.planner.update_position(new_pose)

    def publish(self):
        while not is_shutdown():
            if self.status == IDLE or self.status == PAUSED:
                self.publisher.publish(Float32MultiArray(data=[0, 0]))
            elif self.status == WALKING:
                if not validate_planned(self.planner.planned, self.planner.map):
                    self.planner.plan()
                    self.position.update_plan(self.planner.planned)
                self.operate()
            self.rate.sleep()

    def run(self):
        self.publish()


if __name__ == '__main__':
    print('Started Planning')
    navigation_node = Node()
    navigation_node.run()
