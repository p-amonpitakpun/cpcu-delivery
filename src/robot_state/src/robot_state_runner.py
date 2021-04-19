#!/usr/bin/env python3

import signal
import json
import pika
import queue
import rospy
import threading
import sys
from cv_bridge import CvBridge

CV_BRIDGE = CvBridge()
SET_GOAL = 0
MOVE = 1
SET_MAP = 2

#from sensor_msgs import Image
from robot_state.srv import Planning, Localization

def signal_handler(sig, frame):
    sys.exit(0)


def planning_callback(command):

    print('ROBOT_STATE: Call planning callback function')
    req = json.dumps(command)

    rospy.wait_for_service('planning')

    try:
        planning_req = rospy.ServiceProxy('planning', Planning)
        res = planning_req(req)
        return res.res
    except rospy.ServiceException as e:
        print("ROBOT_STATE: Service call failed: {}".format(e))

def locallization_call():

    print('ROBOT_STATE: Call localization for data')
    req = "calling"
    rospy.wait_for_service('localization')

    try:
        localization_req = rospy.ServiceProxy('localization', Localization)
        return_value = localization_req(req)
        return return_value.real_position, return_value.occupancy_grid_position, CV_BRIDGE.imgmsg_to_cv2(return_value.image).tolist()
    except rospy.ServiceException as e:
        print("ROBOT_STATE: Service call failed: {}".format(e))

def moveCommand_callback(ch, method, properties, body):
    print(" [x] Received move %r" % body)
    command_json = json.loads(body)
    planning_callback({
        "type": 0,
        "goal": command_json["destination"]
    })

def stopCommand_callback(ch, method, properties, body):
    print(" [x] Received stop %r" % body)
    command_json = json.loads(body)
    planning_callback({
        "type": 1,
        "status": 2
    })

def rabbitmq_thread():
    # RabbitMQ 
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()
    channel.queue_declare(queue='moveCommand')
    channel.basic_consume(queue='moveCommand', on_message_callback=moveCommand_callback, auto_ack=True)
    channel.queue_declare(queue='stopCommand')
    channel.basic_consume(queue='stopCommand', on_message_callback=stopCommand_callback, auto_ack=True)
    channel.start_consuming()

def main():

    print('ROBOT STATE ')

    rospy.init_node('robot_state_runner', anonymous=True)
    # rospy.Subscriber(topic, type, callback)
    rabbit_mq_thread = threading.Thread(target=rabbitmq_thread, args=())
    rabbit_mq_thread.start()

    rate = rospy.Rate(10)

    real_position = None
    occupancy_grid_position = None
    image = None

    while True:

        # Localization Data
        real_position, occupancy_grid_position, image = locallization_call()

        # Send the Localization Data to Planning
        command = {
            "type": SET_MAP,
            "payload": {
                "real_position": real_position,
                "occupancy_grid_position": occupancy_grid_position,
                "image": image
            }
        }
        planning_callback(command)

        # Handle Ctrl+c
        signal.signal(signal.SIGINT, signal_handler)

        # Sleep
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    print('started robot_state')
    main()

    
