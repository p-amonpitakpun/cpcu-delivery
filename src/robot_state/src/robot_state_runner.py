#!/usr/bin/env python3

import signal
import json
import pika
import queue
import rospy
import threading
import sys
import numpy
import base64
import io
from cv_bridge import CvBridge
from PIL import Image

CV_BRIDGE = CvBridge()
SET_GOAL = 0
MOVE = 1
SET_MAP = 2
RABBITMQ_CONNECTION_STRING = "amqp://admin:password@localhost:9998"

ROBOT_STATE_WEB_QUEUE = "robotState"
COMMAND_WEB_QUEUE = "webCommand"

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

def webCommand_callback(ch, method, properties, body):
    print(" [x] Received web command %r" % body)
    command_json = json.loads(body)
    planning_callback(command_json)

def get_rabbitmq_connection_channel():
    connection = pika.BlockingConnection(pika.URLParameters(RABBITMQ_CONNECTION_STRING))
    channel = connection.channel()
    return channel

def rabbitmq_thread():
    # RabbitMQ 
    channel = get_rabbitmq_connection_channel()
    channel.queue_declare(queue=COMMAND_WEB_QUEUE)
    channel.basic_consume(queue=COMMAND_WEB_QUEUE, on_message_callback=webCommand_callback, auto_ack=True)
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

    # Rabbitmq sending for main thread
    rabbit_channel = get_rabbitmq_connection_channel()
    rabbit_channel.queue_declare(queue='webCommand')

    while True:

        # Localization Data
        real_position, occupancy_grid_position, image = locallization_call()

        # Send the Localization Data to Planning
        command = {
            "type": SET_MAP,
            "real_position": real_position,
            "occupancy_grid_position": occupancy_grid_position,
            "map": image
        }
        res = planning_callback(command)

        # Send to web
        map_image = Image.fromarray(numpy.array(image).astype(numpy.uint8))
        img_byte_arr = io.BytesIO()
        map_image.save(img_byte_arr, format='PNG')
        command["map"] = base64.b64encode(img_byte_arr.getvalue()).decode("utf-8") 
        command["planning_state"] = json.loads(res) if res else {}

        rabbit_channel.basic_publish(
            exchange="",
            routing_key=ROBOT_STATE_WEB_QUEUE,
            body=json.dumps(command)
        )

        # Handle Ctrl+c
        signal.signal(signal.SIGINT, signal_handler)

        # Sleep
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    print('started robot_state')
    main()

    
