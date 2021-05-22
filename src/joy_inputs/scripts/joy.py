#!/usr/bin/env python

import pprint
import rospy

from inputs import get_gamepad
from std_msgs.msg import Int16MultiArray


pp = pprint.PrettyPrinter(indent=4)

key_map = {
    # 'ABS_X': 0,
    'ABS_Y': 1,
    # 'ABS_RX': 2,
    'ABS_RY': 3,
}


def main():
    pub = rospy.Publisher('joy', Int16MultiArray, queue_size=10)
    rospy.init_node('joy', anonymous=True)
    print('started')
    while not rospy.is_shutdown():
        events = get_gamepad()
        for event in events:
            if event.code in key_map.keys():
                data = [key_map[event.code], int(event.state)]
                msg = Int16MultiArray(data=data)
                pub.publish(msg)
                pp.pprint(event.__dict__)
                pp.pprint(msg)
                print('\n')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
