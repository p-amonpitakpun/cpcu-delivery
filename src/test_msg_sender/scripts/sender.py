#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def sender():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = 'sent {}'.format(rospy.get_time())
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        sender()
    except rospy.ROSInterruptException as e:
        print(e)
