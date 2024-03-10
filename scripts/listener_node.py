#!/usr/bin/env python 

# Simple example of a ROS node that listens to a topic

import rospy

from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def listen():
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo("Listener node started.")
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass

        