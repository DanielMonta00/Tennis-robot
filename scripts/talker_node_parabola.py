#!/usr/bin/env python 

# Generate noisy parabola data to simulate 3D points from OptiTrack
# Publish the noisy data to the topic "chatter"
# The data will be used by the curve_fitter_node to fit a parabola to the data

import rospy
import numpy as np
import random

from std_msgs.msg import String

# Generate parabola

nSamples = 50
t= np.linspace(-10, 10, nSamples)
params = [1, 0, -1, 0, -1, 0, 100]

# Parabola function
def parabola(t, a, b, c, d, e, h, i):
    x = a*t + b + np.random.normal(0, 0.1)
    y = c*t + d + np.random.normal(0, 0.1)
    z = e*t**2 + h*t + i + np.random.normal(0, 0.1)
    return x, y, z

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 1=10hz
    rospy.loginfo("Talker node started.")
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        for index in range(nSamples):
            hello_str = (rospy.get_time(),)+parabola(t[index], *params)
            hello_str = str(hello_str)
            pub.publish(hello_str)
            rospy.loginfo(hello_str)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

        