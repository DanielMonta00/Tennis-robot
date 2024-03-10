#!/usr/bin/env python 

# 3D curve fitting 
# Input data: 3D points (x, y, z) from OptiTrack
# To determine: size of a buffer to store the last n points, 
# fitting method, distance threshold for adding points to the buffer
# Output: Fitted curve in 3D space

# Optional output: 3D plot of the fitted curve and the points
# To do: publish the estimated parameters to a topic
# To do: create another node for the 3D plot
# To do: create a launch file to start both nodes

# The equation of the parabola in space is given by:
# (x,y,z) = (at^2 + bt + c, dt^2 + et + f, gt^2 + ht + i)
# where t is the time, and a, b, c, d, e, f, g, h, i are the coefficients to be determined
# Parabolas with gravity in the z direction and constant speed on xy plane are given by:

# (x,y,z) = (at + c, bt + d, -0.5gt^2+ht+i)

import rospy
from std_msgs.msg import String

import numpy as np 
from scipy.optimize import curve_fit 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D 

plt.ion()

class LIFOBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = []

    def push(self, item):
        if len(self.buffer) >= self.size:
            self.buffer.pop(0)
        self.buffer.append(item)

    def pop(self):
        return self.buffer.pop() if self.buffer else None

    def is_empty(self):
        return len(self.buffer) == 0

    def is_full(self):
        return len(self.buffer) == self.size
    
def parabola(t, a, b, c, d, e, h, i):
    x = a*t + b
    y = c*t + d
    z = e*t**2 + h*t + i
    return x, y, z

def xParab(t, a, b):
    x = a*t + b
    return x

def yParab(t,c, d):
    y = c*t + d
    return y

def zParab(t, e, h, i):
    z = e*t**2 + h*t + i
    return z

def unpack_buffer(buffer):
    t = np.array([item[0] for item in buffer.buffer])
    # first element of buffer is considered t=0
    t = t - t[0]
    x = np.array([item[1] for item in buffer.buffer])
    y = np.array([item[2] for item in buffer.buffer])
    z = np.array([item[3] for item in buffer.buffer])
    return t,x,y,z

# Fit the data to the parabola
def estimate_params(t, x, y, z): #, buffer, current_time):
    # t = np.array([item[0] for item in buffer.buffer])-current_time
    # x = np.array([item[1] for item in buffer.buffer])
    # y = np.array([item[2] for item in buffer.buffer])
    # z = np.array([item[3] for item in buffer.buffer])
    poptx, pcovx = curve_fit(xParab, t,x) 
    popty, pcovy = curve_fit(yParab, t,y) 
    poptz, pcovz = curve_fit(zParab, t,z) 
    poptx = poptx.tolist()
    popty = popty.tolist()
    poptz = poptz.tolist()
    estimated_params = np.array(poptx + popty + poptz)
    return estimated_params

def callback(data):
    fig=plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    # rospy.loginfo("I heard %s", data.data)
    #current_time = rospy.get_time()
    # add data to buffer
    float_data = tuple(map(float, data.data.strip('()').split(',')))
    buffer.push(float_data)
    # fit the data
    # unpack the buffer
    t,x,y,z = unpack_buffer(buffer)
    #np.array(buffer.buffer).T
    print("time")
    print(t)
    if buffer.is_full():
        #estimated_params=estimate_params(buffer,current_time)
        estimated_params=estimate_params(t,x,y,z)
        rospy.loginfo("I estimated %s", estimated_params.round(1))
        x_fit,y_fit,zfit = parabola(t,*estimated_params)
        #draw the parabola
        if animate:
            line=ax.plot(x_fit,y_fit, zfit, color='red')[0]
            line.set_xdata(x_fit)
            line.set_ydata(y_fit)
            line.set_3d_properties(zfit)
            fig.canvas.draw()
            fig.canvas.flush_events()
    if animate: 
        scat=ax.scatter(x,y,z,color='blue')
        #add one point to the plot
        scat.set_offsets(np.array([x, y, z]).T)
        fig.canvas.draw()
        fig.canvas.flush_events()
        

        # x_fit,y_fit,zfit = parabola(t,*estimated_params)
        # # Add one point to the plot
        # ax.scatter(x, y, z, color='blue')
        # ax.plot(x_fit,y_fit, zfit, color='red')   

def listen():
    rospy.init_node('listener', anonymous=True)
    rospy.loginfo("Listener node started.")
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

def animate_curve(x, y, z, x_fit, y_fit, zfit):
    pass

bufferSize = 10
buffer= LIFOBuffer(bufferSize)
animate = False

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
    
        
        




